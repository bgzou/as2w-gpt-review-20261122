#!/usr/bin/env python3
"""Phase C: ground 6D-base + 12-joint force ledger (O-G + 3xP55-G).

Per ground session (window means): model pose = measured q12 + measured quat,
base snapped so lowest wheel touches z=0. Static inversion of wheel contact
forces lambda (4x3) from measured joint torques, under three models:
  M0: leg 12 equations only, min-norm exact solve (base wrench unchecked)
  M1: full 18 equations (6 base + 12 legs) least squares
  M2: M1 with per-joint motor/estimator bias (from suspended P55S/P85DS
      resid mean) subtracted from measured tau before inversion
Also: Fy clamping magnitude needed per wheel to explain hip tau via pure
lateral internal force (nullspace direction), recomputed on V2 poses.

Outputs:
  runs/.../tables/GROUND_FORCE_INFERENCE.csv
  runs/.../tables/JOINT_TORQUE_DECOMPOSITION.csv
  runs/.../tables/C_GROUND_SUMMARY.json
Run: /home/robot/z00945176/isaac_env/venv312/bin/python
"""
import csv
import json
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
XML = REPO / 'deliveries/as2w-t15-layered-delivery-20261117/simulation/mujoco/as2w_scene.xml'
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
R_WHEEL = 0.086
WIN = {'OG': (17, 120), 'P55G1': (10, 30), 'P55G2': (8, 19), 'P55G3': (7, 19)}


def load_bias():
    """Per-joint tau bias = mean over P55S,P85DS of (tau_meas - g_hold)."""
    bias = {}
    with open(RUNS / 'tables' / 'B_SUSPENDED_GRAVITY_RESIDUAL_12JOINT.csv') as f:
        for r in csv.DictReader(f):
            if r['session'] in ('P55S', 'P85DS'):
                bias.setdefault(r['joint'], []).append(
                    (float(r['tau_meas_mean']) - float(r['g_hold_mean'])))
    return {j: float(np.mean(v)) for j, v in bias.items()}


def main():
    bias = load_bias()
    print('motor/estimator bias (N=2 suspended sessions):')
    for j in LEGS:
        print(f'  {j}: {bias[j]:+.3f}')
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS}
    wbi = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE]
    legv = [vadr[n] for n in LEGS]

    force_rows, tau_rows, summary = [], [], {}
    for tag, (t0, t1) in WIN.items():
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        t, q, tau, quat = z['t'], z['q'], z['tau'], z['quat']
        msk = (t >= t0) & (t <= t1)
        qm = q[msk].mean(0)            # 12 joints, model coords
        taum = tau[msk].mean(0)
        quatm = quat[msk].mean(0)
        quatm /= np.linalg.norm(quatm)
        # pose
        d.qpos[:] = 0
        d.qvel[:] = 0
        d.qpos[3:7] = quatm
        for i, n in enumerate(LEGS):
            d.qpos[qadr[n]] = qm[i]
        mujoco.mj_forward(m, d)
        pts = [d.xpos[b] - np.array([0, 0, R_WHEEL]) for b in wbi]
        d.qpos[2] -= min(p[2] for p in pts)  # snap base down so lowest wheel at 0
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        pts = [d.xpos[b] - np.array([0, 0, R_WHEEL]) for b in wbi]
        heights = [p[2] for p in pts]
        g = d.qfrc_bias.copy()  # gravity (qvel=0); includes base rows
        # contact jacobians (translational) at each wheel bottom point
        J = []
        for i in range(4):
            jacp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jacp, np.zeros((3, m.nv)), pts[i], wbi[i])
            J.append(jacp)
        A_base = np.zeros((6, 12))
        A_leg = np.zeros((12, 12))
        for i in range(4):
            A_base[:, 3 * i:3 * i + 3] = J[i][:, :6].T
            A_leg[:, 3 * i:3 * i + 3] = -J[i][:, legv].T
        # required leg torque: contact generalized force cancels gravity
        b_leg = g[legv]                 # gravity generalized force on legs
        b_base = g[:6]
        results = {}
        for model in ('M0', 'M1', 'M2'):
            tau_in = taum - np.array([bias[n] for n in LEGS]) if model == 'M2' else taum
            if model == 'M0':
                lam = np.linalg.lstsq(A_leg, tau_in - b_leg, rcond=None)[0]
                leg_res = A_leg @ lam - (tau_in - b_leg)
                base_res = A_base @ lam - b_base
            else:
                A = np.vstack([A_base, A_leg])
                bb = np.concatenate([b_base, tau_in - b_leg])
                lam = np.linalg.lstsq(A, bb, rcond=None)[0]
                leg_res = A_leg @ lam - (tau_in - b_leg)
                base_res = A_base @ lam - b_base
            results[model] = dict(
                lam=lam.copy(),
                leg_res=float(np.max(np.abs(leg_res))),
                base_res=float(np.max(np.abs(base_res))),
                Fn=[float(lam[3 * i + 2]) for i in range(4)],
                Fx=[float(lam[3 * i]) for i in range(4)],
                Fy=[float(lam[3 * i + 1]) for i in range(4)])
            for i, w in enumerate(WHE):
                force_rows.append(dict(
                    session=tag, window=f'{t0}-{t1}', model=model, wheel=w,
                    Fn_N=round(results[model]['Fn'][i], 2),
                    Fx_N=round(results[model]['Fx'][i], 2),
                    Fy_N=round(results[model]['Fy'][i], 2),
                    wheel_bottom_height_mm=round(heights[i] * 1000, 1),
                    leg_eq_max_resid_Nm=round(results[model]['leg_res'], 3),
                    base_eq_max_resid=round(results[model]['base_res'], 3)))
        # torque decomposition (use M1 lambda, unbiased tau for table)
        lam1 = results['M1']['lam']
        for j, jn in enumerate(LEGS):
            c_tau = float(A_leg[j] @ lam1)
            tau_rows.append(dict(
                session=tag, joint=jn,
                tau_meas_Nm=round(float(taum[j]), 3),
                tau_bias_Nm=round(bias[jn], 3),
                g_hold_gravity_Nm=round(float(b_leg[j]), 3),
                tau_from_contact_Nm=round(c_tau, 3),
                tau_recon_Nm=round(float(b_leg[j] + c_tau), 3),
                recon_resid_Nm=round(float(taum[j] - (b_leg[j] + c_tau)), 3)))
        # Fy clamping magnitude: pure lateral (all wheels +y or paired) internal
        # force col at hips: solve scalars s.t. A_leg @ lam_fy best matches
        # (tau - g - b) restricted to hip rows
        Fy_dirs = []
        for i in range(4):
            Fy_dirs.append(np.concatenate([np.zeros(3 * i), np.array([0, 1, 0]), np.zeros(3 * (3 - i))]))
        D = np.stack(Fy_dirs, 1)  # 12 x 4
        hip_rows = [0, 1, 2, 3]
        rhs = taum - b_leg
        AA = (A_leg[hip_rows] @ D)  # 4x4
        coef = np.linalg.lstsq(AA, rhs[hip_rows], rcond=None)[0]
        summary[tag] = dict(
            window=(t0, t1), pitch_deg=round(float(np.degrees(np.arcsin(
                np.clip(2 * (quatm[0] * quatm[2] - quatm[3] * quatm[1]), -1, 1)))), 2),
            tau_hip=[round(float(x), 3) for x in taum[:4]],
            g_hip=[round(float(x), 3) for x in b_leg[:4]],
            M1_Fn=[round(x, 2) for x in results['M1']['Fn']],
            M1_Fx=[round(x, 2) for x in results['M1']['Fx']],
            M1_Fy=[round(x, 2) for x in results['M1']['Fy']],
            M1_Fn_sum=round(float(sum(results['M1']['Fn'])), 2),
            weight_N=round(float(m.body_mass.sum() * 9.81), 2),
            M1_base_resid=round(results['M1']['base_res'], 3),
            M2_Fn=[round(x, 2) for x in results['M2']['Fn']],
            M2_Fy=[round(x, 2) for x in results['M2']['Fy']],
            M2_base_resid=round(results['M2']['base_res'], 3),
            Fy_clamp_fit_per_wheel=[round(float(c), 2) for c in coef])
        print(tag, json.dumps(summary[tag]))

    def wcsv(path, rows):
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(path, 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=keys)
            wr.writeheader()
            wr.writerows(rows)
        print('wrote', path, len(rows))
    wcsv(RUNS / 'tables' / 'GROUND_FORCE_INFERENCE.csv', force_rows)
    wcsv(RUNS / 'tables' / 'JOINT_TORQUE_DECOMPOSITION.csv', tau_rows)
    json.dump(summary, open(RUNS / 'tables' / 'C_GROUND_SUMMARY.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
