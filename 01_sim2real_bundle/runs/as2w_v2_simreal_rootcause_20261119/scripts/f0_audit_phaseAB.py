#!/usr/bin/env python3
"""Phase A/B audit (read-only): hip-sign physical anchor, tau_est provenance
checks, static-balance completeness (qfrc_bias vs qfrc_passive/constraint),
base-attitude input, residual motion in the refined windows.

Outputs:
  runs/.../tables/F_AB_AUDIT.json
Run: /home/robot/z00945176/isaac_env/venv312/bin/python
"""
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
WIN = {'P55S': (25, 90), 'P85DS': (48, 100), 'OS': (0, 60)}


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS}
    wbi = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE}
    hbi = {f'{l}_hip': mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{l}_hip')
           for l in ('FL', 'FR', 'RL', 'RR')}
    out = dict(model=dict(nq=m.nq, nv=m.nv, mass=float(m.body_mass.sum()),
                          jnt_stiffness_nonzero=int(np.count_nonzero(m.jnt_stiffness)),
                          dof_damping_max=float(m.dof_damping.max()),
                          dof_frictionloss_max=float(m.dof_frictionloss.max()),
                          timestep=m.opt.timestep, integrator=int(m.opt.integrator)))

    # --- 1. hip-sign physical anchor: which q sign moves the wheel INWARD? ---
    # hip axis +x; leg hangs from hip body at y=+0.06 (left) / -0.06 (right).
    anchor = {}
    for leg, side in (('FL', +1), ('FR', -1)):
        row = {}
        for q in (-0.10, 0.0, +0.10):
            d.qpos[:] = 0
            d.qpos[3] = 1.0
            d.qpos[2] = 0.5
            d.qpos[qadr[f'{leg}_thigh']] = 0.8
            d.qpos[qadr[f'{leg}_calf']] = -1.6
            d.qpos[qadr[f'{leg}_hip']] = q
            mujoco.mj_forward(m, d)
            wy = float(d.xpos[wbi[f'{leg}_foot']][1])
            hy = float(d.xpos[hbi[f'{leg}_hip']][1])
            row[f'q={q:+.2f}'] = round(wy - hy, 4)   # + = outward for left leg
        anchor[leg] = row
    out['hip_sign_anchor_wheel_y_minus_hip_y_m'] = anchor
    out['hip_sign_anchor_note'] = ('left leg (FL): q<0 must move wheel INWARD (dy<0) '
                                   'because gravity sags the leg under the pivot')

    # --- 2. per-session window: residual motion, base attitude, passive terms ---
    sess = {}
    for tag, (t0, t1) in WIN.items():
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        t, q, dq, tau, quat = z['t'], z['q'], z['dq'], z['tau'], z['quat']
        msk = (t >= t0) & (t <= t1)
        qm, dqm, taum = q[msk], dq[msk], tau[msk]
        qqm = quat[msk] / np.linalg.norm(quat[msk], axis=1, keepdims=True)
        # base attitude from measured quat (wxyz)
        pitch = np.degrees(np.arcsin(np.clip(
            2 * (qqm[:, 0] * qqm[:, 2] - qqm[:, 3] * qqm[:, 1]), -1, 1)))
        # gravity hold torque with measured quat, no contact
        G = np.zeros((len(qm), 12))
        pas = np.zeros_like(G)
        for i in range(0, len(qm), 10):
            d.qpos[:] = 0
            d.qvel[:] = 0
            d.qpos[2] = 0.6
            d.qpos[3:7] = qqm[i]
            for j, n in enumerate(LEGS):
                d.qpos[qadr[n]] = qm[i, j]
            mujoco.mj_forward(m, d)
            assert d.ncon == 0
            G[i] = d.qfrc_bias[[vadr[n] for n in LEGS]]
            pas[i] = d.qfrc_passive[[vadr[n] for n in LEGS]]
        sel = np.arange(0, len(qm), 10)
        sess[tag] = dict(
            n_frames=int(msk.sum()),
            dq_abs_p95=[round(float(np.percentile(np.abs(dqm[:, j]), 95)), 4) for j in range(12)],
            dq_abs_max=[round(float(np.abs(dqm[:, j]).max()), 4) for j in range(12)],
            q_std=[round(float(qm[:, j].std()), 4) for j in range(12)],
            pitch_mean_deg=round(float(pitch.mean()), 3),
            pitch_std_deg=round(float(pitch.std()), 3),
            roll_mean_deg=round(float(np.degrees(np.arctan2(
                2 * (qqm[:, 0] * qqm[:, 1] + qqm[:, 2] * qqm[:, 3]),
                1 - 2 * (qqm[:, 1] ** 2 + qqm[:, 2] ** 2))).mean()), 3),
            quat_norm_max_dev=round(float(np.abs(np.linalg.norm(quat[msk], axis=1) - 1).max()), 6),
            g_hold_mean=[round(float(G[sel][:, j].mean()), 3) for j in range(12)],
            g_hold_std=[round(float(G[sel][:, j].std()), 3) for j in range(12)],
            passive_abs_max=round(float(np.abs(pas).max()), 6),
            tau_mean=[round(float(taum[:, j].mean()), 3) for j in range(12)],
            resid_mean=[round(float(taum[:, j].mean() - G[sel][:, j].mean()), 3) for j in range(12)],
        )
        print(tag, 'pitch', sess[tag]['pitch_mean_deg'], 'dqP95max',
              max(sess[tag]['dq_abs_p95']), 'passive', sess[tag]['passive_abs_max'])
    out['sessions'] = sess

    # --- 3. tau_est provenance: PD residual independence check ---
    # tau_est is the driver estimate. If it were a pure PD recomputation, then
    # tau - kp*(tgt-q) would be exactly 0 by construction; report the actual
    # residual and the ratio tau/(kp*e) for the hipdiag sessions.
    pd = {}
    for tag, kp in (('P55S', 55.0), ('P85DS', 85.0)):
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        t, q, tau = z['t'], z['q'], z['tau']
        t0, t1 = WIN[tag]
        msk = (t >= t0) & (t <= t1)
        tgt = np.array([0, 0, 0, 0] + [0.8] * 4 + [-1.6] * 4)
        e = tgt[None, :] - q[msk]
        r = tau[msk] - kp * e
        pd[tag] = dict(kp=kp,
                       resid_hip=[round(float(r[:, j].mean()), 3) for j in range(4)],
                       resid_absmax=[round(float(np.abs(r[:, j]).max()), 3) for j in range(12)],
                       ratio_tau_over_kpe_hip=[round(float((tau[msk][:, j].mean())
                                                          / (kp * e[:, j].mean())), 3) for j in range(4)]
                       if np.all(np.abs(e[:, :4].mean(0)) > 1e-6) else None)
    out['pd_residual'] = pd
    print('pd residual', json.dumps(pd))

    (RUNS / 'tables' / 'F_AB_AUDIT.json').write_text(json.dumps(out, indent=1))
    print('wrote F_AB_AUDIT.json')


if __name__ == '__main__':
    main()
