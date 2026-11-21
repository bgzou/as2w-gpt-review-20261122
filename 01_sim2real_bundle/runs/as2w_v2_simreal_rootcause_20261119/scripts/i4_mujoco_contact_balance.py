#!/usr/bin/env python3
"""Phase I4 (MuJoCo side): contact manifold extraction + static balance residual.

Config: M_cylwheel, G scenario, explicit PD kp55/kd2, real init (G3 contract),
20000 steps, last 20% window.  For every wheel contact point we record:
  world position, contact normal, normal force, 2 friction components,
  torsion, rolling torque (mj_contactForce 6-vector in contact frame).
Then compute the whole-body static balance residual over the settled window:
  sum of contact forces + gravity  -> force residual
  sum of (r_i x f_i) about the base CoM -> torque residual
and per-wheel force split (Fn/Ft, contact point spread = manifold extent).

Output: tables/I4_MUJOCO_CONTACT_BALANCE.{csv,json}
Run: /home/robot/z00945176/isaac_env/venv312/bin/python i4_mujoco_contact_balance.py
"""
import csv
import json
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
XML = RUNS / 'mujoco_abl' / 'M_cylwheel' / 'as2w_scene.xml'
NPZ = RUNS / 'qc' / 'a1_timelines' / 'P55G2.npz'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
TGT = np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])
KP, KD = 55.0, 2.0
KP_W = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}


def real_init_q():
    z = np.load(NPZ)
    t, q = z['t'], z['q']
    m = (t >= 8.0) & (t <= 19.0)
    return [round(float(x), 6) for x in q[m].mean(0)]


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    m.actuator_gainprm[:, :] = 0.0
    m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    wheel_bid = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE}

    d.qpos[:] = 0
    d.qpos[3] = 1.0
    q0 = real_init_q()
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = q0[i]
    mujoco.mj_forward(m, d)
    # lowest wheel point -> 2 mm above ground
    lo = 1e9
    for w in WHE:
        bid = wheel_bid[w]
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == bid and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                lo = min(lo, d.geom_xpos[g][2] - m.geom_size[g][0])
    d.qpos[2] += (0.002 - lo)
    mujoco.mj_forward(m, d)

    nstep = 20000
    rec_from = int(nstep * 0.8)
    for step in range(nstep):
        tau = np.zeros(m.nv)
        for i, n in enumerate(LEGS):
            kind = n.split('_')[1]
            e = TGT[i] - d.qpos[qadr[n]]
            tau[vadr[n]] = np.clip(KP * e + KD * (-d.qvel[vadr[n]]),
                                   -KP_W[kind], KP_W[kind])
        for n in WHE:
            tau[vadr[n]] = np.clip(2.0 * (-d.qvel[vadr[n]]), -15, 15)
        d.qfrc_applied[:] = 0
        idx = [vadr[n] for n in LEGS + WHE]
        d.qfrc_applied[idx] = tau[idx]
        mujoco.mj_step(m, d)

    # settled-window manifold statistics
    mujoco.mj_forward(m, d)
    recs = []          # per contact-point per-frame
    per_frame_F = []
    per_frame_T = []
    mass_total = m.body_mass.sum()
    G = np.array([0, 0, -9.81]) * mass_total
    for _ in range(200):
        # step 10 more each frame (keep dynamics alive, sample 200 frames)
        for _ in range(10):
            tau = np.zeros(m.nv)
            for i, n in enumerate(LEGS):
                kind = n.split('_')[1]
                e = TGT[i] - d.qpos[qadr[n]]
                tau[vadr[n]] = np.clip(KP * e + KD * (-d.qvel[vadr[n]]),
                                       -KP_W[kind], KP_W[kind])
            for n in WHE:
                tau[vadr[n]] = np.clip(2.0 * (-d.qvel[vadr[n]]), -15, 15)
            d.qfrc_applied[:] = 0
            d.qfrc_applied[idx] = tau[idx]
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        com_w = (d.xipos * m.body_mass[:, None]).sum(0) / mass_total
        Fsum = np.zeros(3)
        Tsum = np.zeros(3)
        for ci in range(d.ncon):
            c = d.contact[ci]
            g1, g2 = c.geom1, c.geom2
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            wheel = None
            for w in WHE:
                if b1 == wheel_bid[w] or b2 == wheel_bid[w]:
                    wheel = w
            if wheel is None:
                continue
            f6 = np.zeros(6)
            mujoco.mj_contactForce(m, d, ci, f6)
            # contact frame: normal is c.frame[0:3]
            nrm = np.array(c.frame[0:3])
            t1 = np.array(c.frame[3:6])
            t2 = np.array(c.frame[6:9])
            # f6 = [normal, tangent1, tangent2, torsion, roll1, roll2]
            f_world = f6[0] * nrm + f6[1] * t1 + f6[2] * t2
            pos = np.array(c.pos)
            Fsum += f_world
            Tsum += np.cross(pos - com_w, f_world)
            recs.append(dict(frame=_, wheel=wheel, pos=pos.tolist(),
                             normal=nrm.tolist(), fn=float(f6[0]),
                             ft1=float(f6[1]), ft2=float(f6[2]),
                             torsion=float(f6[3]), roll1=float(f6[4]),
                             roll2=float(f6[5]),
                             dist=int(c.dist)))
        per_frame_F.append(Fsum)
        per_frame_T.append(Tsum)

    per_frame_F = np.array(per_frame_F)
    per_frame_T = np.array(per_frame_T)
    # static balance: contact forces must cancel gravity (Fsum + G ~ 0)
    Fres = per_frame_F + G
    F_res_norm = np.linalg.norm(Fres, axis=1)
    T_res_norm = np.linalg.norm(per_frame_T, axis=1)

    # per-wheel summary
    wheel_summary = {}
    for w in WHE:
        rr = [r for r in recs if r['wheel'] == w]
        if not rr:
            wheel_summary[w] = dict(n_points_mean=0.0)
            continue
        npts = len(rr) / 200.0
        fn = np.mean([r['fn'] for r in rr])
        ft = np.mean([np.hypot(r['ft1'], r['ft2']) for r in rr])
        pts = np.array([r['pos'] for r in rr])
        spread = float(np.max(np.linalg.norm(pts - pts.mean(0), axis=1)))
        wheel_summary[w] = dict(
            n_points_mean=round(float(npts), 2),
            Fn_mean=round(float(fn), 2),
            Ft_mean=round(float(ft), 3),
            mu_used=round(float(ft / fn), 3) if fn > 1e-6 else None,
            contact_spread_m=round(spread, 5),
            dist_mean=round(float(np.mean([r['dist'] for r in rr])), 5),
            torsion_mean=round(float(np.mean([r['torsion'] for r in rr])), 5),
        )

    out = dict(
        config='M_cylwheel G explicit PD kp55/kd2 real init',
        total_mass_kg=round(float(mass_total), 3),
        gravity_N=round(float(np.linalg.norm(G)), 2),
        force_residual_N=dict(
            mean=round(float(F_res_norm.mean()), 3),
            max=round(float(F_res_norm.max()), 3),
            p95=round(float(np.percentile(F_res_norm, 95)), 3)),
        torque_residual_Nm=dict(
            mean=round(float(T_res_norm.mean()), 3),
            max=round(float(T_res_norm.max()), 3),
            p95=round(float(np.percentile(T_res_norm, 95)), 3)),
        per_wheel=wheel_summary,
        n_contact_points_total=int(len(recs)),
        notes='MuJoCo gives per-point manifold; PhysX side may only expose net force (UNKNOWN).',
    )
    with open(RUNS / 'tables' / 'I4_MUJOCO_CONTACT_BALANCE.json', 'w') as f:
        json.dump(out, f, indent=1)
    with open(RUNS / 'tables' / 'I4_MUJOCO_CONTACT_BALANCE.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['wheel', 'n_points_mean', 'Fn_mean_N', 'Ft_mean_N',
                    'mu_used', 'contact_spread_m', 'dist_mean_m', 'torsion_mean'])
        for wname, v in wheel_summary.items():
            w.writerow([wname, v.get('n_points_mean'), v.get('Fn_mean'),
                        v.get('Ft_mean'), v.get('mu_used'),
                        v.get('contact_spread_m'), v.get('dist_mean'),
                        v.get('torsion_mean')])
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
