#!/usr/bin/env python3
"""Phase B: suspended fixed-base gravity validation (MuJoCo qfrc_bias).

For each suspended session (P55S, P85DS, OS) take per-frame measured pose
(12 leg joints, model coords) + measured base quat from the a1 npz inside the
refined stable window, set the MuJoCo model fixed base high (no contact),
compute gravity hold torque g(q, quat) = -qfrc_bias at each leg dof, and
compare with measured tau_est (model coords).

Outputs:
  runs/.../tables/B_SUSPENDED_GRAVITY_RESIDUAL_12JOINT.csv
  runs/.../tables/B_SUSPENDED_SUMMARY.json
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
WIN = {'P55S': (25, 90), 'P85DS': (48, 100), 'OS': (0, 60)}
SUB = 10  # subsample every Nth frame (50 Hz effective)


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    jids = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS}
    qadr = [m.jnt_qposadr[jids[n]] for n in LEGS]
    vadr = [m.jnt_dofadr[jids[n]] for n in LEGS]
    print('model loaded: nbody', m.nbody, 'total_mass', round(m.body_mass.sum(), 3),
          'gravity', m.opt.gravity)
    rows, summary = [], {}
    for tag, (t0, t1) in WIN.items():
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        t, q, tau, quat = z['t'], z['q'], z['tau'], z['quat']
        msk = (t >= t0) & (t <= t1)
        tt, qq, taum, qqm = t[msk][::SUB], q[msk][::SUB], tau[msk], quat[msk][::SUB]
        ncon_max = 0
        G = np.zeros((len(tt), 12))
        for i in range(len(tt)):
            d.qpos[:] = 0
            d.qvel[:] = 0
            d.qpos[0:3] = [0, 0, 0.5]  # high, no contact
            d.qpos[3:7] = qqm[i]       # measured base quat (wxyz)
            d.qpos[qadr] = qq[i]       # measured leg pose (model coords == MJCF convention)
            mujoco.mj_forward(m, d)
            assert d.ncon == 0, (tag, i, d.ncon)
            # holding torque at rest: qvel=0 -> equilibrium requires tau_applied = qfrc_bias
            # (gravity-only generalized force). NOTE earlier version used -qfrc_bias,
            # which matched the pre-sign-correction data only by a compensating error.
            G[i] = d.qfrc_bias[vadr]
        for j, jn in enumerate(LEGS):
            resid = taum[:, j].mean() - G[:, j].mean()
            rows.append(dict(
                session=tag, window=f'{t0}-{t1}', n_frames=int(len(tt)),
                joint=jn,
                tau_meas_mean=round(float(taum[:, j].mean()), 3),
                tau_meas_std=round(float(taum[:, j].std()), 3),
                g_hold_mean=round(float(G[:, j].mean()), 3),
                g_hold_std=round(float(G[:, j].std()), 3),
                resid_mean=round(float(resid), 3),
                resid_over_tau_pct=round(100 * resid / taum[:, j].mean(), 1)
                if abs(taum[:, j].mean()) > 0.3 else ''))
        hip = [LEGS.index(f'{l}_hip') for l in ('FL', 'FR', 'RL', 'RR')]
        summary[tag] = dict(
            window=(t0, t1), n_frames=int(len(tt)),
            tau_hip=[round(float(taum[:, j].mean()), 3) for j in hip],
            g_hold_hip=[round(float(G[:, j].mean()), 3) for j in hip],
            resid_hip=[round(float(taum[:, j].mean() - G[:, j].mean()), 3) for j in hip],
            tau_calf=[round(float(taum[:, 8 + k].mean()), 3) for k in range(4)],
            g_hold_calf=[round(float(G[:, 8 + k].mean()), 3) for k in range(4)],
            resid_calf=[round(float(taum[:, 8 + k].mean() - G[:, 8 + k].mean()), 3) for k in range(4)])
        print(tag, json.dumps(summary[tag]))
    out = RUNS / 'tables' / 'B_SUSPENDED_GRAVITY_RESIDUAL_12JOINT.csv'
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    json.dump(summary, open(RUNS / 'tables' / 'B_SUSPENDED_SUMMARY.json', 'w'), indent=1)
    print('wrote', out, len(rows))


if __name__ == '__main__':
    main()
