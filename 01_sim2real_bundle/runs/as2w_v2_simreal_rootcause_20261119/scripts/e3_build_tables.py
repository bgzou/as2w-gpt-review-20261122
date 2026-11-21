#!/usr/bin/env python3
"""Phase E: build the two cross-engine tables.

  ALL_12JOINT_ENGINE_COMPARISON.csv  real vs MuJoCo C1/C1m/C1m_mass vs PhysX,
                                    12 leg joints x 3 scenarios (tau and q)
  PHYSX_MUJOCO_ASSET_PARITY.csv     asset-level parity (mass, COM, inertia,
                                    wheel collision, joint axes/limits, solver)

Real windows: P55S 25-90 (S), P55G2 8-19 (G), OG 17-120 (GO).
"""
import csv
import json
import pathlib

import numpy as np

RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
T = RUNS / 'tables'
LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
SCEN = {'S': ('P55S', 25, 90), 'G': ('P55G2', 8, 19), 'GO': ('OG', 17, 120)}


def real_means():
    out = {}
    for s, (tag, lo, hi) in SCEN.items():
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        m = (z['t'] >= lo) & (z['t'] <= hi)
        out[s] = dict(tau=z['tau'][m].mean(0), q=z['q'][m].mean(0))
    return out


def main():
    real = real_means()
    muj = list(csv.DictReader(open(T / 'MUJOCO_REAL_SIM_MATRIX.csv')))
    muj_lv = json.load(open(T / 'E_MUJOCO_LOADV0.json'))
    physx = {s: json.load(open(T / f'E_PHYSX_{s}.json')) for s in ('S', 'G', 'GO')}

    def muj_vec(row, key):
        # CSV stores hip as 4 scalar cols, thigh/calf as python-list strings
        if key == 'tau':
            hip = [float(row[f'hip_tau_{l}']) for l in ('FL', 'FR', 'RL', 'RR')]
            return hip + eval(row['thigh_tau']) + eval(row['calf_tau'])
        return [float(x) for x in eval(row['hip_q'])] + [None] * 8

    rows = []
    for s in ('S', 'G', 'GO'):
        c1 = next(r for r in muj if r['scen'] == s and r['variant'] == 'C1')
        c1t = muj_vec(c1, 'tau')
        c1q = muj_vec(c1, 'q')
        c1m = muj_lv['c1m'][s]
        c1mm = muj_lv['c1m_mass'][s]
        px = physx[s]
        for i, jn in enumerate(LEGS):
            rows.append([
                s, jn,
                round(float(real[s]['tau'][i]), 3), round(float(real[s]['q'][i]), 4),
                round(float(c1t[i]), 3), (round(float(c1q[i]), 4) if c1q[i] is not None else ''),
                round(float(c1m['hip_tau' if i < 4 else ('thigh_tau' if i < 8 else 'calf_tau')][i % 4]), 3),
                round(float(c1m['hip_q'][i % 4] if i < 4 else (c1m['hip_q'][i % 4])), 4) if i < 4 else '',
                round(float(c1mm['hip_tau' if i < 4 else ('thigh_tau' if i < 8 else 'calf_tau')][i % 4]), 3),
                round(float(c1mm['hip_q'][i % 4]), 4) if i < 4 else '',
                round(float(px['tau_legs'][i]), 3), round(float(px['q_legs'][i]), 4),
            ])
    with open(T / 'ALL_12JOINT_ENGINE_COMPARISON.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['scen', 'joint', 'real_tau', 'real_q',
                    'mujoco_C1_tau', 'mujoco_C1_q', 'mujoco_C1m_tau', 'mujoco_C1m_q',
                    'mujoco_C1m_mass_tau', 'mujoco_C1m_mass_q', 'physx_tau', 'physx_q'])
        w.writerows(rows)
    print('wrote ALL_12JOINT_ENGINE_COMPARISON.csv', len(rows), 'rows')

    # ---- asset parity ----
    px = physx['G']
    parity = [
        ['total_mass_kg', '27.512', '26.0 (10.08 base)', '27.512', '26.0 (255.06 N model weight)',
         'PhysX read-back; C1m matches load-v0, C1 is official'],
        ['base_body_mass_kg', '12.092', '10.58 (10.08 + 0.5 JT128)', '12.092 (11.592 + 0.5 JT128)',
         '10.58 official', 'PhysX merges fixed JT128 into base_link'],
        ['base_com_xyz_m', '0.02999 -0.00071 0.02008', '0.035002 -0.0007562 0.017521',
         '0.020002 -0.0007562 0.017521 (base) + JT128 separate',
         '0.035002 -0.0007562 0.017521 official',
         'PhysX read-back COM is the MERGED base+JT128 COM; whole-body COM equals C1m'],
        ['base_inertia_kgm2', 'from URDF load-v0', '0.05069483 0.13822065 0.1505824 -0.0002906 -0.02480435 0.00027717',
         '0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455',
         'official 0.05069483 ...', 'load-v0 = official x1.15 mass scaling'],
        ['leg_body_masses_kg', 'hip .75 thigh 1.41 calf 1.1 wheel .595', 'same', 'same', 'same',
         'identical across all variants'],
        ['wheel_collision', 'STL mesh FL_foot.STL bbox y[0.0287,0.0805] z+-0.086',
         'cylinder r0.086 h0.052 at body origin (C0)', 'cylinder r0.086 h0.052, pos y+-0.0546 (C1/C1m)',
         'true wheel centre', 'PhysX already carries the H5-correct geometry'],
        ['effective_track_width_m', '0.44 (body 0.33 + 2x0.0546)', '0.3308 (C0)', '0.4400 (C1)',
         '0.44 (wheel STL)', 'PhysX = C1 geometry'],
        ['joint_axes', 'hip (1,0,0); thigh/calf/wheel (0,1,0)', 'same', 'same', 'same', 'identical'],
        ['joint_limits_rad', 'hip +-0.785; thigh FL/FR -2..3.492, RL/RR -1.3..4.19; calf -2.7677..-0.8133',
         'same', 'same', 'same', 'identical'],
        ['effort_limits_Nm', 'hip/thigh 60 calf 90 wheel 15', 'same', 'same', 'same', 'identical'],
        ['joint_damping_armature_frictionloss', 'none in URDF (0/0/0)',
         '0.1 / 0.01 / 0.05 (legs), 0.05 / 0.005 / 0.01 (wheel)',
         'same as C0', 'n/a', 'MJCF adds damping/armature/frictionloss absent from URDF'],
        ['contact_model', 'convex mesh + default material', 'cylinder condim=4 friction 1.0 0.05 0.001',
         'same as C0', 'real tyre', 'different contact primitive and friction model'],
        ['timestep_s', '0.005 (sim), decimation 4', '0.002 implicitfast', '0.002', 'n/a',
         'different integrators'],
        ['PD_contract', 'implicit drive kp55/kd2 (T2) or kp200/kd4 (official)',
         'explicit qfrc_applied kp55/kd2 (T2) or kp200/kd4 (official)', 'same', 'measured kp/kd',
         'same gains, different actuation path'],
    ]
    with open(T / 'PHYSX_MUJOCO_ASSET_PARITY.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['item', 'physx_training_asset', 'mujoco_C0_delivery', 'mujoco_C1m',
                    'real', 'note'])
        w.writerows(parity)
    print('wrote PHYSX_MUJOCO_ASSET_PARITY.csv', len(parity), 'rows')


if __name__ == '__main__':
    main()
