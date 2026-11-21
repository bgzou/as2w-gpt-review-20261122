#!/usr/bin/env python3
"""Phase G / G2 (read-only): per-session FK reconstruction of wheel pose from
REAL timeline data (window-mean q + base quat), and audit of the contact-point
algorithms used in Phase C (naive / f1 tilt-nominal-bottom / analytic tilted
finite-width cylinder).

Per wheel:
  c      = wheel centre (world) = xpos[foot] + R_foot @ [0, s*0.0546, 0]  (C1 real geometry)
  a      = wheel axis (world)   = R_foot @ [0,1,0]
  tilt   = asin(|a_z|)                       -- wheel-plane inclination, NOT hip q
  analytic lowest point of tilted finite-width cylinder (r, half-width w):
     p = c - w*sign(a_z)*a + r*(a*a_z - z_hat)/sqrt(1 - a_z^2)
     min_z = c_z - w*|a_z| - r*sqrt(1 - a_z^2)
  f1 nominal-bottom point (tilt_C1 variant of f1_audit_phaseC.py):
     p_f1 = c + R_foot @ [0, 0, -0.086]
  naive point (old c1_ground_ledger.py):
     p_naive = xpos[foot] - [0, 0, 0.086]
Base z is normalised so the lowest analytic wheel point touches z=0 (ground
plane horizontal in world; no base position in the npz - clearances are
relative and stated as such).

Sessions/windows are exactly the Phase B/C audit windows (f0/f1).
Outputs: tables/G2_WHEEL_POSE_FK.csv, tables/G2_WHEEL_POSE_FK.json
Run: /home/robot/z00945176/isaac_env/venv312/bin/python   (mujoco only; conda py also OK)
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
SIDE = {'FL': +1, 'FR': -1, 'RL': +1, 'RR': -1}
R_WHEEL = 0.086
W_HALF = 0.026          # cylinder size half-width 52 mm -> half 0.026
OFF_Y = 0.0546          # true wheel-centre lateral offset (STL bbox)
WIN = {'OG': (17, 120), 'P55G1': (10, 30), 'P55G2': (8, 19), 'P55G3': (7, 19),
       'P55S': (25, 90), 'P85DS': (48, 100), 'OS': (0, 60)}


def build(tag):
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS}
    wbi = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE]
    z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
    t, q, quat = z['t'], z['q'], z['quat']
    msk = (t >= WIN[tag][0]) & (t <= WIN[tag][1])
    qm = q[msk].mean(0)
    qstd = q[msk].std(0)
    qqm = quat[msk].mean(0)
    qqm /= np.linalg.norm(qqm)
    d.qpos[:] = 0
    d.qvel[:] = 0
    d.qpos[3:7] = qqm
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = qm[i]
    mujoco.mj_forward(m, d)
    return m, d, wbi, qm, qstd, msk.sum()


def wheel_pose(d, wbi, i, w):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, d.xquat[wbi[i]])
    R = R.reshape(3, 3)
    c = d.xpos[wbi[i]] + R @ np.array([0.0, SIDE[w[:2]] * OFF_Y, 0.0])
    a = R @ np.array([0.0, 1.0, 0.0])
    return R, c, a


def analytic_lowest(c, a, r=R_WHEEL, w=W_HALF):
    """Lowest point of a tilted finite-width cylinder (axis a through centre c)."""
    az = a[2]
    s = np.sqrt(max(1.0 - az * az, 1e-12))
    u = (a * az - np.array([0.0, 0.0, 1.0])) / s       # radial dir with min z
    p = c - w * np.sign(az) * a + r * u
    min_z = c[2] - w * abs(az) - r * s
    return p, min_z


def main():
    rows, jsession = [], {}
    for tag in WIN:
        m, d, wbi, qm, qstd, nfr = build(tag)
        per = []
        # first pass: analytic min_z per wheel for ground normalisation
        poses = []
        for i, w in enumerate(WHE):
            R, c, a = wheel_pose(d, wbi, i, w)
            p, min_z = analytic_lowest(c, a)
            poses.append((w, R, c, a, p, min_z))
        dz = -min(pp[5] for pp in poses)
        base_z_after = d.qpos[2] + dz
        for (w, R, c, a, p, min_z) in poses:
            c = c.copy(); c[2] += dz
            p = p.copy(); p[2] += dz
            p_f1 = c + R @ np.array([0.0, 0.0, -R_WHEEL])
            p_naive = d.xpos[wbi[WHE.index(w)]] + np.array([0, 0, dz]) - np.array([0.0, 0.0, R_WHEEL])
            tilt_deg = np.degrees(np.arcsin(min(abs(a[2]), 1.0)))
            d_pt = np.linalg.norm(p - p_f1)
            rows.append(dict(
                session=tag, wheel=w, n_frames=int(nfr),
                hip_q_rad=round(float(qm[WHE.index(w)]), 4),
                hip_q_std=round(float(qstd[WHE.index(w)]), 4),
                axis_z=round(float(a[2]), 5),
                tilt_deg=round(float(tilt_deg), 3),
                centre_xyz=[round(float(v), 4) for v in c],
                analytic_min_z=round(float(p[2]), 4),
                f1_min_z=round(float(p_f1[2]), 4),
                naive_min_z=round(float(p_naive[2]), 4),
                clearance_mm=round(float(max(p[2], 0.0) * 1000), 1),
                dxdy_analytic_vs_f1_mm=[round(float(v) * 1000, 2) for v in (p - p_f1)[:2]],
                dist_analytic_vs_f1_mm=round(float(d_pt * 1000), 2),
                dist_analytic_vs_naive_mm=round(float(np.linalg.norm(p - p_naive) * 1000), 2),
            ))
            per.append(dict(wheel=w, tilt_deg=round(float(tilt_deg), 3),
                            clearance_mm=round(float(max(p[2], 0.0) * 1000), 1),
                            analytic_pt=[round(float(v), 4) for v in p]))
        jsession[tag] = dict(
            window_s=list(WIN[tag]), n_frames=int(nfr),
            base_quat=[round(float(v), 5) for v in d.qpos[3:7]],
            base_z_normalised=round(float(base_z_after), 4),
            hip_q=[round(float(v), 4) for v in qm[:4]],
            hip_q_std=[round(float(v), 4) for v in qstd[:4]],
            thigh_q=[round(float(v), 4) for v in qm[4:8]],
            calf_q=[round(float(v), 4) for v in qm[8:12]],
            wheels=per)

    out_csv = RUNS / 'tables/G2_WHEEL_POSE_FK.csv'
    with open(out_csv, 'w', newline='') as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        # nested lists -> json strings for csv
        for r in rows:
            r2 = {k: (json.dumps(v) if isinstance(v, list) else v) for k, v in r.items()}
            wtr.writerow(r2)
    with open(RUNS / 'tables/G2_WHEEL_POSE_FK.json', 'w') as f:
        json.dump(jsession, f, indent=1)
    print('wrote', out_csv)
    # console digest
    for tag in WIN:
        rr = [r for r in rows if r['session'] == tag]
        print(f"--- {tag} win={WIN[tag]} hip_q={jsession[tag]['hip_q']} "
              f"quat={jsession[tag]['base_quat']}")
        for r in rr:
            print(f"  {r['wheel']}: hip={r['hip_q_rad']:+.3f} tilt={r['tilt_deg']:6.2f}deg "
                  f"axis_z={r['axis_z']:+.4f} clr={r['clearance_mm']:6.1f}mm "
                  f"|p-f1|={r['dist_analytic_vs_f1_mm']:6.2f}mm dy={r['dxdy_analytic_vs_f1_mm'][1]:+6.2f}mm "
                  f"|p-naive|={r['dist_analytic_vs_naive_mm']:6.2f}mm")


if __name__ == '__main__':
    main()
