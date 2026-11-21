#!/usr/bin/env python3
"""Phase G / G2b (read-only): re-run the Phase C contact-force inversion with
CORRECT contact points, after G2 found that f1_audit_phaseC.py's 'tilt-aware'
points were geometrically wrong.

f1 bug (G2 finding): point = xpos[foot] + R_body @ [0, +-off_y, -0.086].
The foot body frame is rotated by thigh+calf hinge rotations about y
(net -67..-86 deg in the measured stance), so R_body @ [0,0,-0.086] points
roughly HORIZONTALLY along the wheel rim (front/back), not at the wheel
bottom. Correct lowest point of a tilted finite-width cylinder:
    p = c - w*sign(a_z)*a + r*(a*a_z - z_hat)/sqrt(1-a_z^2)
verified against brute-force surface sampling (exact agreement).

Modes re-run per session (4 ground sessions, f1 windows):
  naive      : xpos - [0,0,0.086]                    (old c1_ground_ledger)
  f1_tilt    : xpos + R@[0,+-0.0546,-0.086]          (f1 'tilt_C1', buggy, ref)
  analytic   : correctly computed tilted-cylinder lowest point (fixed)
plus analytic dy/dz +-5mm sensitivities.

Outputs: tables/G2_CONTACT_RECHECK.csv, tables/G2_CONTACT_RECHECK.json
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
SIDE = {'FL': +1, 'FR': -1, 'RL': +1, 'RR': -1}
R_WHEEL = 0.086
W_HALF = 0.026
OFF_Y = 0.0546
WIN = {'OG': (17, 120), 'P55G1': (10, 30), 'P55G2': (8, 19), 'P55G3': (7, 19)}


def contact_pts(m, d, wbi, mode, off_y=OFF_Y, dy=0.0, dz=0.0):
    pts, angs = [], []
    for i, w in enumerate(WHE):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, d.xquat[wbi[i]])
        R = R.reshape(3, 3)
        if mode == 'naive':
            pts.append(d.xpos[wbi[i]] + np.array([0.0, 0.0, dz - R_WHEEL]))
            angs.append(np.degrees(np.arcsin(abs((R @ np.array([0, 1.0, 0]))[2]))))
        elif mode == 'f1_tilt':
            pts.append(d.xpos[wbi[i]] + R @ np.array(
                [0.0, SIDE[w[:2]] * (off_y + dy), dz - R_WHEEL]))
            angs.append(np.degrees(np.arcsin(abs((R @ np.array([0, 1.0, 0]))[2]))))
        elif mode == 'analytic':
            s = SIDE[w[:2]]
            o = R @ np.array([0.0, s * off_y, 0.0])          # axis offset -> centre
            c = d.xpos[wbi[i]] + o
            # lateral sensitivity dy shifts the centre along the axis
            if dy:
                c = c + R @ np.array([0.0, s * dy, 0.0])
            a = R @ np.array([0.0, 1.0, 0.0])
            az = a[2]
            sv = np.sqrt(max(1.0 - az * az, 1e-12))
            u = (a * az - np.array([0.0, 0.0, 1.0])) / sv
            p = c - (W_HALF) * np.sign(az) * a + (R_WHEEL + dz) * u
            pts.append(p)
            angs.append(np.degrees(np.arcsin(min(abs(az), 1.0))))
        else:
            raise ValueError(mode)
    return pts, angs


def build(m, d, wbi, tag, **kw):
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS}
    legv = [vadr[n] for n in LEGS]
    z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
    t, q, tau, quat = z['t'], z['q'], z['tau'], z['quat']
    msk = (t >= WIN[tag][0]) & (t <= WIN[tag][1])
    qm, taum = q[msk].mean(0), tau[msk].mean(0)
    qqm = quat[msk].mean(0)
    qqm /= np.linalg.norm(qqm)
    d.qpos[:] = 0
    d.qvel[:] = 0
    d.qpos[3:7] = qqm
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = qm[i]
    mujoco.mj_forward(m, d)
    mode = kw.pop('mode')
    pts, _ = contact_pts(m, d, wbi, mode, **kw)
    d.qpos[2] -= min(p[2] for p in pts)
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    pts, _ = contact_pts(m, d, wbi, mode, **kw)
    heights = [float(p[2]) for p in pts]
    g = d.qfrc_bias.copy()
    A_base = np.zeros((6, 12))
    A_leg = np.zeros((12, 12))
    for i in range(4):
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jacp, np.zeros((3, m.nv)), pts[i], wbi[i])
        A_base[:, 3 * i:3 * i + 3] = jacp[:, :6].T
        A_leg[:, 3 * i:3 * i + 3] = -jacp[:, legv].T
    return dict(A_base=A_base, A_leg=A_leg, b_leg=g[legv], b_base=g[:6],
                taum=taum, heights=heights, pts=pts)


def solve(blk):
    A = np.vstack([blk['A_base'], blk['A_leg']])
    b = np.concatenate([blk['b_base'], blk['taum'] - blk['b_leg']])
    lam = np.linalg.lstsq(A, b, rcond=None)[0]
    rb = blk['A_base'] @ lam - blk['b_base']
    rl = blk['A_leg'] @ lam - (blk['taum'] - blk['b_leg'])
    return lam, rb, rl


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    wbi = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE]
    rows, out = [], {}
    variants = [
        ('naive', dict(mode='naive')),
        ('f1_tilt_buggy', dict(mode='f1_tilt')),
        ('analytic', dict(mode='analytic')),
        ('analytic_dy-5mm', dict(mode='analytic', dy=-0.005)),
        ('analytic_dy+5mm', dict(mode='analytic', dy=+0.005)),
        ('analytic_dz-5mm', dict(mode='analytic', dz=-0.005)),
        ('analytic_dz+5mm', dict(mode='analytic', dz=+0.005)),
    ]
    for tag in WIN:
        sess = {}
        blks = {}
        for name, kw in variants:
            blk = build(m, d, wbi, tag, **{k: v.copy() if isinstance(v, dict) else v
                                           for k, v in kw.items()})
            blks[name] = blk
            lam, rb, rl = solve(blk)
            Fn = [float(lam[3 * i + 2]) for i in range(4)]
            Fy = [float(lam[3 * i + 1]) for i in range(4)]
            Fx = [float(lam[3 * i]) for i in range(4)]
            mu = [float(np.hypot(Fx[i], Fy[i]) / max(Fn[i], 1e-9)) for i in range(4)]
            sess[name] = dict(
                Fn=[round(x, 2) for x in Fn], Fy=[round(x, 2) for x in Fy],
                Fx=[round(x, 2) for x in Fx],
                heights_mm=[round(h * 1000, 1) for h in blk['heights']],
                mu_req=[round(x, 3) for x in mu],
                Fn_sum=round(sum(Fn), 2),
                base_force_resid_max_N=round(float(np.abs(rb[:3]).max()), 2),
                base_moment_resid_max_Nm=round(float(np.abs(rb[3:]).max()), 2),
                leg_resid_max_Nm=round(float(np.abs(rl).max()), 3),
                leg_resid_rms_Nm=round(float(np.sqrt((rl ** 2).mean())), 3))
        # fixed-lambda lever: lambda from naive leg-only, evaluated at analytic pts
        lam_fix = np.linalg.lstsq(blks['naive']['A_leg'],
                                  blks['naive']['taum'] - blks['naive']['b_leg'],
                                  rcond=None)[0]
        dtau_naive2ana = (blks['analytic']['A_leg'] @ lam_fix
                          - blks['naive']['A_leg'] @ lam_fix)
        dtau_f1bug2ana = (blks['analytic']['A_leg'] @ lam_fix
                          - blks['f1_tilt_buggy']['A_leg'] @ lam_fix)
        out[tag] = dict(window=WIN[tag], variants=sess,
                        fixed_lam_legonly_dtau_hip_naive_to_analytic_Nm=[
                            round(float(x), 3) for x in dtau_naive2ana[:4]],
                        fixed_lam_legonly_dtau_hip_f1buggy_to_analytic_Nm=[
                            round(float(x), 3) for x in dtau_f1bug2ana[:4]],
                        fixed_lam_legonly_dtau_all_naive_to_analytic_Nm=[
                            round(float(x), 3) for x in dtau_naive2ana])
        print(f'--- {tag}')
        for name, v in sess.items():
            print(f'  {name:20s} Fn={v["Fn"]} Fy={v["Fy"]} mu={v["mu_req"]} '
                  f'h_mm={v["heights_mm"]} legRMS={v["leg_resid_rms_Nm"]}')
        print(f'  dtau hip naive->analytic: {[round(float(x),3) for x in dtau_naive2ana[:4]]}')
        for name, v in sess.items():
            rows.append(dict(session=tag, variant=name,
                             Fn='/'.join(f'{x:.1f}' for x in v['Fn']),
                             Fy='/'.join(f'{x:.1f}' for x in v['Fy']),
                             Fx='/'.join(f'{x:.1f}' for x in v['Fx']),
                             heights_mm='/'.join(f'{x:.1f}' for x in v['heights_mm']),
                             mu_req='/'.join(f'{x:.2f}' for x in v['mu_req']),
                             Fn_sum=v['Fn_sum'],
                             base_force_resid_N=v['base_force_resid_max_N'],
                             base_moment_resid_Nm=v['base_moment_resid_max_Nm'],
                             leg_resid_rms_Nm=v['leg_resid_rms_Nm']))
    with open(RUNS / 'tables' / 'G2_CONTACT_RECHECK.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (RUNS / 'tables' / 'G2_CONTACT_RECHECK.json').write_text(json.dumps(out, indent=1))
    print('wrote G2_CONTACT_RECHECK.{csv,json}')


if __name__ == '__main__':
    main()
