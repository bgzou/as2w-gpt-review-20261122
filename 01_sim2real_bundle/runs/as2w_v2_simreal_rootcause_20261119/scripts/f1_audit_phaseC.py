#!/usr/bin/env python3
"""Phase C audit (read-only): inversion identifiability, contact-point
sensitivity, friction feasibility, and the fixed-lambda wheel-centre lever.

Key corrections audited here (vs c1_ground_ledger.py):
  1. contact point is rebuilt with the wheel body rotation (hip roll tilts the
     wheel axis), not as world xpos - [0,0,r];
  2. the real collision geometry offset (C1, +/-54.6 mm) is included;
  3. base/leg residual groups are reported separately in their own units;
  4. weighted vs unweighted least squares;
  5. contact-moment augmentation rank test.

Outputs: runs/.../tables/F_C_INVERSION_AUDIT.csv, F_C_INVERSION_AUDIT.json
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
WIN = {'OG': (17, 120), 'P55G1': (10, 30), 'P55G2': (8, 19), 'P55G3': (7, 19)}


def build(m, d, tag, off_y=0.0, tilt=True, dz=0.0):
    """Set pose from window means; return A, b, contact points, heights."""
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS}
    wbi = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE]
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

    def contact_pts():
        pts = []
        for i, w in enumerate(WHE):
            R = np.zeros((9,))
            mujoco.mju_quat2Mat(R, d.xquat[wbi[i]])
            R = R.reshape(3, 3)
            if tilt:
                off = R @ np.array([0.0, SIDE[w[:2]] * off_y, -R_WHEEL + dz])
            else:
                off = np.array([0.0, SIDE[w[:2]] * off_y, -R_WHEEL + dz])
            pts.append(d.xpos[wbi[i]] + off)
        return pts

    pts = contact_pts()
    d.qpos[2] -= min(p[2] for p in pts)
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    pts = contact_pts()
    heights = [float(p[2]) for p in pts]
    g = d.qfrc_bias.copy()
    J = []
    for i in range(4):
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jacp, np.zeros((3, m.nv)), pts[i], wbi[i])
        J.append(jacp)
    legv = [vadr[n] for n in LEGS]
    A_base = np.zeros((6, 12))
    A_leg = np.zeros((12, 12))
    for i in range(4):
        A_base[:, 3 * i:3 * i + 3] = J[i][:, :6].T
        A_leg[:, 3 * i:3 * i + 3] = -J[i][:, legv].T
    return dict(A_base=A_base, A_leg=A_leg, b_leg=g[legv], b_base=g[:6],
                taum=taum, heights=heights, pts=pts, qm=qm, quat=qqm)


def solve(blk, weighted=False):
    A = np.vstack([blk['A_base'], blk['A_leg']])
    b = np.concatenate([blk['b_base'], blk['taum'] - blk['b_leg']])
    w = np.ones(18)
    if weighted:
        w[:3] = 1 / 255.0     # base force rows, N scale
        w[3:6] = 1 / 20.0     # base moment rows, Nm scale
        w[6:] = 1 / 15.0      # leg rows, Nm scale
    lam = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)[0]
    rb = blk['A_base'] @ lam - blk['b_base']
    rl = blk['A_leg'] @ lam - (blk['taum'] - blk['b_leg'])
    return lam, rb, rl


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    rows, out = [], {}
    for tag in WIN:
        base = build(m, d, tag, off_y=0.0, tilt=True)
        A = np.vstack([base['A_base'], base['A_leg']])
        sv = np.linalg.svd(A, compute_uv=False)
        sv_leg = np.linalg.svd(base['A_leg'], compute_uv=False)
        lam0, rb0, rl0 = solve(base)
        lamw, rbw, rlw = solve(base, weighted=True)
        # M0 (leg only)
        lam_m0 = np.linalg.lstsq(base['A_leg'], base['taum'] - base['b_leg'], rcond=None)[0]
        rb_m0 = base['A_base'] @ lam_m0 - base['b_base']
        # contact-moment augmentation: 3 moment components per wheel
        jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS}
        vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS}
        legv = [vadr[n] for n in LEGS]
        wbi = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE]
        M = np.zeros((12, 12))
        for i in range(4):
            jacr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, np.zeros((3, m.nv)), jacr, base['pts'][i], wbi[i])
            M[:, 3 * i:3 * i + 3] = -jacr[:, legv].T
        Aaug = np.hstack([base['A_leg'], M])
        rank_aug = int(np.linalg.matrix_rank(Aaug, tol=1e-8))
        lam_aug = np.linalg.lstsq(Aaug, base['taum'] - base['b_leg'], rcond=None)[0]

        def pack(lam, rb, rl):
            return dict(
                Fn=[round(float(lam[3 * i + 2]), 2) for i in range(4)],
                Fx=[round(float(lam[3 * i]), 2) for i in range(4)],
                Fy=[round(float(lam[3 * i + 1]), 2) for i in range(4)],
                base_force_resid_max_N=round(float(np.abs(rb[:3]).max()), 3),
                base_moment_resid_max_Nm=round(float(np.abs(rb[3:]).max()), 3),
                leg_resid_max_Nm=round(float(np.abs(rl).max()), 3),
                leg_resid_rms_Nm=round(float(np.sqrt((rl ** 2).mean())), 3))

        variants = {}
        for name, kw in (
                ('naive_C0', dict(off_y=0.0, tilt=False)),
                ('tilt_C0', dict(off_y=0.0, tilt=True)),
                ('tilt_C1', dict(off_y=0.0546, tilt=True)),
                ('tilt_C1_dy-5mm', dict(off_y=0.0546 - 0.005, tilt=True)),
                ('tilt_C1_dy+5mm', dict(off_y=0.0546 + 0.005, tilt=True)),
                ('tilt_C1_dy-10mm', dict(off_y=0.0546 - 0.010, tilt=True)),
                ('tilt_C1_dy+10mm', dict(off_y=0.0546 + 0.010, tilt=True)),
                ('tilt_C1_dz-5mm', dict(off_y=0.0546, tilt=True, dz=-0.005)),
                ('tilt_C1_dz+5mm', dict(off_y=0.0546, tilt=True, dz=+0.005))):
            blk = build(m, d, tag, **kw)
            lam, rb, rl = solve(blk)
            variants[name] = pack(lam, rb, rl)
            variants[name]['heights_mm'] = [round(h * 1000, 1) for h in blk['heights']]
            variants[name]['lam'] = lam
        # fixed-lambda lever: keep naive_C0 lambda, evaluate at tilt_C1 contacts
        lam_fix = np.linalg.lstsq(base['A_leg'], base['taum'] - base['b_leg'], rcond=None)[0]
        b1 = build(m, d, tag, off_y=0.0546, tilt=True)
        dtau = b1['A_leg'] @ lam_fix - base['A_leg'] @ lam_fix
        # friction requirement (baseline tilt_C1)
        lam = variants['tilt_C1']['lam']
        mu_req = [float(np.hypot(lam[3 * i], lam[3 * i + 1]) / max(lam[3 * i + 2], 1e-9))
                  for i in range(4)]
        # naive world-down contact height check (what c1 actually used)
        out[tag] = dict(
            window=WIN[tag],
            A_dim=list(A.shape), A_rank=int(np.linalg.matrix_rank(A)),
            A_singular_values=[round(float(x), 4) for x in sv],
            A_cond=round(float(sv[0] / sv[-1]), 1),
            A_leg_rank=int(np.linalg.matrix_rank(base['A_leg'])),
            A_leg_cond=round(float(sv_leg[0] / sv_leg[-1]), 1),
            unweighted=pack(lam0, rb0, rl0),
            weighted=pack(lamw, rbw, rlw),
            M0=dict(Fn=[round(float(lam_m0[3 * i + 2]), 2) for i in range(4)],
                    Fy=[round(float(lam_m0[3 * i + 1]), 2) for i in range(4)],
                    base_force_resid_max_N=round(float(np.abs(rb_m0[:3]).max()), 3),
                    base_moment_resid_max_Nm=round(float(np.abs(rb_m0[3:]).max()), 3)),
            contact_moment_rank=rank_aug,
            contact_moment_minnorm_absmax_Nm=round(float(np.abs(lam_aug[12:]).max()), 4),
            variants={k: {kk: vv for kk, vv in v.items() if kk != 'lam'}
                      for k, v in variants.items()},
            fixed_lambda_dtau_hip_Nm=[round(float(dtau[j]), 3) for j in range(4)],
            fixed_lambda_dtau_all_Nm=[round(float(x), 3) for x in dtau],
            mu_required_per_wheel=[round(x, 3) for x in mu_req],
            mu_required_max=round(max(mu_req), 3),
            Fn_sum=round(float(lam[[2, 5, 8, 11]].sum()), 2),
        )
        print(tag, json.dumps({k: v for k, v in out[tag].items()
                               if k not in ('variants',)}))
        for name, v in variants.items():
            rows.append(dict(session=tag, variant=name,
                             Fn='/'.join(f'{x:.1f}' for x in v['Fn']),
                             Fx='/'.join(f'{x:.1f}' for x in v['Fx']),
                             Fy='/'.join(f'{x:.1f}' for x in v['Fy']),
                             heights_mm='/'.join(f'{x:.1f}' for x in v['heights_mm']),
                             base_force_resid_N=v['base_force_resid_max_N'],
                             base_moment_resid_Nm=v['base_moment_resid_max_Nm'],
                             leg_resid_Nm=v['leg_resid_max_Nm']))
    with open(RUNS / 'tables' / 'F_C_INVERSION_AUDIT.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (RUNS / 'tables' / 'F_C_INVERSION_AUDIT.json').write_text(json.dumps(out, indent=1))
    print('wrote F_C_INVERSION_AUDIT.{csv,json}')


if __name__ == '__main__':
    main()
