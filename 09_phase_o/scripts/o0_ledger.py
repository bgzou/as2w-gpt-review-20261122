#!/usr/bin/env python3
"""Phase O / O0: corrected co-configuration static ledger.

Fixes three implementation issues flagged in the Phase O task book:
  O0-A  tilted finite-width cylinder lowest point now includes the end-cap term
        p = c - w*sign(a_z)*a + r*(a*a_z - z_hat)/sqrt(1-a_z^2)
        with the *geom* axis (geom_xmat column 2), not the foot-body y axis.
  O0-B  friction cone uses an inscribed N-gon (projection <= mu*Fn*cos(pi/N))
        and, as an outer bound, the circumscribed polygon (<= mu*Fn); the old
        8-gon was the circumscribed one and is not "conservative".
  O0-C  hip generalized force is computed only from that leg's own wheel,
        about the hinge anchor and projected on the hinge axis, and is compared
        with A[hip_dof] @ f.

Equilibrium: qfrc_bias = qfrc_applied + J^T f  ->  A f = b,
b = qfrc_bias - [0_base; tau_est]; unknown joint correction d_tau = b_joint - (A f)_joint.
"""
import json
import pathlib

import mujoco
import numpy as np
from scipy.optimize import linprog

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
BASE = ROOT / 'input' / 'as2w_scene_base.xml'
NPZ = ROOT / 'input' / 'P55G2.npz'
SCENES = ROOT / 'scenes'
OUT = ROOT / 'results'
OUT.mkdir(parents=True, exist_ok=True)

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
R_WHEEL = 0.086
W_HALF = 0.026
MU_GRID = [0.20, 0.25, 0.30, 0.365, 0.40, 0.44, 0.50, 0.80]
N_POLY = 32
OLD_INERTIAL = ('<inertial pos="0.035002 -0.0007562 0.017521" mass="10.08"\n'
                '                fullinertia="0.05069483 0.13822065 0.1505824 -0.0002906 -0.02480435 0.00027717"/>')
NEW_INERTIAL = ('<inertial pos="0.020002 -0.0007562 0.017521" mass="11.592"\n'
                '                fullinertia="0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455"/>')


def load_anchor():
    z = np.load(NPZ)
    t = z['t']
    m = (t >= 8.0) & (t <= 19.0)
    quat = z['quat'][m].mean(0)
    quat /= np.linalg.norm(quat)
    return dict(q=z['q'][m].mean(0), tau=z['tau'][m].mean(0), quat=quat,
                roll_deg=float(np.degrees(z['roll'][m].mean())),
                pitch_deg=float(np.degrees(z['pitch'][m].mean())))


def make_model(loadv0):
    txt = BASE.read_text()
    if loadv0:
        assert OLD_INERTIAL in txt
        txt = txt.replace(OLD_INERTIAL, NEW_INERTIAL)
    p = SCENES / ('O0_' + ('loadv0' if loadv0 else 'delivery')) / 'as2w_scene.xml'
    p.parent.mkdir(parents=True, exist_ok=True)
    lnk = p.parent / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(ROOT / 'input' / 'meshes')
    p.write_text(txt)
    return p


def wheel_cyl(m, d, wheel):
    """Return centre, axis (world), radius, half width of the wheel cylinder."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, wheel)
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != bid:
            continue
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CYLINDER:
            continue
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        c = np.array(d.geom_xpos[g], dtype=float)
        R = np.array(d.geom_xmat[g], dtype=float).reshape(3, 3)
        a = R[:, 2] / np.linalg.norm(R[:, 2])   # cylinder local z = its axis
        r = float(m.geom_size[g][0])
        w = float(m.geom_size[g][1])
        return c, a, r, w
    raise RuntimeError('no cylinder for ' + wheel)


def lowest_point(c, a, r=R_WHEEL, w=W_HALF):
    az = float(a[2])
    s = np.sqrt(max(1.0 - az * az, 1e-12))
    u = (a * az - np.array([0.0, 0.0, 1.0])) / s
    p = c - w * np.sign(az) * a + r * u
    p_old = c + r * u
    return p, p_old, s


def build_ledger(xml_path, anc, use_old=False):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    d.qpos[:] = 0
    d.qpos[3:7] = anc['quat']
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = anc['q'][i]
    mujoco.mj_forward(m, d)

    info = {}
    lows = []
    for w in WHE:
        c, a, r, hw = wheel_cyl(m, d, w)
        p, p_old, s = lowest_point(c, a, r, hw)
        info[w] = dict(centre=[float(x) for x in c], axis=[float(x) for x in a],
                       tilt_deg=float(np.degrees(np.arcsin(min(abs(a[2]), 1.0)))),
                       r=r, w=hw,
                       p_new=[float(x) for x in p], p_old=[float(x) for x in p_old],
                       dz_mm=float((p[2] - p_old[2]) * 1000.0),
                       dxy_mm=[float((p[0] - p_old[0]) * 1000.0),
                               float((p[1] - p_old[1]) * 1000.0)])
        lows.append(p[2])
    # normalise base height with the *corrected* lowest points
    d.qpos[2] += -min(lows)
    mujoco.mj_forward(m, d)
    for w in WHE:
        c, a, r, hw = wheel_cyl(m, d, w)
        p, p_old, s = lowest_point(c, a, r, hw)
        info[w]['p_new'][2] = float(p[2])
        info[w]['p_old'][2] = float(p_old[2])
        info[w]['centre'][2] = float(c[2])
    zvals = [info[w]['p_new'][2] for w in WHE]

    bias = d.qfrc_bias.copy()
    applied = np.zeros(m.nv)
    for i, n in enumerate(LEGS):
        applied[vadr[n]] = anc['tau'][i]
    b = bias - applied

    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    cols, points = [], []
    for w in WHE:
        p = np.array(info[w]['p_old' if use_old else 'p_new'])
        points.append(p)
        mujoco.mj_jac(m, d, jacp, jacr, p, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w))
        for c in (2, 0, 1):
            cols.append(jacp[c, :].copy())
    A = np.stack(cols, axis=1)
    order = [vadr[n] for n in LEGS + WHE]
    return dict(m=m, d=d, A=A, b=b, A_joint=A[order, :], b_joint=b[order],
                points=points, info=info, z_spread_mm=float((max(zvals) - min(zvals)) * 1000),
                order=order, vadr=vadr, qadr=qadr, jid=jid)


def cone_rows(mu, inscribed=True, n=N_POLY):
    """N-gon friction rows for 12 vars. Inscribed: projection <= mu*Fn*cos(pi/N)."""
    scale = mu * (np.cos(np.pi / n) if inscribed else 1.0)
    rows = []
    for i in range(4):
        for k in range(n):
            th = 2.0 * np.pi * k / n
            r = np.zeros(12)
            r[3 * i + 0] = -scale
            r[3 * i + 1] = np.cos(th)
            r[3 * i + 2] = np.sin(th)
            rows.append(r)
        r = np.zeros(12)
        r[3 * i] = -1.0          # Fn >= 0
        rows.append(r)
    return np.stack(rows)


def solve(led, mu, tol, inscribed=True, objective=None):
    A, b = led['A'], led['b']
    A_j, b_j = led['A_joint'], led['b_joint']
    A_ub = np.vstack([A_j, -A_j, cone_rows(mu, inscribed)])
    b_ub = np.concatenate([b_j + tol, -b_j + tol,
                           np.zeros(cone_rows(mu, inscribed).shape[0])])
    c = np.zeros(12) if objective is None else objective
    return linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A[0:6, :], b_eq=b[0:6],
                   bounds=[(None, None)] * 12, method='highs')


def min_tol(led, mu, inscribed=True, hi=200.0):
    if solve(led, mu, 0.0, inscribed).status == 0:
        return 0.0
    lo = 0.0
    for _ in range(26):
        mid = 0.5 * (lo + hi)
        if solve(led, mu, mid, inscribed).status == 0:
            hi = mid
        else:
            lo = mid
    return hi


def cone_violation(x, mu):
    v = []
    for i in range(4):
        fn, fx, fy = x[3 * i], x[3 * i + 1], x[3 * i + 2]
        v.append(max(0.0, float(np.hypot(fx, fy) - mu * fn)))
    return v


def hip_check(led, x, mu):
    m, d = led['m'], led['d']
    out = {}
    for hn in LEGS[:4]:
        leg = hn[:2]
        w = leg + '_foot'
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, hn)
        axis = np.array(d.xaxis[j], dtype=float)
        anchor = np.array(d.xanchor[j], dtype=float)
        i = WHE.index(w)
        f = np.array([x[3 * i + 1], x[3 * i + 2], x[3 * i]])  # (Fx, Fy, Fn=z)
        p = np.array(led['points'][i])
        r = p - anchor
        tau_direct = float(axis @ np.cross(r, f))
        tau_jac = float(led['A_joint'][led['order'].index(led['vadr'][hn]), :] @ x)
        out[hn] = dict(axis=[round(float(v), 4) for v in axis],
                       anchor=[round(float(v), 4) for v in anchor],
                       r=[round(float(v), 4) for v in r],
                       force_Fx_Fy_Fn=[round(float(v), 3) for v in f],
                       tau_axis_cross=round(tau_direct, 4),
                       tau_jacobian=round(tau_jac, 4),
                       diff=round(tau_direct - tau_jac, 5),
                       ryFz=round(float(r[1] * f[2]), 3),
                       rzFy=round(float(-r[2] * f[1]), 3))
    return out


def run_asset(loadv0, anc):
    led = build_ledger(make_model(loadv0), anc)
    m = led['m']
    res = dict(asset=('loadv0' if loadv0 else 'delivery'),
               mass_kg=round(float(np.sum(m.body_mass)), 3),
               contact_points=[[round(float(v), 5) for v in p] for p in led['points']],
               wheel_info={w: led['info'][w] for w in WHE},
               four_point_z_spread_mm=round(led['z_spread_mm'], 3),
               base_bias=[round(float(v), 3) for v in led['b'][0:6]],
               rows=[])
    x025 = None
    for mu in MU_GRID:
        ti = min_tol(led, mu, True)
        to = min_tol(led, mu, False)
        row = dict(mu=mu, t_star_inscribed=round(float(ti), 4),
                   t_star_circumscribed=round(float(to), 4))
        r = solve(led, mu, ti, True)
        if r.status == 0:
            x = r.x
            if abs(mu - 0.25) < 1e-9:
                x025 = x
            dt = led['b_joint'] - led['A_joint'] @ x
            row.update(contact_forces_N=[[round(float(v), 2) for v in x[3 * i:3 * i + 3]]
                                         for i in range(4)],
                       dtau_Nm=[round(float(v), 3) for v in dt],
                       max_abs_dtau=round(float(np.max(np.abs(dt))), 3),
                       base_residual=[round(float(v), 4) for v in
                                      (led['A'][0:6, :] @ x - led['b'][0:6])],
                       cone_violation= [round(float(v), 5) for v in cone_violation(x, mu)])
            fy = []
            fn = []
            for i in range(4):
                c1 = np.zeros(12); c1[3 * i + 2] = 1.0
                c2 = np.zeros(12); c2[3 * i] = 1.0
                trange = max(2.0, ti + 0.25)
                a1 = solve(led, mu, trange, True, c1)
                a2 = solve(led, mu, trange, True, -c1)
                a3 = solve(led, mu, trange, True, c2)
                a4 = solve(led, mu, trange, True, -c2)
                fy.append([round(float(a1.x[3 * i + 2]), 2) if a1.status == 0 else None,
                           round(float(a2.x[3 * i + 2]), 2) if a2.status == 0 else None])
                fn.append([round(float(a3.x[3 * i]), 2) if a3.status == 0 else None,
                           round(float(a4.x[3 * i]), 2) if a4.status == 0 else None])
            row['Fy_range_N'] = fy
            row['Fn_range_N'] = fn
            row['Fy_range_crosses_zero'] = [None if (a is None or b is None) else bool(a <= 0 <= b)
                                            for a, b in fy]
            row['range_tol_Nm'] = round(float(trange), 3)
        res['rows'].append(row)
    if x025 is not None:
        res['hip_check_mu0.25'] = hip_check(led, x025, 0.25)
    old = build_ledger(make_model(loadv0), anc, use_old=True)
    res['old_points_mu0.25'] = dict(
        t_star_inscribed=round(float(min_tol(old, 0.25, True)), 4),
        t_star_circumscribed=round(float(min_tol(old, 0.25, False)), 4),
        contact_points=[[round(float(v), 5) for v in p] for p in old['points']])
    return res


def main():
    anc = load_anchor()
    out = dict(schema='o0_ledger_v1', mujoco_version=mujoco.__version__,
               n_poly=N_POLY, mu_grid=MU_GRID,
               anchor=dict(q=[round(float(x), 5) for x in anc['q']],
                           tau=[round(float(x), 4) for x in anc['tau']],
                           quat=[round(float(x), 6) for x in anc['quat']],
                           pitch_deg=round(anc['pitch_deg'], 4),
                           roll_deg=round(anc['roll_deg'], 4)),
               assets=[])
    for loadv0 in (False, True):
        r = run_asset(loadv0, anc)
        out['assets'].append(r)
        print(json.dumps({k: v for k, v in r.items() if k != 'rows'}, ensure_ascii=False),
              flush=True)
        for row in r['rows']:
            print(json.dumps(row, ensure_ascii=False), flush=True)
    (OUT / 'O0_ledger.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'O0_ledger.json')


if __name__ == '__main__':
    main()
