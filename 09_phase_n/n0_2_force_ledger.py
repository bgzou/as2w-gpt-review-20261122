#!/usr/bin/env python3
"""N0-2 / N0-3: co-configuration static force ledger for P55G2 + fixed-mu LP.

Builds the 6D base + 12 leg-joint equilibrium at the *real* P55G2 configuration
(real joint angles and real IMU quaternion), with four rigid cylinder contacts,
and solves a linear program in the 12 contact-force components.

Equilibrium (MuJoCo convention, qacc = 0):
    qfrc_bias = qfrc_applied(tau_est) + J^T f
=>  A f = b,  b = qfrc_bias - [0_base; tau_est]

Unknown joint-side correction  d_tau = b_joint - (A f)_joint  is reported, never
treated as a real transmission loss.

Contact cone: octagon inscribed in sqrt(Fx^2+Fy^2) <= mu*Fn (conservative).
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
MU_GRID = [0.20, 0.25, 0.30, 0.365, 0.40, 0.44, 0.50, 0.60, 0.80]
NEW_INERTIAL = ('<inertial pos="0.020002 -0.0007562 0.017521" mass="11.592"\n'
                '                fullinertia="0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455"/>')
OLD_INERTIAL = ('<inertial pos="0.035002 -0.0007562 0.017521" mass="10.08"\n'
                '                fullinertia="0.05069483 0.13822065 0.1505824 -0.0002906 -0.02480435 0.00027717"/>')


def load_anchor():
    z = np.load(NPZ)
    t = z['t']
    m = (t >= 8.0) & (t <= 19.0)
    quat = z['quat'][m].mean(0)
    quat = quat / np.linalg.norm(quat)
    return dict(
        q=z['q'][m].mean(0), tau=z['tau'][m].mean(0), quat=quat,
        roll_deg=float(np.degrees(z['roll'][m].mean())),
        pitch_deg=float(np.degrees(z['pitch'][m].mean())),
        t0=float(t[m][0]), t1=float(t[m][-1]),
    )


def make_model(loadv0=False):
    txt = BASE.read_text()
    if loadv0:
        assert OLD_INERTIAL in txt
        txt = txt.replace(OLD_INERTIAL, NEW_INERTIAL)
    p = SCENES / ('N0_' + ('loadv0' if loadv0 else 'delivery')) / 'as2w_scene.xml'
    p.parent.mkdir(parents=True, exist_ok=True)
    lnk = p.parent / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(ROOT / 'input' / 'meshes')
    p.write_text(txt)
    return p


def wheel_lowest_point(m, d, body_id):
    """Lowest point of the wheel cylinder in world coordinates."""
    best = None
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body_id:
            continue
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CYLINDER:
            continue
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        c = np.array(d.geom_xpos[g])
        # cylinder axis = body y axis
        axis = np.array(d.xmat[body_id]).reshape(3, 3)[:, 1]
        axis = axis / np.linalg.norm(axis)
        down = np.array([0.0, 0.0, -1.0])
        perp = down - np.dot(down, axis) * axis
        n = np.linalg.norm(perp)
        if n < 1e-9:
            continue
        p = c + float(m.geom_size[g][0]) * (perp / n)
        if best is None or p[2] < best[2]:
            best = p
    return best


def build_ledger(xml_path, anc):
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
    # place so the lowest wheel just touches z=0
    lows = [wheel_lowest_point(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)) for n in WHE]
    d.qpos[2] += -min(p[2] for p in lows)
    mujoco.mj_forward(m, d)

    bias = d.qfrc_bias.copy()
    applied = np.zeros(m.nv)
    leg_dofs = [vadr[n] for n in LEGS]
    applied[leg_dofs] = anc['tau']
    b = bias - applied

    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    cols, points = [], []
    for i, n in enumerate(WHE):
        p = wheel_lowest_point(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n))
        points.append(p)
        mujoco.mj_jac(m, d, jacp, jacr, p, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n))
        for c in (2, 0, 1):   # per-wheel order: Fn=vertical z, Fx=longitudinal x, Fy=lateral y
            cols.append(jacp[c, :].copy())
    A = np.stack(cols, axis=1)          # (nv, 12)
    order = [vadr[n] for n in LEGS + WHE]      # canonical leg-major order
    return dict(m=m, d=d, A=A, b=b, A_joint=A[order, :], b_joint=b[order],
                points=points, bias=bias, applied=applied, order=order,
                leg_dofs=leg_dofs, qadr=qadr, vadr=vadr)


def cone_rows(mu):
    """8 linear inequalities (octagon) per wheel: rows over 12 vars."""
    rows = []
    s2 = np.sqrt(2.0)
    for i in range(4):
        base = np.zeros(12)
        def row(fn, fx, fy):
            r = base.copy()
            r[3 * i + 0] = fn
            r[3 * i + 1] = fx
            r[3 * i + 2] = fy
            return r
        rows += [row(-mu, 1, 0), row(-mu, -1, 0), row(-mu, 0, 1), row(-mu, 0, -1),
                 row(-s2 * mu, 1, 1), row(-s2 * mu, 1, -1),
                 row(-s2 * mu, -1, 1), row(-s2 * mu, -1, -1)]
        # Fn >= 0
        r = base.copy(); r[3 * i] = -1.0; rows.append(r)
    return np.stack(rows)


def solve(led, mu, tol, objective=None):
    A, b = led['A'], led['b']
    A_eq = A[0:6, :]
    b_eq = b[0:6]
    A_j = led['A_joint']
    b_j = led['b_joint']
    # |b_j - A_j x| <= tol
    A_ub = [A_j, -A_j]
    b_ub = [b_j + tol, -b_j + tol]
    A_ub = np.vstack(A_ub + [cone_rows(mu)])
    b_ub = np.concatenate(b_ub + [np.zeros(cone_rows(mu).shape[0])])
    c = np.zeros(12) if objective is None else objective
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq,
                  bounds=[(None, None)] * 12, method='highs')
    return res


def min_tol(led, mu, hi=200.0):
    lo = 0.0
    r = solve(led, mu, lo)
    if r.status == 0:
        return 0.0, r
    best = None
    for _ in range(28):
        mid = 0.5 * (lo + hi)
        r = solve(led, mu, mid)
        if r.status == 0:
            hi = mid
            best = r
        else:
            lo = mid
    return hi, best


def main():
    anc = load_anchor()
    out = dict(schema='n0_2_force_ledger_v1', mujoco_version=mujoco.__version__,
               anchor=dict(q=[round(float(x), 5) for x in anc['q']],
                           tau=[round(float(x), 4) for x in anc['tau']],
                           quat=[round(float(x), 6) for x in anc['quat']],
                           roll_deg=round(anc['roll_deg'], 4),
                           pitch_deg=round(anc['pitch_deg'], 4),
                           window=[anc['t0'], anc['t1']]),
               runs=[])

    for loadv0 in (False, True):
        xml = make_model(loadv0)
        led = build_ledger(xml, anc)
        m, d = led['m'], led['d']
        mass = float(np.sum(m.body_mass))
        pts = [[round(float(x), 5) for x in p] for p in led['points']]
        base = dict(asset=('loadv0' if loadv0 else 'delivery'), mass_kg=round(mass, 3),
                    contact_points=pts,
                    bias_base=[round(float(x), 3) for x in led['b'][0:6]],
                    bias_joint_max=round(float(np.max(np.abs(led['b_joint']))), 3))
        for mu in MU_GRID:
            tol, r = min_tol(led, mu)
            entry = dict(mu=mu, minmax_joint_correction_Nm=round(float(tol), 4),
                         feasible_with_zero_correction=bool(tol < 1e-9))
            # fixed 2 Nm band: comparable across mu
            r2 = solve(led, mu, 2.0)
            entry['feasible_with_2Nm_band'] = bool(r2.status == 0)
            if r2.status == 0:
                x = r2.x
                dt = led['b_joint'] - led['A_joint'] @ x
                entry['contact_forces_N'] = [[round(float(v), 2) for v in row]
                                             for row in x.reshape(4, 3)]
                entry['dtau_Nm'] = [round(float(v), 3) for v in dt]
                entry['max_abs_dtau'] = round(float(np.max(np.abs(dt))), 3)
                fy = []
                for i in range(4):
                    c = np.zeros(12); c[3 * i + 2] = 1.0
                    a = solve(led, mu, 2.0, c)
                    bq = solve(led, mu, 2.0, -c)
                    fy.append([round(float(a.x[3 * i + 2]), 2) if a.status == 0 else None,
                               round(float(bq.x[3 * i + 2]), 2) if bq.status == 0 else None])
                entry['Fy_range_N_2Nm'] = fy
                fn = []
                for i in range(4):
                    c = np.zeros(12); c[3 * i] = 1.0
                    a = solve(led, mu, 2.0, c)
                    bq = solve(led, mu, 2.0, -c)
                    fn.append([round(float(a.x[3 * i]), 2) if a.status == 0 else None,
                               round(float(bq.x[3 * i]), 2) if bq.status == 0 else None])
                entry['Fn_range_N_2Nm'] = fn
            out['runs'].append({**base, **entry})
            print(json.dumps(out['runs'][-1], ensure_ascii=False), flush=True)

        # per-joint detail at mu=0.25, 2 Nm band + G2-force residual
        r2 = solve(led, 0.25, 2.0)
        if r2.status == 0:
            x = r2.x
            g2 = np.array([48.22, 0.0, 17.59, 48.38, 0.0, -17.83,
                           79.47, 0.0, 5.73, 79.48, 0.0, -5.48])
            # per-hip moment decomposition  M_x = r_y F_z - r_z F_y
            hip_xyz, decomp = {}, {}
            for i, hn in enumerate(LEGS[:4]):
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, hn)
                hp = np.array(d.xpos[bid])
                hip_xyz[hn] = [round(float(v), 4) for v in hp]
                rows = []
                tot = 0.0
                for k, wn in enumerate(WHE):
                    r = np.array(led['points'][k]) - hp
                    fn, fx, fy = x[3 * k], x[3 * k + 1], x[3 * k + 2]
                    m_x = r[1] * fn - r[2] * fy
                    rows.append(dict(wheel=wn, r=[round(float(v), 4) for v in r],
                                     ryFz=round(float(r[1] * fn), 3),
                                     rzFy=round(float(-r[2] * fy), 3),
                                     Mx=round(float(m_x), 3)))
                    tot += m_x
                decomp[hn] = dict(terms=rows, total_Mx=round(float(tot), 3))
            detail = dict(
                asset=('loadv0' if loadv0 else 'delivery'), mu=0.25, band_Nm=2.0,
                joints=LEGS + WHE,
                bias=[round(float(v), 3) for v in led['b_joint']],
                tau_est=[round(float(v), 3) for v in anc['tau']] + [0.0] * 4,
                contact_generalized=[round(float(v), 3) for v in (led['A_joint'] @ x)],
                residual=[round(float(v), 3) for v in
                          (led['b_joint'] - led['A_joint'] @ x)],
                g2_force_residual=[round(float(v), 3) for v in
                                   (led['b_joint'] - led['A_joint'] @ g2)],
                hip_positions=hip_xyz,
                hip_moment_decomposition=decomp,
            )
            out.setdefault('joint_detail', {})[detail['asset']] = detail
            print('DETAIL ' + json.dumps(detail, ensure_ascii=False), flush=True)

    (OUT / 'N0_2_force_ledger.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'N0_2_force_ledger.json')


if __name__ == '__main__':
    main()

