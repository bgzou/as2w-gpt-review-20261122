#!/usr/bin/env python3
"""Phase P / P2: operator and conservation tests.

1. finite-difference check of the contact-point Jacobian (mj_jac jacp) against
   the FK contact point at several configurations;
2. hip generalized force from the axis cross product vs A[hip_dof] @ f, for
   random physically admissible force sets (not just the LP optimum);
3. one-sidedness / base residual bookkeeping of the O0 solution;
4. PhysX per-contact conservation: not available in the archived JSON
   (record_contacts=False) -> UNKNOWN.
"""
import importlib.util
import json
import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('o0', ROOT / 'code' / 'o0_ledger.py')
o0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(o0)

OUT = ROOT / 'results'
OUT.mkdir(parents=True, exist_ok=True)
RNG = np.random.default_rng(20261122)


def setup(loadv0, anc, dq=None):
    xml = o0.make_model(loadv0)
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in o0.LEGS + o0.WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in o0.LEGS + o0.WHE}
    d.qpos[:] = 0
    d.qpos[3:7] = anc['quat']
    for i, n in enumerate(o0.LEGS):
        d.qpos[qadr[n]] = anc['q'][i] + (0.0 if dq is None else dq[i])
    mujoco.mj_forward(m, d)
    return m, d, qadr


def contact_pts(m, d):
    pts = []
    for w in o0.WHE:
        c, a, r, hw = o0.wheel_cyl(m, d, w)
        p, _, _ = o0.lowest_point(c, a, r, hw)
        pts.append(p)
    return pts


def main():
    anc = o0.load_anchor()
    out = dict(schema='p2_operator_tests_v1', fd=[], hip_random=[], o0_bookkeeping={},
               physx_conservation='UNKNOWN (archived JSON has record_contacts=False; '
                                   'net_forces_w only)')
    for loadv0 in (False, True):
        m, d, qadr = setup(loadv0, anc)
        # --- 1) finite-difference Jacobian
        eps = 1e-6
        base_pts = contact_pts(m, d)
        jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
        test_dofs = []
        for n in ('FL_hip', 'FR_thigh', 'RL_calf'):
            jidn = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            test_dofs.append((n, int(m.jnt_qposadr[jidn]), int(m.jnt_dofadr[jidn])))
        test_dofs += [('base_x', 0, 0), ('base_y', 1, 1), ('base_z', 2, 2)]
        for name, qidx, dof in test_dofs:
            mujoco.mj_jac(m, d, jacp, jacr, base_pts[0],
                          mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_foot'))
            ana = jacp[:, dof].copy()
            # track the SAME material point: the geometric "lowest point" changes
            # material identity as the wheel tilts, so FD of that function is not
            # the Jacobian; FD of a fixed local point is.
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_foot')
            Rb = np.array(d.xmat[bid]).reshape(3, 3)
            p_loc = Rb.T @ (base_pts[0] - np.array(d.xpos[bid]))

            def world_of_local():
                Rn = np.array(d.xmat[bid]).reshape(3, 3)
                return Rn @ p_loc + np.array(d.xpos[bid])

            q0 = d.qpos.copy()
            d.qpos[:] = q0
            d.qpos[qidx] += eps
            mujoco.mj_forward(m, d)
            pp = world_of_local()
            d.qpos[:] = q0
            d.qpos[qidx] -= eps
            mujoco.mj_forward(m, d)
            pm = world_of_local()
            d.qpos[:] = q0
            mujoco.mj_forward(m, d)
            fd = (pp - pm) / (2 * eps)
            out['fd'].append(dict(asset=('loadv0' if loadv0 else 'delivery'), dof=name,
                                  analytic=[round(float(x), 6) for x in ana],
                                  fd=[round(float(x), 6) for x in fd],
                                  max_abs_diff=round(float(np.max(np.abs(ana - fd))), 8)))
        # --- 2) random admissible force sets vs hip Jacobian
        led = o0.build_ledger(o0.make_model(loadv0), anc)
        lm, ld = led['m'], led['d']
        mu = 0.6
        for trial in range(5):
            f = []
            for _ in range(4):
                fn = float(RNG.uniform(20, 100))
                th = float(RNG.uniform(0, 2 * np.pi))
                ft = float(RNG.uniform(0, mu * fn * 0.8))
                f += [fn, ft * np.cos(th), ft * np.sin(th)]   # (Fn, Fx, Fy) in LP order
            f = np.array(f)
            for hn in o0.LEGS[:4]:
                leg = hn[:2]
                i = o0.WHE.index(leg + '_foot')
                j = mujoco.mj_name2id(lm, mujoco.mjtObj.mjOBJ_JOINT, hn)
                axis = np.array(ld.xaxis[j]); anchor = np.array(ld.xanchor[j])
                r = np.array(led['points'][i]) - anchor
                fvec = np.array([f[3 * i + 1], f[3 * i + 2], f[3 * i]])
                tau_cross = float(axis @ np.cross(r, fvec))
                tau_jac = float(led['A_joint'][led['order'].index(led['vadr'][hn]), :] @ f)
                out['hip_random'].append(dict(asset=('loadv0' if loadv0 else 'delivery'),
                                              trial=trial, hip=hn,
                                              tau_cross=round(tau_cross, 5),
                                              tau_jac=round(tau_jac, 5),
                                              diff=round(tau_cross - tau_jac, 6)))
        # --- 3) O0 bookkeeping
        r = o0.solve(led, 0.25, 2.16)
        if r.status == 0:
            x = r.x
            out['o0_bookkeeping'][('loadv0' if loadv0 else 'delivery')] = dict(
                base_resid_N=round(float(np.max(np.abs(led['A'][0:3, :] @ x - led['b'][0:3]))), 6),
                base_resid_Nm=round(float(np.max(np.abs(led['A'][3:6, :] @ x - led['b'][3:6]))), 6),
                min_Fn=round(float(min(x[0::3])), 4),
                max_cone_violation=round(float(max(o0.cone_violation(x, 0.25))), 6),
                max_joint_resid=round(float(np.max(np.abs(led['b_joint'] - led['A_joint'] @ x))), 4))
    (OUT / 'P2_operator_tests.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k not in ('hip_random',)},
                     ensure_ascii=False)[:2000])
    print('wrote', OUT / 'P2_operator_tests.json')


if __name__ == '__main__':
    main()
