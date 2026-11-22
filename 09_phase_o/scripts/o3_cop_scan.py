#!/usr/bin/env python3
"""Phase O / O3 (limited S1): lateral CoP shift within the tread width.

The contact point is allowed to move along the wheel axis projected on the
ground, s in [-w, +w] with w = 0.026 m, keeping z = 0 (a deformed tyre patch is
flat on the ground).  This is a *geometrically bounded* CoP probe, not a free
contact model: the shift stays inside the 52 mm tread width.

Only a common shift (same s for all four wheels) plus a rear-only shift are
scanned; a full per-wheel grid is not justified before an independent Fn
measurement (Phase O task book §5).
"""
import importlib.util
import json
import pathlib

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('o0', ROOT / 'code' / 'o0_ledger.py')
o0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(o0)

OUT = ROOT / 'results'
OUT.mkdir(parents=True, exist_ok=True)


def shifted_points(base_led, s_common=0.0, s_rear=0.0):
    pts = []
    for i, w in enumerate(o0.WHE):
        info = base_led['info'][w]
        p = np.array(info['p_new'], dtype=float)
        a = np.array(info['axis'], dtype=float)
        ah = np.array([a[0], a[1], 0.0])
        ah = ah / max(np.linalg.norm(ah), 1e-9)
        s = s_common + (s_rear if w[:2] in ('RL', 'RR') else 0.0)
        q = p + s * ah
        q[2] = 0.0
        pts.append(q)
    return pts


def rebuild(base_led, pts):
    """Recompute A with new application points on the same model/data."""
    import mujoco
    m, d = base_led['m'], base_led['d']
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    cols = []
    for i, w in enumerate(o0.WHE):
        mujoco.mj_jac(m, d, jacp, jacr, pts[i],
                      mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w))
        for c in (2, 0, 1):
            cols.append(jacp[c, :].copy())
    A = np.stack(cols, axis=1)
    led = dict(base_led)
    led['A'] = A
    led['A_joint'] = A[base_led['order'], :]
    led['points'] = pts
    return led


def main():
    anc = o0.load_anchor()
    out = dict(schema='o3_cop_scan_v1', note='S1 limited CoP probe, |s| <= 0.026 m',
               rows=[])
    for loadv0 in (False, True):
        base = o0.build_ledger(o0.make_model(loadv0), anc)
        cases = [('common', s) for s in (-0.026, -0.013, 0.0, 0.013, 0.026)]
        cases += [('rear', s) for s in (-0.026, 0.013, 0.026)]
        for kind, s in cases:
            pts = shifted_points(base, s if kind == 'common' else 0.0,
                                 s if kind == 'rear' else 0.0)
            led = rebuild(base, pts)
            for mu in (0.25, 0.50):
                t = o0.min_tol(led, mu, True)
                row = dict(asset=('loadv0' if loadv0 else 'delivery'),
                           kind=kind, shift_m=s, mu=mu, t_star=round(float(t), 4))
                r = o0.solve(led, mu, max(2.0, t + 0.25), True)
                if r.status == 0:
                    x = r.x
                    fy = []
                    for i in range(4):
                        c1 = np.zeros(12); c1[3 * i + 2] = 1.0
                        a1 = o0.solve(led, mu, max(2.0, t + 0.25), True, c1)
                        a2 = o0.solve(led, mu, max(2.0, t + 0.25), True, -c1)
                        fy.append([round(float(a1.x[3 * i + 2]), 2) if a1.status == 0 else None,
                                   round(float(a2.x[3 * i + 2]), 2) if a2.status == 0 else None])
                    row['Fy_range_N'] = fy
                    row['rear_crosses_zero'] = [None if (a is None or b is None) else bool(a <= 0 <= b)
                                                for a, b in fy[2:]]
                out['rows'].append(row)
                print(json.dumps(row, ensure_ascii=False), flush=True)
    (OUT / 'O3_cop_scan.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'O3_cop_scan.json')


if __name__ == '__main__':
    main()
