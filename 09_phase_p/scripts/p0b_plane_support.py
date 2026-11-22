#!/usr/bin/env python3
"""Phase P / P0-B: ground-plane fit, support subsets and minimal vertical
compliance model.

1. least-squares plane z = a x + b y + c through the four corrected lowest points;
   report tilt, residuals and peak-to-peak.
2. for every 3-point subset: plane through the three, signed distance of the 4th
   (geometric feasibility of 3-wheel support).
3. constrained static LP on 3-wheel and 2-wheel support subsets (non-contacting
   wheels get exactly zero force).
4. minimal compliance model Fn_i = k * max(0, penetration_i) for assumed k and
   plane tilt, with c solved so that sum Fn = W; report Fn split and the
   implied CoP vs the CoP required by the base wrench.
"""
import importlib.util
import json
import itertools
import pathlib

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('o0', ROOT / 'code' / 'o0_ledger.py')
o0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(o0)

OUT = ROOT / 'results'
OUT.mkdir(parents=True, exist_ok=True)
G = 9.81


def plane_fit(pts):
    A = np.array([[p[0], p[1], 1.0] for p in pts])
    z = np.array([p[2] for p in pts])
    coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    res = z - A @ coef
    tilt = float(np.degrees(np.arctan(np.hypot(coef[0], coef[1]))))
    return dict(a=float(coef[0]), b=float(coef[1]), c=float(coef[2]),
                tilt_deg=round(tilt, 4),
                residual_mm=[round(float(r) * 1000, 3) for r in res],
                peak_to_peak_mm=round(float(res.max() - res.min()) * 1000, 3))


def lp_subset(led, mu, tol, active):
    """LP with only `active` wheel indices carrying force."""
    A = led['A']
    cols = []
    for i in range(4):
        if i in active:
            cols += [3 * i, 3 * i + 1, 3 * i + 2]
    A_sel = A[:, cols]
    Aj = A_sel[led['order'], :]
    bj = led['b_joint']
    n = len(cols)
    scale = mu * np.cos(np.pi / 32)
    rows = []
    for k, i in enumerate(active):
        for t in range(32):
            th = 2 * np.pi * t / 32
            r = np.zeros(n)
            r[3 * k + 0] = -scale
            r[3 * k + 1] = np.cos(th)
            r[3 * k + 2] = np.sin(th)
            rows.append(r)
        r = np.zeros(n); r[3 * k] = -1.0
        rows.append(r)
    A_ub = np.vstack([Aj, -Aj, np.array(rows)])
    b_ub = np.concatenate([bj + tol, -bj + tol, np.zeros(len(rows))])
    from scipy.optimize import linprog
    res = linprog(np.zeros(n), A_ub=A_ub, b_ub=b_ub,
                  A_eq=A_sel[0:6, :], b_eq=led['b'][0:6],
                  bounds=[(None, None)] * n, method='highs')
    return res, cols


def min_tol_subset(led, mu, active, hi=200.0):
    r0, _ = lp_subset(led, mu, 0.0, active)
    if r0.status == 0:
        return 0.0
    lo = 0.0
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        r, _ = lp_subset(led, mu, mid, active)
        if r.status == 0:
            hi = mid
        else:
            lo = mid
    return hi


def compliance(pts, W, k, tilt_deg):
    """Given plane tilt, find c so that sum Fn = W; return Fn and penetration."""
    th = np.radians(tilt_deg)
    a, b = np.sin(th), 0.0
    def fn_of_c(c):
        f = []
        for p in pts:
            pen = a * p[0] + b * p[1] + c - p[2]
            f.append(k * max(0.0, pen))
        return np.array(f)
    lo, hi = -0.05, 0.05
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if fn_of_c(mid).sum() < W:
            lo = mid
        else:
            hi = mid
    c = 0.5 * (lo + hi)
    f = fn_of_c(c)
    pen = np.array([a * p[0] + b * p[1] + c - p[2] for p in pts])
    return f, pen, c


def main():
    anc = o0.load_anchor()
    out = dict(schema='p0b_plane_support_v1',
               note=('plane fit is a model calculation, not a measured ground slope; '
                     'compliance k is an assumed conditional parameter'),
               assets=[])
    for loadv0 in (False, True):
        led = o0.build_ledger(o0.make_model(loadv0), anc)
        asset = 'loadv0' if loadv0 else 'delivery'
        W = float(np.sum(led['m'].body_mass) * G)
        pts = [list(map(float, p)) for p in led['points']]
        entry = dict(asset=asset, weight_N=round(W, 2),
                     points=[[round(v, 5) for v in p] for p in pts],
                     horizontal_spread_mm=round((max(p[2] for p in pts) -
                                                 min(p[2] for p in pts)) * 1000, 3),
                     plane=plane_fit(pts), subsets=[])
        # 3-point subsets
        for sub in itertools.combinations(range(4), 3):
            coef, *_ = np.linalg.lstsq(
                np.array([[pts[i][0], pts[i][1], 1.0] for i in sub]),
                np.array([pts[i][2] for i in sub]), rcond=None)
            j = [i for i in range(4) if i not in sub][0]
            d = pts[j][2] - (coef[0] * pts[j][0] + coef[1] * pts[j][1] + coef[2])
            t3 = min_tol_subset(led, 0.25, sub)
            entry['subsets'].append(dict(support=list(sub), lifted=j,
                                         gap_mm=round(float(d) * 1000, 3),
                                         t_star_3wheel=round(float(t3), 4)))
        # 2-point subsets
        for sub in itertools.combinations(range(4), 2):
            t2 = min_tol_subset(led, 0.25, sub)
            entry.setdefault('two_wheel', []).append(
                dict(support=list(sub), t_star_2wheel=round(float(t2), 4)))
        # compliance
        comp = []
        for k in (2e4, 5e4, 1e5, 2e5):
            for tilt in (0.0, 0.253, 0.5):
                f, pen, c = compliance(pts, W, k, tilt)
                comp.append(dict(k_N_per_m=k, tilt_deg=tilt, c_m=round(c, 5),
                                 Fn=[round(float(x), 2) for x in f],
                                 penetration_mm=[round(float(x) * 1000, 3) for x in pen],
                                 all_four_loaded=bool(np.all(f > 1.0)),
                                 fn_sum=round(float(f.sum()), 2)))
        entry['compliance'] = comp
        out['assets'].append(entry)
        print(json.dumps({k: v for k, v in entry.items() if k not in ('compliance',)},
                         ensure_ascii=False), flush=True)
    (OUT / 'P0B_plane_support.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'P0B_plane_support.json')


if __name__ == '__main__':
    main()
