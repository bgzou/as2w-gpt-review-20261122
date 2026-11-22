#!/usr/bin/env python3
"""Phase P / P0-A: re-audit the S1 CoP scan geometry.

The Phase O o3_cop_scan.py started from the *axial end* point p_new (at
t = -w*sign(a_z) along the cylinder axis) and moved +-26 mm along the horizontal
projection of the axis.  Half of those points fall outside |t| <= w.

This script
  1. computes the axial coordinate t = (p - c).a for every old scan point and
     lists the out-of-tread ones;
  2. reproduces s=0 -> O0 p_new exactly and checks t* against O0 (tolerance 0.01);
  3. scans only *inward* along the axis, d = 0/5/10/20 mm, per wheel using
     sign(a_z) so that "inward" is consistent, and reports the required vertical
     deformation d*|a_z| (the point is put back on z=0) and the resulting
     t*, Fy intervals and cross-zero flags.
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


def main():
    anc = o0.load_anchor()
    out = dict(schema='p0a_s1_geometry_v1',
               note=('axial coordinate t=(p-c).a must satisfy |t|<=w=0.026; '
                     'inward scan only, point kept on z=0 -> reported deformation'),
               assets=[])
    for loadv0 in (False, True):
        led = o0.build_ledger(o0.make_model(loadv0), anc)
        info = led['info']
        asset = 'loadv0' if loadv0 else 'delivery'
        base = dict(asset=asset, wheels={}, old_scan_violations=[], scans=[])
        # 1) axial coordinate of p_new and of the old o3 scan points
        for w in o0.WHE:
            c = np.array(info[w]['centre'], float)
            a = np.array(info[w]['axis'], float)
            ww = float(info[w]['w'])
            p_new = np.array(info[w]['p_new'], float)
            t0 = float((p_new - c) @ a)
            ah = np.array([a[0], a[1], 0.0])
            ah = ah / max(np.linalg.norm(ah), 1e-9)
            rows = []
            for s in (-0.026, -0.013, 0.0, 0.013, 0.026):
                p = p_new + s * ah
                t = float((p - c) @ a)
                rows.append(dict(s=s, t_mm=round(t * 1000, 2),
                                 inside=bool(abs(t) <= ww + 1e-9)))
            base['wheels'][w] = dict(t_pnew_mm=round(t0 * 1000, 2), w_mm=ww * 1000,
                                     tilt_deg=info[w]['tilt_deg'], scan=rows)
            bad = [r for r in rows if not r['inside']]
            if bad:
                base['old_scan_violations'].append(dict(wheel=w,
                                                        points=[r['s'] for r in bad]))
        # 2) s=0 must reproduce O0
        t0_star = o0.min_tol(led, 0.25, True)
        base['t_star_s0'] = round(float(t0_star), 4)
        base['t_star_o0_reference'] = 2.1573 if not loadv0 else 1.3841
        base['s0_matches_o0'] = bool(abs(t0_star - base['t_star_o0_reference']) < 0.01)
        # 3) legal inward scan
        for d_mm in (0.0, 5.0, 10.0, 20.0):
            pts = []
            deform = []
            for w in o0.WHE:
                c = np.array(info[w]['centre'], float)
                a = np.array(info[w]['axis'], float)
                p = np.array(info[w]['p_new'], float)
                d = d_mm / 1000.0
                q = p + d * np.sign(a[2]) * a
                deform.append(abs(q[2]) * 1000.0)
                q[2] = 0.0
                pts.append(q)
            led2 = dict(led)
            import mujoco
            m, dd = led['m'], led['d']
            jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
            cols = []
            for i, w in enumerate(o0.WHE):
                mujoco.mj_jac(m, dd, jacp, jacr, pts[i],
                              mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w))
                for cidx in (2, 0, 1):
                    cols.append(jacp[cidx, :].copy())
            A = np.stack(cols, axis=1)
            led2['A'] = A
            led2['A_joint'] = A[led['order'], :]
            led2['points'] = pts
            for mu in (0.25, 0.50):
                t = o0.min_tol(led2, mu, True)
                row = dict(d_mm=d_mm, mu=mu, t_star=round(float(t), 4),
                           deformation_mm=[round(x, 3) for x in deform],
                           points=[[round(float(v), 5) for v in p] for p in pts])
                tr = max(2.0, t + 0.25)
                r = o0.solve(led2, mu, tr, True)
                if r.status == 0:
                    x = r.x
                    fy = []
                    for i in range(4):
                        c1 = np.zeros(12); c1[3 * i + 2] = 1.0
                        a1 = o0.solve(led2, mu, tr, True, c1)
                        a2 = o0.solve(led2, mu, tr, True, -c1)
                        fy.append([round(float(a1.x[3 * i + 2]), 2) if a1.status == 0 else None,
                                   round(float(a2.x[3 * i + 2]), 2) if a2.status == 0 else None])
                    row['Fy_range_N'] = fy
                    row['crosses_zero'] = [None if (a is None or b is None) else bool(a <= 0 <= b)
                                           for a, b in fy]
                base['scans'].append(row)
                print(json.dumps(row, ensure_ascii=False), flush=True)
        out['assets'].append(base)
        print(json.dumps({k: v for k, v in base.items() if k != 'scans'}, ensure_ascii=False),
              flush=True)
    (OUT / 'P0A_s1_geometry.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'P0A_s1_geometry.json')


if __name__ == '__main__':
    main()
