#!/usr/bin/env python3
"""Phase P / P1-C: cross-session co-configuration ledger (P55G1 / P55G2 / P55G3 / OG).

Each session uses its own window, q, quat and tau_est; the same corrected
contact-point geometry, asset and mu are used.  OG keeps its own controller
contract (kp=200) — only tau_est is used as the joint-side input, so the ledger
is a conditional "what contact forces would be needed", not an OG validation.
"""
import importlib.util
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('o0', ROOT / 'code' / 'o0_ledger.py')
o0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(o0)

NPZ_DIR = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / 'input_npz')
OUT = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else ROOT / 'results')
WIN = {'P55G1': (10, 30), 'P55G2': (8, 19), 'P55G3': (7, 19), 'OG': (17, 120)}


def anchor_of(tag):
    z = np.load(NPZ_DIR / f'{tag}.npz')
    t = z['t']
    m = (t >= WIN[tag][0]) & (t <= WIN[tag][1])
    q = z['quat'][m].mean(0)
    q = q / np.linalg.norm(q)
    return dict(q=z['q'][m].mean(0), tau=z['tau'][m].mean(0), quat=q,
                pitch_deg=float(np.degrees(z['pitch'][m].mean())),
                roll_deg=float(np.degrees(z['roll'][m].mean()))), int(m.sum())


def main():
    out = dict(schema='p1c_cross_session_v1',
               note='same corrected geometry/asset/mu; per-session q/quat/tau_est',
               sessions=[])
    for tag in ('P55G1', 'P55G2', 'P55G3', 'OG'):
        p = NPZ_DIR / f'{tag}.npz'
        if not p.exists():
            out['sessions'].append(dict(session=tag, status='BLOCKED',
                                        reason=f'{p} not found'))
            print(json.dumps(out['sessions'][-1]), flush=True)
            continue
        anc, nfr = anchor_of(tag)
        row = dict(session=tag, window=list(WIN[tag]), n_frames=nfr,
                   pitch_deg=round(anc['pitch_deg'], 3), roll_deg=round(anc['roll_deg'], 3),
                   hip_q=[round(float(x), 4) for x in anc['q'][:4]],
                   tau_est_hip=[round(float(x), 3) for x in anc['tau'][:4]])
        for loadv0 in (False, True):
            led = o0.build_ledger(o0.make_model(loadv0), anc)
            t = o0.min_tol(led, 0.25, True)
            key = 'loadv0' if loadv0 else 'delivery'
            row[key] = dict(t_star=round(float(t), 4),
                            z_spread_mm=round(float(led['z_spread_mm']), 3))
            tr = max(2.0, t + 0.25)
            r = o0.solve(led, 0.25, tr, True)
            if r.status == 0:
                x = r.x
                fy = []
                for i in range(4):
                    c = np.zeros(12); c[3 * i + 2] = 1.0
                    a1 = o0.solve(led, 0.25, tr, True, c)
                    a2 = o0.solve(led, 0.25, tr, True, -c)
                    fy.append([round(float(a1.x[3 * i + 2]), 2) if a1.status == 0 else None,
                               round(float(a2.x[3 * i + 2]), 2) if a2.status == 0 else None])
                row[key]['Fy_range_N'] = fy
                row[key]['crosses_zero'] = [None if (a is None or b is None) else bool(a <= 0 <= b)
                                            for a, b in fy]
                row[key]['Fn'] = [round(float(x[3 * i]), 1) for i in range(4)]
        out['sessions'].append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    (OUT / 'P1C_cross_session.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'P1C_cross_session.json')


if __name__ == '__main__':
    main()
