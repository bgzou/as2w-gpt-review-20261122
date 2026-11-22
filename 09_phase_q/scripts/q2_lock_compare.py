#!/usr/bin/env python3
"""Phase Q / Q2: compare the baseline pitch_p2 trajectory with the two lock
ablations (base xy lock, wheel q lock) over the same 0-260 s window."""
import json
import pathlib
import sys

import numpy as np

RES = pathlib.Path(sys.argv[1])
OUT = pathlib.Path(sys.argv[2])
FILES = {
    'A_baseline': 'p1a_dt005_pitch_series_360s.json',
    'B_lock_base_xy': 'q_p1b_lock_base_xy_260s.json',
    'C_lock_wheel_q': 'q_p1b_lock_wheel_q_260s.json',
}
TS = [60, 120, 180, 210, 240, 260]


def pitch_of(q):
    x, y, z, w = q
    return float(np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))


def load(path):
    j = json.loads(path.read_text())
    s = j['series']
    t = np.array([r['t'] for r in s])
    rp = np.array([r['root_pose'] for r in s])
    q16 = np.array([r['q16'] for r in s])
    dq16 = np.array([r['dq16'] for r in s])
    fn = np.array([r['Fn'] for r in s])
    return dict(t=t, rp=rp, q16=q16, dq16=dq16, fn=fn,
                pitch=np.array([pitch_of(r['root_pose'][3:7]) for r in s]),
                lock=j.get('lock', {}))


def main():
    rows = []
    for name, fname in FILES.items():
        p = RES / fname
        if not p.exists():
            rows.append(dict(run=name, status='MISSING', file=str(p)))
            continue
        d = load(p)
        t = d['t']
        row = dict(run=name, lock=d['lock'], n=len(t), t_end=round(float(t[-1]), 1))
        row['samples'] = {}
        for T in TS:
            i = int(np.argmin(np.abs(t - T)))
            row['samples'][str(T)] = dict(
                pitch=round(float(d['pitch'][i]), 3),
                base_x=round(float(d['rp'][i, 0]), 4),
                base_y=round(float(d['rp'][i, 1]), 4),
                wheel_q=[round(float(x), 3) for x in d['q16'][i, 12:16]],
                Fn=[round(float(x), 1) for x in d['fn'][i]])
        # 180-260 window
        m = (t >= 180) & (t <= min(260, t[-1]))
        row['w180_260'] = dict(
            pitch_change=round(float(d['pitch'][m][-1] - d['pitch'][m][0]), 3),
            base_x_change=round(float(d['rp'][m][-1, 0] - d['rp'][m][0, 0]), 4),
            wheel_q_change=[round(float(x), 3) for x in
                            (d['q16'][m][-1, 12:16] - d['q16'][m][0, 12:16])],
            wheel_dq_p99=round(float(np.percentile(np.abs(d['dq16'][m][:, 12:16]), 99)), 4),
            n_unloaded=int(np.sum(d['fn'][m].min(1) < 5.0)),
            Fn_min=[round(float(x), 1) for x in d['fn'][m].min(0)])
        # last 60 s
        m2 = t >= (t[-1] - 60.0)
        row['last60'] = dict(
            base_xy=round(float(np.linalg.norm(d['rp'][m2][-1, 0:2] - d['rp'][m2][0, 0:2])), 5),
            wheel_dq_p99=round(float(np.percentile(np.abs(d['dq16'][m2][:, 12:16]), 99)), 4),
            pitch_drift=round(float(d['pitch'][m2][-1] - d['pitch'][m2][0]), 3),
            n_unloaded=int(np.sum(d['fn'][m2].min(1) < 5.0)))
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    (OUT / 'Q2_lock_ablation.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'Q2_lock_ablation.json')


if __name__ == '__main__':
    main()
