#!/usr/bin/env python3
"""Phase P / P1-A analysis: last-60 s stationarity and contact events from the
20 Hz series produced by i_common_series.py."""
import json
import pathlib
import sys

import numpy as np

RES = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else
                   '/root/gpufree-data/as2w-cpro/phase_klm_20261122/phase_p_20261122/results')
OUT = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else RES)


def pitch_of(q):
    # root_pose quat order (x,y,z,w)
    x, y, z, w = q
    return float(np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))


def analyse(p):
    j = json.loads(p.read_text())
    s = j.get('series')
    if not s:
        return dict(run=p.stem, error='no series')
    t = np.array([r['t'] for r in s])
    rp = np.array([r['root_pose'] for r in s])
    rv = np.array([r['root_vel'] for r in s])
    q16 = np.array([r['q16'] for r in s])
    dq16 = np.array([r['dq16'] for r in s])
    fn = np.array([r['Fn'] for r in s])
    pitch = np.array([pitch_of(r['root_pose'][3:7]) for r in s])

    def window(t0, t1):
        m = (t >= t0) & (t <= t1)
        if m.sum() < 5:
            return None
        dq_w = dq16[m][:, 12:16]
        dq_l = dq16[m][:, 0:12]
        v = np.linalg.norm(rv[m][:, 0:2], axis=1)
        return dict(
            n=int(m.sum()),
            wheel_q_change=[round(float(x), 4) for x in
                            (q16[m][-1, 12:16] - q16[m][0, 12:16])],
            wheel_dq_p99=round(float(np.percentile(np.abs(dq_w), 99)), 4),
            wheel_dq_max=round(float(np.abs(dq_w).max()), 4),
            leg_dq_p99=round(float(np.percentile(np.abs(dq_l), 99)), 4),
            base_xy_drift_m=round(float(np.linalg.norm(rp[m][-1, 0:2] - rp[m][0, 0:2])), 5),
            base_speed_p99=round(float(np.percentile(v, 99)), 5),
            pitch_drift_deg=round(float(pitch[m][-1] - pitch[m][0]), 3),
            pitch_p99_abs_dq=round(float(np.percentile(np.abs(pitch[m]), 99)), 3),
            Fn_mean=[round(float(x), 1) for x in fn[m].mean(0)],
            Fn_min=[round(float(x), 1) for x in fn[m].min(0)],
            n_unloaded_samples=int(np.sum(fn[m].min(1) < 5.0)),
        )

    # contact-loss events over the whole run
    events = []
    unloaded = fn.min(1) < 5.0
    if unloaded.any():
        idx = np.where(unloaded)[0]
        start = idx[0]
        for k in range(1, len(idx)):
            if idx[k] != idx[k - 1] + 1:
                events.append(dict(t0=round(float(t[start]), 2),
                                   t1=round(float(t[idx[k - 1]]), 2),
                                   wheels=[int(i) for i in np.where(fn[idx[k - 1]] < 5.0)[0]]))
                start = idx[k]
        events.append(dict(t0=round(float(t[start]), 2), t1=round(float(t[idx[-1]]), 2),
                           wheels=[int(i) for i in np.where(fn[idx[-1]] < 5.0)[0]]))
    return dict(
        run=p.stem, n_samples=len(t), dt_series=round(float(np.median(np.diff(t))), 4),
        base_x_total=round(float(rp[-1, 0] - rp[0, 0]), 4),
        base_y_total=round(float(rp[-1, 1] - rp[0, 1]), 4),
        wheel_rotation_total=[round(float(x), 3) for x in (q16[-1, 12:16] - q16[0, 12:16])],
        last60=window(max(0.0, t[-1] - 60.0), t[-1]),
        window_180_240=window(180.0, 240.0),
        unload_events=events[:20],
    )


def main():
    rows = []
    for p in sorted(RES.glob('p1a_*.json')):
        if p.name.endswith('_analysis.json'):
            continue
        r = analyse(p)
        rows.append(r)
        print(json.dumps(r, ensure_ascii=False), flush=True)
    (OUT / 'P1A_series_analysis.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'P1A_series_analysis.json')


if __name__ == '__main__':
    main()
