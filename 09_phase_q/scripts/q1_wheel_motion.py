#!/usr/bin/env python3
"""Phase Q / Q1: last-60 s wheel motion decomposition from the Phase P 20 Hz series.

For each wheel: net dq, total variation sum|dq|, signed mean dq, RMS, P95/P99,
sign-change count, and a contact-slip proxy
    v_slip_long ~ vx_base + r * dq_wheel   (rolling direction, r = 0.086 m)
    v_slip_lat ~ vy_base
NOTE: this is a *model-calculation proxy* using the wheel hinge rate and the base
velocity, not the true contact-point velocity (contact position not recorded).
"""
import json
import pathlib
import sys

import numpy as np

RES = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else
                   '/root/gpufree-data/as2w-cpro/phase_klm_20261122/phase_p_20261122/results')
OUT = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else RES)
R_WHEEL = 0.086


def analyse(p):
    j = json.loads(p.read_text())
    s = j.get('series')
    if not s:
        return dict(run=p.stem, error='no series')
    t = np.array([r['t'] for r in s])
    dq16 = np.array([r['dq16'] for r in s])
    rv = np.array([r['root_vel'] for r in s])
    m = t >= (t[-1] - 60.0)
    dq = dq16[m][:, 12:16]
    tt = t[m]
    dt = float(np.median(np.diff(tt)))
    vx, vy = rv[m][:, 0], rv[m][:, 1]
    out = dict(run=p.stem, window_s=[round(float(tt[0]), 2), round(float(tt[-1]), 2)],
               dt=round(dt, 4), wheels=[])
    for i in range(4):
        q = dq[:, i]
        net = float(q.sum() * dt)
        tv = float(np.abs(q).sum() * dt)
        sign = np.sign(q)
        nz = sign[sign != 0]
        changes = int(np.sum(nz[1:] != nz[:-1])) if len(nz) > 1 else 0
        slip_long = vx + R_WHEEL * q
        out['wheels'].append(dict(
            wheel=i,
            net_dq_rad=round(net, 5),
            total_variation_rad=round(tv, 5),
            abs_mean_dq=round(float(np.abs(q).mean()), 5),
            signed_mean_dq=round(float(q.mean()), 5),
            rms=round(float(np.sqrt(np.mean(q ** 2))), 5),
            p95=round(float(np.percentile(np.abs(q), 95)), 5),
            p99=round(float(np.percentile(np.abs(q), 99)), 5),
            sign_changes=changes,
            v_slip_long_mean=round(float(slip_long.mean()), 5),
            v_slip_long_p95=round(float(np.percentile(np.abs(slip_long), 95)), 5),
            v_slip_lat_mean=round(float(vy.mean()), 5),
        ))
    # classification
    tv_sum = sum(w['total_variation_rad'] for w in out['wheels'])
    net_sum = sum(abs(w['net_dq_rad']) for w in out['wheels'])
    out['tv_vs_net'] = round(tv_sum / max(net_sum, 1e-9), 2)
    out['classification'] = ('OSCILLATORY_WHEEL_MICROMOTION' if tv_sum > 5 * max(net_sum, 1e-6)
                             else 'NET_WHEEL_MOTION')
    return out


def main():
    rows = []
    for p in sorted(RES.glob('p1a_*.json')):
        r = analyse(p)
        rows.append(r)
        print(json.dumps(r, ensure_ascii=False), flush=True)
    (OUT / 'Q1_wheel_motion.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'Q1_wheel_motion.json')


if __name__ == '__main__':
    main()
