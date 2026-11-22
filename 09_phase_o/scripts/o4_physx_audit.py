#!/usr/bin/env python3
"""Phase O / O4: audit existing PhysX long-run JSONs for wheel unload, rolling
and base drift.  Read-only: no simulation is run.

For every checkpoint of every run in the Phase K/L experiment directory:
  * per-wheel Fn, count of wheels with Fn < 5 N,
  * wheel joint angles (qpos of the four wheel hinges) and their total change,
  * base x/y/z and pitch,
  * instantaneous max |dq| over the 12 leg joints.
"""
import json
import pathlib
import sys

import numpy as np

RES = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else
                   '/root/gpufree-data/as2w-cpro/phase_hij_20261121/experiments/p0b_dt_20261122/results')
OUT = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else
                   '/root/gpufree-data/as2w-cpro/phase_klm_20261122/runs/as2w_phase_o_20261122/results')


def main():
    summary = []
    for p in sorted(RES.glob('*.json')):
        j = json.loads(p.read_text())
        cks = sorted(j.get('checkpoints', []), key=lambda c: c['t_s'])
        if not cks:
            continue
        rows = []
        for c in cks:
            fn = np.array(c.get('Fn', [np.nan] * 4), dtype=float)
            q = np.array(c.get('joint_q', [np.nan] * 16), dtype=float)
            dq = np.array(c.get('joint_dq', [np.nan] * 16), dtype=float)
            rp = c.get('root_pose_w', [np.nan] * 7)
            rows.append(dict(
                t=c['t_s'], pitch=c.get('pitch_deg'),
                base_xyz=[round(float(v), 4) for v in rp[0:3]],
                Fn=[round(float(v), 1) for v in fn],
                n_unloaded=int(np.sum(fn < 5.0)),
                wheel_q=[round(float(v), 3) for v in q[12:16]],
                dq_leg_max=round(float(np.nanmax(np.abs(dq[0:12]))), 4),
            ))
        wheel_total = [round(float(rows[-1]['wheel_q'][i] - rows[0]['wheel_q'][i]), 3)
                       for i in range(4)]
        fn_series = np.array([r['Fn'] for r in rows], dtype=float)
        entry = dict(run=p.stem, dt=j.get('dt'), nstep=j.get('nstep'),
                     pitch_init=j.get('pitch_init_deg'), hip_offset=j.get('hip_offset_rad'),
                     rows=rows,
                     any_unloaded=bool(np.any(fn_series < 5.0)),
                     unloaded_events=[[r['t'], r['Fn']] for r in rows if r['n_unloaded'] > 0],
                     wheel_angle_change_rad=wheel_total,
                     base_xy_drift_m=[round(float(rows[-1]['base_xyz'][0] - rows[0]['base_xyz'][0]), 4),
                                      round(float(rows[-1]['base_xyz'][1] - rows[0]['base_xyz'][1]), 4)],
                     base_z_change_m=round(float(rows[-1]['base_xyz'][2] - rows[0]['base_xyz'][2]), 4),
                     pitch_change_deg=round(float(rows[-1]['pitch'] - rows[0]['pitch']), 3))
        summary.append(entry)
        print(json.dumps({k: v for k, v in entry.items() if k != 'rows'}, ensure_ascii=False),
              flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'O4_physx_audit.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'O4_physx_audit.json')


if __name__ == '__main__':
    main()
