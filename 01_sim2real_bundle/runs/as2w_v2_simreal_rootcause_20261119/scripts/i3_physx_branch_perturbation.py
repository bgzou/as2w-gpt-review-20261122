#!/usr/bin/env python3
"""I3: PhysX branch stability under small single-factor perturbations.

Baseline = G3 G-scenario real init, cyl wheel, kp55/kd2, implicit PD,
dt=0.005, nstep=12000 (matches G3 so baseline lands on G3's +4.01 deg cyl/real
branch). Each condition is run twice in independent app processes.

Conditions (14 runs):
  base          x2
  pitch +2 / -2 deg (root base)   x2 each
  hip   +0.02 / -0.02 rad (all 4 hips same direction) x2 each
  wheel-ground gap 0 mm / 4 mm    x2 each   (2 mm == baseline)

Branch-stability rule (from task):
  same branch if cross-condition settled-pitch range < 0.3 deg AND
  hip-q range < 0.02 rad; otherwise initial-value-sensitive / multi-branch.
"""
import csv
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import i_common as C  # noqa: E402

OUTD = C.ABL / 'i3_runs'
LOGD = C.ABL / 'i3_logs'
TABLE = C.TABLES / 'I3_PHYSX_BRANCH_PERTURBATION'
PRIM = 'cyl'
DT = 0.005
NSTEP = 12000

CONDS = [
    ('base', dict()),
    ('pitch_p2', dict(pitch_deg=2.0)),
    ('pitch_m2', dict(pitch_deg=-2.0)),
    ('hip_p002', dict(hip_offset=0.02)),
    ('hip_m002', dict(hip_offset=-0.02)),
    ('gap0', dict(gap_mm=0.0)),
    ('gap4', dict(gap_mm=4.0)),
]


def build_runs():
    runs = []
    for name, extra in CONDS:
        for rep in (1, 2):
            p = dict(prim=PRIM, init='real', mode='implicit', dt=DT,
                     nstep=NSTEP, kp=55.0, kd=2.0, friction=1.0,
                     outdir=str(OUTD))
            p.update(extra)
            runs.append((f'{name}_r{rep}', p))
    return runs


def analyze():
    rows = []
    for name, extra in CONDS:
        reps = []
        for rep in (1, 2):
            f = OUTD / f'{name}_r{rep}.json'
            if f.exists():
                reps.append(json.loads(f.read_text()))
        if len(reps) < 2:
            rows.append(dict(condition=name, perturbation=json.dumps(extra),
                             n_runs=len(reps), pitch_deg=float('nan'),
                             hip_q=[], Fn=[], settle_time_s=float('nan'),
                             repeat_dpitch=float('nan'),
                             repeat_dhipq=float('nan'),
                             notes='BLOCKED: missing replicate'))
            continue
        p = [r['pitch_deg'] for r in reps]
        hq = [np.array(r['hip_q']) for r in reps]
        fn = [np.array(r['Fn']) for r in reps]
        st = [r['settle_time_s'] for r in reps]
        rows.append(dict(
            condition=name, perturbation=json.dumps(extra), n_runs=2,
            pitch_deg=round(float(np.mean(p)), 3),
            hip_q=[round(float(x), 4) for x in np.mean(hq, 0)],
            Fn=[round(float(x), 2) for x in np.mean(fn, 0)],
            settle_time_s=round(float(np.mean(st)), 2),
            repeat_dpitch=round(float(abs(p[0] - p[1])), 3),
            repeat_dhipq=round(float(np.abs(hq[0] - hq[1]).max()), 4),
            settle_ok=bool(all(r['settle_ok'] for r in reps)),
            notes='implicit PD dt0.005; cyl; real init baseline',
        ))

    ok = [r for r in rows if np.isfinite(r.get('pitch_deg', float('nan')))]
    if ok:
        prange = max(r['pitch_deg'] for r in ok) - min(r['pitch_deg'] for r in ok)
        hqmat = np.array([r['hip_q'] for r in ok])
        hqrange = float((hqmat.max(0) - hqmat.min(0)).max())
        same_branch = bool(prange < 0.3 and hqrange < 0.02)
        verdict = ('SAME_BRANCH_STABLE_ATTRACTOR' if same_branch
                   else 'INIT_SENSITIVE_MULTIBRANCH')
        summary = dict(
            n_conditions=len(ok),
            cross_condition_pitch_range_deg=round(prange, 3),
            cross_condition_hip_q_range_rad=round(hqrange, 4),
            same_branch_rule='pitch_range<0.3deg AND hip_q_range<0.02rad',
            verdict=verdict,
            max_repeat_dpitch_deg=round(max(r['repeat_dpitch'] for r in ok), 3),
            max_repeat_dhipq_rad=round(max(r['repeat_dhipq'] for r in ok), 4),
        )
    else:
        summary = dict(verdict='NO_DATA')

    fields = ['condition', 'perturbation', 'n_runs', 'pitch_deg', 'hip_q',
              'Fn', 'settle_time_s', 'repeat_dpitch', 'repeat_dhipq',
              'settle_ok', 'notes']
    with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({
                'condition': r['condition'],
                'perturbation': r['perturbation'],
                'n_runs': r['n_runs'],
                'pitch_deg': r.get('pitch_deg'),
                'hip_q': '/'.join(f'{x:.4f}' for x in r.get('hip_q', [])),
                'Fn': '/'.join(f'{x:.2f}' for x in r.get('Fn', [])),
                'settle_time_s': r.get('settle_time_s'),
                'repeat_dpitch': r.get('repeat_dpitch'),
                'repeat_dhipq': r.get('repeat_dhipq'),
                'settle_ok': r.get('settle_ok'),
                'notes': r.get('notes', ''),
            })
    with open(TABLE.with_suffix('.json'), 'w') as f:
        json.dump(dict(summary=summary, rows=rows), f, indent=1)
    print('SUMMARY', json.dumps(summary))
    print('wrote', TABLE)


if __name__ == '__main__':
    if '--worker' in sys.argv:
        C.worker_cli()
    else:
        C.drive(pathlib.Path(__file__).resolve(), build_runs(), LOGD)
        analyze()
