#!/usr/bin/env python3
"""G3 digest: branch sensitivity (spread across inits) + engine gap per primitive."""
import csv
import json
import pathlib

RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
T = RUNS / 'tables' / 'G3_ENGINE_PRIMITIVE_MATRIX'

rows = json.loads(T.with_suffix('.json').read_text())
by = {(r['engine'], r['scenario'], r['wheel_primitive'], r['init_condition']): r
      for r in rows}

print('=== CSV parse check ===')
with open(T.with_suffix('.csv'), newline='') as f:
    rd = list(csv.DictReader(f))
print('CSV rows:', len(rd), '| JSON rows:', len(rows))

print('\n=== Branch sensitivity: spread of settled pitch/hip_q across 3 inits ===')
print(f"{'engine':7s} {'scen':4s} {'prim':4s} {'pitch spread':>12s} "
      f"{'hip_q spread (max-min per joint)':>34s}  all_same_branch")
for scen in ('G', 'GO'):
    for prim in ('cyl', 'mesh'):
        for eng in ('mujoco', 'physx'):
            rs = [by.get((eng, scen, prim, i)) for i in ('real', 'nearflat', 'crouch')]
            if any(r is None for r in rs):
                print(f'{eng:7s} {scen:4s} {prim:4s}  MISSING')
                continue
            ps = [r['pitch_deg'] for r in rs]
            pspread = max(ps) - min(ps)
            hq = [r['hip_q'] for r in rs]
            if all(len(x) == 4 for x in hq):
                hspread = max(max(x[k] for x in hq) - min(x[k] for x in hq)
                              for k in range(4))
            else:
                hspread = float('nan')
            same = (pspread < 1.0) and (hspread < 0.1)
            print(f'{eng:7s} {scen:4s} {prim:4s} {pspread:11.2f}d '
                  f'{hspread:33.3f}  {same}')

print('\n=== Engine gap (MuJoCo - PhysX) for same primitive/scenario/init ===')
print(f"{'scen':4s} {'prim':4s} {'init':9s} {'MJ pitch':>9s} {'PX pitch':>9s} "
      f"{'gap':>7s}   MJ hip_q / PX hip_q")
for scen in ('G', 'GO'):
    for prim in ('cyl', 'mesh'):
        for init in ('real', 'nearflat', 'crouch'):
            mj = by.get(('mujoco', scen, prim, init))
            px = by.get(('physx', scen, prim, init))
            if not mj or not px:
                continue
            gap = mj['pitch_deg'] - px['pitch_deg']
            print(f'{scen:4s} {prim:4s} {init:9s} {mj["pitch_deg"]:8.2f} '
                  f'{px["pitch_deg"]:8.2f} {gap:6.2f}   '
                  f'{mj["hip_q"]} / {px["hip_q"]}')

print('\n=== Measured-branch reproduction (pitch -6.1, hip_q ~0.21-0.28) ===')
for r in rows:
    if r['scenario'] in ('G', 'GO') and r.get('pitch_deg') is not None:
        p = r['pitch_deg']
        if abs(p - (-6.1)) < 1.5:
            print('MATCH', r['engine'], r['scenario'], r['wheel_primitive'],
                  r['init_condition'], 'pitch', p, 'hip_q', r['hip_q'])

print('\n=== BLOCKED / failed settles ===')
for r in rows:
    if not r.get('settle_ok', True):
        print('NOT-SETTLED', r['engine'], r['scenario'], r['wheel_primitive'],
              r['init_condition'], r.get('notes', ''))
