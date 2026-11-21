#!/usr/bin/env python3
"""Phase E: MuJoCo same-asset control run.

The PhysX asset under test is the CURRENT training URDF (load-v0):
base mass 10.08 x 1.15 = 11.592 kg, base COM x shifted -15 mm (0.035002 ->
0.020002), wheel collision = true STL at the real wheel centre (track 0.44 m).
The Phase D MuJoCo matrix instead used the delivery MJCF (official 10.08 kg,
COM 0.035002). This script builds C1m = C1 (wheel-centre fix) + load-v0 base
mass/COM/inertia and reruns S/G/GO, so PhysX vs MuJoCo is a same-asset
comparison and the residual gap can be attributed to the contact model, not
to the mass/COM difference.

Run:
  /home/robot/z00945176/isaac_env/venv312/bin/python e2_mujoco_loadv0.py
"""
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import d1_closed_loop_matrix as d1  # noqa: E402

RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
C1_XML = RUNS / 'mujoco_c1' / 'as2w_scene.xml'
OUT_XML = RUNS / 'mujoco_c1m' / 'as2w_scene.xml'

# load-v0 base_link inertial (URDF). PhysX merges the 0.5 kg JT128 into base,
# so the runtime base body is 12.092 kg; MuJoCo keeps JT128 separate, hence the
# base body here stays 11.592 kg and the total stays the same 27.512 kg.
OLD = ('<inertial pos="0.035002 -0.0007562 0.017521" mass="10.08"\n'
       '                fullinertia="0.05069483 0.13822065 0.1505824 -0.0002906 -0.02480435 0.00027717"/>')
# C1m: load-v0 mass AND COM shift (the actual training asset)
NEW = ('<inertial pos="0.020002 -0.0007562 0.017521" mass="11.592"\n'
       '                fullinertia="0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455"/>')
# C1m_mass: load-v0 mass only, official COM (separates mass from COM effects)
NEW_MASS = ('<inertial pos="0.035002 -0.0007562 0.017521" mass="11.592"\n'
            '                fullinertia="0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455"/>')


def build_variant(tag, replacement):
    d = RUNS / f'mujoco_{tag}'
    d.mkdir(parents=True, exist_ok=True)
    xp = d / 'as2w_scene.xml'
    lnk = d / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(C1_XML.parent / 'meshes')
    txt = C1_XML.read_text()
    assert OLD in txt
    xp.write_text(txt.replace(OLD, replacement))
    return xp


def build_fixbase(tag, free_xml):
    dd = RUNS / f'mujoco_s{tag}'
    dd.mkdir(parents=True, exist_ok=True)
    xp = dd / 'as2w_scene.xml'
    xp.write_text(free_xml.read_text().replace(
        '<freejoint name="root"/>',
        '<!-- root fixed for suspended strap scenario -->'))
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(C1_XML.parent / 'meshes')
    return xp


def main():
    out = {}
    for tag, repl in (('c1m', NEW), ('c1m_mass', NEW_MASS)):
        free = build_variant(tag, repl)
        fb = build_fixbase(tag, free)
        out[tag] = {}
        for scen in ('S', 'G', 'GO'):
            r = d1.run_case(free if scen != 'S' else fb, scen)
            r['variant'] = tag
            out[tag][scen] = r
            print(tag, scen, r, flush=True)
    (RUNS / 'tables' / 'E_MUJOCO_LOADV0.json').write_text(json.dumps(out, indent=1))
    print('wrote E_MUJOCO_LOADV0.json')


if __name__ == '__main__':
    main()
