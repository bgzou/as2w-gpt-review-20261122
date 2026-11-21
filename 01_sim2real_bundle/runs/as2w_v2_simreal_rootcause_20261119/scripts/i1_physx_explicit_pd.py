#!/usr/bin/env python3
"""I1-explicit: PhysX explicit-PD control group vs G3 implicit-PD.

G3 PhysX used ImplicitActuatorCfg (PD inside the PhysX solver). MuJoCo used
explicit qfrc_applied PD at 500 Hz. This script removes the implicit-drive
confound: drive stiffness/damping are zeroed and tau = kp*(tgt-q)+kd*(0-dq)
(kp=55,kd=2) is injected every sim step via set_joint_effort_target_index
(-> PhysX set_dof_actuation_forces). Wheels: tau = clip(-2*dq, -15, 15).

Runs (6):
  explicit  dt=0.002 nstep=20000 (40 s)  x {cyl,mesh} x {real,nearflat}
  implicit  dt=0.002 nstep=20000 (40 s)  x {cyl,mesh} x {real}   <- same-dt control

Outputs tables/I1_PHYSX_EXPLICIT_PD.{csv,json} with G3 implicit rows merged for
side-by-side comparison.
"""
import csv
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import i_common as C  # noqa: E402

OUTD = C.ABL / 'i1_runs'
LOGD = C.ABL / 'i1_logs'
TABLE = C.TABLES / 'I1_PHYSX_EXPLICIT_PD'
G3_TABLE = C.TABLES / 'G3_ENGINE_PRIMITIVE_MATRIX.json'

DT = 0.002
NSTEP = 20000


def build_runs():
    runs = []
    for prim in ('cyl', 'mesh'):
        for init in ('real', 'nearflat'):
            tag = f'exp_{prim}_{init}'
            runs.append((tag, dict(prim=prim, init=init, mode='explicit',
                                   dt=DT, nstep=NSTEP, kp=55.0, kd=2.0,
                                   friction=1.0, outdir=str(OUTD))))
    for prim in ('cyl', 'mesh'):
        tag = f'imp_dt002_{prim}_real'
        runs.append((tag, dict(prim=prim, init='real', mode='implicit',
                               dt=DT, nstep=NSTEP, kp=55.0, kd=2.0,
                               friction=1.0, outdir=str(OUTD))))
    return runs


def merge():
    rows = []
    for tag in ('exp_cyl_real', 'exp_cyl_nearflat', 'exp_mesh_real',
                'exp_mesh_nearflat', 'imp_dt002_cyl_real', 'imp_dt002_mesh_real'):
        f = OUTD / f'{tag}.json'
        if f.exists():
            r = json.loads(f.read_text())
            r['run_tag'] = tag
            rows.append(r)
        else:
            rows.append(dict(run_tag=tag, mode=tag.split('_')[0],
                             wheel_primitive='?', init_condition='?',
                             pitch_deg=float('nan'), settle_ok=False,
                             notes=f'BLOCKED: no result {f.name}'))
    # merge G3 implicit (dt=0.005) G rows for comparison
    if G3_TABLE.exists():
        for r in json.loads(G3_TABLE.read_text()):
            if r.get('engine') == 'physx' and r.get('scenario') == 'G':
                rr = dict(r)
                rr['run_tag'] = f"G3_imp_dt005_{r['wheel_primitive']}_{r['init_condition']}"
                rr['mode'] = 'implicit_dt005'
                rr['dt'] = 0.005
                rows.append(rr)
    fields = ['run_tag', 'mode', 'dt', 'wheel_primitive', 'init_condition',
              'hip_tau_Nm', 'hip_q', 'pitch_deg', 'Fn', 'settle_ok',
              'settle_time_s', 'notes']
    with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({
                'run_tag': r.get('run_tag'),
                'mode': r.get('mode'),
                'dt': r.get('dt'),
                'wheel_primitive': r.get('wheel_primitive'),
                'init_condition': r.get('init_condition'),
                'hip_tau_Nm': '/'.join(f'{x:.2f}' for x in r.get('hip_tau_Nm', [])),
                'hip_q': '/'.join(f'{x:.4f}' for x in r.get('hip_q', [])),
                'pitch_deg': r.get('pitch_deg'),
                'Fn': '/'.join(f'{x:.1f}' for x in r.get('Fn', [])),
                'settle_ok': r.get('settle_ok'),
                'settle_time_s': r.get('settle_time_s'),
                'notes': r.get('notes', ''),
            })
    with open(TABLE.with_suffix('.json'), 'w') as f:
        json.dump(rows, f, indent=1)
    print('wrote', TABLE, len(rows), 'rows')


if __name__ == '__main__':
    if '--worker' in sys.argv:
        C.worker_cli()
    else:
        C.drive(pathlib.Path(__file__).resolve(), build_runs(), LOGD)
        merge()
