#!/usr/bin/env python3
"""I4-PhysX: contact info extraction + static equilibrium residual.

One baseline run (G scenario, cyl, real init, implicit kp55/kd2, dt=0.005,
nstep=12000) with ContactSensorCfg(track_contact_points=True,
track_friction_forces=True, max_contact_data_count_per_prim=64).

Per-contact-point individual positions/forces are NOT exposed by the PhysX
ContactSensor (only the average contact position per sensor body plus net
normal/friction force vectors) -> per-point rows are marked UNKNOWN with the
attempted API documented.

Static balance (settled window means):
  sum Fz vs weight (26.0 kg * 9.81 = 255.06 N, model mass convention)
  sum Fx, sum Fy vs 0
  CoP = sum(x_i*Fz_i)/sum(Fz_i)
  moment residual about CoM and about world origin
"""
import csv
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import i_common as C  # noqa: E402

OUTD = C.ABL / 'i4_runs'
LOGD = C.ABL / 'i4_logs'
TABLE = C.TABLES / 'I4_PHYSX_CONTACT_BALANCE'
TAG = 'base_contacts'
DT = 0.005
NSTEP = 12000


def build_runs():
    return [(TAG, dict(prim='cyl', init='real', mode='implicit', dt=DT,
                       nstep=NSTEP, kp=55.0, kd=2.0, friction=1.0,
                       record_contacts=True, outdir=str(OUTD)))]


def analyze():
    f = OUTD / f'{TAG}.json'
    if not f.exists():
        rows = [dict(item='ALL', value=None, unit='', evidence='UNKNOWN',
                     notes=f'BLOCKED: no result {f.name}')]
        _write(rows, dict(verdict='BLOCKED'))
        return
    r = json.loads(f.read_text())
    cs = r['contact_series']
    fn_vec = np.array(cs['Fn_vec'])          # (T,4,3)
    fn_n = np.array(cs['Fn_normal'])         # (T,4,3)
    fn_f = np.array(cs['Fn_friction'])       # (T,4,3)
    cpos = np.array(cs['contact_pos'])       # (T,4,1,3)
    com = np.array(cs['root_com'])           # (T,3)
    cpos = cpos[:, :, 0, :]                  # (T,4,3)

    F = fn_vec.mean(0)                       # (4,3)
    N = fn_n.mean(0)
    Fr = fn_f.mean(0)
    P = cpos.mean(0)                         # (4,3)
    com_m = com.mean(0)
    W = C.WEIGHT_N
    sumF = F.sum(0)
    sumFz = float(sumF[2])
    sumFx = float(sumF[0])
    sumFy = float(sumF[1])
    fz = F[:, 2]
    if abs(fz.sum()) > 1e-6:
        cop = (P[:, :2] * fz[:, None]).sum(0) / fz.sum()
    else:
        cop = np.array([np.nan, np.nan])
    # moment about CoM (gravity acts at CoM -> zero gravity moment)
    r_rel = P - com_m[None, :]
    M_com = np.cross(r_rel, F).sum(0)
    # moment about world origin including weight
    M0 = np.cross(P, F).sum(0) + np.cross(com_m, np.array([0, 0, -W]))
    contact_pos_known = bool(np.isfinite(P).all())

    rows = []
    def add(item, value, unit, evidence, notes):
        rows.append(dict(item=item, value=value, unit=unit,
                         evidence=evidence, notes=notes))
    add('sum_Fz', round(sumFz, 2), 'N', 'SIM_RUN_CONFIRMED',
        'sum of per-wheel net normal-ish contact force z')
    add('weight_model', round(W, 2), 'N', 'MODEL_CALCULATION',
        '26.0 kg (delivery model mass) * 9.81; NOT weighed')
    add('sum_Fz_minus_weight', round(sumFz - W, 2), 'N', 'MODEL_CALCULATION',
        'static vertical residual')
    add('sum_Fx', round(sumFx, 2), 'N', 'SIM_RUN_CONFIRMED', 'horizontal residual x')
    add('sum_Fy', round(sumFy, 2), 'N', 'SIM_RUN_CONFIRMED', 'horizontal residual y')
    add('Fn_per_wheel', '/'.join(f'{v:.2f}' for v in np.linalg.norm(F, axis=1)),
        'N', 'SIM_RUN_CONFIRMED', 'FL/FR/RL/RR net contact force magnitude')
    add('Fn_normal_per_wheel', '/'.join(f'{v:.2f}' for v in np.linalg.norm(N, axis=1)),
        'N', 'SIM_RUN_CONFIRMED', 'net_normal_forces_w magnitude')
    add('Fn_friction_per_wheel', '/'.join(f'{v:.2f}' for v in np.linalg.norm(Fr, axis=1)),
        'N', 'SIM_RUN_CONFIRMED', 'net_friction_forces_w magnitude')
    add('CoP_xy', f'{cop[0]:.4f}/{cop[1]:.4f}' if np.isfinite(cop).all() else 'UNKNOWN',
        'm', 'SIM_RUN_CONFIRMED' if contact_pos_known else 'UNKNOWN',
        'Fz-weighted mean of average contact positions')
    add('contact_pos_per_wheel',
        '/'.join('[' + ','.join(f'{x:.3f}' for x in p) + ']' for p in P)
        if contact_pos_known else 'UNKNOWN',
        'm', 'SIM_RUN_CONFIRMED' if contact_pos_known else 'UNKNOWN',
        'ContactSensor contact_pos_w (average per body)')
    add('moment_about_CoM', f'{np.linalg.norm(M_com):.3f}', 'Nm',
        'MODEL_CALCULATION' if contact_pos_known else 'UNKNOWN',
        '|sum (r_i-com) x F_i|; static should be ~0')
    add('moment_about_origin', f'{np.linalg.norm(M0):.3f}', 'Nm',
        'MODEL_CALCULATION' if contact_pos_known else 'UNKNOWN',
        '|sum r_i x F_i + com x (0,0,-W)|')
    add('per_contact_point', 'UNKNOWN', '', 'UNKNOWN',
        'PhysX ContactSensor exposes only net force + average contact_pos_w; '
        'no per-contact-point position/force arrays. Tried: net_forces_w, '
        'net_normal_forces_w, net_friction_forces_w, contact_pos_w, '
        'force_matrix_w (needs filter_prim_paths_expr), '
        'max_contact_data_count_per_prim. Individual points not exposed.')

    summary = dict(
        verdict='SIM_RUN_CONFIRMED',
        sum_Fz=round(sumFz, 2), weight_model=round(W, 2),
        vertical_residual=round(sumFz - W, 2),
        horizontal_residual=[round(sumFx, 2), round(sumFy, 2)],
        CoP_xy=[round(float(cop[0]), 4), round(float(cop[1]), 4)]
        if np.isfinite(cop).all() else None,
        moment_about_CoM_Nm=round(float(np.linalg.norm(M_com)), 3)
        if contact_pos_known else None,
        moment_about_origin_Nm=round(float(np.linalg.norm(M0)), 3)
        if contact_pos_known else None,
        per_contact_point='UNKNOWN',
        baseline_run=dict(pitch_deg=r['pitch_deg'], hip_q=r['hip_q'],
                          hip_tau_Nm=r['hip_tau_Nm'], Fn=r['Fn'],
                          settle_ok=r['settle_ok']),
    )
    _write(rows, summary)


def _write(rows, summary):
    with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['item', 'value', 'unit', 'evidence', 'notes'])
        w.writeheader()
        for r in rows:
            w.writerow(r)
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
