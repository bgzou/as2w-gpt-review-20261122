#!/usr/bin/env python3
"""Phase H2: per-frame tau_PD = kp*(q_des-q)+kd*(0-dq)+tau_ff vs recorded tau_est.

Contract provenance (H1, RAW_DATA_CONFIRMED):
  - custom hipdiag sessions: debug_lowcmd FixedStand rows -> leg kp/kd, T2 targets frozen, tau_ff=0
  - official sessions: lowcmd_capture ai_stand_up hold -> kp=200/kd=4, frozen targets, tau_ff=0
Window definitions from Phase B/C adjudication (G2 uses same).
Outputs: tables/H2_TORQUE_PD_CONSISTENCY.{csv,json}, tables/H2_SESSION_TAU_STATS.csv
"""
import json
import pathlib

import numpy as np

RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
QC = RUNS / 'qc' / 'a1_timelines'
JN = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
      'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
      'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']

# window + contract per session (00_SOURCE_AND_CONTRACT.md §3, RAW_DATA_CONFIRMED)
SESS = {
    'OG':    dict(win=(17, 120), kp=200., kd=4.,  tgt=np.array([-0.15, .15, -0.15, .15] + [.6]*4 + [-1.2]*4)),
    'OS':    dict(win=(0, 60),   kp=200., kd=4.,  tgt=np.array([-0.15, .15, -0.15, .15] + [.6]*4 + [-1.2]*4)),
    'P55G1': dict(win=(10, 30),  kp=55.,  kd=2.,  tgt=np.array([0]*4 + [.8]*4 + [-1.6]*4)),
    'P55G2': dict(win=(8, 19),   kp=55.,  kd=2.,  tgt=np.array([0]*4 + [.8]*4 + [-1.6]*4)),
    'P55G3': dict(win=(7, 19),   kp=55.,  kd=2.,  tgt=np.array([0]*4 + [.8]*4 + [-1.6]*4)),
    'P55S':  dict(win=(25, 90),  kp=55.,  kd=2.,  tgt=np.array([0]*4 + [.8]*4 + [-1.6]*4)),
    'P85DS': dict(win=(48, 100), kp=85.,  kd=3.2, tgt=np.array([0]*4 + [.8]*4 + [-1.6]*4)),
}


def fit(x, y):
    A = np.vstack([x, np.ones_like(x)]).T
    (slope, inter), res, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = slope * x + inter
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return float(slope), float(inter), 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')


def plateau_frac(v):
    """Fraction of samples within 1% of the extreme magnitude (flat-top detector)."""
    m = np.abs(v).max()
    if m < 1e-9:
        return 0.0
    return float((np.abs(np.abs(v) - m) < 0.01 * m).mean())


def main():
    (RUNS / 'tables').mkdir(exist_ok=True)
    rows, jrows = [], []
    for tag, meta in SESS.items():
        d = np.load(QC / f'{tag}.npz')
        t, q, dq, tau = d['t'], d['q'], d['dq'], d['tau']
        temp = d['temp']
        m = (t >= meta['win'][0]) & (t < meta['win'][1])
        kp, kd, tgt = meta['kp'], meta['kd'], meta['tgt']
        e = tgt[None, :] - q[m]                    # (n,12)
        tau_pd = kp * e + kd * (0.0 - dq[m])       # tau_ff=0
        tau_est = tau[m]
        n = int(m.sum())
        # session-level joint loop
        for j, jn in enumerate(JN):
            x, y = tau_pd[:, j], tau_est[:, j]
            resid = y - x
            slope, inter, r2 = fit(x, y)
            corr = float(np.corrcoef(x, y)[0, 1]) if x.std() > 1e-12 and y.std() > 1e-12 else float('nan')
            front = jn.startswith(('FL', 'FR'))
            jrows.append(dict(
                session=tag, joint=jn, group='front' if front else 'rear',
                n=n,
                pd_mean=float(x.mean()), est_mean=float(y.mean()),
                pd_std=float(x.std()), est_std=float(y.std()),
                est_p95=float(np.percentile(y, 95)), est_absmax=float(np.abs(y).max()),
                resid_mean=float(resid.mean()), resid_std=float(resid.std()),
                resid_p95abs=float(np.percentile(np.abs(resid), 95)),
                resid_absmax=float(np.abs(resid).max()),
                corr=corr, fit_slope=slope, fit_intercept=inter, fit_r2=r2,
                est_plateau_frac=plateau_frac(y),
                e_q_mean=float(e[:, j].mean()), e_q_absmax=float(np.abs(e[:, j]).max()),
                temp_mean=float(temp[m][:, j].mean()), temp_max=float(temp[m][:, j].max()),
                # temporal correlation of residual with time and with temp (derating check)
                resid_time_corr=float(np.corrcoef(np.arange(n), resid)[0, 1]) if resid.std() > 1e-12 else float('nan'),
                resid_temp_corr=float(np.corrcoef(temp[m][:, j], resid)[0, 1]) if temp[m][:, j].std() > 1e-9 and resid.std() > 1e-12 else float('nan'),
            ))
        # session aggregate: worst joints
        rr = np.abs(tau_est - tau_pd)
        rows.append(dict(
            session=tag, window=meta['win'], n=n,
            kp=kp, kd=kd,
            resid_rms_all=float(np.sqrt((rr ** 2).mean())),
            resid_absmax_all=float(rr.max()),
            worst_joint=JN[int(rr.mean(axis=0).argmax())],
            est_absmax_all=float(np.abs(tau_est).max()),
            est_plateau_any=float(max(plateau_frac(tau_est[:, j]) for j in range(12))),
            temp_range_all=[float(temp[m].min()), float(temp[m].max())],
        ))
        print(f'{tag}: n={n} resid RMS={rows[-1]["resid_rms_all"]:.3f} Nm '
              f'max={rows[-1]["resid_absmax_all"]:.3f} Nm worst={rows[-1]["worst_joint"]} '
              f'temp {rows[-1]["temp_range_all"]}')
    (RUNS / 'tables').mkdir(exist_ok=True)
    import csv as _csv
    with open(RUNS / 'tables' / 'H2_TORQUE_PD_CONSISTENCY.csv', 'w', newline='') as f:
        w = _csv.DictWriter(f, fieldnames=list(jrows[0].keys()))
        w.writeheader()
        w.writerows(jrows)
    with open(RUNS / 'tables' / 'H2_SESSION_TAU_STATS.csv', 'w', newline='') as f:
        w = _csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(RUNS / 'tables' / 'H2_TORQUE_PD_CONSISTENCY.json', 'w') as f:
        json.dump(dict(per_joint=jrows, per_session=rows), f, indent=1, ensure_ascii=False)
    print('wrote H2 tables:', len(jrows), 'joint rows,', len(rows), 'sessions')


if __name__ == '__main__':
    main()
