#!/usr/bin/env python3
"""Phase A2: curated stable windows + 10s segmentation + change-point on hip tau
+ PD/contract verification + session_qc cross-check. Model coordinates.

Curated windows (from experiment_manifest.csv text; garbled CSV cells resolved
by reading the manifest notes, source tagged in 01 doc):
  OG 17-120, OS 0-60, P55G1 10-30, P55G2 7-20, P55G3 6-20,
  P55S 21-110, P85DS 42-122
"""
import csv
import json
import pathlib

import numpy as np

RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
DOCS = pathlib.Path('/home/robot/z00945176/Rob2Nav/docs/as2w_v2_simreal_rootcause_20261119')
V2 = pathlib.Path('/home/robot/z00945176/Rob2Nav/docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119')
JN = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
      'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
      'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WIN = {'OG': (17, 120), 'OS': (0, 60), 'P55G1': (10, 30), 'P55G2': (7, 20),
       'P55G3': (6, 20), 'P55S': (21, 110), 'P85DS': (42, 122)}
KP = {'OG': 200, 'OS': 200, 'P55G1': 55, 'P55G2': 55, 'P55G3': 55, 'P55S': 55, 'P85DS': 85}
TGT = {'official': np.array([-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6, -1.2, -1.2, -1.2, -1.2]),  # V2 model=native (hip sign +1)
       'hipdiag': np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])}


def main():
    out_rows, seg_rows, cp_rows = [], [], []
    summary = {}
    for tag, (t0, t1) in WIN.items():
        z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
        t, q, tau, e = z['t'], z['q'], z['tau'], z['e']
        pitch, roll = z['pitch'], z['roll']
        kind = 'official' if tag in ('OG', 'OS') else 'hipdiag'
        tgt = TGT[kind]
        m = (t >= t0) & (t <= t1)
        assert m.sum() > 1000, (tag, m.sum())
        # --- PD consistency: tau_est vs kp*e (kd term ~0 in steady state)
        for j, jn in enumerate(JN):
            tau_pd = KP[tag] * (tgt[j] - q[m, j])
            resid = tau[m, j] - tau_pd
            out_rows.append(dict(
                session=tag, joint=jn, window=f'{t0}-{t1}', n=int(m.sum()),
                st_q_mean=round(float(q[m, j].mean()), 4),
                st_q_std=round(float(q[m, j].std()), 4),
                st_tau_mean=round(float(tau[m, j].mean()), 3),
                st_tau_std=round(float(tau[m, j].std()), 3),
                st_tau_p5=round(float(np.percentile(tau[m, j], 5)), 3),
                st_tau_p95=round(float(np.percentile(tau[m, j], 95)), 3),
                e_q_mean=round(float(e[m, j].mean()), 4),
                pd_resid_mean=round(float(resid.mean()), 3),
                pd_resid_rms=round(float(np.sqrt((resid ** 2).mean())), 3)))
        # --- 10s segmentation inside window
        for a in np.arange(t0, t1, 10):
            b = min(a + 10, t1)
            mm = m & (t >= a) & (t < b)
            if mm.sum() < 500:
                continue
            row = dict(session=tag, seg=f'{a:.0f}-{b:.0f}', n=int(mm.sum()),
                       pitch_deg=round(float(np.degrees(pitch[mm].mean())), 2),
                       roll_deg=round(float(np.degrees(roll[mm].mean())), 2))
            for j, jn in enumerate(JN[:4]):
                row[f'{jn}_tau'] = round(float(tau[mm, j].mean()), 3)
                row[f'{jn}_q'] = round(float(q[mm, j].mean()), 4)
            for j, jn in enumerate(JN[8:]):
                row[f'{jn}_tau'] = round(float(tau[mm, j].mean()), 3)
            seg_rows.append(row)
        # --- change-point scan on hip tau inside window (10s cumulative)
        for j, jn in enumerate(JN[:4]):
            x = tau[m, j]; tt = t[m]
            # binary segmentation on mean shift, min segment 5s
            cp = []
            def scan(lo, hi, depth=0):
                if hi - lo < 2500 or depth > 2:
                    return
                x_ = x[lo:hi]
                best, bi = 0.0, -1
                csum = np.cumsum(x_) - x_[0]
                n = np.arange(1, len(x_))
                mu_all = x_.mean()
                # SSE reduction split
                sse = ((x_ - mu_all) ** 2).sum()
                for i in n[500:-500:100]:
                    l, r = x_[:i], x_[i:]
                    s = ((l - l.mean()) ** 2).sum() + ((r - r.mean()) ** 2).sum()
                    red = sse - s
                    if red > best:
                        best, bi = red, i
                if bi > 0 and best > 2.0 * x_.var():  # meaningful threshold
                    cp.append((round(float(tt[lo + bi]), 1),
                               round(float(l.mean()), 3), round(float(r.mean()), 3)))
                    scan(lo, lo + bi, depth + 1)
                    scan(lo + bi, hi, depth + 1)
            scan(0, len(x))
            cp_rows.append(dict(session=tag, joint=jn,
                                full_std=round(float(x.std()), 3),
                                seg_std_minmax=f'{x.std():.2f}',
                                changepoints=json.dumps(cp)))
        summary[tag] = dict(
            window=(t0, t1), n=int(m.sum()),
            hip_tau=[round(float(tau[m, j].mean()), 3) for j in range(4)],
            calf_tau=[round(float(tau[m, 8 + j].mean()), 3) for j in range(4)],
            thigh_tau=[round(float(tau[m, 4 + j].mean()), 3) for j in range(4)],
            hip_q=[round(float(q[m, j].mean()), 4) for j in range(4)],
            pitch_deg=round(float(np.degrees(pitch[m].mean())), 2),
            roll_deg=round(float(np.degrees(roll[m].mean())), 2))
        print(tag, summary[tag])
    def wcsv(path, rows):
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(path, 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=keys)
            wr.writeheader()
            wr.writerows(rows)
        print('wrote', path, len(rows))
    (RUNS / 'tables').mkdir(exist_ok=True, parents=True)
    wcsv(DOCS / 'REAL_V2_ALL_JOINT_WINDOWS.csv', out_rows)
    wcsv(RUNS / 'tables' / 'A_SEG10_WINDOWS.csv', seg_rows)
    wcsv(RUNS / 'tables' / 'A_CHANGEPOINT_HIP.csv', cp_rows)
    json.dump(summary, open(RUNS / 'tables' / 'A_SUMMARY_MODEL_COORDS.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
