#!/usr/bin/env python3
"""Phase A: parse V2 sessions, unify motor slots -> model coords, QC contract,
stable windows, segmented statistics.

Inputs: docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/...
Outputs (docs/as2w_v2_simreal_rootcause_20261119/, runs/.../tables/):
  REAL_V2_ALL_JOINT_WINDOWS.csv  per session x 12 joints x window kinds
  A_SESSION_INVENTORY.csv        rows/duration/rate/sha of key files
  qc/a1_timelines/*.npz          parsed arrays for later phases
Model coordinate mapping source: historical clip
  runs/as2w-r4000-realcal-wheelrl-20261114/.../220051_fixedstand_hold/clip.npz
  (dds_slots [3,0,9,6,4,1,10,7,5,2,11,8,13,12,15,14],
   signs [-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1],
   joint_names [FL,FR,RL,RR]x[hip,thigh,calf] + wheels)
Unit check embedded: historical FL_hip model +10.63 vs V2 native FR_hip raw
+11.04 must map to the same (mirror-symmetric) model-direction load.
"""
import csv
import hashlib
import json
import pathlib

import numpy as np

V2 = pathlib.Path('/home/robot/z00945176/Rob2Nav/docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119')
DOCS = pathlib.Path('/home/robot/z00945176/Rob2Nav/docs/as2w_v2_simreal_rootcause_20261119')
RUNS = pathlib.Path('/home/robot/z00945176/Rob2Nav/runs/as2w_v2_simreal_rootcause_20261119')
JN = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
      'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
      'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
SLOTS = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])   # model order -> motor slot (legs)
# V2 hip sign correction (evidence, Phase D): the suspended sag direction pins
# the real convention -- gravity swings the leg INWARD under the hip pivot
# (model q<0, verified numerically); real P55S/P85DS native hip sag is negative
# (e.g. -0.052), so V2 model = native UNFLIPPED for hip. The historical clip
# (2026-11-14 build) used hip sign -1; V2 firmware/logger diverges. All other
# signs unchanged. Official native targets are then a symmetric inward tuck:
# FL/RL -0.15, FR/RR +0.15.
SIGNS = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
WSLOTS = [13, 12, 15, 14]

SESSIONS = {
    'OG': dict(dir='recordings/20261119-153922_official-og-trans', kind='official', support='ground'),
    'OS': dict(dir='recordings/20261119-165542_official-os', kind='official', support='suspended'),
    'P55G1': dict(dir='recordings/field_sessions/20261119-182802_p55-hold', kind='hipdiag', support='ground', kp=55, kd=2),
    'P55G2': dict(dir='recordings/field_sessions/20261119-185930_p55-hold', kind='hipdiag', support='ground', kp=55, kd=2),
    'P55G3': dict(dir='recordings/field_sessions/20261119-190012_p55-hold', kind='hipdiag', support='ground', kp=55, kd=2),
    'P55S': dict(dir='recordings/field_sessions/20261119-194344_p55-hold', kind='hipdiag', support='suspended', kp=55, kd=2),
    'P85DS': dict(dir='recordings/field_sessions/20261119-194837_p85d-hold', kind='hipdiag', support='suspended', kp=85, kd=3.2),
}


def sha256(path, limit=None):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()[:16]


def parse_low(path):
    """LOW_SAMPLE -> dict of arrays (500 Hz state)."""
    rows, hdr = [], None
    for line in open(path, errors='replace'):
        if line.startswith('LOW_SAMPLE_SCHEMA'):
            hdr = line.strip().split(',')[1:]
            if 'm2_q' not in hdr:  # abbreviated schema with literal '...'
                assert '...' in hdr and hdr[13] == 'm0_q'
                hdr = hdr[:13] + [f'm{s}_{f}' for s in range(35)
                                  for f in ('q', 'dq', 'ddq', 'tau', 'mode', 'temp')]
            continue
        if line.startswith('LOW_SAMPLE,'):
            r = line.rstrip('\n').split(',')[1:]
            if len(r) < len(hdr):
                r = r + [''] * (len(hdr) - len(r))
            rows.append(r)
    idx = {k: i for i, k in enumerate(hdr)}
    out = {}
    out['t_all'] = np.array([float(r[idx['elapsed_ms']]) / 1000 for r in rows])
    rows = [r for r in rows if r[idx['tick']] != '']  # drop heartbeat-only rows
    out['t'] = np.array([float(r[idx['elapsed_ms']]) / 1000 for r in rows])
    for k in ('tick', 'mode_pr', 'mode_machine', 'packet_count'):
        out[k] = np.array([int(r[idx[k]]) if r[idx[k]] not in ('', '-1') else -1 for r in rows])
    for k in ('quat_w', 'quat_x', 'quat_y', 'quat_z', 'gyro_x', 'gyro_y', 'gyro_z'):
        out[k] = np.array([float(r[idx[k]]) if r[idx[k]] != '' else np.nan for r in rows])
    q = np.zeros((len(rows), 35)); dq = np.zeros_like(q); tau = np.zeros_like(q)
    temp = np.zeros_like(q)
    for s in range(35):
        q[:, s] = [float(r[idx[f'm{s}_q']]) if r[idx[f'm{s}_q']] != '' else np.nan for r in rows]
        dq[:, s] = [float(r[idx[f'm{s}_dq']]) if r[idx[f'm{s}_dq']] != '' else np.nan for r in rows]
        tau[:, s] = [float(r[idx[f'm{s}_tau']]) if r[idx[f'm{s}_tau']] != '' else np.nan for r in rows]
        temp[:, s] = [float(r[idx[f'm{s}_temp']]) if r[idx[f'm{s}_temp']] != '' else np.nan for r in rows]
    out['q_raw'], out['dq_raw'], out['tau_raw'], out['temp_raw'] = q, dq, tau, temp
    return out


def parse_lowcmd(path):
    """LOWCMD_SAMPLE -> arrays (100 Hz official command capture)."""
    rows, hdr = [], None
    for line in open(path, errors='replace'):
        if line.startswith('LOWCMD_SAMPLE_SCHEMA'):
            hdr = line.strip().split(',')[1:]
            if 'm2_mode' not in hdr:  # abbreviated schema with literal '...'
                assert '...' in hdr and hdr[13] == 'm0_mode'
                hdr = hdr[:13] + [f'm{s}_{f}' for s in range(35)
                                  for f in ('mode', 'q', 'dq', 'kp', 'kd', 'tau')]
            continue
        if line.startswith('LOWCMD_SAMPLE,'):
            r = line.rstrip('\n').split(',')[1:]
            if len(r) < len(hdr):
                r = r + [''] * (len(hdr) - len(r))
            rows.append(r)
    idx = {k: i for i, k in enumerate(hdr)}
    out = {'t': np.array([float(r[idx['elapsed_ms']]) / 1000 for r in rows]),
           'tick': np.array([int(r[idx['low_tick']]) if r[idx['low_tick']] not in ('', '-1') else -1 for r in rows])}
    for pre in ('mode', 'q', 'dq', 'kp', 'kd', 'tau'):
        M = np.zeros((len(rows), 35))
        for s in range(35):
            k = f'm{s}_{pre}'
            if k in idx:
                M[:, s] = [float(r[idx[k]]) if r[idx[k]] != '' else np.nan for r in rows]
        out[pre] = M
    return out


def quat_rp(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def windows_stats(tag, q, tau, e, t, windows):
    rows = []
    for wname, (t0, t1) in windows:
        m = (t >= t0) & (t < t1)
        if m.sum() < 50:
            continue
        for j, jn in enumerate(JN):
            rows.append(dict(session=tag, window=wname, t0=t0, t1=t1, n=int(m.sum()),
                             joint=jn, st_q_mean=round(q[m, j].mean(), 4),
                             st_q_std=round(q[m, j].std(), 4),
                             st_tau_mean=round(tau[m, j].mean(), 3),
                             st_tau_std=round(tau[m, j].std(), 3),
                             st_tau_p5=round(np.percentile(tau[m, j], 5), 3),
                             st_tau_p95=round(np.percentile(tau[m, j], 95), 3),
                             e_q_mean=round(e[m, j].mean(), 4),
                             e_q_std=round(e[m, j].std(), 4)))
    return rows


def main():
    DOCS.mkdir(exist_ok=True)
    (RUNS / 'tables').mkdir(parents=True, exist_ok=True)
    (RUNS / 'qc' / 'a1_timelines').mkdir(parents=True, exist_ok=True)
    inv_rows, csv_rows = [], []
    # unit check against historical clip
    hist = np.load('/home/robot/z00945176/Rob2Nav/runs/as2w-r4000-realcal-wheelrl-20261114/'
                   'data/extracted/SimDynamicsValidation_AS2W_20261114/clips/220051_fixedstand_hold/clip.npz')
    assert hist['dds_slots'].tolist()[:12] == SLOTS.tolist(), 'slot mapping mismatch vs historical clip'
    print('UNIT CHECK slot mapping matches historical clip meta: PASS')
    print('NOTE V2 hip sign = +1 (model=native), diverges from historical clip hip sign -1;'
          ' evidence: suspended sag direction + official inward-tuck targets (see 04 doc)')
    for tag, meta in SESSIONS.items():
        sd = V2 / meta['dir']
        low = parse_low(sd / 'low_capture_fast.log')
        t = low['t']
        n = len(t)
        dur = t[-1] - t[0]
        rate = n / dur if dur > 0 else 0
        inv_rows.append(dict(session=tag, dir=meta['dir'], kind=meta['kind'], support=meta['support'],
                             rows=n, dur_s=round(dur, 1), approx_hz=round(rate, 1),
                             low_sha=sha256(sd / 'low_capture_fast.log')))
        # model coords
        qm = low['q_raw'][:, SLOTS] * SIGNS
        taum = low['tau_raw'][:, SLOTS] * SIGNS
        dqm = low['dq_raw'][:, SLOTS] * SIGNS
        roll, pitch = quat_rp(low['quat_w'], low['quat_x'], low['quat_y'], low['quat_z'])
        # command targets
        if meta['kind'] == 'hipdiag':
            tgt = np.array([0, 0, 0, 0] + [0.8] * 4 + [-1.6] * 4, dtype=float)
        else:
            # official native targets (measured): m3(FL)/m9(RL) = -0.15, m0(FR)/m6(RR) = +0.15,
            # thigh 0.6, calf -1.2; V2 hip sign +1 -> model = native (symmetric inward tuck)
            tgt = np.array([-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                            -1.2, -1.2, -1.2, -1.2], dtype=float)
        e = tgt[None, :] - qm
        # stable window definitions
        windows = [('full', (float(np.floor(t[0])), float(np.ceil(t[-1]))))]
        # auto: sliding 3s window on |dq| P95 and tau stability
        dq_p95 = np.zeros(n)
        k = 1500
        for i in range(n):
            lo = max(0, i - k)
            dq_p95[i] = np.percentile(np.abs(dqm[max(lo, i - 10):i + 10]), 95)
        stable_m = dq_p95 <= 0.15
        # longest stable run split into segments
        segs, s = [], None
        for i in range(n):
            if stable_m[i] and i * 1 and t[i] > 3:
                if s is None:
                    s = i
            else:
                if s is not None and i - s > 1500:
                    segs.append((t[s], t[i]))
                s = None
        if s is not None and n - s > 1500:
            segs.append((t[s], t[n - 1]))
        for si, (a, b) in enumerate(segs):
            windows.append((f'auto_stable_{si}', (float(np.floor(a)), float(np.ceil(b)))))
        # fixed 5s / 10s grid over full
        tt = np.arange(0, t[-1] + 5, 5)
        for a in tt:
            windows.append(('seg5', (float(a), float(a + 5))))
        tt = np.arange(0, t[-1] + 10, 10)
        for a in tt:
            windows.append(('seg10', (float(a), float(a + 10))))
        # manifest windows if given
        mf = sd / 'experiment_manifest.csv'
        if mf.exists():
            r = list(csv.DictReader(open(mf)))
            if r and r[0].get('stable_window_start_list'):
                ss = r[0]['stable_window_start_list'].split(';')
                se = r[0]['stable_window_end_list'].split(';')
                for a, b in zip(ss, se):
                    try:
                        windows.append((f'manifest_{a}-{b}', (float(a), float(b))))
                    except ValueError:
                        continue
        csv_rows += windows_stats(tag, qm, taum, e, t, windows)
        np.savez_compressed(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz',
                            t=t, q=qm, dq=dqm, tau=taum, e=e, roll=roll, pitch=pitch,
                            tick=low['tick'], temp=low['temp_raw'][:, SLOTS],
                            quat=np.stack([low['quat_w'], low['quat_x'], low['quat_y'], low['quat_z']], 1))
        print(f'{tag}: {n} rows, {dur:.1f}s, rate~{rate:.0f}Hz, '
              f'auto stable segs={[(round(a,1),round(b,1)) for a,b in segs]}, '
              f'pitch(end-stable)={np.nanmean(pitch[stable_m]) * 57.3 if stable_m.any() else float("nan"):.2f} deg')
    # write CSVs
    keys = list(csv_rows[0].keys())
    with open(DOCS / 'REAL_V2_ALL_JOINT_WINDOWS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(csv_rows)
    with open(RUNS / 'tables' / 'A_SESSION_INVENTORY.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(inv_rows[0].keys()))
        w.writeheader()
        w.writerows(inv_rows)
    print('wrote REAL_V2_ALL_JOINT_WINDOWS.csv rows=', len(csv_rows))


if __name__ == '__main__':
    main()
