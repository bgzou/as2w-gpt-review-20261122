#!/usr/bin/env python3
"""Phase H3: torque time-series replay on the C0 delivery candidate (MuJoCo).

Modes (P55G2 window 8-19 s real, 500 Hz):
  A  closed PD  kp=55/kd=2, T2 frozen, wheels -kd*dq (contract baseline)
  B  closed PD  with contract read back from debug_lowcmd FixedStand rows
                (H1.3 proved std=0 => B expected == A; validates read pipeline)
  C  open-loop  legs tau = interp(real tau_est(t))   [sensitivity only, H1.7]
  D  open-loop  legs tau = kp*(tgt - q_real(t)) + kd*(0 - dq_real(t))
Stages: S = suspended fixed base (M_cylwheel_susp), G = ground free base
        (M_cylwheel, init wheel clearance 2 mm from real initial pose).
Metrics: settled-window means vs real window (A/B, closed); per-frame
         q tracking RMS/drift vs real (C/D, open-loop).
Run: /home/robot/z00959834  -- use venv312 or conda python with mujoco.
"""
import csv
import json
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'mujoco_abl'
LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHEELS = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
T2 = np.array([0.]*4 + [0.8]*4 + [-1.6]*4)
# stage -> (reference session, window): suspended stage replays the suspended
# session (like-for-like), ground stage replays the ground session.
STAGE_REF = {'S': ('P55S', 25., 90.), 'G': ('P55G2', 8., 19.)}
DT = 0.002
DUR = 65.  # longest window; each run uses its own reference window length


def load_model(stage):
    sub = 'M_cylwheel_susp' if stage == 'S' else 'M_cylwheel'
    m = mujoco.MjModel.from_xml_path(str(ABL / sub / 'as2w_scene.xml'))
    m.actuator_gainprm[:, :] = 0.0   # neutralize factory position/velocity
    m.actuator_biasprm[:, :] = 0.0   # actuators (ctrl=0 would pull to q=0)
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHEELS}
    qa = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHEELS}
    va = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHEELS}
    return m, d, qa, va


def set_pose(m, d, qa, stage, q0, quat0):
    d.qpos[:] = 0
    d.qvel[:] = 0
    if stage == 'G':
        d.qpos[0:4] = quat0  # wxyz
        # base xy zero; find wheel clearance with base z=0 then lift
        for n in LEGS:
            d.qpos[qa[n]] = q0[LEGS.index(n)]
        mujoco.mj_forward(m, d)
        foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_foot'),
                    mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_foot'),
                    mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_foot'),
                    mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_foot')]
        minz = min(d.xpos[b, 2] for b in foot_ids) - 0.086
        d.qpos[2] = -minz + 0.002  # lowest wheel point 2 mm off ground
    else:
        d.qpos[2] = 0.5
        for n in LEGS:
            d.qpos[qa[n]] = q0[LEGS.index(n)]
    mujoco.mj_forward(m, d)


def ctrl_pd(d, qa, va, kp, kd, tgt):
    tau = np.zeros(12)
    for i, n in enumerate(LEGS):
        tau[i] = kp * (tgt[i] - d.qpos[qa[n]]) + kd * (0.0 - d.qvel[va[n]])
    wtau = np.zeros(4)
    for i, n in enumerate(WHEELS):
        wtau[i] = kd * (0.0 - d.qvel[va[n]])
    return tau, wtau


def run(stage, mode, real, nsteps, verbose=True):
    m, d, qa, va = load_model(stage)
    t_r, q_r, dq_r, tau_r, quat0 = real
    q0 = q_r[0]
    set_pose(m, d, qa, stage, q0, quat0)
    kp, kd = 55., 2.
    hist_q = np.zeros((nsteps, 12)); hist_tau = np.zeros((nsteps, 12))
    for k in range(nsteps):
        tau_real_t = tau_r[k]; q_real_t = q_r[k]; dq_real_t = dq_r[k]
        if mode in ('A', 'B'):
            tau, wtau = ctrl_pd(d, qa, va, kp, kd, T2)
        elif mode == 'C':
            tau = tau_real_t.copy()
            wtau = np.array([kd * (0.0 - d.qvel[va[n]]) for n in WHEELS])
        elif mode == 'D':
            tau = kp * (T2 - q_real_t) + kd * (0.0 - dq_real_t)
            wtau = np.array([kd * (0.0 - d.qvel[va[n]]) for n in WHEELS])
        else:
            raise ValueError(mode)
        d.qfrc_applied[:] = 0
        for i, n in enumerate(LEGS):
            d.qfrc_applied[va[n]] = tau[i]
        for i, n in enumerate(WHEELS):
            d.qfrc_applied[va[n]] = wtau[i]
        hist_q[k] = [d.qpos[qa[n]] for n in LEGS]
        hist_tau[k] = tau
        mujoco.mj_step(m, d)
    return hist_q, hist_tau


def resample(tag, win):
    z = np.load(RUNS / 'qc' / 'a1_timelines' / f'{tag}.npz')
    t, q, dq, tau, quat = z['t'], z['q'], z['dq'], z['tau'], z['quat']
    msk = (t >= win[0]) & (t < win[1])
    n = int(round((win[1] - win[0]) / DT))
    tt = np.linspace(win[0], win[1] - DT, n)
    q_r = np.stack([np.interp(tt, t[msk], q[msk, j]) for j in range(12)], 1)
    dq_r = np.stack([np.interp(tt, t[msk], dq[msk, j]) for j in range(12)], 1)
    tau_r = np.stack([np.interp(tt, t[msk], tau[msk, j]) for j in range(12)], 1)
    return (tt - win[0], q_r, dq_r, tau_r, quat[msk][0]), n


def main():
    rows = []
    for stage, mode in [('S', 'A'), ('S', 'C'), ('S', 'D'),
                        ('G', 'A'), ('G', 'B'), ('G', 'C'), ('G', 'D')]:
        tag, w0, w1 = STAGE_REF[stage]
        real, nsteps = resample(tag, (w0, w1))
        t_r, q_r, dq_r, tau_r, quat0 = real
        qmean_real, taumean_real = q_r.mean(0), tau_r.mean(0)
        hist_q, hist_tau = run(stage, mode, real, nsteps)
        if mode in ('A', 'B'):
            sl = slice(int(5 / DT), None)  # settled last 5 s
            qm, tm = hist_q[sl].mean(0), hist_tau[sl].mean(0)
            rows.append(dict(stage=stage, mode=mode, ref=tag, kind='closed-settled',
                             q_err_rms=float(np.sqrt(((qm - qmean_real) ** 2).mean())),
                             q_err_max=float(np.abs(qm - qmean_real).max()),
                             tau_err_rms=float(np.sqrt(((tm - taumean_real) ** 2).mean())),
                             tau_err_max=float(np.abs(tm - taumean_real).max()),
                             q_drift_end=float(np.abs(hist_q[-1] - q_r[-1]).max()),
                             pitch_deg_end=float('nan')))
        else:
            e = hist_q - q_r
            rows.append(dict(stage=stage, mode=mode, ref=tag, kind='open-track',
                             q_err_rms=float(np.sqrt((e ** 2).mean())),
                             q_err_max=float(np.abs(e).max()),
                             tau_err_rms=float('nan'), tau_err_max=float('nan'),
                             q_drift_end=float(np.abs(hist_q[-1] - q_r[-1]).max()),
                             pitch_deg_end=float('nan')))
        r = rows[-1]
        for j, jn in enumerate(LEGS[:4]):
            if mode in ('A', 'B'):
                sl = slice(int(5 / DT), None)
                rows.append(dict(stage=stage, mode=mode, ref=tag, kind=f'hip:{jn}',
                                 q_err_rms=float(hist_q[sl, j].mean() - qmean_real[j]),
                                 q_err_max=float(abs(hist_tau[sl, j].mean() - taumean_real[j])),
                                 tau_err_rms=float('nan'), tau_err_max=float('nan'),
                                 q_drift_end=float(hist_q[-1, j] - q_r[-1, j]),
                                 pitch_deg_end=float('nan')))
            else:
                rows.append(dict(stage=stage, mode=mode, ref=tag, kind=f'hip:{jn}',
                                 q_err_rms=float(np.sqrt(((hist_q[:, j] - q_r[:, j]) ** 2).mean())),
                                 q_err_max=float(np.abs(hist_q[:, j] - q_r[:, j]).max()),
                                 tau_err_rms=float('nan'), tau_err_max=float('nan'),
                                 q_drift_end=float(hist_q[-1, j] - q_r[-1, j]),
                                 pitch_deg_end=float('nan')))
        print(f"{stage}-{mode} (ref {tag}): {r['kind']} qErrRMS={r['q_err_rms']:.4f} "
              f"qErrMax={r['q_err_max']:.4f} tauErrRMS={r['tau_err_rms']:.4f} drift={r['q_drift_end']:.4f}")
    with open(RUNS / 'tables' / 'H3_TORQUE_REPLAY.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    with open(RUNS / 'tables' / 'H3_TORQUE_REPLAY.json', 'w') as f:
        json.dump(rows, f, indent=1)
    print('wrote H3_TORQUE_REPLAY.{csv,json}')


if __name__ == '__main__':
    main()
