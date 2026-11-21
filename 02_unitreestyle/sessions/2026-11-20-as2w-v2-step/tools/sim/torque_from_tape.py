"""Compute the simulated PD torque demand from a sensor tape.

tau_i = 55*(q_target_i - q_i) + 2*(0 - dq_i)   for the 12 leg axes
(same formula used for the hardware estimate, so the numbers are comparable).

Usage: torque_from_tape.py <tape.npz> <label>
"""
import json
import sys
from pathlib import Path

import numpy as np

LEGS = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"]
KP, KD = 55.0, 2.0

tape = Path(sys.argv[1])
label = sys.argv[2] if len(sys.argv) > 2 else tape.stem
d = np.load(tape, allow_pickle=False)
q = d["q_rad"][:, :, :12]
dq = d["dq_rad_s"][:, :, :12]
leg_q_tgt = d["leg_q_target"]          # absolute target, (frames, trials, 12)

err = leg_q_tgt - q
tau = KP * err + KD * (0.0 - dq)

# zero-command frames only would be ideal; the tape mixes scenarios, so report
# both the overall mean and the frames where the command is near zero.
cmd = d["command_si"]
rest = np.abs(cmd).sum(-1) < 0.01
tau_rest = tau[rest] if rest.any() else tau
q_rest = leg_q_tgt[rest] if rest.any() else leg_q_tgt
q_meas_rest = q[rest] if rest.any() else q

report = {
    "label": label,
    "tape": tape.name,
    "frames": int(q.shape[0]),
    "trials": int(q.shape[1]),
    "rest_frames": int(rest.sum()) if rest.any() else 0,
    "tracking_err_mean_rest_rad": round(float(np.abs(q_rest - q_meas_rest).mean()), 4),
    "mean_abs_tau_rest_nm": round(float(np.abs(tau_rest).mean()), 3),
    "per_joint_mean_abs_tau_rest_nm": {
        k: round(float(v), 2) for k, v in zip(LEGS, np.abs(tau_rest).reshape(-1, 12).mean(0))
    },
    "hip_abs_tau_rest_nm": {
        k: round(float(v), 2)
        for k, v in zip(LEGS, np.abs(tau_rest).reshape(-1, 12).mean(0)) if "hip" in k
    },
    "overall_mean_abs_tau_nm": round(float(np.abs(tau).mean()), 3),
}
print(json.dumps(report, indent=1))
out = tape.with_suffix(".torque.json")
out.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
print("wrote", out)
