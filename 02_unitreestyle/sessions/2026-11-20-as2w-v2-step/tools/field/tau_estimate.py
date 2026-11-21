"""Estimate the PD torque the real robot had to produce during the zero-B stand.

tau_i = kp*(q_target - q_measured) + kd*(dq_target - dq_measured)
      = 55*err + 2*(0 - dq)          (legs, position mode)

Compare with the T2 simulation numbers from the 2026-11-19/20 field calibration
(front hip < 0.7 Nm in Isaac; 11-12 Nm measured on the hardware).
"""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120\debug_policy.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
LEGS = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"]
REQ = 149
KP, KD = 55.0, 2.0

rows = []
with LOG.open("r", encoding="utf-8", errors="replace") as f:
    for line in f:
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        if p[2].upper() != "WALK":
            continue
        try:
            q = np.array([float(x) for x in p[25:41]])
            dq = np.array([float(x) for x in p[41:57]])
            req = np.array([float(p[REQ + 4 * i]) for i in range(16)])
        except (ValueError, IndexError):
            continue
        rows.append((q, dq, req))

q = np.array([r[0] for r in rows])
dq = np.array([r[1] for r in rows])
tgt = np.array([r[2] for r in rows])

tau = KP * (tgt[:, :12] - q[:, :12]) + KD * (0.0 - dq[:, :12])
mean_tau = tau.mean(0)
abs_mean = np.abs(tau).mean(0)

report = {
    "frames": len(rows),
    "leg_tracking_error_rad_mean": round(float(np.abs(tgt[:, :12] - q[:, :12]).mean()), 4),
    "leg_dq_rad_s_mean_abs": round(float(np.abs(dq[:, :12]).mean()), 4),
    "estimate": "tau = 55*(q_tgt-q) + 2*(0-dq)  [Nm]",
    "per_joint_mean_tau_nm": {k: round(float(v), 2) for k, v in zip(LEGS, mean_tau)},
    "per_joint_mean_abs_tau_nm": {k: round(float(v), 2) for k, v in zip(LEGS, abs_mean)},
    "hip_abs_tau_mean_nm": {k: round(float(v), 2) for k, v in zip(LEGS, abs_mean) if "hip" in k},
    "reference": {
        "isaac_front_hip_nm": "<0.7 (2026-11-19 T2 static comparison)",
        "hardware_measured_front_hip_nm": "11-12 (same session)",
        "note": "this estimate is PD-only; the real hardware adds friction/gearbox losses",
    },
}
Path(r"X:\Items\as2w\.stage58\field20261120\torque-estimate.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
