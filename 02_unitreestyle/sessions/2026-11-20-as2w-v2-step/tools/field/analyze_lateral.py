"""Analyse the CPRO lateral/yaw trial (v2-step jitter weight, sticks live)."""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120b\debug_policy.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ, APP = 149, 213

phases = {}
rows = []
with LOG.open("r", encoding="utf-8", errors="replace") as f:
    for line in f:
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        phase = p[2]
        phases[phase] = phases.get(phase, 0) + 1
        if phase.upper() != "WALK":
            continue
        try:
            ns = float(p[1])
            cmd = np.array([float(x) for x in p[5:8]])
            gyro = np.array([float(x) for x in p[15:18]])
            quat = np.array([float(x) for x in p[18:22]])
            q_meas = np.array([float(x) for x in p[25:41]])
            req = np.array([float(p[REQ + 4 * i]) for i in range(16)])
        except (ValueError, IndexError):
            continue
        rows.append((ns, cmd, gyro, quat, q_meas, req))

print("phases:", phases)
print("walk rows:", len(rows))
if not rows:
    raise SystemExit(1)

t = np.array([(r[0] - rows[0][0]) / 1e9 for r in rows])
cmd = np.array([r[1] for r in rows])
gyro = np.array([r[2] for r in rows])
quat = np.array([r[3] for r in rows])
q = np.array([r[4][:12] - DEFAULT for r in rows])
tgt = np.array([r[5][:12] - DEFAULT for r in rows])

dt = float(np.median(np.diff(t)))
p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
track = float(np.mean(np.abs(tgt - q)))

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

# yaw tracking: commanded wz vs measured body yaw rate
sec = cmd[:, 2]
active = np.abs(sec) > 0.02
yaw_ratio = float(np.mean(gyro[active, 2] / sec[active])) if active.any() else float("nan")
yaw_mean_cmd = float(np.mean(sec[active])) if active.any() else 0.0
yaw_mean_meas = float(np.mean(gyro[active, 2])) if active.any() else 0.0

report = {
    "log": LOG.name,
    "walk_frames": len(rows),
    "duration_s": round(float(t[-1]), 3),
    "cmd_max_abs": [round(float(np.abs(cmd[:, i]).max()), 4) for i in range(3)],
    "cmd_active_fraction": {
        "vx": float(np.mean(np.abs(cmd[:, 0]) > 0.02)),
        "vy": float(np.mean(np.abs(cmd[:, 1]) > 0.02)),
        "wz": float(np.mean(np.abs(cmd[:, 2]) > 0.02)),
    },
    "yaw_cmd_mean_active": round(yaw_mean_cmd, 4),
    "yaw_meas_mean_active": round(yaw_mean_meas, 4),
    "yaw_tracking_ratio": round(yaw_ratio, 3),
    "leg_target_p2p_rad_mean": round(p2p, 4),
    "step_delta_rad_mean": round(delta, 4),
    "signflip_fraction": round(flip, 4),
    "leg_tracking_error_rad_mean": round(track, 4),
    "pitch_deg_mean": round(float(pitch.mean()), 2),
    "pitch_deg_range": [round(float(pitch.min()), 2), round(float(pitch.max()), 2)],
    "roll_deg_mean": round(float(roll.mean()), 2),
    "roll_deg_range": [round(float(roll.min()), 2), round(float(roll.max()), 2)],
}
Path(r"X:\Items\as2w\.stage58\field20261120b\trial-analysis.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
