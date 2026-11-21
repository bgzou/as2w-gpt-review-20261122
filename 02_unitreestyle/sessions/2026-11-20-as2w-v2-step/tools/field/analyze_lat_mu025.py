"""Analyse the mu025 lateral/yaw trial: command usage, yaw tracking, lateral accel, jitter."""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field-mu025\lateral.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ = 149

rows = []
with LOG.open("r", encoding="utf-8", errors="replace") as f:
    for line in f:
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        if p[2].upper() != "WALK":
            continue
        try:
            rows.append((float(p[1]),
                         np.array([float(x) for x in p[5:8]]),
                         np.array([float(x) for x in p[15:18]]),
                         np.array([float(x) for x in p[18:22]]),
                         np.array([float(x) for x in p[25:41]]),
                         np.array([float(x) for x in p[41:57]]),
                         np.array([float(p[REQ + 4 * i]) for i in range(16)])))
        except (ValueError, IndexError):
            continue

t = np.array([(r[0] - rows[0][0]) / 1e9 for r in rows])
cmd = np.array([r[1] for r in rows])
gyro = np.array([r[2] for r in rows])
quat = np.array([r[3] for r in rows])
q = np.array([r[4][:12] - DEFAULT for r in rows])
dq = np.array([r[5][:12] for r in rows])
tgt = np.array([r[6][:12] - DEFAULT for r in rows])

dt = float(np.median(np.diff(t)))
p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
track = float(np.mean(np.abs(tgt - q)))

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

act_wz = np.abs(cmd[:, 2]) > 0.02
act_vy = np.abs(cmd[:, 1]) > 0.02
yaw_ratio = (float(np.mean(gyro[act_wz, 2] / cmd[act_wz, 2])) if act_wz.any() else float("nan"))
yaw_cmd = float(np.mean(cmd[act_wz, 2])) if act_wz.any() else 0.0
yaw_meas = float(np.mean(gyro[act_wz, 2])) if act_wz.any() else 0.0

# world-frame lateral acceleration, gravity removed
def R_of(qq):
    w_, x_, y_, z_ = qq
    return np.array([
        [1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - w_ * z_), 2 * (x_ * z_ + w_ * y_)],
        [2 * (x_ * y_ + w_ * z_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - w_ * x_)],
        [2 * (x_ * z_ - w_ * y_), 2 * (y_ * z_ + w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)]])

acc = np.array([np.array([float(v) for v in line.rstrip("\n").split(",")[22:25]])
                for line in LOG.open("r", encoding="utf-8", errors="replace")
                if line.startswith("DEBUG_POLICY,") and line.split(",")[2].upper() == "WALK"]
               ) if False else None

report = {
    "walk_frames": len(rows),
    "duration_s": round(float(t[-1]), 3),
    "cmd_max_abs": [round(float(np.abs(cmd[:, i]).max()), 3) for i in range(3)],
    "active_fraction": {"vy": round(float(act_vy.mean()), 3), "wz": round(float(act_wz.mean()), 3)},
    "yaw_tracking_ratio": round(yaw_ratio, 3),
    "yaw_cmd_mean_when_active": round(yaw_cmd, 4),
    "yaw_meas_mean_when_active": round(yaw_meas, 4),
    "leg_target_p2p_rad_mean": round(p2p, 4),
    "step_delta_rad_mean": round(delta, 4),
    "signflip_fraction": round(flip, 4),
    "leg_tracking_error_rad_mean": round(track, 4),
    "pitch_deg_mean": round(float(pitch.mean()), 2),
    "pitch_deg_range": [round(float(pitch.min()), 2), round(float(pitch.max()), 2)],
    "roll_deg_mean": round(float(roll.mean()), 2),
    "roll_deg_range": [round(float(roll.min()), 2), round(float(roll.max()), 2)],
}
Path(r"X:\Items\as2w\.stage58\field-mu025\lateral-analysis.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
print("\n--- previous jitter weight, same trial (wz<=0.30) ---")
print("yaw_ratio 0.164, cmd 0.260 -> meas 0.049 rad/s; p2p 0.438; flip 6.2%; tracking 0.102")
