"""Analyse the raised-envelope joystick trial (vy<=0.20, wz<=0.30)."""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120c.csv")
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
            rows.append((
                float(p[1]),
                np.array([float(x) for x in p[5:8]]),    # cmd
                np.array([float(x) for x in p[15:18]]),  # gyro
                np.array([float(x) for x in p[22:25]]),  # accel
                np.array([float(x) for x in p[18:22]]),  # quat
                np.array([float(p[REQ + 4 * i]) for i in range(16)]),
                np.array([float(x) for x in p[25:41]]),
            ))
        except (ValueError, IndexError):
            continue

t = np.array([(r[0] - rows[0][0]) / 1e9 for r in rows])
cmd = np.array([r[1] for r in rows])
gyro = np.array([r[2] for r in rows])
acc = np.array([r[3] for r in rows])
quat = np.array([r[4] for r in rows])
tgt = np.array([r[5][:12] - DEFAULT for r in rows])
q = np.array([r[6][:12] - DEFAULT for r in rows])

dt = float(np.median(np.diff(t)))
p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
track = float(np.mean(np.abs(tgt - q)))

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

# segment split: where are the commands?
act_wz = np.abs(cmd[:, 2]) > 0.02
act_vy = np.abs(cmd[:, 1]) > 0.02


def ratio(cmd_v, meas_v, mask):
    if not mask.any():
        return float("nan"), 0.0, 0.0
    return (float(np.mean(meas_v[mask] / cmd_v[mask])),
            float(np.mean(cmd_v[mask])), float(np.mean(meas_v[mask])))


yaw_ratio, yaw_cmd, yaw_meas = ratio(cmd[:, 2], gyro[:, 2], act_wz)
# lateral: body-frame accel y (positive = left?) as a motion proxy
lat_ratio, lat_cmd, lat_acc = ratio(cmd[:, 1], acc[:, 1], act_vy)

report = {
    "walk_frames": len(rows),
    "duration_s": round(float(t[-1]), 3),
    "cmd_max_abs": [round(float(np.abs(cmd[:, i]).max()), 4) for i in range(3)],
    "active_fraction": {"vy": float(act_vy.mean()), "wz": float(act_wz.mean())},
    "yaw_tracking_ratio": round(yaw_ratio, 3),
    "yaw_cmd_mean_when_active": round(yaw_cmd, 4),
    "yaw_meas_mean_when_active": round(yaw_meas, 4),
    "lateral_accel_ratio_per_cmd": round(lat_ratio, 3),
    "lateral_cmd_mean_when_active": round(lat_cmd, 4),
    "lateral_accel_mean_when_active": round(lat_acc, 4),
    "leg_target_p2p_rad_mean": round(p2p, 4),
    "step_delta_rad_mean": round(delta, 4),
    "signflip_fraction": round(flip, 4),
    "leg_tracking_error_rad_mean": round(track, 4),
    "pitch_deg_mean": round(float(pitch.mean()), 2),
    "pitch_deg_range": [round(float(pitch.min()), 2), round(float(pitch.max()), 2)],
    "roll_deg_mean": round(float(roll.mean()), 2),
    "roll_deg_range": [round(float(roll.min()), 2), round(float(roll.max()), 2)],
}
Path(r"X:\Items\as2w\.stage58\field20261120c-analysis.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
