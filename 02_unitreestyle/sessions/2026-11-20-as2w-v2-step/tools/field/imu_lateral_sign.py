"""Lateral-direction evidence from IMU: does cmd_vy>0 (policy 'left') move the
body toward its own +y (left) or -y (right)?

Method
  * body-frame accelerometer includes gravity -> rotate to world, subtract g.
  * project world acceleration onto the body's LEFT axis (R * [0,1,0]).
  * compare the mean lateral acceleration while the vy command is active with
    the mean while it is zero (that difference cancels any sensor bias/lean).
  * integrate the bias-corrected lateral acceleration over the active segment
    to get the velocity change (m/s).
"""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120c.csv")

rows = []
with LOG.open("r", encoding="utf-8", errors="replace") as f:
    for line in f:
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        if p[2].upper() != "WALK":
            continue
        try:
            cmd = np.array([float(x) for x in p[5:8]])
            quat = np.array([float(x) for x in p[18:22]])   # wxyz
            acc = np.array([float(x) for x in p[22:25]])    # body frame, incl. g
        except (ValueError, IndexError):
            continue
        rows.append((cmd, quat, acc))

cmd = np.array([r[0] for r in rows])
quat = np.array([r[1] for r in rows])
acc = np.array([r[2] for r in rows])
n = len(rows)


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# world-frame acceleration with gravity removed (assumes accel includes +g on z)
lat = np.zeros(n)   # acceleration along the body's LEFT axis, world-rotated
for i in range(n):
    R = quat_to_rot(quat[i])
    a_world = R @ acc[i]
    a_world[2] -= 9.81
    left_world = R @ np.array([0.0, 1.0, 0.0])
    lat[i] = float(a_world @ left_world)

act = np.abs(cmd[:, 1]) > 0.02
rest = np.abs(cmd[:, 1]) <= 0.005
mean_act = float(lat[act].mean()) if act.any() else float("nan")
mean_rest = float(lat[rest].mean()) if rest.any() else float("nan")
dt = 0.02
delta_v = (mean_act - mean_rest) * dt * int(act.sum())

# sign of the vertical check (sanity: world z accel should be ~0 after removing g)
report = {
    "frames": n,
    "vy_active_frames": int(act.sum()),
    "vy_rest_frames": int(rest.sum()),
    "mean_lateral_accel_active_m_s2": round(mean_act, 4),
    "mean_lateral_accel_rest_m_s2": round(mean_rest, 4),
    "bias_corrected_lateral_accel_m_s2": round(mean_act - mean_rest, 4),
    "integrated_delta_v_left_m_s": round(delta_v, 3),
    "cmd_vy_mean_active": round(float(cmd[act, 1].mean()), 3) if act.any() else None,
    "interpretation": (
        "cmd_vy > 0 asks the policy for body +y = LEFT. "
        "If the bias-corrected lateral acceleration (and the integrated delta-v) "
        "is NEGATIVE the body actually accelerates to the RIGHT -> the vy command "
        "sign is inverted relative to the trained frame."
    ),
}
Path(r"X:\Items\as2w\.stage58\field20261120c-lateral-imu.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
