"""Compare yaw/lateral response between the 0.5-trained and mu025-trained weights
at matched command amplitude, using the two CPRO joystick logs."""
import json
from pathlib import Path

import numpy as np

LOGS = {
    "jitter(0.5)": Path(r"X:\Items\as2w\.stage58\field20261120c.csv"),
    "mu025": Path(r"X:\Items\as2w\.stage58\field-mu025\lateral2.csv"),
}


def load(path):
    rows = []
    for line in path.open("r", encoding="utf-8", errors="replace"):
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        if p[2].upper() != "WALK":
            continue
        try:
            rows.append((np.array([float(x) for x in p[5:8]]),
                         np.array([float(x) for x in p[15:18]]),
                         np.array([float(x) for x in p[18:22]]),
                         np.array([float(x) for x in p[22:25]])))
        except (ValueError, IndexError):
            continue
    return rows


def R_of(qq):
    w, x, y, z = qq
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


out = {}
for label, path in LOGS.items():
    rows = load(path)
    if not rows:
        out[label] = "no rows"
        continue
    cmd = np.array([r[0] for r in rows])
    gyro = np.array([r[1] for r in rows])
    quat = np.array([r[2] for r in rows])
    acc = np.array([r[3] for r in rows])

    # body-frame lateral acceleration with gravity removed, projected on LEFT axis
    lat = np.zeros(len(rows))
    for i in range(len(rows)):
        Rw = R_of(quat[i])
        a_w = Rw @ acc[i]
        a_w[2] -= 9.81
        left = Rw @ np.array([0.0, 1.0, 0.0])
        lat[i] = float(a_w @ left)

    res = {"frames": len(rows), "cmd_max": [round(float(np.abs(cmd[:, i]).max()), 3) for i in range(3)]}
    for lo, hi, tag in ((0.02, 0.15, "wz 0.02-0.15"), (0.15, 0.99, "wz 0.15-0.3")):
        m = (np.abs(cmd[:, 2]) >= lo) & (np.abs(cmd[:, 2]) < hi)
        if m.sum() >= 10:
            res[tag] = {
                "frames": int(m.sum()),
                "cmd_mean": round(float(np.mean(cmd[m, 2])), 3),
                "meas_mean": round(float(np.mean(gyro[m, 2])), 3),
                "ratio": round(float(np.mean(gyro[m, 2] / cmd[m, 2])), 3),
            }
    for lo, tag in ((0.05, "vy>0.05"), (-0.99, "vy<-0.05")):
        m = (cmd[:, 1] > lo) if lo > 0 else (cmd[:, 1] < -0.05)
        if m.sum() >= 10:
            res[tag] = {
                "frames": int(m.sum()),
                "cmd_mean": round(float(np.mean(cmd[m, 1])), 3),
                "lateral_accel_mean": round(float(np.mean(lat[m])), 4),
                "ratio": round(float(np.mean(lat[m] / cmd[m, 1])), 3),
            }
    out[label] = res

print(json.dumps(out, indent=1))
Path(r"X:\Items\as2w\.stage58\field-mu025\yaw-lateral-compare.json").write_text(
    json.dumps(out, indent=1) + "\n", encoding="utf-8")
