"""Signed regression slopes: measured yaw rate vs commanded wz, and body-left
acceleration vs commanded vy. Robust to mixed command signs."""
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


def slope(cmd, meas):
    denom = float(np.sum(cmd * cmd))
    return float(np.sum(cmd * meas) / denom) if denom > 0 else float("nan")


out = {}
for label, path in LOGS.items():
    rows = load(path)
    cmd = np.array([r[0] for r in rows])
    gyro = np.array([r[1] for r in rows])
    quat = np.array([r[2] for r in rows])
    acc = np.array([r[3] for r in rows])
    lat = np.zeros(len(rows))
    for i in range(len(rows)):
        Rw = R_of(quat[i])
        a_w = Rw @ acc[i]
        a_w[2] -= 9.81
        left = Rw @ np.array([0.0, 1.0, 0.0])
        lat[i] = float(a_w @ left)

    wz, vy = cmd[:, 2], cmd[:, 1]
    mw = np.abs(wz) > 0.02
    mv = np.abs(vy) > 0.02
    out[label] = {
        "walk_frames": len(rows),
        "wz_active_frames": int(mw.sum()),
        "wz_cmd_abs_mean": round(float(np.abs(wz[mw]).mean()), 3) if mw.any() else None,
        "yaw_slope": round(slope(wz[mw], gyro[mw, 2]), 3) if mw.any() else None,
        "vy_active_frames": int(mv.sum()),
        "vy_cmd_abs_mean": round(float(np.abs(vy[mv]).mean()), 3) if mv.any() else None,
        "lateral_slope": round(slope(vy[mv], lat[mv]), 3) if mv.any() else None,
    }
print(json.dumps(out, indent=1))
Path(r"X:\Items\as2w\.stage58\field-mu025\yaw-lateral-slopes.json").write_text(
    json.dumps(out, indent=1) + "\n", encoding="utf-8")
