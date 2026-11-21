"""Analyse the mu025 zero-command trial (same pipeline as the jitter-weight run)."""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field-mu025\zerob.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ = 149

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
            rows.append((float(p[1]),
                         np.array([float(x) for x in p[5:8]]),
                         np.array([float(x) for x in p[18:22]]),
                         np.array([float(x) for x in p[25:41]]),
                         np.array([float(x) for x in p[41:57]]),
                         np.array([float(p[REQ + 4 * i]) for i in range(16)])))
        except (ValueError, IndexError):
            continue

t = np.array([(r[0] - rows[0][0]) / 1e9 for r in rows])
cmd = np.array([r[1] for r in rows])
quat = np.array([r[2] for r in rows])
q = np.array([r[3][:12] - DEFAULT for r in rows])
dq = np.array([r[4][:12] for r in rows])
tgt = np.array([r[5][:12] - DEFAULT for r in rows])

dt = float(np.median(np.diff(t)))
p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
track = float(np.mean(np.abs(tgt - q)))

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

tau = 55.0 * (tgt - q) + 2.0 * (0.0 - dq)
LEGS = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf"]

report = {
    "log": LOG.name,
    "phases": phases,
    "walk_frames": len(rows),
    "duration_s": round(float(t[-1]), 3),
    "command_abs_max": float(np.abs(cmd).max()),
    "leg_target_p2p_rad_mean": round(p2p, 4),
    "step_delta_rad_mean": round(delta, 4),
    "signflip_fraction": round(flip, 4),
    "leg_tracking_error_rad_mean": round(track, 4),
    "pitch_deg_mean": round(float(pitch.mean()), 2),
    "pitch_deg_range": [round(float(pitch.min()), 2), round(float(pitch.max()), 2)],
    "roll_deg_mean": round(float(roll.mean()), 2),
    "roll_deg_range": [round(float(roll.min()), 2), round(float(roll.max()), 2)],
    "hip_abs_tau_nm": {k: round(float(v), 2) for k, v in zip(LEGS, np.abs(tau).mean(0)) if "hip" in k},
    "all_abs_tau_nm": round(float(np.abs(tau).mean()), 2),
}
Path(r"X:\Items\as2w\.stage58\field-mu025\zerob-analysis.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))
print("\n--- reference: jitter weight (0.5 asset) zero-B ---")
print("p2p 0.360 / flip 8.2% / tracking 0.101 / pitch -0.51 (-3.41..+0.74) / roll -2.49 (-4.40..-0.03)")
print("hip |tau|: FL 6.19 FR 3.98 RL 4.18 RR 5.91")
