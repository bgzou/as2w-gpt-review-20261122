"""Analyse the CPRO zero-B trial logs for the v2-step jitter weight."""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120\debug_policy.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ = 149   # requested_targets (q,dq,kp,kd)x16
APP = 213   # applied_targets  (q,dq,kp,kd)x16

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
            quat = np.array([float(x) for x in p[18:22]])
            q_meas = np.array([float(x) for x in p[25:41]])
            req = np.array([float(p[REQ + 4 * i]) for i in range(16)])
            app = np.array([float(p[APP + 4 * i]) for i in range(16)])
        except (ValueError, IndexError):
            continue
        rows.append((ns, cmd, quat, q_meas, req, app))

print("phases:", phases)
print("walk rows:", len(rows))
if not rows:
    raise SystemExit(1)

t = np.array([(r[0] - rows[0][0]) / 1e9 for r in rows])
tgt = np.array([r[4][:12] - DEFAULT for r in rows])
applied = np.array([r[5][:12] - DEFAULT for r in rows])
q = np.array([r[3][:12] - DEFAULT for r in rows])
quat = np.array([r[2] for r in rows])
cmd = np.array([r[1] for r in rows])

dt = float(np.median(np.diff(t)))
p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
p2p_max = float((tgt.max(0) - tgt.min(0)).max())
delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
track = float(np.mean(np.abs(tgt - q)))
sig = tgt[:, 0] - tgt[:, 0].mean()
spec = np.abs(np.fft.rfft(sig))
freqs = np.fft.rfftfreq(sig.size, d=dt)
fpeak = float(freqs[1:][np.argmax(spec[1:])])

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

report = {
    "log": LOG.name,
    "walk_frames": len(rows),
    "duration_s": round(float(t[-1]), 3),
    "sample_dt_s": round(dt, 5),
    "command_abs_max": float(np.abs(cmd).max()),
    "leg_target_p2p_rad_mean": round(p2p, 4),
    "leg_target_p2p_rad_max": round(p2p_max, 4),
    "step_delta_rad_mean": round(delta, 4),
    "signflip_fraction": round(flip, 4),
    "leg_tracking_error_rad_mean": round(track, 4),
    "dominant_freq_hz": round(fpeak, 2),
    "pitch_deg_mean": round(float(pitch.mean()), 2),
    "pitch_deg_min": round(float(pitch.min()), 2),
    "pitch_deg_max": round(float(pitch.max()), 2),
    "roll_deg_mean": round(float(roll.mean()), 2),
    "roll_deg_min": round(float(roll.min()), 2),
    "roll_deg_max": round(float(roll.max()), 2),
}
Path(r"X:\Items\as2w\.stage58\field20261120\zero-b-analysis.json").write_text(
    json.dumps(report, indent=1) + "\n", encoding="utf-8")
print(json.dumps(report, indent=1))

print("\n--- vs 2026-11-19 field baseline (old clamp weight, 10 Hz legs) ---")
print("p2p      %.3f   vs 0.46-0.50" % p2p)
print("signflip %.4f  vs 0.30" % flip)
print("f_dom    %.2f Hz vs 4.5 Hz" % fpeak)
print("tracking %.4f  vs 0.161" % track)
print("pitch    %.2f (%.2f..%.2f) vs -0.53 (-4.91..+2.86)" % (pitch.mean(), pitch.min(), pitch.max()))
print("roll     %.2f (%.2f..%.2f)" % (roll.mean(), roll.min(), roll.max()))
