#!/usr/bin/env python3
"""Build the baseline-vs-jitter comparison table (clean and DR2 replays)."""
import json
from pathlib import Path

import numpy as np

SRC = Path("/root/gpufree-data/as2w-runs")
LEG = 12
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
RUNS = [
    ("base@DR0", SRC / "v2-stephard-eval-600"),
    ("base@DR2", SRC / "cmp2-base-dr2"),
    ("jit@DR0", SRC / "cmp-jit-dr0"),
    ("jit@DR2", SRC / "cmp2-jit-dr2"),
]
MODES = ("left", "right", "yaw_left", "yaw_right")


def tape_metrics(snap: Path):
    f = snap / "flat-sensor-tape.npz"
    if not f.exists():
        return None
    d = np.load(f, allow_pickle=False)
    tgt = d["next_target"][:, :, :LEG]
    q = d["q_rad"][:, :, :LEG] - DEFAULT
    p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
    delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
    flip = float(np.mean((tgt[1:] * tgt[:-1]) < 0))
    track = float(np.mean(np.abs(tgt - q)))
    sig = tgt[:, 0, :].sum(axis=1)
    sig = sig - sig.mean()
    spec = np.abs(np.fft.rfft(sig))
    freqs = np.fft.rfftfreq(sig.size, d=0.02)
    fpk = float(freqs[1:][np.argmax(spec[1:])]) if sig.size > 8 else float("nan")
    return {
        "leg_target_p2p_rad": round(p2p, 4),
        "step_delta_rad": round(delta, 4),
        "signflip_fraction": round(flip, 4),
        "leg_tracking_error_rad": round(track, 4),
        "dominant_freq_hz": round(fpk, 2),
    }


out = {}
for label, snap in RUNS:
    rep = json.loads((snap / "stage-report.json").read_text())
    gate = json.loads((snap / "flat-gait-gate.json").read_text())
    entry = {
        "snapshot": snap.name,
        "gate_qualified": gate.get("qualified"),
        "gate_failures": gate.get("failures"),
        "tape": tape_metrics(snap),
        "modes": {},
    }
    for m in MODES:
        x = rep["modes"][m]
        entry["modes"][m] = {
            "tracking": round(x["tracking_ratio_min"] or 0, 3),
            "wheels_2plus": x["wheels_with_2plus_events"],
            "tilt_p95": round(x["tilt_p95_deg"], 2),
            "support_min": round(x["four_wheel_support_min"], 3),
            "falls": x.get("falls"),
        }
    out[label] = entry

(SRC / "jitter-compare.json").write_text(json.dumps(out, indent=1) + "\n")

# markdown
lines = ["| run | gate | left | yaw_left | yaw_right | right | flip | delta | p2p | f_peak | track err |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
for label, e in out.items():
    t = e["tape"] or {}
    m = e["modes"]
    lines.append(
        f"| {label} | {'PASS' if e['gate_qualified'] else 'FAIL'} | "
        + " | ".join(f"{m[k]['wheels_2plus']}/4 t{m[k]['tracking']:.2f}" for k in ("left", "yaw_left", "yaw_right", "right"))
        + f" | {t.get('signflip_fraction','-')} | {t.get('step_delta_rad','-')} | {t.get('leg_target_p2p_rad','-')} | "
          f"{t.get('dominant_freq_hz','-')} | {t.get('leg_tracking_error_rad','-')} |")
md = "\n".join(lines) + "\n"
(SRC / "jitter-compare.md").write_text(md)
print(md)
