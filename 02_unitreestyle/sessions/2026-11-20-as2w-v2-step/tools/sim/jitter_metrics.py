#!/usr/bin/env python3
"""Jitter metrics from a fixed-replay sensor tape (deployment-style signal analysis).

Computes, from what the controller actually commanded:
  * leg-target p2p (rad)
  * per-step action delta (rad, mean |target_t - target_{t-1}|)
  * sign-flip fraction (fraction of leg dims flipping sign between steps)
  * leg tracking error (|target - q|)
and the dominant frequency of the leg-target signal (DFT peak, like the field
report's 4.5 Hz observation).

Usage: jitter_metrics.py <snapshot_dir> [label]
"""
import json
import sys
from pathlib import Path

import numpy as np

LEG = 12


def metrics(tape: Path):
    d = np.load(tape, allow_pickle=False)
    tgt = d["next_target"][:, :, :LEG]          # (frames, trials, 12) offsets in rad
    q = d["q_rad"][:, :, :LEG]
    default = np.array([0.0, 0.8, -1.6] * 4)
    q_off = q - default
    frames, trials, _ = tgt.shape

    p2p = float(np.mean(tgt.max(0) - tgt.min(0)))
    delta = float(np.mean(np.abs(np.diff(tgt, axis=0))))
    prod = tgt[1:] * tgt[:-1]
    flip = float(np.mean(prod < 0))
    track = float(np.mean(np.abs(tgt - q_off)))

    # dominant frequency of the summed leg-target signal over the trial axis
    sig = tgt[:, 0, :].sum(axis=1)
    sig = sig - sig.mean()
    if sig.size > 8:
        spec = np.abs(np.fft.rfft(sig))
        freqs = np.fft.rfftfreq(sig.size, d=0.02)
        fpeak = float(freqs[1:][np.argmax(spec[1:])])
    else:
        fpeak = float("nan")

    return {
        "frames": int(frames), "trials": int(trials),
        "leg_target_p2p_rad": round(p2p, 4),
        "step_delta_rad": round(delta, 4),
        "signflip_fraction": round(flip, 4),
        "leg_tracking_error_rad": round(track, 4),
        "dominant_freq_hz": round(fpeak, 2),
    }


def main():
    snap = Path(sys.argv[1])
    label = sys.argv[2] if len(sys.argv) > 2 else snap.name
    out = {"label": label, "snapshot": str(snap)}
    tape = snap / "flat-sensor-tape.npz"
    out["metrics"] = metrics(tape) if tape.exists() else "not_run"
    gate = snap / "flat-gait-gate.json"
    if gate.exists():
        g = json.loads(gate.read_text())
        out["gate_qualified"] = g.get("qualified")
        out["gate_failures"] = g.get("failures")
    rep = snap / "stage-report.json"
    if rep.exists():
        r = json.loads(rep.read_text())
        out["modes"] = {
            m: {
                "tracking": round(r["modes"][m]["tracking_ratio_min"] or 0, 3),
                "wheels_with_2plus": r["modes"][m]["wheels_with_2plus_events"],
                "tilt_p95_deg": round(r["modes"][m]["tilt_p95_deg"], 2),
                "support": round(r["modes"][m]["four_wheel_support_min"], 3),
            } for m in ("left", "right", "yaw_left", "yaw_right")
        }
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()

