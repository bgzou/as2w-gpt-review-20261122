#!/usr/bin/env python3
"""Rank screened checkpoints by worst-case margin against the gait gate.

For vx modes the gate needs four-wheel support >= 0.90; for vy/wz modes each
wheel needs >= 2 valid lift events per trial (min across trials and wheels).
"""
import json, sys
from pathlib import Path

base = Path("/root/gpufree-data/as2w-runs")
labels = sys.argv[1:] or ["900", "1000", "1100", "1500", "1800", "1900", "endpoint"]
rows = []
for label in labels:
    d = base / f"v2-baseline-eval-{label}"
    g = d / "flat-gait-gate.json"
    if not g.exists():
        continue
    gate = json.loads(g.read_text())
    cases = gate["cases"]
    margins = {}
    for mode in ("forward", "reverse"):
        margins[mode] = cases[mode]["min_four_wheel_support"] / 0.90
    for mode in ("left", "right", "yaw_left", "yaw_right"):
        margins[mode] = min(cases[mode]["min_events_per_wheel"]) / 2.0
    worst = min(margins.values())
    sha = (d / "checkpoint.sha256").read_text().split()[0]
    rows.append((worst, label, sha, gate["qualified"], margins))
rows.sort(reverse=True)
for worst, label, sha, ok, margins in rows:
    print(f"{label:9s} qualified={ok!s:5s} worst_margin={worst:6.3f} sha={sha[:12]}")
    print("          " + "  ".join(f"{m}:{v:.2f}" for m, v in margins.items()))
if rows:
    best = rows[0]
    print(f"\nBEST_PARENT v2-baseline-eval-{best[1]} sha={best[2]}")
