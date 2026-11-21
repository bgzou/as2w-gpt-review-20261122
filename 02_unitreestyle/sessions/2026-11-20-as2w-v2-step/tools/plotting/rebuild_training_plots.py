"""Rebuild training-data extracts and plots from all fetched runs."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

GH2 = Path(r"X:\Items\as2w\.stage58\gh2")
STAGE = Path(r"X:\Items\as2w\.stage58\gh-stage-20261120")
DATA, PLOTS = STAGE / "data", STAGE / "plots"
plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})

RUNS = {
    "v2-steponly600": "step-only 600",
    "v2-steprear600": "rear-weight 600",
    "v2-stephard600": "hardest 600 (delivered DR0)",
    "v2-jitter600": "DR2+jitter 600",
    "v2-mu025-600": "mu025 DR2+jitter 600",
    "v2-tanh-dr1-600": "DR1 600",
    "v2-tanh-dr2-400": "DR2 400",
    "v2-tanh-dr3-300": "DR3 300",
}

logs = {}
for run in RUNS:
    f = GH2 / f"{run}.reward-components.jsonl"
    if f.exists():
        logs[run] = [json.loads(l) for l in f.open()]
print("runs loaded:", {k: len(v) for k, v in logs.items()})
print("keys (latest run):", sorted(logs["v2-mu025-600"][0].keys())[:20])

KEYS = ["executed_signflip_fraction", "executed_delta_mean", "executed_abs_mean",
        "swing_event", "event_right_RR", "event_left_RR", "event_right_RL",
        "support_right_RR", "tracking"]

# 鈹€鈹€ summary csv 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
lines = ["run,updates," + ",".join(KEYS)]
for run, rows in logs.items():
    last = rows[-1]
    vals = []
    for k in KEYS:
        if k in last:
            vals.append(f"{last[k]:.6f}")
        elif k == "tracking" and "swing_event" in last:
            vals.append("")
        else:
            vals.append("")
    lines.append(f"{run},{len(rows)}," + ",".join(vals))
(DATA / "training-runs-summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

# 鈹€鈹€ downsampled curves csv 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cos = ["run", "update", "executed_signflip_fraction", "executed_delta_mean",
       "executed_abs_mean", "event_right_RR", "event_left_RR", "swing_event"]
out = [",".join(cos)]
for run, rows in logs.items():
    for r in rows[::10]:
        def g(k):
            return f"{r[k]:.6f}" if k in r else ""
        out.append(f"{run},{r['update']}," + ",".join(g(k) for k in cos[2:]))
(DATA / "training-curves-downsampled.csv").write_text("\n".join(out) + "\n", encoding="utf-8")

# 鈹€鈹€ P6: jitter metrics across runs 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
ax = axes[0]
for run, label in RUNS.items():
    if run not in logs:
        continue
    rows = logs[run]
    if "executed_signflip_fraction" not in rows[0]:
        continue
    upd = [r["update"] for r in rows]
    ax.plot(upd, [r["executed_signflip_fraction"] * 100 for r in rows], lw=1, label=label)
ax.set_xlabel("update"); ax.set_ylabel("sign-flip [%]")
ax.set_title("executed-action sign-flip fraction per run")
ax.legend(fontsize=7)

ax = axes[1]
for run, label in RUNS.items():
    if run not in logs:
        continue
    rows = logs[run]
    if "swing_event" not in rows[0]:
        continue
    upd = [r["update"] for r in rows]
    ax.plot(upd, [r["swing_event"] for r in rows], lw=1, label=label)
ax.set_xlabel("update"); ax.set_ylabel("swing_event reward term")
ax.set_title("swing event reward (task progress)")
ax.legend(fontsize=7)
fig.savefig(PLOTS / "p6-training-curves.png")
plt.close(fig)

# 鈹€鈹€ P8: right-wheel lift events (the unsolved direction) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
fig, ax = plt.subplots(figsize=(9, 3.2))
for run, label in RUNS.items():
    if run not in logs:
        continue
    rows = logs[run]
    if "event_right_RR" not in rows[0]:
        continue
    ax.plot([r["update"] for r in rows], [r["event_right_RR"] for r in rows], lw=1, label=label)
ax.set_ylim(bottom=0)
ax.set_xlabel("update"); ax.set_ylabel("event_right_RR")
ax.set_title("event_right_RR: final value 0.000000 for every arm (peak during training <= 0.0015)")
ax.legend(fontsize=7)
fig.savefig(PLOTS / "p8-right-RR-events.png")
plt.close(fig)

print("wrote", sorted(p.name for p in PLOTS.glob("*.png")))

