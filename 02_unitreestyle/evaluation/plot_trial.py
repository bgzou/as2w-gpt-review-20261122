"""Plot measured joint angles, policy targets and normal force from a flat replay."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("trace", type=Path)
parser.add_argument("scenario")
parser.add_argument("output", type=Path)
parser.add_argument("--trial", type=int, default=0)
args = parser.parse_args()

with np.load(args.trace) as trace:
    trials = np.flatnonzero(trace["labels"] == args.scenario)
    if not 0 <= args.trial < len(trials):
        parser.error(f"{args.scenario!r} has {len(trials)} saved trials")
    env = trials[args.trial]
    failure = np.flatnonzero(trace["fall"][:, env])
    end = int(failure[0]) if len(failure) else len(trace["q"])
    angles = trace["q"][:end, env, :12]
    targets = trace["target_trace"][:end, env]
    normal = trace["wheel_data"][:end, env, :, 15]

fig, axes = plt.subplots(4, 4, figsize=(15, 10), sharex=True, layout="constrained")
time = np.arange(end) * 0.02
for column, leg in enumerate(("FL", "FR", "RL", "RR")):
    axes[0, column].set_title(leg)
    for row in range(3):
        joint = 3 * column + row
        ax = axes[row, column]
        ax.plot(time, np.rad2deg(targets[:, 0, joint]), color="0.7", linewidth=0.7, label="Before LPF")
        ax.plot(time, np.rad2deg(targets[:, 1, joint]), color="#e68515", linewidth=0.9, label="After LPF")
        ax.plot(time, np.rad2deg(angles[:, joint]), color="#1769aa", linewidth=1, label="Measured")
    force = axes[3, column]
    force.plot(time, normal[:, column], color="#267845", linewidth=1)
    force.axhline(3, color="0.4", linewidth=0.7, linestyle="--")
    force.set_xlabel("Time (s)")

for row, label in enumerate(("Hip (deg)", "Thigh (deg)", "Calf (deg)", "Normal force (N)")):
    axes[row, 0].set_ylabel(label)
axes[0, 0].legend(fontsize=8)
fig.suptitle(f"{args.scenario}, trial {args.trial}")
fig.savefig(args.output, dpi=125)
