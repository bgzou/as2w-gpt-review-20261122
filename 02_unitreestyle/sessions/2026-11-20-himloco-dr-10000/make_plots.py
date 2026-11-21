import json, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

S = r"X:\Items\as2w\_repo_as2w\sessions\2026-11-20-himloco-dr-10000"

axes = ["FL_hip","FL_thigh","FL_calf","FR_hip","FR_thigh","FR_calf",
        "RL_hip","RL_thigh","RL_calf","RR_hip","RR_thigh","RR_calf"]
none = [0.209,-0.159,0.006,-0.193,-0.004,0.053,0.144,0.237,0.336,-0.145,0.239,0.331]
with_off = [0.176,-0.112,0.142,-0.128,-0.305,-0.053,0.021,0.020,0.105,-0.076,0.117,0.199]

fig, ax = plt.subplots(figsize=(11,4.6))
x = np.arange(len(axes)); w=0.4
ax.bar(x-w/2, none, w, label="no gravity compensation (36 samples)", color="#c0392b")
ax.bar(x+w/2, with_off, w, label="static offset +0.21..+0.34 (last samples)", color="#2980b9")
ax.axhline(0, color="k", lw=.8)
ax.set_xticks(x); ax.set_xticklabels(axes, rotation=45, ha="right", fontsize=8)
ax.set_ylabel("cmd - actual (rad)\npositive = PD cannot hold, leg sags")
ax.set_title("AS2-W HIMLoco DR model_18200 — real-robot joint sink (POLICY_TRACK)")
ax.legend(fontsize=8); ax.grid(axis="y", alpha=.3)
fig.tight_layout(); fig.savefig(f"{S}/plots/p1-sink-per-axis.png", dpi=160); plt.close(fig)

rep = json.load(open(f"{S}/analysis/matrix-mu025-report.json"))
cmds = [c for c in ("handoff","forward","reverse","arc_left","arc_right","stop_same") if c in rep["by_command"]]
ev   = [rep["by_command"][c]["e_vx"] for c in cmds]
term = [rep["by_command"][c]["terminated"] for c in cmds]
pmax = [rep["by_command"][c]["pitch_max"] for c in cmds]
pmin = [rep["by_command"][c]["pitch_min"] for c in cmds]

fig, axs = plt.subplots(1,3, figsize=(13,4.2))
axs[0].bar(cmds, ev, color="#27ae60"); axs[0].set_title("vx tracking error (m/s)"); axs[0].tick_params(axis="x", rotation=30)
axs[1].bar(cmds, term, color="#8e44ad"); axs[1].set_title("terminated trials (of 12)")
axs[2].bar(cmds, pmax, color="#e67e22", label="pitch max"); axs[2].bar(cmds, pmin, color="#3498db", label="pitch min")
axs[2].set_title("pitch range (deg)"); axs[2].legend(fontsize=8); axs[2].tick_params(axis="x", rotation=30)
for a in axs: a.grid(axis="y", alpha=.3)
fig.suptitle("AS2-W HIMLoco DR model_18200 — fixed matrix, 4 poses x 3 loads, friction 0.25, 72 trials")
fig.tight_layout(); fig.savefig(f"{S}/plots/p2-capability-by-command.png", dpi=160); plt.close(fig)
print("PLOTS_OK")
