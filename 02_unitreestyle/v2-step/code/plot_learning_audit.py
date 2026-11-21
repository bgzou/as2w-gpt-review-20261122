"""Plot PPO curves alongside independent gait observations; never infer gait success from loss."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument("root",type=Path);a=p.parse_args()
trend=json.loads((a.root/"base-training-trends.json").read_text())["tensorboard"]
comparison_file=a.root/"base-checkpoint-comparison.json"
if not comparison_file.exists(): comparison_file=a.root/"base-u1001-u2001-u3001-comparison.json"
comparison=json.loads(comparison_file.read_text())
fig,axes=plt.subplots(2,2,figsize=(12,8))
for ax,tag,title in [
 (axes[0,0],"Train/mean_reward","PPO training return (100-record block means)"),
 (axes[0,1],"Policy/mean_std","Gaussian exploration std (100-record block means)"),
 (axes[1,0],"Loss/value","Value loss (100-record block means)")]:
 blocks=trend[tag]["blocks"]
 ax.plot([(b["first_step"]+b["last_step"])/2 for b in blocks],[b["mean"] for b in blocks])
 ax.set_title(title,fontsize=10);ax.set_xlabel("PPO update index");ax.grid(alpha=.25)
 if tag in ("Policy/mean_std","Loss/value"):ax.set_yscale("log")
ax=axes[1,1]
xs=sorted(map(int,comparison))
for mode in ("forward","reverse"):
 ax.plot(xs,[comparison[str(i)]["audit"][mode]["support_pct_0_to_4"][4] for i in xs],
         marker="o",label=mode)
ax.set_ylim(0,100);ax.set_title("Independent fixed replay: four-wheel support",fontsize=10)
ax.set_ylabel("% of valid frames");ax.set_xlabel("Completed updates (checkpoint index + 1)")
ax.grid(alpha=.25);ax.legend()
fig.suptitle("Unified baseline: numerical PPO stability is not gait qualification")
fig.text(.5,.01,"Fixed replay uses the original intent-audit protocol; do not compare directly with DR protocol percentages.",
         ha="center",fontsize=8)
fig.tight_layout(rect=(0,.03,1,.96))
fig.savefig(a.root/"base-learning-vs-motion.png",dpi=150)
plt.close(fig)
