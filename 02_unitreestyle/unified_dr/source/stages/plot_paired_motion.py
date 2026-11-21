"""Plot fixed, declared trial zero. Aggregate qualification remains in gate.json."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument("selection",type=Path);p.add_argument("out",type=Path);a=p.parse_args()
a.out.mkdir(exist_ok=True,parents=True)
with np.load(a.selection/"incoming/flat-dr0/flat.npz") as f:
 old={k:f[k] for k in f.files}
with np.load(a.selection/"update-250/flat-dr0/flat.npz") as f:
 new={k:f[k] for k in f.files}
t=np.arange(len(new["q"]))*0.02
meta={"scope":"Nominal flat, seed 319, first trial per command; figures are illustrative, gate.json covers all six trials.",
      "old":"stage input","new":"DR3 update 250","plots":[]}
for mode in ("forward","reverse","left","yaw_left"):
 idx=int(np.flatnonzero(new["labels"]==mode)[0])
 assert old["labels"][idx]==mode
 fig,axes=plt.subplots(4,3,figsize=(15,9),sharex=True)
 for j,ax in enumerate(axes.flat):
  ax.plot(t,np.rad2deg(old["q"][:,idx,j]),label="input actual",lw=1,alpha=.8)
  ax.plot(t,np.rad2deg(new["q"][:,idx,j]),label="u250 actual",lw=1)
  # evaluate.py stores absolute target angles (reference already added).
  ax.plot(t,np.rad2deg(new["target_trace"][:,idx,1,j]),
          "--",label="u250 filtered target",lw=.8,alpha=.8)
  ax.set_title(("FL","FR","RL","RR")[j//3]+" "+("hip","thigh","calf")[j%3],fontsize=9)
  ax.grid(alpha=.25)
  if j%3==0: ax.set_ylabel("deg")
  if j//3==3: ax.set_xlabel("time (s)")
 axes[0,0].legend(fontsize=7)
 fig.suptitle(f"{mode}: nominal flat / seed 319 / trial 0 (not aggregate verdict)")
 fig.tight_layout()
 file=f"{mode}-joint-angles.png";fig.savefig(a.out/file,dpi=140);plt.close(fig)
 meta["plots"].append(file)
 fig,axes=plt.subplots(4,1,figsize=(13,8),sharex=True)
 for w,ax in enumerate(axes):
  ax.plot(t,old["normal_load"][:,idx,w],label="input",lw=1,alpha=.8)
  ax.plot(t,new["normal_load"][:,idx,w],label="u250",lw=1,alpha=.85)
  ax.axhline(3,ls=":",color="gray",lw=.7,label="3 N contact threshold")
  ax.set_ylabel(("FL","FR","RL","RR")[w]+" (N)");ax.grid(alpha=.25)
 axes[0].legend(fontsize=8);axes[-1].set_xlabel("time (s)")
 fig.suptitle(f"{mode}: wheel normal forces / nominal flat / seed 319 / trial 0")
 fig.tight_layout()
 file=f"{mode}-wheel-normal-load.png";fig.savefig(a.out/file,dpi=140);plt.close(fig)
 meta["plots"].append(file)
(a.out/"context.json").write_text(json.dumps(meta,indent=2)+"\n")
print(a.out)
