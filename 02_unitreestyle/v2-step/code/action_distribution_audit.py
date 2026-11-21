"""Offline Gaussian/clipping diagnostic; no modifications to training or checkpoints."""
import argparse, importlib.util, json, math
from pathlib import Path
import numpy as np
import torch
p=argparse.ArgumentParser()
p.add_argument("source",type=Path)
p.add_argument("base",type=Path)
p.add_argument("tape",type=Path)
p.add_argument("output",type=Path)
a=p.parse_args()
helpers=(a.source/"delivery/export.py",a.source.parents[1]/"model/export.py")
helper=next(path for path in helpers if path.is_file())
spec=importlib.util.spec_from_file_location("actor_export",helper)
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
with np.load(a.tape) as tape:
 obs=torch.from_numpy(tape["observation"].reshape(-1,58)[::10].copy())
result={"scope":"Offline fixed nominal observation tape, not sampled on-policy rollout statistics",
        "observation_tape":str(a.tape),"frames":len(obs),"clip_interval":[-1,1],
        "checkpoints":[]}
paths=[(f"base-{i}",a.base/"train"/f"model_{i}.pt") for i in (0,500,1000,1500,2000,2500,2800,3000)]
for level in (1,2,3):
 paths.extend((f"dr{level}-{label}",a.output.parent/f"stage-{level}/train/{label}.pt")
              for label in ("update-250","update-500","endpoint"))
for idx,path in paths:
 if not path.exists():continue
 d=torch.load(path,weights_only=True,map_location="cpu")
 sd=d["actor_state_dict"]["distribution.std_param"]
 actor=mod.load_actor(path)
 with torch.no_grad():
  mean=actor(obs)
  dist=torch.distributions.Normal(mean,sd)
  prob=dist.cdf(torch.full_like(mean,-1)) + 1-dist.cdf(torch.full_like(mean,1))
 result["checkpoints"].append({"runner_index":idx,"checkpoint":str(path),
   "mean_absolute_action_quantiles":torch.quantile(mean.abs().flatten(),torch.tensor([.5,.9,.99])).tolist(),
   "std_per_joint":sd.tolist(),"std_mean":sd.mean().item(),
   "expected_stochastic_clip_fraction_on_fixed_tape":prob.mean().item(),
   "expected_clip_fraction_per_joint":prob.mean(0).tolist(),
   "deterministic_mean_clip_fraction_on_fixed_tape":(mean.abs()>=1).float().mean().item(),
   "gaussian_clip_fraction_lower_bound_any_mean_per_joint":
      [max(0,1-2/(v*math.sqrt(2*math.pi))) for v in sd.tolist()]})
a.output.write_text(json.dumps(result,indent=2)+"\n")
for row in result["checkpoints"]:
 print(row["runner_index"],round(row["std_mean"],3),
       round(row["expected_stochastic_clip_fraction_on_fixed_tape"],4),
       round(row["deterministic_mean_clip_fraction_on_fixed_tape"],4))
