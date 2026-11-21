import json
from pathlib import Path
import subprocess
import sys
import numpy as np

def summary(path):
    with np.load(path/"flat.npz") as a:
        force=a["wheel_data"][...,15].clip(min=0.)
        valid=~np.maximum.accumulate(a["fall"],axis=0)
        u=a["target_trace"][:,:,0,:]/.25
        out={}
        for case in np.unique(a["labels"]):
            mask=(a["labels"]==case)[None,:]&valid
            mask[:50]=False
            f=force[mask]
            if not len(f) or not f.sum()>0:
                continue
            # A one- or two-sample reversal must cross 1.3 action units in both
            # directions. Subtracting a fixed joint zero would cancel here.
            j=np.flatnonzero(a["labels"]==case)
            z=u[50:,j]
            ok=valid[50:,j]
            left=z[1:-2]-z[:-3]
            right1=z[1:-2]-z[2:-1]
            right2=z[2:-1]-z[3:]
            plateau=np.abs(z[2:-1]-z[1:-2])<.25
            one=(left*right1>0)&(np.minimum(abs(left),abs(right1))>1.3)
            two=(left*right2>0)&(np.minimum(abs(left),abs(right2))>1.3)&plateau
            count=int(((one|two)&ok[3:,:,None]).sum())
            out[str(case)]={"short_reversals":count,"load_share":(f.sum(0)/f.sum()).tolist(),
                            "duty":(f>=3.).mean(0).tolist(),
                            "force_p05_p50_p95_N":np.quantile(f,[.05,.5,.95],axis=0).tolist()}
        return out

def compare(base,candidate):
    original=candidate/"gate.json"
    archive=candidate/"historical-gate.json"
    if not archive.exists():
        archive.write_bytes(original.read_bytes())
    checker=Path(__file__).resolve().parents[1]/"continuation1220/gate.py"
    subprocess.run([sys.executable,str(checker),str(base),str(candidate)],check=True,stdout=sys.stderr)
    result=json.loads(original.read_text())
    a,b=summary(base),summary(candidate)
    result["wheel_load_baseline"]=a
    result["wheel_load_candidate"]=b
    for case,old in a.items():
        if case not in b:
            result["failures"].append(case+": missing force trace")
            continue
        for k,tol in (("load_share",.08),("duty",.12)):
            if np.max(np.abs(np.subtract(old[k],b[case][k])))>tol:
                result["failures"].append(case+": wheel "+k)
    if json.loads((candidate/"run.json").read_text()).get("profile")=="raised-v1":
        from height_audit import assess
        heights=assess(base,candidate)
        result["height_audit"]=heights
        result["failures"].extend(heights["failures"])
    result["gait_preserved"]=not result["failures"]
    original.write_text(json.dumps(result,indent=2))
    return result
