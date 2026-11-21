"""Compare a continued checkpoint to the frozen update-1220 gait."""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import butter,sosfiltfilt

old,new=map(Path,sys.argv[1:])
def load(path):
    with np.load(path/"flat.npz") as file:
        arrays={k:file[k] for k in ("q","vel","fall","labels","wheel_data","target_trace")}
    return arrays,json.loads((path/"flat-motion-audit.json").read_text())
ref,ref_report=load(old); cand,cand_report=load(new)
assert np.array_equal(ref["labels"],cand["labels"])
legs=("FL","FR","RL","RR")
anchor={"left":1,"right":1,"yaw_left":3,"yaw_right":2}
metrics={}
failures=[]
old_count=new_count=0
sos=butter(2,5.,fs=50.,btype="highpass",output="sos")
grid=(np.arange(80)+.5)/80

def phase_diagnostic(record,ids,w):
    phase_samples=[]; seq=[]; periods=[]
    for i in ids:
        f=np.flatnonzero(record["fall"][:,i])
        end=f[0] if len(f) else len(record["q"])
        s=record["wheel_data"][:end,i,w,15]>=3.
        tz=[k for k in range(50,end-2) if s[k:k+3].all() and not s[k-3:k].any()]
        for a,b in zip(tz,tz[1:]):
            if 18<=b-a<=55:
                q=record["q"][a:b+1,i,:12]
                phase=np.linspace(0,1,len(q))
                phase_samples.append(np.stack([np.interp(grid,phase,v) for v in q.T],axis=1))
                seq.append((phase,q)); periods.append((b-a)*.02)
    if len(seq)<3: return None
    median=np.median(phase_samples,axis=0)
    ms=[]
    for x,q in seq:
        fitted=np.stack([np.interp(x,grid,v,period=1.) for v in median.T],axis=1)
        ms.append(float(np.mean(sosfiltfilt(sos,q-fitted,axis=0)**2)))
    return dict(cycles=len(seq),period_s=float(np.median(periods)),measured_residual_rms_deg=float(np.rad2deg(np.sqrt(np.mean(ms)))))

def spikes(record,ids):
    u=(record["target_trace"][50:][:,ids,0,:]-np.tile([0.,.8,-1.6],4))/.25
    sign=np.where(u>=.9999,1,np.where(u<=-.9999,-1,0))
    n=0
    for row in np.moveaxis(sign,0,-1).reshape(-1,len(u)):
        k=np.flatnonzero(np.diff(row))+1
        n+=sum(b-a<=2 and row[a]!=0 and row[a-1]==row[b]==-row[a] for a,b in zip(k,k[1:]))
    return int(n)

for c,old_row in ref_report.items():
    ids=np.flatnonzero(ref["labels"]==c)
    v=cand_report[c]
    metrics[c]={"baseline":old_row,"candidate":v}
    if v["survived"]<old_row["survived"]:
        failures.append(f"{c}: falls")
    if not v["mean_body_vx_vy_wz"]:
        failures.append(f"{c}: no scored frames")
        continue
    if c in ("forward","reverse","stand"):
        if v["support_pct_0_to_4"][4]<old_row["support_pct_0_to_4"][4]-2.:
            failures.append(f"{c}: loss of four-wheel stance")
        if abs(v["mean_body_vx_vy_wz"][0]-old_row["mean_body_vx_vy_wz"][0])>.04:
            failures.append(f"{c}: longitudinal speed")
    if c not in anchor and c!="mixed":
        continue
    axes=(1,) if c in ("left","right") else ((2,) if c in ("yaw_left","yaw_right") else (0,1,2))
    limits=(.05,.03,.06)
    for axis in axes:
        if abs(v["mean_body_vx_vy_wz"][axis]-old_row["mean_body_vx_vy_wz"][axis])>limits[axis]:
            failures.append(f"{c}: tracking axis {axis}")
    for i,(a,b) in enumerate(zip(old_row["lift_event_counts_FL_FR_RL_RR"],v["lift_event_counts_FL_FR_RL_RR"])):
        if a>=6 and b<.75*a: failures.append(f"{c}: {legs[i]} lifts")
    if c in anchor:
        a,b=(phase_diagnostic(x,ids,anchor[c]) for x in (ref,cand))
        metrics[c].update(phase_baseline=a,phase_candidate=b)
        if a and (not b or b["cycles"]<.75*a["cycles"] or abs(b["period_s"]/a["period_s"]-1.)>.15):
            failures.append(f"{c}: cadence")
        if a and b and b["measured_residual_rms_deg"]>1.10*a["measured_residual_rms_deg"]:
            failures.append(f"{c}: increased measured phase residual")
        before,after=(np.rad2deg(np.diff(np.quantile(x["q"][50:][:,ids,:12],[.05,.95],axis=(0,1)),axis=0)[0]) for x in (ref,cand))
        if np.any((before>=8.) & (after<.8*before)):
            failures.append(f"{c}: reduced active joint excursion")
    a=spikes(ref,ids); b=spikes(cand,ids)
    metrics[c].update(baseline_shorts=a,candidate_shorts=b)
    old_count+=a; new_count+=b
result=dict(gait_preserved=not failures,failures=failures,spikes_baseline=old_count,spikes_candidate=new_count,case_metrics=metrics)
(new/"gate.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps({k:v for k,v in result.items() if k!="case_metrics"}))
