"""Score one fixed nominal/stress replay on absolute motion and paired behavior."""
import json
from pathlib import Path
import sys
import numpy as np

MODES=("stand","forward","reverse","left","right","yaw_left","yaw_right","mixed")
AXES={"forward":(0,),"reverse":(0,),"left":(1,),"right":(1,),
      "yaw_left":(2,),"yaw_right":(2,),"mixed":(0,1,2)}
WHEELS=("FL","FR","RL","RR")

def high_frequency_rms_deg(values, dt=.02):
    """RMS of 5–20 Hz motion in each joint after removing its mean."""
    if len(values)<75:
        return None
    window=np.hanning(len(values))
    signal=(values-values.mean(0))*window[:,None]
    spectrum=np.fft.rfft(signal,axis=0)
    freq=np.fft.rfftfreq(len(values),d=dt)
    band=(freq>=5.)&(freq<=20.)
    variance=2.*(abs(spectrum[band])**2).sum(0)/(len(window)*(window**2).sum())
    return np.rad2deg(np.sqrt(variance)).tolist()


def metrics(path):
    r={}
    is_slope=path.stem=="slope"
    with np.load(path) as d:
        q=d["q"][50:,:,:12]
        alive=(np.cumsum(d["fall"],axis=0)==0)
        forces=d["normal_load"] if "normal_load" in d else d["wheel_data"][...,15]
        height=d["clearance"] if "clearance" in d else d["wheel_data"][...,2]-.086
        touching=forces[50:]>=3
        slip=d["lateral_slip"][50:] if "lateral_slip" in d else None
        targets=d["target_trace"][50:]
        for m in MODES:
            ii=np.flatnonzero(d["labels"]==m)
            trials=[]
            for trial,i in enumerate(ii):
                use=alive[50:,i]
                if not use.any():
                    trials.append({"trial":trial,"survived":False,"err":2.,"support":0.,"lifts":[0]*4,
                                   "loaded_slip_p95":None,"wheel_target":None,"swings":[]})
                    continue
                c=touching[use,i]
                e=(d["vel"][50:,i][use]-d["cmd"][50:,i][use])
                axes=AXES.get(m,(0,1,2))
                err=float(np.max(np.abs(e[...,list(axes)].mean(0))))
                lift=(height[50:,i][use]>=.010)&~c
                lifts=[]; cycles=[]
                for w in range(4):
                    d1=np.diff(np.r_[False,lift[:,w],False].astype(int))
                    starts=np.flatnonzero(d1==1)
                    ends=np.flatnonzero(d1==-1)
                    times=starts[(ends-starts)>=3]
                    lifts.append(int(len(times)))
                    cycles.extend(np.diff(times).tolist())
                slip95=float(np.quantile(slip[use,i][c],.95)) if slip is not None and c.any() else None
                trials.append({"trial":trial,"survived":bool(alive[-1,i]),"err":err,
                    "support_count_fraction":(np.bincount(c.sum(1),minlength=5)/len(c)).tolist(),
                    "wheel_loaded_fraction":c.mean(0).tolist(),
                    "wheel_normal_n_p05":np.quantile(forces[50:,i][use],.05,axis=0).tolist(),
                    "wheel_normal_n_p95":np.quantile(forces[50:,i][use],.95,axis=0).tolist(),
                    "joint_high_band_rms_deg":{
                        "actual":high_frequency_rms_deg(q[:,i][use]),
                        "pre_filter_target":high_frequency_rms_deg(targets[:,i,0][use]),
                        "post_filter_target":high_frequency_rms_deg(targets[:,i,1][use]),
                    },
                    "tracking_mae":np.abs(e).mean(0).tolist(),
                    "support":float(c.all(1).mean()),"lifts":lifts,"loaded_slip_p95":slip95,
                    "nonwheel_fraction":float(d["nonwheel"][50:,i][use].mean()),
                    "tilt_p95_deg":float(np.quantile(d["tilt"][50:,i][use],.95)),
                    "wheel_target":float(abs(d["wheel_target"][50:,i][use]).mean()),
                    "swings":cycles})
            valid=alive[50:,ii]
            angles=q[:,ii][valid]
            slipp=[x["loaded_slip_p95"] for x in trials if x["loaded_slip_p95"] is not None]
            jitter=[max(x["joint_high_band_rms_deg"]["actual"])
                    for x in trials if x.get("joint_high_band_rms_deg",{}).get("actual") is not None]
            r[m]={"trials":trials,"joint_high_band_worst_axis_median_deg":float(np.median(jitter)) if jitter else None,"survived":sum(x["survived"] for x in trials),
                  "mean_err":float(np.mean([x['err'] for x in trials])),
                  "worst_trial_err":max(x["err"] for x in trials),
                  "support":float(np.mean([x["support"] for x in trials])),
                  "cadence_hz":float(50./np.median(cycle_samples)) if (cycle_samples:=[s for x in trials for s in x["swings"]]) else None,
                  "lifts":np.asarray([x["lifts"] for x in trials]).sum(0).tolist(),
                  "loaded_slip_p95_median":float(np.median(slipp)) if slipp else None,
                  "excursion_deg":np.rad2deg(np.quantile(angles,.95,axis=0)-np.quantile(angles,.05,axis=0)).tolist()
                     if len(angles) else [0.]*12}
    return r

def score(path, reference):
    a,b=metrics(path),metrics(reference)
    slope=path.stem=="slope"
    failures=[]
    for m in MODES:
        v,old=a[m],b[m]
        if v["survived"]<old["survived"] or v["survived"]<5:
            failures.append(m+": survival")
        if v["mean_err"]>(.18 if m.startswith("yaw") else (.15 if slope else .10)):
            failures.append(m+": tracking")
        if any(x.get("nonwheel_fraction",0)>.02 for x in v["trials"]):
            failures.append(m+": sustained non-wheel support")
        if m in ("stand","forward","reverse"):
            floor=.70 if slope else .92
            if v["support"]<max(floor,old["support"]-(.08 if slope else .02)):
                failures.append(m+": rolling support")
        if m in ("left","right","yaw_left","yaw_right"):
            if sum(x["survived"] and min(x["lifts"])>=2 for x in v["trials"])<3:
                failures.append(m+": missing repeated stepping on all four wheels")
            if any(x["wheel_target"] is None or x["wheel_target"]>1e-4 for x in v["trials"]):
                failures.append(m+": nonzero wheel target")
        if m in ("left","right","yaw_left","yaw_right","mixed"):
            for w,(x,y) in enumerate(zip(old["lifts"],v["lifts"])):
                if x>=6 and y<.75*x:
                    failures.append(f"{m}/{WHEELS[w]}: fewer lifts")
            if any(x>=8. and y<.8*x for x,y in zip(old["excursion_deg"],v["excursion_deg"])):
                failures.append(m+": smaller joint excursion")
    return {"cases":a,"failures":failures}

if __name__=="__main__":
    candidate,ref,dest=map(Path,sys.argv[1:])
    result=score(candidate,ref)
    tmp=dest.with_suffix(".tmp")
    tmp.write_text(json.dumps(result,indent=2)+"\n")
    tmp.replace(dest)
    print(candidate.parent.name,candidate.stem,result["failures"],flush=True)
