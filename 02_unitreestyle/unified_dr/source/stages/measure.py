"""Calibrate and score a complete unloaded swing from a fixed flat replay."""
import json
from pathlib import Path
import sys

import numpy as np

MODES=("left","right","yaw_left","yaw_right")
WHEELS=("FL","FR","RL","RR")
DT=.02
PROFILE=np.load(Path(__file__).with_name("tire_profile.npy"))
CONTACT_OFF=2.
CONTACT_ON=4.
MIN_HEIGHT=.010
OFF_FRAMES=3
ON_FRAMES=2

def geometry(q):
    x,y,z,w=np.moveaxis(q,-1,0)
    a=np.stack((2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)),axis=-1)
    normal=np.array([0.,0.,1.])
    az=a[...,2:3]
    f=((az[...,0]+1)*256).clip(0,512)
    i=f.astype(np.int64).clip(max=511)
    wheel=np.arange(4)
    u=(f-i)[...,None]
    p=PROFILE[wheel,i]*(1-u)+PROFILE[wheel,i+1]*u
    radial=normal-az*a
    r=p[...,0:1]*a-p[...,1:2]*radial/np.linalg.norm(radial,axis=-1,keepdims=True).clip(min=.01)
    return a,r

def observations(path):
    with np.load(path) as f:
        labels,fall,w=(f[k] for k in ('labels','fall','wheel_data'))
    a,r=geometry(w[...,3:7])
    v=w[...,7:10]+np.cross(w[...,10:13],r)
    return labels,fall,w[...,15],w[...,2]+r[...,2],abs(np.sum(v*a,axis=-1))

def swings(force,height):
    off=on=0
    armed=False
    start=None
    peak=0.
    for i,(f,h) in enumerate(zip(force,height,strict=True)):
        off=off+1 if f<=CONTACT_OFF else 0
        on=on+1 if f>=CONTACT_ON else 0
        if on>=ON_FRAMES and start is None:
            armed=True
        if off==1 and armed:
            peak=float(h)
        elif off and armed:
            peak=max(peak,float(h))
        if off==OFF_FRAMES and armed:
            start=i-OFF_FRAMES+1
            armed=False
        if start is not None:
            peak=max(peak,float(h))
            if on==ON_FRAMES:
                end=i-ON_FRAMES+1
                yield end,(end-start)*DT,peak
                start=None
                armed=True

def records(path):
    labels,falls,f,h,slip=observations(path)
    data={}
    for mode in MODES:
        wheels={}
        for wi,wheel in enumerate(WHEELS):
            trials=[]
            for i in np.flatnonzero(labels==mode):
                failed=np.flatnonzero(falls[:,i])
                end=int(failed[0]) if len(failed) else len(f)
                events=[event for event in swings(f[:end,i,wi],h[:end,i,wi])
                        if event[0]>=50 and event[2]>=MIN_HEIGHT]
                contact=f[50:end,i,wi]>=CONTACT_ON
                trials.append((end,events,slip[50:end,i,wi][contact]))
            wheels[wheel]=trials
        data[mode]=wheels
    return data

def calibrate(path):
    result={}
    for mode,wheels in records(path).items():
        result[mode]={}
        for wheel,trials in wheels.items():
            events=[e for _,seq,_ in trials for e in seq]
            n=len(events)
            if n<6:
                result[mode][wheel]={"eligible":False,"baseline_events":n}
                continue
            dur,peak=np.array(events)[:,1:].T
            gaps=[DT*(b[0]-a[0]) for _,seq,_ in trials for a,b in zip(seq,seq[1:])]
            gap=float(np.quantile(gaps,.75)) if gaps else 1.
            result[mode][wheel]={"eligible":True,"baseline_events":n,
                "min_s":round(max(OFF_FRAMES*DT,float(np.quantile(dur,.1))-.02),2),
                "max_s":round(max(.18,float(np.quantile(dur,.9))+.06),2),
                "min_m":round(max(MIN_HEIGHT,float(np.quantile(peak,.15))-.002),4),
                "max_m":round(max(float(np.quantile(peak,.90))+.012,.025),4),
                "min_interval_s":round(max(.24,.70*gap),2),
                "gap_budget_s":round(min(2.,max(.65,1.35*gap)),2)}
    return result

def report(path, reference):
    output={}
    for mode,wheels in records(path).items():
        output[mode]={}
        for wheel,trials in wheels.items():
            spec=reference[mode][wheel]
            all_events=[e for _,seq,_ in trials for e in seq]
            valid=lambda e:spec.get("eligible") and (spec["min_s"]-.001<=e[1]<=spec["max_s"]+.001 and spec["min_m"]<=e[2]<=spec["max_m"])
            seqs=[[e for e in seq if valid(e)] for _,seq,_ in trials]
            good=[e for seq in seqs for e in seq]
            longest=max((max(np.diff([50,*[e[0] for e in seq],max(50,end)]))*DT
                         for (end,_,_),seq in zip(trials,seqs,strict=True)),default=0.)
            peaks=[e[2]*1000 for e in all_events]
            slip=np.concatenate([x for _,_,x in trials])
            output[mode][wheel]={"events":len(all_events),"calibrated_events":len(good),
                 "peak_median_mm":float(np.median(peaks)) if peaks else None,
                 "longest_calibrated_gap_s":round(float(longest),3),
                 "loaded_slip_p95_m_s":float(np.quantile(slip,.95)) if len(slip) else None}
    return output

if __name__=="__main__":
    cmd,trace,dst,*extra=sys.argv[1:]
    result=calibrate(Path(trace)) if cmd=="calibrate" else report(Path(trace),json.loads(Path(extra[0]).read_text()))
    Path(dst).write_text(json.dumps(result,indent=2)+"\n")
