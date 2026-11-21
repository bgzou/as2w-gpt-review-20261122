from pathlib import Path
import json
import sys
import numpy as np
from scipy import signal

run=Path(sys.argv[1])
paths=[run/"baseline/flat.npz",run/"review/u3220/training-best/flat.npz"]
cases={"left":1,"right":1,"yaw_left":3,"yaw_right":2}
dt=.02
fs=1/dt
hi=signal.butter(2,5,btype="highpass",fs=fs,output="sos")
phase_grid=np.arange(80)/80

def events(l):
    mask=l>=3
    return np.array([i for i in range(50,len(mask)-2) if mask[i:i+3].all() and not mask[i-3:i].any()])

def spectral(x,ranges=((.5,4),(5,18))):
    f,ps=signal.periodogram(x[50:],fs=fs,window="hann",detrend="constant",axis=0)
    # Pool power over trials before picking peaks.
    ps=ps.mean(axis=1)
    peaks=[]
    for lo,up in ranges:
        ok=(f>=lo)&(f<=up)
        peaks.append((f[ok][ps[ok].argmax(axis=0)],ps[ok].max(axis=0)))
    return peaks

def phase_fit(a,loads,anchor):
    # Hold out an entire trial, remove each cycle's DC offset, and see
    # how much across-cycle power the common touchdown template accounts for.
    cycles=[]; owners=[]; lengths=[]
    for j in range(a.shape[1]):
        k=events(loads[:,j,anchor])
        for s,e in zip(k,k[1:]):
            if not 18<=e-s<=55:
                continue
            q=a[s:e+1,j]
            y=np.stack([np.interp(phase_grid,np.linspace(0,1,len(q)),z) for z in q.T],axis=1)
            cycles.append(y-y.mean(axis=0)); owners.append(j); lengths.append((e-s)*dt)
    y=np.array(cycles); owners=np.array(owners)
    ss=np.zeros(a.shape[2]); base=np.zeros_like(ss)
    for test in np.unique(owners):
        held=y[owners==test]
        pred=y[owners!=test].mean(axis=0)
        ss+=((held-pred)**2).sum(axis=(0,1))
        base+=(held**2).sum(axis=(0,1))
    return len(lengths),float(np.median(lengths)),1-np.divide(ss,base,out=np.full_like(ss,np.nan),where=base>0)

def contact_ratio(a,loads):
    out=[]
    for foot in range(4):
        event=np.zeros(loads.shape[:2],dtype=bool)
        for j in range(a.shape[1]):
            for k in events(loads[:,j,foot]):
                event[max(50,k-2):min(len(event),k+4),j]=True
        rest=(~event)[50:]
        joint=a[50:,:,foot*3:(foot+1)*3]
        count=event[50:].sum()
        out.append((int(count),np.sqrt((joint[event[50:]]**2).mean(axis=0)/(joint[rest]**2).mean(axis=0)).round(2).tolist() if count else []))
    return out

def analyze(path):
    results={}
    with np.load(path) as f:
        labels=f["labels"]; names=f["joints"][:12].tolist()
        for case in dict.fromkeys(labels):
            d=labels==case
            q=np.rad2deg(f["q"][:,d,:12])
            targets=np.rad2deg(f["target_trace"][:,d])
            raw,app=targets[:,:,0],targets[:,:,1]
            loads=f["wheel_data"][:,d][...,15]
            # e has a load/geometry offset as well as a dynamic component.
            err=app-q
            q_hi=signal.sosfiltfilt(hi,q,axis=0)
            err_hi=signal.sosfiltfilt(hi,err,axis=0)
            def rms(a): return np.sqrt((a[50:]**2).mean(axis=(0,1)))
            def quick(a):
                return [None if not np.isfinite(x) else round(float(x),3) for x in a]
            q_s=spectral(q)
            a_s=spectral(app)
            def vr(mask): return float(np.mean(mask))
            result=dict(q_peaks_low=quick(q_s[0][0]),q_peaks_high=quick(q_s[1][0]),applied_peaks_high=quick(a_s[1][0]),
                q_high_rms=quick(rms(q_hi)),tracking_rms=quick(rms(err)),tracking_mean=quick(err[50:].mean((0,1))),
                tracking_high_rms=quick(rms(err_hi)),contact_q_high=contact_ratio(q_hi,loads),
                applied_std=quick(app[50:].std(axis=(0,1))), applied_high_rms=quick(rms(signal.sosfiltfilt(hi,app,axis=0))),
                raw_std=quick(raw[50:].std(axis=(0,1))),
                contact_err_high=contact_ratio(err_hi,loads),four_support=vr((loads[50:]>=3).all(axis=-1)))
            if case in cases:
                stats={}
                for label,series in (("q",q),("err",err),("q_high",q_hi),("err_high",err_hi),("raw",raw)):
                    n,period,fit=phase_fit(series,loads,cases[case])
                    stats[label]=quick(fit)
                result.update(cycles=n,period=period,phase=stats)
            results[case]=result
    return results,names

a,names=analyze(paths[0])
b,_=analyze(paths[1])
(Path(sys.argv[2])/"results.json").write_text(json.dumps({"joints":names,"1220":a,"3160":b},indent=2)+"\n")
for c in a:
    print(c, "4wheel", round(100*a[c]["four_support"],2),round(100*b[c]["four_support"],2))
    if "cycles" in b[c]:
        for run,d in (("1220",a[c]),("3160",b[c])):
            row=d["phase"]
            print(run,"cycles",d["cycles"],"period",d["period"],
                "median phase R2 q/err/fast_q/fast_err/raw",
                [round(float(np.nanmedian(np.asarray(row[k],dtype=float))),2) for k in ("q","err","q_high","err_high","raw")])
    for i in [np.argmax(b[c]["q_high_rms"])] + ([6] if c=="yaw_right" else [2] if c in ("forward","reverse") else []):
        print(names[i], *[
           (d["q_peaks_low"][i],d["q_peaks_high"][i],d["applied_peaks_high"][i],
            d["q_high_rms"][i],d["tracking_mean"][i],d["tracking_high_rms"][i],
            d["contact_q_high"][i//3],d.get("phase",{}).get("err",[None]*12)[i])
           for d in (a[c],b[c])])
