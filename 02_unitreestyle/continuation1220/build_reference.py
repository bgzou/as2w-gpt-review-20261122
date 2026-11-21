import sys
import numpy as np
from scipy.signal import savgol_filter

src,dst=sys.argv[1:]
cases=("left","right","yaw_left","yaw_right")
anchor=(1,1,3,2)
phase=np.arange(128)/128
with np.load(src) as a:
    labels=a["labels"]
    f=a["wheel_data"][...,15]
    q=a["target_trace"][:,:,0,:]
    q=(q-np.tile([0.,.8,-1.6],4))/.25
    fall=a["fall"]
refs=[]; periods=[]
for c,w in zip(cases,anchor,strict=True):
    cycles=[]; duration=[]
    for e in np.flatnonzero(labels==c):
        failed=np.flatnonzero(fall[:,e])
        end=int(failed[0]) if len(failed) else len(q)
        cts=f[:end,e,w]>=3.
        starts=[k for k in range(50,end-2) if cts[k:k+3].all() and not cts[k-3:k].any()]
        for first,last in zip(starts,starts[1:]):
            if 18<=last-first<=55:
                x=np.linspace(0,1,last-first+1)
                cycles.append(np.stack([np.interp(phase,x,v) for v in q[first:last+1,e].T],axis=1))
                duration.append((last-first)*.02)
    assert len(duration)>=12,c
    refs.append(savgol_filter(np.median(cycles,axis=0),7,3,axis=0,mode="wrap").clip(-1,1))
    periods.append(float(np.median(duration)))
    print(c,len(duration),periods[-1])
np.savez(dst,reference=np.asarray(refs,dtype=np.float32),period=np.asarray(periods,dtype=np.float32),anchor=anchor)
