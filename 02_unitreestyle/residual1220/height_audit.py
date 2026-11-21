"""Flat-ground acceptance for the optional body and swing height changes."""
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

from height import CASES

def measure(path):
    with np.load(path/"flat.npz") as a:
        body=a["pose"][...,2]
        w=a["wheel_data"]
        valid=~np.maximum.accumulate(a["fall"],axis=0)
        valid[:50]=False
        labels=a["labels"]
        clearance=np.empty(w.shape[:3])
        for i,name in enumerate(a["joints"][-4:]):
            mesh=Path(__file__).resolve().parents[1]/"assets/as2w/meshes"/f"{name}.STL"
            v=trimesh.load_mesh(mesh,process=True).vertices
            hull=v[ConvexHull(v).vertices]
            wheel=w[:,:,i,:].reshape(-1,17)
            up=Rotation.from_quat(wheel[:,3:7]).as_matrix()[:,2,:]
            low=np.concatenate([(u@hull.T).min(axis=1) for u in np.array_split(up,32)])
            clearance[:,:,i]=(wheel[:,2]+low).reshape(body.shape)
        result={}
        for case in CASES:
            ii=np.flatnonzero(labels==case)
            z=body[:,ii][valid[:,ii]]
            med=[];counts=[]
            for i in range(4):
                peaks=[]
                for j in ii:
                    air=(w[:,j,i,15]<3.) & valid[:,j]
                    edges=np.diff(np.r_[False,air,False].astype(int))
                    for a0,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
                        if b-a0<3 or a0==0 or b==len(air) or not valid[[a0-1,b],j].all():
                            continue
                        peak=float(clearance[a0:b,j,i].max())
                        if peak>.01:
                            peaks.append(peak)
                counts.append(len(peaks))
                med.append(float(np.median(peaks)) if peaks else None)
            result[case]=dict(body_m=float(np.median(z)) if len(z) else None,
                              peak_m=med,complete_swings=counts)
        return result

def assess(base,candidate):
    old,new=measure(base),measure(candidate)
    failures=[]
    for case in CASES:
        a,b=old[case],new[case]
        if a["body_m"] is None or b["body_m"] is None or not .005<=b["body_m"]-a["body_m"]<=.020:
            failures.append(case+": body height outside +5 to +20 mm")
        if case not in CASES[3:7]:
            continue
        compared=0
        for i,n in enumerate(a["complete_swings"]):
            if n<6:
                continue
            compared+=1
            if b["complete_swings"][i]<.75*n or b["peak_m"][i] is None or not .005<=b["peak_m"][i]-a["peak_m"][i]<=.025:
                failures.append(f"{case}: wheel {i} swing count or peak height")
        if not compared:
            failures.append(case+": insufficient reference swing events")
    return dict(baseline=old,candidate=new,failures=failures)
