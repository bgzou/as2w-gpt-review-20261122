import json
from pathlib import Path
import sys
import numpy as np
import trimesh
from scipy.signal import periodogram
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

p=Path(sys.argv[1])
with np.load(p/"flat.npz") as f:
    a={k:f[k] for k in f.files}
labels=a["labels"]
w=a["wheel_data"]
res=[]
for i,name in enumerate(a["joints"][-4:]):
    path=Path(__file__).resolve().parents[1] / "assets/as2w/meshes"/f"{name}.STL"
    vertices=trimesh.load_mesh(path,process=True).vertices
    vertices=vertices[ConvexHull(vertices).vertices]
    per=w[:,:,i,:].reshape(-1,17)
    rotation=Rotation.from_quat(per[:,3:7]).as_matrix()
    rel=np.einsum("nci,vi->nvc",rotation,vertices)
    z=rel[:,:,2]
    lowest=z.min(axis=1)
    select=z<=lowest[:,None]+.002
    contact_point=(rel*select[:,:,None]).sum(axis=1)/select.sum(axis=1)[:,None]
    point_velocity=per[:,7:10]+np.cross(per[:,10:13],contact_point)
    axis=rotation[:,:2,1]
    axis/=np.linalg.norm(axis,axis=1)[:,None].clip(min=1e-6)
    direction=np.stack((axis[:,1],-axis[:,0]),axis=-1)
    slip_lat=(point_velocity[:,:2]*axis).sum(axis=1)
    slip_lon=(point_velocity[:,:2]*direction).sum(axis=1)
    res.append(np.stack((per[:,2]+lowest,slip_lat,slip_lon),1).reshape(*w.shape[:2],3))
clear,lateral,longitudinal=np.moveaxis(np.stack(res,2),-1,0)
normal=w[...,15]
supported=normal>=3.
reports={}
for label in dict.fromkeys(labels):
    ids=labels==label
    valid=~(np.cumsum(a["fall"][:,ids],axis=0)>0)
    valid[:50]=False
    wheels=valid[:,:,None] & supported[:,ids]
    n=supported[:,ids].sum(axis=2)[valid]
    tilt=a["tilt"][:,ids][valid]
    q=a["pose"][:,ids,3:]
    x,y,z,t=np.moveaxis(q,-1,0)
    roll=np.arctan2(2*(t*x+y*z),1-2*(x*x+y*y))
    pitch=np.arcsin(np.clip(2*(t*y-z*x),-1,1))
    rates=np.rad2deg(np.hypot(np.diff(roll,axis=0),np.diff(pitch,axis=0)))/.02
    speed=a["vel"][:,ids][valid]
    def qtile(values,qs):
        if not values.size:
            return None
        return np.round(np.quantile(values,qs),4).tolist()
    lift=(clear[:,ids]>.01)&(~supported[:,ids])&valid[:,:,None]
    durations=[]
    counts=[]
    for foot in range(4):
        per_foot=[]
        for env in range(lift.shape[1]):
            transitions=np.diff(np.r_[False,lift[:,env,foot],False].astype(int))
            runs=np.flatnonzero(transitions==-1)-np.flatnonzero(transitions==1)
            per_foot.extend(runs[runs>=3]*.02)
        durations.extend(per_foot)
        counts.append(len(per_foot))
    angles=qtile(a["q"][:,ids,:12][valid],[.05,.5,.95])
    body=qtile(np.rad2deg(np.stack([roll,pitch],axis=2)[valid]),[.05,.95])
    survivors=valid[-1]
    f_hz=[None]*14
    if survivors.any():
        signal=np.concatenate([a["q"][50:,ids,:12],roll[50:,:,None],pitch[50:,:,None]],axis=2)[:,survivors]
        freq,power=periodogram(signal,fs=50.,window="hann",detrend="linear",axis=0)
        band=(freq>=.6)&(freq<=20.)
        peak=freq[band][np.argmax(power[band],axis=0)]
        dom=np.median(peak,axis=0)
        lo,hi=np.quantile(signal,[.05,.95],axis=(0,1))
        f_hz=[round(float(f),2) if high-low>=.03 else None for f,low,high in zip(dom,lo,hi,strict=True)]
    phase_metrics={}
    if "swing_lift_ref_m" in a:
        # The 9 mm reference and 5 N load match the training reward masks.
        cmd=a["cmd"][:,ids]
        gait_active=(np.abs(cmd[...,1])+.28*np.abs(cmd[...,2]))>.04
        desired=(a["swing_lift_ref_m"][:,ids]>.009)&(valid&gait_active)[:,:,None]
        loaded=normal[:,ids]>=5.
        clearance=clear[:,ids]>.01
        desired_frames=desired.sum(axis=(0,1))
        def pct_of_reference(mask):
            counts=(desired&mask).sum(axis=(0,1))
            return [round(100.*count/total,2) if total else None
                    for count,total in zip(counts,desired_frames,strict=True)]
        phase_metrics={
            "reference_swing_frames_FL_FR_RL_RR":desired_frames.tolist(),
            "loaded_during_reference_swing_pct_FL_FR_RL_RR":pct_of_reference(loaded),
            "unloaded_and_clear_during_reference_swing_pct_FL_FR_RL_RR":
                pct_of_reference(~loaded&clearance),
        }
    reports[str(label)]={
        "survived":int(valid[-1].sum()),
        "trials":int(ids.sum()),
        "mean_body_vx_vy_wz":np.round(speed.mean(0),4).tolist() if speed.size else None,
        "valid_frames":int(valid.sum()), "spectrum_trials":int(survivors.sum()),
        "tilt_p95_deg":qtile(tilt,.95),
        "roll_pitch_rate_p95_deg_s":qtile(rates[valid[1:]&valid[:-1]],.95),
        "support_pct_0_to_4":np.round(np.bincount(n,minlength=5)*100/len(n),2).tolist() if len(n) else None,
        "nonwheel_contact_frames":int(a["nonwheel"][:,ids][valid].sum()),
        "loaded_mesh_min_p05_mm":qtile(1000*clear[:,ids][wheels],[0,.05]),
        "abs_lateral_slip_median_p95_m_s":qtile(np.abs(lateral[:,ids][wheels]),[.5,.95]),
        "abs_longitudinal_slip_median_p95_m_s":qtile(np.abs(longitudinal[:,ids][wheels]),[.5,.95]),
        "measured_wheel_speed_abs_p95_rad_s":qtile(np.abs(w[...,16][:,ids][valid]),.95),
        "lift_event_counts_FL_FR_RL_RR":counts,
        "lift_events_Hz_per_wheel_FL_FR_RL_RR":[round(n/(ids.sum()*5),2) for n in counts],
        "joint_angles_p05_p50_p95_rad":angles,
        "joint_dominant_Hz":f_hz[:12],
        "roll_pitch_dominant_Hz":f_hz[12:],
        "body_roll_pitch_p05_p95_deg":body,
        "lift_duration_median_s":round(float(np.median(durations)),3) if durations else None,
        **phase_metrics,
   }
(p/"flat-motion-audit.json").write_text(json.dumps(reports,indent=2,default=float)+"\n")
for key,value in reports.items(): print(key,json.dumps(value,default=float))
