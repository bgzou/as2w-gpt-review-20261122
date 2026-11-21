import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument("run",type=Path)
parser.add_argument("output",type=Path)
args=parser.parse_args()
paths=(args.run/"baseline/flat.npz",args.run/"review/u3220/training-best/flat.npz")
data=[]
for p in paths:
    with np.load(p) as f:
        data.append({k:f[k] for k in ("labels","joints","q","fall")})
assert np.array_equal(data[0]["labels"],data[1]["labels"])
assert np.array_equal(data[0]["joints"],data[1]["joints"])
joint_names=[str(n) for n in data[0]["joints"][:12]]
time=.02*(np.arange(len(data[0]["q"]))+1)
selected={}
for name in dict.fromkeys(data[0]["labels"]):
    eligible=np.flatnonzero(data[0]["labels"]==name)
    idx=eligible[len(eligible)//2]
    assert not any(run["fall"][:,idx].any() for run in data),name
    selected[name]=(idx,[np.rad2deg(run["q"][:,idx,:12]) for run in data])

groups={
    "lateral":(("left","Left step    vy +0.30 m/s"),("right","Right step    vy -0.30 m/s")),
    "turn":(("yaw_left","Left turn    wz +0.60 rad/s"),("yaw_right","Right turn    wz -0.60 rad/s")),
    "rolling":(("forward","Forward    vx +0.60 m/s"),("reverse","Reverse    vx -0.60 m/s")),
    "stand-mixed":(("stand","Stand    zero command"),("mixed","Mixed    vx +0.45, vy +0.20, wz +0.45")),
}
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.top":False,"axes.spines.right":False})
for title,cases in groups.items():
    fig,axs=plt.subplots(4,6,figsize=(16,8),sharex=True,facecolor="#fbfcfe")
    fig.subplots_adjust(left=.065,right=.988,top=.82,bottom=.09,wspace=.30,hspace=.46)
    fig.text(.065,.945,"Measured joint angles",fontsize=23,weight="bold",color="#18273d")
    fig.text(.065,.901,"AS2-W Unitree-style  |  PhysX nominal flat ground  |  matched trial 4 of 6",fontsize=11,color="#41516a")
    for j in range(12):
        panel_row,part=divmod(j,3)
        values=[x[:,j] for label,_ in cases for x in selected[label][1]]
        low,high=min(v.min() for v in values),max(v.max() for v in values)
        margin=max((high-low)*.07,.2)
        for case_idx,(label,_) in enumerate(cases):
            ax=axs[panel_row,3*case_idx+part]
            baseline,best=selected[label][1]
            ax.set_facecolor("#fbfcfe")
            line1,=ax.plot(time,baseline[:,j],color="#718093",lw=.95,label="1220 seed")
            line2,=ax.plot(time,best[:,j],color="#007a82",lw=1.15,label="3160 return best")
            ax.set_title(joint_names[j].replace("_"," "),loc="left",fontsize=10,weight="bold")
            ax.set_ylim(low-margin,high+margin)
            ax.set_xlim(0,6)
            ax.set_xticks([0,2,4,6])
            ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
            ax.grid(axis="y",color="#dce6ef",lw=.7)
            ax.tick_params(colors="#41516a",labelsize=8)
            if part==0:
                ax.set_ylabel("deg")
            if panel_row==3:
                ax.set_xlabel("Time (s)")
    for i,(_,label) in enumerate(cases):
        fig.text(.065+i*.465,.86,label,fontsize=14,weight="bold",color="#18273d")
    fig.legend(handles=[line1,line2],loc="upper right",bbox_to_anchor=(.989,.96),ncol=2,frameon=False)
    fig.savefig(args.output/f"AS2W-3160-joint-angles-{title}.png",dpi=145)
    plt.close(fig)

with (args.output/"AS2W-3160-joint-angles-timeseries.csv").open("w",newline="") as file:
    writer=csv.writer(file)
    writer.writerow(["scenario","trial","time_s","checkpoint_update"]+[j+"_deg" for j in joint_names])
    for label,(idx,traces) in selected.items():
        for update,q in zip((1220,3160),traces,strict=True):
            writer.writerows([label,4,f"{t:.2f}",update,*[round(v,5) for v in row]] for t,row in zip(time,q,strict=True))

with (args.output/"AS2W-3160-joint-angles-quantiles.csv").open("w",newline="") as file:
    writer=csv.writer(file)
    writer.writerow(["scenario","checkpoint_update","joint","p05_deg","p50_deg","p95_deg","p95_minus_p05_deg"])
    for label in selected:
        for update,run in zip((1220,3160),data,strict=True):
            mask=np.flatnonzero(run["labels"]==label)
            valid=~(np.cumsum(run["fall"][:,mask],axis=0)>0)[50:]
            q=np.rad2deg(run["q"][50:,mask,:12][valid])
            slices=np.quantile(q,[.05,.50,.95],axis=0)
            for name,(lo,median,hi) in zip(joint_names,slices.T,strict=True):
                writer.writerow([label,update,name,round(lo,5),round(median,5),round(hi,5),round(hi-lo,5)])
