from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from scipy.signal import butter,sosfiltfilt

root,p=map(Path,sys.argv[1:])
path=root/"review/u3220/training-best/flat.npz"
a=np.load(path)
fig,ax=plt.subplots(3,2,figsize=(13,7.4),sharex=True,gridspec_kw={"height_ratios":[1.3,1,1]})
fig.subplots_adjust(left=.085,right=.98,bottom=.085,top=.79,wspace=.27,hspace=.20)
fig.text(.085,.952,"AS2-W  |  Update 3160",fontsize=23,weight="bold")
fig.text(.085,.912,"Front-left calf on the same control clock · PhysX flat ground · trial 4 of 6",fontsize=11,color="#41516a")
t=.02*(np.arange(len(a["q"]))+1)
high=butter(2,5,btype="highpass",fs=50,output="sos")
for col,(key,title) in enumerate((("forward","Forward rolling  |  vx +0.60 m/s"),("left","Left stepping  |  vy +0.30 m/s"))):
    i=np.flatnonzero(a["labels"]==key)[3]
    actual=np.rad2deg(a["q"][:,i,2])
    raw,goal=np.rad2deg(a["target_trace"][:,i,:,2].T)
    load=a["wheel_data"][:,i,:,15]
    ax[0,col].set_title(title,loc="left",fontsize=12,weight="bold")
    ax[0,col].plot(t,raw,color="#9a7258",lw=.9,linestyle="--",label="Clipped target before LPF")
    ax[0,col].plot(t,goal,color="#e27729",lw=1.05,label="Applied target")
    ax[0,col].plot(t,actual,color="#19778c",lw=1.4,label="Measured")
    ax[0,col].set_ylim(-111,-69)
    ax[1,col].plot(t,sosfiltfilt(high,goal),color="#e27729",lw=1,label="Applied target")
    ax[1,col].plot(t,sosfiltfilt(high,actual),color="#19778c",lw=1.2,label="Measured")
    ax[1,col].set_ylim(-11,11)
    for j,(side,color) in enumerate(zip(("FL","FR","RL","RR"),("#286b87","#a7562b","#568651","#806ca8"),strict=True)):
        ax[2,col].plot(t,load[:,j],label=side,color=color,lw=.95 if j==0 else .75,alpha=1 if j==0 else .6)
    ax[2,col].axhline(3,color="#333",lw=.8,ls=":")
    ax[2,col].set_ylim(-5,None)
    ax[2,col].legend(ncol=4,frameon=False,fontsize=8,loc="upper right")
    ax[2,col].set_xlabel("Time since command (s)")
    for row in range(3):
        curr=ax[row,col]
        curr.set_xlim(2,5)
        curr.spines[["top","right"]].set_visible(False)
        curr.grid(alpha=.2)
for i,label in enumerate(("Joint angle (deg)","5 Hz high-pass (deg)","Wheel load (N)")):
    ax[i,0].set_ylabel(label)
handles,labels=ax[0,1].get_legend_handles_labels()
fig.legend(handles,labels,loc="upper right",bbox_to_anchor=(.984,.926),ncol=3,fontsize=9,frameon=False)
fig.savefig(p/"phase-diagnostic-3160.png",dpi=145)
