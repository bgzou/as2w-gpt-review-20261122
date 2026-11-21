"""Render Isaac Lab physics trajectories by replaying recorded poses."""
from pathlib import Path
import json
import subprocess
import sys
import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation
import torch
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

root=Path(__file__).resolve().parents[3]
out=Path(sys.argv[1])
template=root/"review/u3220/model/as2w_flat.xml"
modes=("baseline","review/u3220/training-best","review/u3220/endpoint")
titles=("1220 / seed","3160 / return best","3220 / endpoint")
accent=("#4ce3b1","#ffab76","#7db4ff")
run=Path(sys.argv[2])
backend=sys.argv[3]
width,height=1536,415
panel_w,panel_h=506,286
fps=25
font_root=Path("/usr/share/fonts/truetype/dejavu")
def font(size,bold=False):
    return ImageFont.truetype(str(font_root/("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")),size)

def scene():
    xml=ET.parse(template).getroot()
    asset=xml.find("asset")
    for element in asset.findall("mesh"):
        old=Path(element.get("file"))
        element.set("file",str(root/"assets/as2w/meshes"/old.name))
    ET.SubElement(asset,"texture",name="grid",type="2d",builtin="checker",width="128",height="128",
                  rgb1=".19 .22 .27",rgb2=".29 .32 .37")
    ET.SubElement(asset,"material",name="gridMat",texture="grid",texrepeat="4 4",texuniform="true",reflectance="0")
    world=xml.find("worldbody")
    floor=world.find("geom[@name='ground']")
    floor.set("material","gridMat")
    if backend=="mujoco":
        install_inertia(world)
        floor.set("friction",".8 .005 .0001")
        for geom in world.findall(".//geom"):
            geom.set("friction",".8 .005 .0001")
        # Use the original training controller at 50 Hz and four 5 ms substeps.
        xml.find("option").set("timestep",".005")
        xml.find("option").set("integrator","implicitfast")
    ET.SubElement(world,"light",pos="1 -1 4",dir="-.2 .2 -1",diffuse=".85 .85 .85",castshadow="true")
    ET.SubElement(world,"light",pos="0 3 3",dir="0 -1 -1",diffuse=".35 .4 .48",castshadow="false")
    ET.SubElement(xml,"visual")
    ET.SubElement(xml.find("visual"),"global",offwidth=str(panel_w),offheight=str(panel_h))
    for g in world.findall(".//geom"):
        if g.get("mesh","").endswith("_foot"):
            g.set("rgba",".24 .28 .33 1")
    return mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding="unicode"))

def install_inertia(world):
    urdf=ET.parse(root/"calibration/candidates/load-v0/as2w-load-v0.urdf")
    parent=world.find("body[@name='base_link']/inertial")
    fixed=urdf.find("joint[@name='JT128']/origin")
    position=np.fromstring(fixed.get("xyz"),sep=" ")
    rotation=Rotation.from_euler("xyz",np.fromstring(fixed.get("rpy"),sep=" ")).as_matrix()
    links=[]
    for name,p,r in (("base_link",np.zeros(3),np.eye(3)),("JT128",position,rotation)):
        spec=urdf.find(f"link[@name='{name}']/inertial")
        origin=spec.find("origin")
        relative=Rotation.from_euler("xyz",np.fromstring(origin.get("rpy"),sep=" ")).as_matrix()
        align=r@relative
        mat=spec.find("inertia")
        ixx,iyy,izz,ixy,ixz,iyz=(float(mat.get(k)) for k in ("ixx","iyy","izz","ixy","ixz","iyz"))
        tensor=np.array(((ixx,ixy,ixz),(ixy,iyy,iyz),(ixz,iyz,izz)))
        center=p+r@np.fromstring(origin.get("xyz"),sep=" ")
        links.append((float(spec.find("mass").get("value")),center,align@tensor@align.T))
    total=sum(m for m,_,_ in links)
    com=sum(m*p for m,p,_ in links)/total
    tensor=np.zeros((3,3))
    for mass,center,moment in links:
        loc=center-com
        tensor+=moment+mass*((loc@loc)*np.eye(3)-np.outer(loc,loc))
    parent.attrib.clear()
    parent.set("mass",str(total))
    parent.set("pos"," ".join(map(str,com)))
    parent.set("fullinertia"," ".join(str(tensor[i,j]) for i,j in ((0,0),(1,1),(2,2),(0,1),(0,2),(1,2))))

chapters=[
    ("forward","Forward",(.60,0.,0.)),
    ("reverse","Reverse",(-.60,0.,0.)),
    ("left","Step left",(0.,.30,0.)),
    ("right","Step right",(0.,-.30,0.)),
    ("yaw_left","Turn left",(0.,0.,.60)),
    ("yaw_right","Turn right",(0.,0.,-.60)),
]
legs=tuple(f"{side}_{kind}" for side in ("FL","FR","RL","RR") for kind in ("hip","thigh","calf"))
wheels=tuple(f"{side}_foot" for side in ("FL","FR","RL","RR"))
joints=legs+wheels
home=np.tile([0.,.8,-1.6],4)
alpha_leg=1.-np.exp(-2*np.pi*10*.02)
alpha_wheel=1.-np.exp(-2*np.pi*5*.02)
timesteps=300
torch.set_num_threads(1)
model=scene()
q_ids=[int(model.joint(name).qposadr[0]) for name in joints]
v_ids=[int(model.joint(name).dofadr[0]) for name in joints]
drives=[model.actuator(name).id for name in joints]
foot_bodies={model.body(name).id:i for i,name in enumerate(wheels)}
base=model.body("base_link").id
ground=model.geom("ground").id
contact_force=np.zeros(6)
effort=model.actuator_ctrlrange[drives,1]

def actor(path):
    sd=torch.load(path,map_location="cpu",weights_only=True)["actor_state_dict"]
    layers=[k for k in range(0,8,2)]
    @torch.inference_mode()
    def infer(obs):
        x=torch.as_tensor(obs,dtype=torch.float32)
        for k in layers:
            x=torch.nn.functional.linear(x,sd[f"mlp.{k}.weight"],sd[f"mlp.{k}.bias"])
            if k<layers[-1]:
                x=torch.nn.functional.elu(x)
        return x.clamp(-1.,1.).numpy()
    print(path.parent.name,"std",float(sd["distribution.std_param"].mean()),flush=True)
    return infer

def rollout(policy,command):
    data=mujoco.MjData(model)
    data.qpos[:3]=[0,0,.390]
    data.qpos[q_ids[:12]]=home
    mujoco.mj_forward(model,data)
    target=np.zeros(16)
    obsvel=np.zeros(6)
    history={k:[] for k in ("pose","q","vel","tilt","target_trace","load","fall")}
    for i in range(-50,timesteps):
        cmd=command if i>=0 else np.zeros(3)
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_XBODY,base,obsvel,1)
        gravity=-data.xmat[base].reshape(3,3)[2]
        pos=np.r_[data.qpos[q_ids[:12]]-home,np.zeros(4)]
        obs=np.r_[cmd,obsvel[:3],gravity,pos,data.qvel[v_ids]*.05,target[:12]/.25,target[12:]/12.,.37]
        action=policy(obs)*.25
        target[:12]+=alpha_leg*(action-target[:12])
        wheel=np.clip(cmd[0]/.086,-12,12)
        target[12:]+=alpha_wheel*(wheel-target[12:])
        if wheel==0:
            target[12:][np.abs(target[12:])<.02]=0.
        load=np.zeros(4)
        for _ in range(4):
            current_pos=data.qpos[q_ids[:12]]
            current_vel=data.qvel[v_ids]
            torque=np.r_[55*(home+target[:12]-current_pos)-2*current_vel[:12],
                         2*(target[12:]-current_vel[12:])]
            data.ctrl[drives]=np.clip(torque,-effort,effort)
            mujoco.mj_step(model,data)
            for n,contact in enumerate(data.contact):
                if ground not in contact.geom:
                    continue
                wheel_geom=contact.geom[1] if contact.geom[0]==ground else contact.geom[0]
                foot=foot_bodies.get(int(model.geom_bodyid[wheel_geom]))
                if foot is not None:
                    mujoco.mj_contactForce(model,data,n,contact_force)
                    load[foot]+=max(0.,contact_force[0])*abs(contact.frame[2])/4
        if i<0:
            continue
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_XBODY,base,obsvel,1)
        upright=data.xmat[base,8]
        for k,value in zip(history,(data.qpos[:7],data.qpos[q_ids],obsvel[[3,4,2]],
                                    np.rad2deg(np.arccos(np.clip(upright,-1,1))),
                                    (home+action,home+target[:12]),load,upright<.52),strict=True):
            history[k].append(np.array(value))
    return {k:np.array(v) for k,v in history.items()}

def trial_from_physx(trace,key):
    indices=np.flatnonzero(trace["labels"]==key)
    # Select the middle trial in original evaluation order for every weight.
    i=indices[len(indices)//2]
    values={k:trace[k][:,i].copy() for k in ("q","pose","vel","tilt","fall","target_trace")}
    values["pose"]=values["pose"][:,[0,1,2,6,3,4,5]]
    values["load"]=trace["wheel_data"][:,i,:,15].copy()
    return values

if backend=="mujoco":
    actors=[actor(run/m/"best.pt") for m in modes]
else:
    traces=[]
    for m in modes:
        with np.load(run/m/"flat.npz") as a:
            assert tuple(a["joints"])==joints
            traces.append({k:a[k] for k in a.files})
output=out/f"AS2W-1220-3160-3220-{backend}.mp4"
encoder=subprocess.Popen(
    ["ffmpeg","-hide_banner","-loglevel","error","-y","-f","rawvideo","-pix_fmt","rgb24",
     "-s",f"{width}x{height+1}","-r",str(fps),"-i","-","-an","-c:v","libx264","-preset","fast",
     "-crf","21","-pix_fmt","yuv420p","-movflags","+faststart",str(output)],
    stdin=subprocess.PIPE)
reports={}
try:
    with mujoco.Renderer(model,height=panel_h,width=panel_w) as renderer:
        data=mujoco.MjData(model)
        camera=mujoco.MjvCamera()
        camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        camera.azimuth=128
        camera.elevation=-19
        camera.distance=1.60
        for key,label,cmd in chapters:
            if backend=="mujoco":
                samples=[rollout(policy,cmd) for policy in actors]
            else:
                samples=[trial_from_physx(trace,key) for trace in traces]
            falls=[next(iter(np.flatnonzero(a["fall"])),None) for a in samples]
            reports[key]=[]
            for title,a,fall in zip(titles,samples,falls,strict=True):
                n=len(a["q"]) if fall is None else int(fall)
                scored=slice(50,n)
                reports[key].append(dict(checkpoint=title,fall_s=None if fall is None else round(float(fall)*.02,2),
                         four_wheel_pct=float((a["load"][scored]>=3).all(1).mean()*100) if n>50 else None,
                         mean_v=a["vel"][scored].mean(0).tolist() if n>50 else None,
                         joint_span_deg=np.rad2deg(np.diff(np.quantile(a["q"][scored,:12],[.05,.95],axis=0),axis=0)[0]).tolist() if n>50 else None))
            for idx in range(0,timesteps,2):
                frame=Image.new("RGB",(width,height+1),"#10151e")
                draw=ImageDraw.Draw(frame)
                draw.text((10,7),label,font=font(22,True),fill="#f5f8ff")
                draw.text((182,10),f"cmd: vx {cmd[0]:+.2f}  vy {cmd[1]:+.2f} m/s  wz {cmd[2]:+.2f} rad/s",font=font(17),fill="#adbbcf")
                engine="MuJoCo closed-loop / load-v0 inertia" if backend=="mujoco" else "Isaac Lab / PhysX recorded poses rendered in MuJoCo"
                draw.text((930,10),f"{engine}   t={idx*.02:.2f}s",font=font(16),fill="#e5edf9")
                for j,(title,color,a,fall) in enumerate(zip(titles,accent,samples,falls,strict=True)):
                    at=idx if fall is None else min(idx,max(0,fall-1))
                    pose=a["pose"][at]
                    data.qpos[:7]=pose
                    data.qpos[q_ids]=a["q"][at]
                    mujoco.mj_forward(model,data)
                    camera.lookat[:]=[pose[0],pose[1],.36]
                    renderer.update_scene(data,camera=camera)
                    x=j*512+3
                    frame.paste(Image.fromarray(renderer.render()),(x,36))
                    draw.rectangle((x,36,x+panel_w,62),fill="#18202c")
                    draw.text((x+7,40),title,font=font(17,True),fill=color)
                    v=a["vel"][at]
                    draw.text((x+8,329),f"vx {v[0]:+.2f}    vy {v[1]:+.2f}    wz {v[2]:+.2f}     tilt {a['tilt'][at]:.1f} deg",font=font(16),fill="#dce6f5")
                    for w,(leg,load) in enumerate(zip(("FL","FR","RL","RR"),a["load"][at],strict=True)):
                        start=x+8+w*125
                        draw.text((start,354),f"{leg} {load:3.0f} N",font=font(16),fill=color if load>=3 else "#aab6c9")
                        draw.rectangle((start,376,start+min(112,max(0,load)),379),fill=color if load>=3 else "#63758a")
                    if fall is not None and idx>=fall:
                        draw.rectangle((x+132,157,x+370,194),fill="#7e2731")
                        draw.text((x+147,163),f"FALL  {float(fall)*.02:.2f}s",font=font(21,True),fill="#fff")
                draw.text((10,397),"Wheel bars are vertical normal force; highlighted at 3 N and above. Each clip is an independent nominal trial.",font=font(14),fill="#a9b8cb")
                encoder.stdin.write(frame.tobytes())
            print(backend,key,"falls",falls,flush=True)
finally:
    encoder.stdin.close()
    if encoder.wait():
        raise RuntimeError("encoding failed")
(out/f"{backend}-summary.json").write_text(json.dumps(reports,indent=2)+"\n")
