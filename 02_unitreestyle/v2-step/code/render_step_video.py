"""Visual-only replay of saved PhysX trajectories. No MuJoCo physics integration."""
from pathlib import Path
import argparse,hashlib,json,subprocess,xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont

p=argparse.ArgumentParser()
for key in ("data","checkpoint","template","meshes","out"):
 p.add_argument("--"+key,type=Path,required=True)
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
with np.load(a.data) as f:d={k:f[k] for k in f.files}
xml=ET.parse(a.template).getroot()
for mesh in xml.find("asset").findall("mesh"):
 mesh.set("file",str(a.meshes/Path(mesh.get("file")).name))
asset=xml.find("asset")
ET.SubElement(asset,"texture",name="replay_grid",type="2d",builtin="checker",width="128",height="128",rgb1=".20 .23 .27",rgb2=".30 .33 .37")
ET.SubElement(asset,"material",name="replay_grid_mat",texture="replay_grid",texrepeat="4 4",texuniform="true")
world=xml.find("worldbody")
world.find("geom[@name='ground']").set("material","replay_grid_mat")
ET.SubElement(world,"light",pos="1 -2 4",dir="-.2 .2 -1",diffuse=".9 .9 .9",castshadow="true")
ET.SubElement(world,"light",pos="0 3 3",dir="0 -1 -1",diffuse=".4 .4 .45",castshadow="false")
visual=xml.find("visual")
if visual is None:visual=ET.SubElement(xml,"visual")
ET.SubElement(visual,"global",offwidth="640",offheight="400")
m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding="unicode"))
state=mujoco.MjData(m)
jids=np.array([int(m.joint(str(j)).qposadr[0]) for j in d["joints"]])
bids=np.array([m.body(n+"_foot").id for n in ("FL","FR","RL","RR")])
def set_pose(frame,idx):
 pose=d["pose"][frame,idx]
 state.qpos[:3]=pose[:3];state.qpos[3:7]=pose[[6,3,4,5]]
 state.qpos[jids]=d["q"][frame,idx]
 mujoco.mj_forward(m,state)
 return pose
errors=[]
for frame in range(0,len(d["q"]),10):
 for idx in range(len(d["labels"])):
  set_pose(frame,idx)
  errors.extend(np.linalg.norm(state.xpos[bids]-d["wheel_data"][frame,idx,:,:3],axis=-1).tolist())
kinematics={"max_wheel_link_position_error_m":float(max(errors)),"p99_error_m":float(np.quantile(errors,.99))}
assert max(errors)<.005,kinematics
print("KINEMATICS_OK",kinematics,flush=True)
renderer=mujoco.Renderer(m,width=640,height=400)
fontdir=Path("/usr/share/fonts/truetype/dejavu")
def font(n,bold=False):return ImageFont.truetype(str(fontdir/("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")),n)
chapters=("left","right","yaw_left","yaw_right")
video=a.out/"AS2W-step-policy-3of4.mp4"
encoder=subprocess.Popen(["ffmpeg","-hide_banner","-loglevel","error","-y","-f","rawvideo","-pix_fmt","rgb24","-s","1280x720","-r","25","-i","-","-c:v","libx264","-threads","4","-preset","fast","-crf","19","-pix_fmt","yuv420p","-movflags","+faststart",str(video)],stdin=subprocess.PIPE)
meta={"checkpoint_sha256":hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
      "trajectory_sha256":hashlib.sha256(a.data.read_bytes()).hexdigest(),
      "physics":"Isaac Lab / PhysX recorded trajectory","renderer":"MuJoCo visual-only forward kinematics",
      "mujoco_version":mujoco.__version__,"policy_label":"step_policy 45b36ee7 (left/yaw_left/yaw_right 4/4, right 3/4)",
      "status":"3 of 4 lateral directions qualified; right RR is a geometry limit","seed":319,"trial_rule":"first of six trials, fixed in advance",
      "terrain":"nominal flat","fps":25,"speed":1,"kinematics":kinematics,"chapters":[]}
try:
 for block,label in enumerate(chapters):
  idx=int(np.flatnonzero(d["labels"]==label)[0])
  cmd=d["cmd"][0,idx]
  meta["chapters"].append({"mode":label,"start_s":block*6,"trial_index":idx,"command":cmd.tolist()})
  for k in range(0,300,2):
   pose=set_pose(k,idx)
   frame=Image.new("RGB",(1280,720),"#101722");draw=ImageDraw.Draw(frame)
   draw.text((16,7),f"{block+1:02d}  {label}    |    step policy (hardest endpoint)",font=font(25,True),fill="#edf4ff")
   draw.text((16,41),f"command vx {cmd[0]:+.2f} m/s   vy {cmd[1]:+.2f} m/s   wz {cmd[2]:+.2f} rad/s",font=font(18),fill="#b9c8df")
   draw.text((926,12),"left/yaw 4/4 | right 3/4",font=font(19,True),fill="#ffae76")
   draw.text((926,43),f"1x speed | t={k*.02:.2f}s | trial 0",font=font(16),fill="#bcc9dc")
   for side,azimuth in enumerate((127,220)):
    cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth=azimuth;cam.elevation=-18;cam.distance=1.75
    cam.lookat[:]=pose[:3]+[0,0,-.04]
    renderer.update_scene(state,camera=cam)
    frame.paste(Image.fromarray(renderer.render()),(side*640,80))
    draw.rectangle((side*640,80,side*640+640,109),fill="#192333")
    draw.text((side*640+12,85),"front oblique" if side==0 else "side / rear oblique",font=font(17,True),fill="#75dbbe")
   force=d["normal_load"][k,idx];q=np.rad2deg(d["q"][k,idx,:12])
   vel=d["vel"][k,idx]
   draw.text((16,491),f"actual vx {vel[0]:+.2f}   vy {vel[1]:+.2f}   wz {vel[2]:+.2f}     tilt {d['tilt'][k,idx]:.1f} deg     loaded wheels {int((force>=3).sum())}/4",font=font(20,True),fill="#e5eefb")
   for w,name in enumerate(("FL","FR","RL","RR")):
    x=16+(w%2)*640;y=534+(w//2)*55
    color="#6ae0b4" if force[w]>=3 else "#ffad71"
    draw.text((x,y),f"{name}  {force[w]:6.1f} N",font=font(19,True),fill=color)
    draw.text((x+170,y),f"hip {q[w*3]:+6.1f}  thigh {q[w*3+1]:+6.1f}  calf {q[w*3+2]:+6.1f} deg",font=font(17),fill="#d1ddec")
   draw.text((16,653),"Isaac PhysX trajectory | MuJoCo visual replay (no physics stepping) | flat | seed 319",font=font(16),fill="#acbcd3")
   draw.text((16,684),"Wheel normal forces and joint angles are recorded Isaac data; green contact threshold = 3 N.",font=font(15),fill="#acbcd3")
   if block==1 and k==100:frame.save(a.out/"preview.jpg",quality=92)
   encoder.stdin.write(frame.tobytes())
  print("RENDERED",label,"trial",idx,flush=True)
finally:
 encoder.stdin.close();code=encoder.wait();renderer.close()
 if code:raise RuntimeError(f"ffmpeg failed {code}")
(a.out/"video-provenance.json").write_text(json.dumps(meta,indent=2)+"\n")
print("VIDEO_OK",video,flush=True)
