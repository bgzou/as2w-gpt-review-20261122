"""MuJoCo closed-loop evaluation of the AS2-W HIMLoco DR policy (model_18200)."""
import argparse, math, sys
import numpy as np, mujoco
sys.path.insert(0, "/root/as2w/him_arc")
from deploy_ref_controller import Controller

DEF = np.array([0., .8, -1.6] * 4 + [0.] * 4, dtype=np.float32)
POL2MJ = [leg * 4 + p for leg in range(4) for p in range(3)]
FOOT2MJ = [j * 4 + 3 for j in range(4)]
DT = 0.02

ap = argparse.ArgumentParser()
ap.add_argument("--vx", type=float, default=0.0)
ap.add_argument("--wz", type=float, default=0.0)
ap.add_argument("--seconds", type=float, default=8.0)
ap.add_argument("--policy", default="/root/as2w/deploy/model_18200/policy.onnx")
ap.add_argument("--scene", default="scene_v2.xml")
ap.add_argument("--csv", default="")
ap.add_argument("--out", default="")
ap.add_argument("--width", type=int, default=640)
ap.add_argument("--height", type=int, default=480)
a = ap.parse_args()

m = mujoco.MjModel.from_xml_path(a.scene)
d = mujoco.MjData(m)
sub = int(round(DT / m.opt.timestep))
mujoco.mj_resetData(m, d)
d.qpos[0:3] = [0, 0, 0.33]
d.qpos[3:7] = [1, 0, 0, 0]
for i in range(12):
    d.qpos[7 + POL2MJ[i]] = DEF[i]
mujoco.mj_forward(m, d)

ctl = Controller(a.policy, DEF)
cmd = np.array([a.vx, 0.0, a.wz], dtype=np.float32)
rows = []
import subprocess
ff = None; ren = None; cam = None
if a.out:
    ff = subprocess.Popen(["ffmpeg","-y","-f","rawvideo","-pix_fmt","rgb24","-s",f"{a.width}x{a.height}",
                           "-r","25","-i","-","-an","-vcodec","libx264","-pix_fmt","yuv420p","-crf","23",a.out],
                          stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ren = mujoco.Renderer(m, a.height, a.width)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING; cam.trackbodyid = 1
    cam.distance = 1.8; cam.elevation = -18; cam.azimuth = 130
steps = int(a.seconds / DT)
for k in range(steps):
    q = np.zeros(16, dtype=np.float32); dq = np.zeros(16, dtype=np.float32)
    for i in range(12):
        q[i] = d.qpos[7 + POL2MJ[i]]; dq[i] = d.qvel[6 + POL2MJ[i]]
    for j in range(4):
        q[12 + j] = 0.0; dq[12 + j] = d.qvel[6 + FOOT2MJ[j]]
    w, x, y, z = d.qpos[3:7]
    R = np.array([[1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                  [2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x)],
                  [2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y)]])
    g = (R.T @ np.array([0, 0, -1.])).astype(np.float32)
    res = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_XBODY, 1, res, 1)
    obs = ctl.obs(cmd, res[0:3].astype(np.float32), g, q, dq)
    out = ctl.act(obs, cmd)
    for i in range(12):
        d.ctrl[i] = out["leg_position"][i]   # actuator index, not joint index
    for j in range(4):
        d.ctrl[12 + j] = out["wheel_velocity"][j]   # actuators 12..15 = wheels
    for _ in range(sub):
        mujoco.mj_step(m, d)
    pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    rows.append([k*DT, d.qpos[0], d.qpos[1], d.qpos[2], d.qvel[0], pitch, yaw,
                 out["wheel_velocity"][0], d.qvel[6+FOOT2MJ[0]]])
    if ff is not None and k % 2 == 0:
        ren.update_scene(d, cam); ff.stdin.write(ren.render().tobytes())

arr = np.array(rows)
steady = arr[arr[:, 0] > 2.0]
print(f"RESULT vx={a.vx:+.2f} wz={a.wz:+.2f} "
      f"x_end={arr[-1,1]:+.2f} y_end={arr[-1,2]:+.2f} z_end={arr[-1,3]:.3f} "
      f"z_min={arr[:,3].min():.3f} z_max={arr[:,3].max():.3f} "
      f"pitch_max={np.abs(arr[:,5]).max():.1f}deg "
      f"yaw_end={arr[-1,6]:+.0f}deg vx_mean={steady[:,4].mean():+.3f} wheel_dq_mean={steady[:,8].mean():+.2f}")
if a.csv:
    np.savetxt(a.csv, arr, delimiter=",", header="t,x,y,z,vx,pitch_deg,yaw_deg,wcmd,wheel_dq", comments="")
if ff is not None:
    ff.stdin.close(); ff.wait()
