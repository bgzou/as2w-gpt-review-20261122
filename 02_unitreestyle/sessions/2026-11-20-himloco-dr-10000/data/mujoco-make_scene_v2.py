"""Rebuild the AS2-W MJCF with stable implicit actuators.

Legs: <position kp=55 kv=2>   (kv enters the implicit damping term)
Wheels: <velocity kv=2>       (implicit velocity servo; the old explicit
                               torque loop was unstable at 1.25 ms)
Wheel joints get a little armature/damping so the servo has finite bandwidth.
"""
import re, mujoco

src = open("as2w_flat.xml").read()

# wheel joints: armature + damping
src = re.sub(r'<joint name="(\w+_foot)" actuatorfrcrange="-15 15" axis="0 1 0" />',
             r'<joint name="\1" actuatorfrcrange="-15 15" axis="0 1 0" armature="0.01" damping="0.05" />',
             src)

legs = ["FL_hip","FL_thigh","FL_calf","FR_hip","FR_thigh","FR_calf",
        "RL_hip","RL_thigh","RL_calf","RR_hip","RR_thigh","RR_calf"]
feet = ["FL_foot","FR_foot","RL_foot","RR_foot"]
leg_xml = "".join(
    f'<position name="a_{j}" joint="{j}" kp="55" kv="2" ctrllimited="true" ctrlrange="-3 3" />'
    for j in legs)
foot_xml = "".join(
    f'<velocity name="a_{j}" joint="{j}" kv="2" ctrllimited="true" ctrlrange="-12 12" />'
    for j in feet)
src = re.sub(r'<actuator>.*?</actuator>', f'<actuator>{leg_xml}{foot_xml}</actuator>', src, flags=re.S)
open("scene_v2.xml", "w").write(src)

m = mujoco.MjModel.from_xml_path("scene_v2.xml")
print("SCENE_OK nq", m.nq, "nv", m.nv, "nu", m.nu, "ngeom", m.ngeom, "dt", m.opt.timestep)
print("act0", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 0),
      "act12", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 12))
