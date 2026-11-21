"""Generate the field contract from constants used by the runtime."""
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from runtime import DDS_SLOT, DEFAULT, DT, JOINTS, KD, KP, LEG_ALPHA, MAX_COMMAND, RADIUS, STATE_SIGN, WHEEL_ALPHA

def field(i,name,unit,source,transform):
    return dict(index=i,name=name,unit=unit,source=source,transform=transform)

def contract(original_urdf):
    limits={j.get("name"):j.find("limit").attrib for j in ET.parse(original_urdf).findall("joint") if j.find("limit") is not None}
    motors=[dict(joint=j,dds_state_slot=int(DDS_SLOT[i]),dds_state_sign=int(STATE_SIGN[i]),
                 default_rad=round(float(DEFAULT[i]),6),kp=float(KP[i]),kd=float(KD[i]),tau_ff_Nm=0.,
                 effort_limit_sim_Nm=float(limits[j]["effort"]),velocity_limit_sim_rad_s=float(limits[j]["velocity"]))
             for i,j in enumerate(JOINTS)]
    obs=[field(i,n,u,"served command (body +x forward, +y left, +z up)","identity")
         for i,(n,u) in enumerate((("vx","m/s"),("vy","m/s"),("wz","rad/s")))]
    obs += [field(3+i,"gyro_"+a,"rad/s","IMU gyro in body frame","identity") for i,a in enumerate("xyz")]
    obs += [field(6+i,"projected_gravity_"+a,"unit","IMU body-to-world wxyz","inverse rotate world [0,0,-1]")
            for i,a in enumerate("xyz")]
    for i,joint in enumerate(JOINTS):
        obs += [field(9+i,joint+"_q_relative","rad","LowState mapped q","q minus the default" if i<12 else "constant zero")]
        obs += [field(25+i,joint+"_dq_scaled","scaled rad/s","LowState mapped dq","scale 0.05")]
        obs += [field(41+i,joint+"_previous_filtered_target","unitless","previous software wrapper output",
                      "divide offset by 0.25 rad" if i<12 else "divide wheel speed by 12 rad/s")]
    obs += [field(57,"height_constant","m","training constant","0.37")]
    return dict(onnx={"opset":17,"input":["observation",["batch",58],"float32"],
                      "output":["leg_action",["batch",12],"float32"],"hidden_state":False},
                policy_dt_s=DT,physics_dt_s=.005,command_limit_si=[round(float(v),2) for v in MAX_COMMAND],
                features=sorted(obs,key=lambda row:row["index"]),runtime_environment_overrides={},
                training_reset_default_root_height_m=.39, motors=motors, nominal_legs_kp_kd=[55,2],nominal_wheels_kp_kd=[0,2],
                leg_scale_rad=.25,clip_policy=[-1,1],wheel_target_limit_rad_s=12.,
                wheel_radius_m=round(float(RADIUS),3),wheel_snap_zero_threshold_rad_s=.02,
                filter={"kind":"matched pole","leg_hz":10,"wheel_hz":5,"leg_alpha":float(LEG_ALPHA),"wheel_alpha":float(WHEEL_ALPHA)},
                policy_axis_order=list(JOINTS),imu_quat="body_to_world wxyz",control_features="nominal chain",
                deployment_sensor_age_limit_s=.04,zero_offset_source="site calibration provided by deployer",
                action_offset_reference="environment.py init_state.joint_pos (same as observation reference)",
                policy_entry="external stand handoff; initialize prior leg offset as measured q minus its default and wheel prior to zero",
                policy_exit="outer machine owns the final hold or damping transition and motor stream")
if __name__=="__main__":
    out=contract(sys.argv[1])
    assert len(out["features"])==58
    Path(sys.argv[2]).write_text(json.dumps(out,indent=2)+"\n")
