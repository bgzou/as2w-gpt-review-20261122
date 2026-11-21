"""AS2-W 50 Hz native controller math and CPRO state adapter."""
from pathlib import Path
import argparse
import json
import math
import time

import numpy as np
import onnxruntime as ort

JOINTS=tuple(f"{leg}_{joint}" for leg in ("FL","FR","RL","RR") for joint in ("hip","thigh","calf"))+(
    "FL_foot","FR_foot","RL_foot","RR_foot")
DEFAULT=np.array([0.,.8,-1.6]*4+[0.]*4,np.float32)
DDS_SLOT=np.array([3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14])
STATE_SIGN=np.array([-1.,1.,1.]*4+[1.]*4,np.float32)
KP=np.array([55.]*12+[0.]*4,np.float32)
KD=np.full(16,2.,np.float32)
MAX_COMMAND=np.array([.8,.45,.8],np.float32)
DT=.02
LEG_SCALE=np.float32(.25)
WHEEL_LIMIT=np.float32(12.)
RADIUS=np.float32(.086)
LEG_ALPHA=np.float32(-math.expm1(-2.*math.pi*10.*DT))
WHEEL_ALPHA=np.float32(-math.expm1(-2.*math.pi*5.*DT))

def projected_gravity(quat_wxyz):
    q=np.asarray(quat_wxyz,dtype=np.float32)
    if np.any(abs(np.sum(q*q,axis=-1)-1.)>.02):
        raise ValueError("Invalid IMU orientation, expected a unit wxyz quaternion")
    w,x,y,z=np.moveaxis(q,-1,0)
    return np.stack((2*(w*y-x*z),-2*(w*x+y*z),2*(x*x+y*y)-1),axis=-1)

def observation(q,dq,gyro,quat,command,prior):
    q_rel=np.asarray(q,dtype=np.float32)-DEFAULT
    q_rel[...,12:]=0.
    return np.concatenate((command,gyro,projected_gravity(quat),q_rel,dq*np.float32(.05),
                           prior[...,:12]/LEG_SCALE,prior[...,12:]/WHEEL_LIMIT,
                           np.full_like(q_rel[...,:1],.37)),axis=-1).astype(np.float32)

def wrap(prior,raw,command):
    target=np.empty_like(prior)
    leg=np.clip(raw,-1.,1.)*LEG_SCALE
    target[...,:12]=prior[...,:12]+LEG_ALPHA*(leg-prior[...,:12])
    wheel=np.clip(command[...,:1]/RADIUS,-WHEEL_LIMIT,WHEEL_LIMIT)
    filt=prior[...,12:]+WHEEL_ALPHA*(wheel-prior[...,12:])
    target[...,12:]=np.where((wheel==0)&(np.abs(filt)<.02),0.,filt)
    return target

def load_session(path):
    session=ort.InferenceSession(str(path),providers=["CPUExecutionProvider"])
    if ([(x.name,x.type,x.shape[-1]) for x in session.get_inputs()] != [("observation","tensor(float)",58)]
            or [(x.name,x.type,x.shape[-1]) for x in session.get_outputs()] != [("leg_action","tensor(float)",12)]):
        raise ValueError("Unexpected AS2-W ONNX interface")
    return session

def decode_lowstate(q_slots,dq_slots,zero_dds):
    q=np.asarray(q_slots,dtype=np.float32)[DDS_SLOT]
    dq=np.asarray(dq_slots,dtype=np.float32)[DDS_SLOT]
    return (q-zero_dds)*STATE_SIGN,dq*STATE_SIGN

class Controller:
    def __init__(self,model):
        self.session=load_session(model)
        self.prior=None

    def enter(self,q):
        self.prior=np.zeros(16,dtype=np.float32)
        self.prior[:12]=q[:12]-DEFAULT[:12]

    def exit(self):
        self.prior=None

    def step(self,q,dq,gyro,quat,command):
        if self.prior is None:
            raise ValueError("Controller requires explicit stand handoff")
        for value,n in zip((q,dq,gyro,quat,command),(16,16,3,4,3),strict=True):
            if value.shape!=(n,) or not np.isfinite(value).all():
                raise ValueError("Nonfinite or incorrectly shaped control input")
        if np.any(np.abs(command)>MAX_COMMAND+1e-6):
            raise ValueError("Command exceeds the trained envelope")
        obs=observation(q,dq,gyro,quat,command,self.prior)
        if obs[8]>-.52:
            raise ValueError("Body tilt outside learned range")
        raw=self.session.run(["leg_action"],{"observation":obs[None]})[0][0]
        if not np.isfinite(raw).all():
            raise ValueError("Nonfinite actor output")
        self.prior=wrap(self.prior,raw,command)
        q_cmd=np.zeros(16,np.float32)
        q_cmd[:12]=DEFAULT[:12]+self.prior[:12]
        dq_cmd=np.zeros(16,np.float32)
        dq_cmd[12:]=self.prior[12:]
        return dict(q=q_cmd,dq=dq_cmd,kp=KP.copy(),kd=KD.copy(),tau=np.zeros(16,np.float32),
                    observation=obs,raw_action=raw,prior_after=self.prior.copy())

def shadow(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    import threading
    lock=threading.Lock()
    state=None

    ChannelFactoryInitialize(args.domain,args.interface)
    def receive(msg):
        nonlocal state
        m=msg.motor_state
        frame=(time.monotonic(),[m[i].q for i in range(16)],[m[i].dq for i in range(16)],
               np.array(msg.imu_state.gyroscope,np.float32),np.array(msg.imu_state.quaternion,np.float32))
        with lock:
            state=frame
    subscriber=ChannelSubscriber("rt/lowstate",LowState_)
    subscriber.Init(receive,1)
    controller=Controller(args.model)
    cal=json.loads(Path(args.zero_map).read_text())
    if cal.get("joints")!=list(JOINTS):
        raise ValueError("Calibration joint order must match policy order")
    zeros=np.asarray(cal["zero_dds_rad"],dtype=np.float32)
    if zeros.shape!=(16,) or not np.isfinite(zeros).all():
        raise ValueError("Expected 16 finite DDS zero positions")
    command=np.asarray(args.command,dtype=np.float32)
    if not math.isfinite(args.seconds) or args.seconds<=0:
        raise ValueError("Shadow duration must be finite and positive")
    start=time.monotonic()
    tick=start
    with Path(args.output).open("x",encoding="utf-8") as out:
        while tick-start<args.seconds:
            time.sleep(max(0.,tick-time.monotonic()))
            now=time.monotonic()
            with lock:
                frame=state
            if frame is not None and now-frame[0]<=.04:
                received,q_slots,dq_slots,gyro,quat=frame
                q,dq=decode_lowstate(q_slots,dq_slots,zeros)
                try:
                    if controller.prior is None:
                        controller.enter(q)
                    target=controller.step(q,dq,gyro,quat,command)
                    record={k:v.tolist() for k,v in target.items()}
                    record.update(t_s=now-start,sensor_age_s=now-received,q_slots=q_slots,dq_slots=dq_slots,
                                  gyro=gyro.tolist(),quat_wxyz=quat.tolist(),command=command.tolist())
                except ValueError as e:
                    controller.exit()
                    record={"t_s":now-start,"error":str(e)}
            else:
                controller.exit()
                record={"t_s":now-start,"error":"LowState missing or stale"}
            out.write(json.dumps(record,allow_nan=False)+"\n")
            tick=max(tick+DT,now+DT if now-tick>=DT else tick+DT)

if __name__=="__main__":
    p=argparse.ArgumentParser(description="CPRO 50 Hz LowState shadow replay")
    p.add_argument("--model",required=True,type=Path)
    p.add_argument("--interface",required=True)
    p.add_argument("--domain",type=int,default=0)
    p.add_argument("--zero-map",required=True)
    p.add_argument("--seconds",type=float,default=10.)
    p.add_argument("--command",nargs=3,type=float,default=(0.,0.,0.))
    p.add_argument("--output",required=True)
    shadow(p.parse_args())
