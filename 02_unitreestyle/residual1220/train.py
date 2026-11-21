"""Resume the original 58-input AS2W policy with a contact-phase spike penalty."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--root",type=Path,required=True)
parser.add_argument("--out",type=Path,required=True)
parser.add_argument("--iterations",type=int,required=True)
parser.add_argument("--offset",type=int,default=1220)
parser.add_argument("--resume",type=Path)
parser.add_argument("--weight",type=float,default=.02)
parser.add_argument("--envs",type=int,default=8192)
import os
from height import PROFILE, set_posture, references, penalty
parser.add_argument("--profile",choices=("original",PROFILE),default=os.getenv("AS2W_PROFILE","original"))
args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"common"))
app=AppLauncher(headless=True).app
env=None
try:
    import numpy as np
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, RADIUS, WHEELS, tensor
    from learner import train
    from policy import read_actor, compose, gate

    @configclass
    class ContactScene(AS2WSceneCfg):
        contacts: ContactSensorCfg | None = None

    @configclass
    class ContinueCfg(AS2WCfg):
        scene=ContactScene(num_envs=args.envs,env_spacing=2.5,replicate_physics=True)

    class ContinueEnv(AS2WEnv):
        def __init__(self,cfg):
            super().__init__(cfg)
            self.base_ckpt=torch.load(args.root/"source/best.pt",map_location="cpu",weights_only=False)
            self.base_actor=read_actor(self.base_ckpt["actor_state_dict"],self.device)
            self.sensor=self.scene["contacts"]
            self.sensor_ids=[self.sensor.body_names.index(w) for w in WHEELS]
            with np.load(args.root/"source/reference.npz") as f:
                self.ref=torch.as_tensor(f["reference"],device=self.device)
                self.period=torch.as_tensor(f["period"],device=self.device)
                self.anchors=torch.as_tensor(f["anchor"],device=self.device)
            n=self.num_envs; dev=self.device
            self.rows=torch.arange(n,device=dev)
            self.gait_case=torch.full((n,),-1,device=dev,dtype=torch.long)
            self.clock=torch.zeros(n,device=dev)
            self.period_est=torch.ones(n,device=dev)
            self.landed=torch.zeros(n,dtype=torch.bool,device=dev)
            self.pending_contact=torch.zeros(n,dtype=torch.long,device=dev)
            self.pending_air=torch.zeros_like(self.pending_contact)
            self.was_air=torch.zeros_like(self.landed)
            self.age=torch.zeros_like(self.pending_contact)
            self.last3=torch.zeros(3,n,12,device=dev)
            self.action_legs=torch.zeros(n,12,device=dev)
            self.spike_mean=torch.zeros((),device=dev)
            with np.load(args.root/"source/flat.npz") as baseline:
                duties=[]
                for name in ("left","right","yaw_left","yaw_right"):
                    ix=baseline["labels"]==name
                    valid=~np.maximum.accumulate(baseline["fall"][:,ix],axis=0)[50:]
                    forces=baseline["wheel_data"][...,15][50:,ix]
                    duties.append((forces[valid]>=3.).mean(axis=0))
                self.duties=torch.tensor(np.stack(duties),device=dev,dtype=torch.float)
            if args.profile == PROFILE:
                arrays = references(args.root/"source")
                self.body_ref,self.wheel_ref,self.swing_ref = [torch.as_tensor(a,device=dev) for a in arrays]
                self.wheel_body_ids = [self.robot.body_names.index(w) for w in WHEELS]
            self.support_sum=torch.zeros(n,4,device=dev)
            self.phase_coverage=torch.zeros((),device=dev)

        def _pre_physics_step(self,actions):
            frame=self._frame()
            frozen=self.base_actor(frame)
            old_gate=gate(frame[:,:3])
            interval = round(2.5 / self.step_dt)
            env_ids = (self.episode_length_buf.remainder(interval) == 0).nonzero(as_tuple=True)[0]
            self._sample_commands(env_ids)
            if self.exercise is not None:
                self.command[:] = self.exercise
            actions = compose(frozen, actions, torch.minimum(old_gate,gate(self.command)))
            self.action_legs.copy_(actions)
            wheel = actions[:, 12:] * 12. if self.cfg.him else self.command[:, :1].expand(-1, 4) / RADIUS
            self.target[:, :12].lerp_(actions[:, :12] * .25, self.alpha_leg)
            self.target[:, 12:].lerp_(wheel.clamp(-12., 12.), self.alpha_wheel)
            if not self.cfg.him:
                zero = (wheel == 0) & (self.target[:, 12:].abs() < .02)
                self.target[:, 12:].masked_fill_(zero, 0.)


        def _reset_idx(self,env_ids):
            if env_ids is None:
                env_ids=self.rows
            super()._reset_idx(env_ids)
            self.gait_case[env_ids]=-1
            self.clock[env_ids]=0
            self.landed[env_ids]=False
            self.pending_contact[env_ids]=0
            self.pending_air[env_ids]=0
            self.was_air[env_ids]=False
            self.age[env_ids]=0
            self.last3[:,env_ids]=0

        def _get_rewards(self):
            legacy=super()._get_rewards()
            vx,vy,wz=self.command.unbind(1)
            side=(vx.abs()<1e-6)&(wz.abs()<1e-6)&(vy.abs()>=.15)&(vy.abs()<=.45)
            turn=(vx.abs()<1e-6)&(vy.abs()<1e-6)&(wz.abs()>=.35)&(wz.abs()<=.8)
            plane=self.terrain.terrain_types < self.cfg.scene.terrain.terrain_generator.num_cols*3/8
            code=torch.where(side,(vy<0).long(),torch.where(turn,2+(wz<0).long(),-1))
            code=torch.where(plane,code,-1)
            changed=code!=self.gait_case
            self.gait_case.copy_(code)
            safe=code.clamp_min(0)
            self.clock.copy_(torch.where(changed,0.,self.clock+self.step_dt))
            self.landed.logical_and_(~changed)
            self.period_est.copy_(torch.where(changed,self.period[safe],self.period_est))
            load=tensor(self.sensor.data.net_normal_forces_w)[:,self.sensor_ids,2]
            loaded=load[self.rows,self.anchors[safe]]>=3.
            self.pending_contact.copy_(torch.where(loaded & ~changed,self.pending_contact+1,0))
            self.pending_air.copy_(torch.where(~loaded & ~changed,self.pending_air+1,0))
            self.was_air.copy_(~changed & (self.was_air | (self.pending_air>=3)))
            impact=self.was_air&(self.pending_contact==3)&(code>=0)
            elapsed=(self.clock-2*self.step_dt)
            consistent=impact&self.landed&(elapsed>=.36)&(elapsed<=1.10)
            self.period_est.copy_(torch.where(consistent,.8*self.period_est+.2*elapsed,self.period_est))
            self.landed|=impact
            self.clock.copy_(torch.where(impact,2*self.step_dt,self.clock))
            self.was_air.logical_and_(~impact)
            self.age.copy_(torch.where(changed,0,self.age+1))
            index=(self.clock/self.period_est).remainder(1.)*self.ref.shape[1]
            i=index.long().clamp_max(self.ref.shape[1]-1)
            template=self.ref[safe,i].lerp(self.ref[safe,(i+1)%self.ref.shape[1]],(index-i)[:,None])
            residual=self.action_legs-template
            d0=residual-self.last3[0]
            d1=self.last3[0]-self.last3[1]
            d2=self.last3[1]-self.last3[2]
            one=torch.where(d0*d1<0,torch.minimum(d0.abs(),d1.abs()),0.)
            two=torch.where((d0*d2<0)&(d1.abs()<.25),torch.minimum(d0.abs(),d2.abs()),0.)
            short=(torch.maximum(one,two)-.65).clamp(0.,1.35).square().mean(1)
            use=(code>=0)&self.landed&(self.clock<=1.25*self.period_est)&(self.age>=5)&~impact
            short=short*use
            self.last3[2].copy_(self.last3[1])
            self.last3[1].copy_(self.last3[0])
            self.last3[0].copy_(residual)
            self.spike_mean.copy_(short.mean())
            self.phase_coverage.copy_(use.float().mean())
            length=(self.period[safe]/self.step_dt).round().long().clamp_min(1)
            start=changed | (self.age.remainder(length)==0)
            self.support_sum.masked_fill_(start[:,None],0.)
            self.support_sum.add_((load>=3.).float())
            end=(code>=0)&(self.age.remainder(length)==length-1)&(self.age>=length)
            duty_error=(self.support_sum/length[:,None]-self.duties[safe]).abs()
            duty=(duty_error-.12).clamp_min(0.).square().mean(1)*length*end
            cost=args.weight*short+.10*duty
            if args.profile == PROFILE:
                cost=cost+penalty(self,code,index,use,plane)
            return legacy-self.step_dt*cost

    input_manifest=json.loads((args.root/"source/run.json").read_text())
    asset=(ROOT / input_manifest["asset"]).resolve(strict=True)
    assert hashlib.sha256(asset.read_bytes()).hexdigest()==input_manifest["asset_sha256"]
    cfg=ContinueCfg(rough=True,randomize=True,seed=42)
    cfg.scene.robot.spawn.asset_path=str(asset)
    cfg.scene.robot.spawn.usd_dir=str(args.root/"usd")
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")
    if args.profile == PROFILE:
        set_posture(cfg)
    cfg.configure_terrain()
    env=ContinueEnv(cfg)
    assert cfg.observation_space==58 and abs(env.step_dt-.020)<1e-8
    train(env,args,args.out)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    if env is not None:
        env.close()
    app.close()
