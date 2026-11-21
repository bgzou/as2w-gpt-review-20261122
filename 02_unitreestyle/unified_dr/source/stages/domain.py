"""Cumulative AS2-W domain randomization stages."""
import json
from pathlib import Path
import math
import torch

from environment import DomainEvents,tensor

SETTINGS=json.loads(Path(__file__).with_name("domain_config.json").read_text())

def set_contact_domains(cfg,level):
    if level==0:
        return
    cfg.events=DomainEvents()
    d=SETTINGS["contact"]
    cfg.events.material.params.update(static_friction_range=tuple(d["static_friction"]),
        dynamic_friction_range=tuple(d["dynamic_friction"]),restitution_range=tuple(d["restitution"]),
        make_consistent=True)
    cfg.events.mass.params["mass_distribution_params"]=tuple(d["mass_scale"])
    cfg.events.com.params["com_range"]={key:(-d["com_half_range_m"],d["com_half_range_m"]) for key in "xyz"}
    cfg.events.drive_gains.params.update(
        stiffness_distribution_params=tuple(d["drive_gain_scale"]),
        damping_distribution_params=tuple(d["drive_gain_scale"]))

class DomainState:
    def __init__(self,env,level,seed):
        self.env=env
        self.level=level
        n,d=env.num_envs,env.device
        self.rng=torch.Generator(device=d).manual_seed(seed+10007)
        self.bias=torch.zeros(n,58,device=d)
        self.noise=torch.zeros(58,device=d)
        self.bias_scale=torch.zeros_like(self.noise)
        self.delay=torch.zeros(n,2,device=d,dtype=torch.bool)
        self.prev_target=torch.zeros_like(env.target)
        self.applied=torch.zeros_like(env.target)
        self.push_time=torch.zeros(n,device=d)
        sensor=SETTINGS["sensors"]
        for sl,noise,bias,scale in [
            (slice(3,6),"gyro_noise_rad_s","gyro_bias_rad_s",1.),
            (slice(9,21),"position_noise_rad","position_bias_rad",1.),
            (slice(25,41),"velocity_noise_rad_s","velocity_bias_rad_s",.05)]:
            self.noise[sl]=scale*sensor[noise]
            self.bias_scale[sl]=scale*sensor[bias]
        self.reset(torch.arange(n,device=d))

    def sample(self,shape):
        return torch.rand(shape,device=self.env.device,generator=self.rng)

    def reset(self,ids):
        n=len(ids)
        if self.level>=2:
            self.bias[ids]=(2*self.sample((n,58))-1)*self.bias_scale
            self.delay[ids]=self.sample((n,2))<.5
            self.prev_target[ids]=self.env.target[ids]
            self.applied[ids]=self.env.target[ids]
        if self.level>=3:
            lo,hi=SETTINGS["push"]["interval_s"]
            self.push_time[ids]=lo+(hi-lo)*self.sample((n,))

    def observations(self,value):
        if self.level>=2:
            actor=value["policy"]
            value["policy"]=actor+self.bias+(2*self.sample(actor.shape)-1)*self.noise
        return value

    def step(self):
        e=self.env
        if self.level>=2:
            self.applied[:,:12]=torch.where(self.delay[:,0,None],self.prev_target[:,:12],e.target[:,:12])
            self.applied[:,12:]=torch.where(self.delay[:,1,None],self.prev_target[:,12:],e.target[:,12:])
            self.prev_target.copy_(e.target)
        if self.level>=3:
            self.push_time.sub_(e.step_dt)
            ids=(self.push_time<=0).nonzero(as_tuple=True)[0]
            n=len(ids)
            if n:
                a=2*math.pi*self.sample((n,))
                lo,hi=SETTINGS["push"]["horizontal_delta_v_m_s"]
                magnitude=lo+(hi-lo)*self.sample((n,))
                vel=tensor(e.robot.data.root_vel_w)[ids].clone()
                vel[:,:2]+=torch.stack((a.cos(),a.sin()),1)*magnitude[:,None]
                e.robot.write_root_velocity_to_sim(vel,env_ids=ids)
                lo,hi=SETTINGS["push"]["interval_s"]
                self.push_time[ids]=lo+(hi-lo)*self.sample((n,))

    def apply(self):
        e=self.env
        pos=tensor(e.robot.data.default_joint_pos)[:,e.leg_ids]+self.applied[:,:12]
        e.robot.set_joint_position_target(pos,joint_ids=e.leg_ids)
        e.robot.set_joint_velocity_target(self.applied[:,12:],joint_ids=e.wheel_ids)
