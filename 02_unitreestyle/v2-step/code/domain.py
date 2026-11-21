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
    from isaaclab.managers import SceneEntityCfg

    cfg.events.mass.params.update(
        asset_cfg=SceneEntityCfg("robot", body_names="base_link"),
        mass_distribution_params=tuple(d["base_mass_add_kg"]),
        operation="add",
    )
    cfg.events.com.params["com_range"] = {
        axis: tuple(bounds) for axis, bounds in d["base_com_delta_m"].items()
    }
    if level < 2:
        cfg.events.drive_gains = None
    else:
        cfg.events.drive_gains.params.update(
            stiffness_distribution_params=tuple(d["drive_gain_scale"]),
            damping_distribution_params=tuple(d["drive_gain_scale"]),
        )


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
        # ── motor jitter (2026-11-20) ──────────────────────────────────────
        # Real-robot feedback (unitree-unified 抖动与饱和 2026-11-19): the
        # deployed policy showed 0.46-0.50 rad p2p leg targets, 30% sign-flip
        # frames and ~0.16 rad tracking error. Model that class of disturbance
        # so the policy learns to damp it instead of relying on dither:
        #   * per-step uniform dither on the applied leg/wheel targets
        #   * per-episode, per-joint kp/kd multiplier (unequal motor response)
        jitter=SETTINGS.get("jitter",{})
        self.jitter_on=bool(jitter.get("enabled",False)) and level>=int(jitter.get("min_level",2))
        self.jitter_rad=float(jitter.get("target_noise_rad",0.))
        self.jitter_wheel=float(jitter.get("target_noise_wheel_rad_s",0.))
        self.gain_fraction=float(jitter.get("gain_jitter_fraction",0.))
        self.torque_noise=float(jitter.get("torque_noise_nm",0.))
        self.gain_scale=torch.ones(n,16,device=d)
        self.reset(torch.arange(n,device=d))


    def sample(self,shape):
        return torch.rand(shape,device=self.env.device,generator=self.rng)

    def reset(self,ids):
        n=len(ids)
        if self.level>=2:
            self.bias[ids]=(2*self.sample((n,58))-1)*self.bias_scale
            sensor = SETTINGS["sensors"]
            self.delay[ids] = ((self.sample((n, 2)) < sensor["delay_probability"])
                               & (sensor["delay_ticks_max"] == 1))
            self.prev_target[ids]=self.env.target[ids]
            self.applied[ids]=self.env.target[ids]
        if self.jitter_on and self.gain_fraction>0:
            # Per-episode, per-joint kp/kd multiplier: real motors do not all
            # answer the same PD command (cogging, gearbox friction, temperature).
            scale=1.+(2*self.sample((n,16))-1)*self.gain_fraction
            self.gain_scale[ids]=scale
            robot=self.env.robot
            base_kp=tensor(robot.data.default_joint_stiffness)[ids].clone()
            base_kd=tensor(robot.data.default_joint_damping)[ids].clone()
            robot.write_joint_stiffness_to_sim(base_kp*scale,joint_ids=None,env_ids=ids)
            robot.write_joint_damping_to_sim(base_kd*scale,joint_ids=None,env_ids=ids)
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
        wheel=self.applied[:,12:].clone()
        if self.jitter_on:
            # Per-step dither on the applied targets: this is the disturbance
            # that shows up on the real robot as motor/encoder jitter. It is
            # re-drawn every control step, so a policy cannot average it away
            # inside one control period.
            if self.jitter_rad>0:
                pos=pos+(2*self.sample(pos.shape)-1)*self.jitter_rad
            if self.jitter_wheel>0:
                wheel=wheel+(2*self.sample(wheel.shape)-1)*self.jitter_wheel
        e.robot.set_joint_position_target(pos,joint_ids=e.leg_ids)
        e.robot.set_joint_velocity_target(wheel,joint_ids=e.wheel_ids)

