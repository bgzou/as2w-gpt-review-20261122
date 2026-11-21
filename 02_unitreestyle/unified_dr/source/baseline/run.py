"""Train AS2-W velocity tracking with command-conditioned wheel and leg rewards."""
import argparse
import json
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser()
parser.add_argument("--out",type=Path,required=True)
parser.add_argument("--asset",type=Path,required=True)
parser.add_argument("--num-envs",type=int,default=4096)
parser.add_argument("--iterations",type=int,required=True)
parser.add_argument("--seed",type=int,default=42)
parser.add_argument("--rollout-steps",type=int,default=48)
args=parser.parse_args()
args.resume=None
args.resume_update=None
args.previous_best=None
app=AppLauncher(headless=True).app
env=None

try:
    import numpy as np
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass, instantiate
    from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh
    from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, WHEELS, tensor
    from swing_reward import SwingReward
    from training import train

    source=Path(__file__).resolve().parent
    weights=json.loads((source/"reward_config.json").read_text())

    @configclass
    class ContactSceneCfg(AS2WSceneCfg):
        contacts: ContactSensorCfg | None = None

    @configclass
    class TaskCfg(AS2WCfg):
        scene=ContactSceneCfg(num_envs=args.num_envs,env_spacing=2.5,replicate_physics=True)

    class Task(AS2WEnv):
        def __init__(self,cfg):
            super().__init__(cfg)
            self.contact=self.scene["contacts"]
            self.sensor_wheels=[self.contact.body_names.index(w) for w in WHEELS]
            self.wheel_bodies=[self.robot.body_names.index(w) for w in WHEELS]
            self.nonwheel=[i for i,n in enumerate(self.contact.body_names) if n not in WHEELS]
            self.trunk=[i for i in self.nonwheel if "base" in self.contact.body_names[i].lower()
                        or "head" in self.contact.body_names[i].lower()]
            assert self.nonwheel and self.trunk, self.contact.body_names
            generator=instantiate(cfg.scene.terrain.terrain_generator,device=self.device)
            np.testing.assert_allclose(generator.terrain_origins, tensor(self.terrain.terrain_origins).cpu().numpy(),atol=1e-5)
            m=generator.terrain_mesh
            self.mesh=convert_to_warp_mesh(m.vertices,m.faces,self.device)
            self.swing=SwingReward(self.num_envs,self.device,json.loads((source/"calibration.json").read_text()))
            self.last_command=torch.zeros_like(self.command)
            self.command_age=torch.zeros(self.num_envs,device=self.device)
            self.roll_missing=torch.zeros(self.num_envs,4,device=self.device)
            self.trunk_time=torch.zeros_like(self.command_age)
            self.last_normal_velocity=torch.zeros_like(self.roll_missing)
            self.touchdown_speed=torch.zeros_like(self.roll_missing)
            self.last_loaded=torch.zeros_like(self.roll_missing,dtype=torch.bool)
            self.down=torch.tensor([0.,0.,-1.],device=self.device)
            lower,upper=tensor(self.robot.data.joint_pos_limits)[:,self.leg_ids].unbind(-1)
            margin=(upper-lower)*weights["soft_joint_limit_margin_fraction"]
            self.soft_lower,self.soft_upper=lower+margin,upper-margin
            self.limit_margin=margin.clamp_min(.01)
            self.metric_sums=torch.zeros(12,device=self.device)
            self.metric_steps=0
            self.rollup_names=("swing_event","swing_overdue","loaded_lateral_slip","landing_speed",
                               "nonwheel_contact","joint_soft_limit","rolling_missing_wheel",
                               "rolling_samples","stepping_samples","terrain_hits","trunk_termination","rolling_support_fraction")

        def _reset_idx(self,ids):
            super()._reset_idx(ids)
            if hasattr(self,"swing"):
                ids=slice(None) if ids is None else ids
                self.swing.reset(ids)
                for value in (self.last_command,self.command_age,self.roll_missing,self.trunk_time,self.last_normal_velocity,self.touchdown_speed,self.last_loaded):
                    value[ids]=0

        def _get_dones(self):
            fail,timeout=super()._get_dones()
            if not hasattr(self,"trunk_time"):
                return fail,timeout
            forces=tensor(self.contact.data.net_normal_forces_w)
            hit=forces[:,self.trunk].norm(dim=-1).amax(1)>=weights["nonwheel_contact_force_n"]
            self.trunk_time.copy_(torch.where(hit,self.trunk_time+self.step_dt,0.))
            return fail|(self.trunk_time>=weights["body_contact_stop_s"]),timeout

        def _get_rewards(self):
            reward=super()._get_rewards()
            dt=self.step_dt
            vx,vy,wz=self.command.unbind(1)
            x,y,z=(self.command.abs()>1e-6).unbind(1)
            side=~x&~z&(vy.abs()>=.15)
            turn=~x&~y&(wz.abs()>=.35)
            mode=torch.where(side,(vy<0).long(),torch.where(turn,2+(wz<0).long(),-1))
            switch=(self.command-self.last_command).abs().amax(1)>1e-6
            self.last_command.copy_(self.command)
            self.command_age.copy_(torch.where(switch,0.,self.command_age+dt))
            data=self.robot.data
            ids=self.wheel_bodies
            position=tensor(data.body_link_pos_w)[:,ids]
            ground,_,normal,_=raycast_mesh(position-self.down*.40,self.down.expand_as(position),
                                             self.mesh,max_dist=2.,return_normal=True)
            hit=torch.isfinite(ground).all(-1)&torch.isfinite(normal).all(-1)
            normal=torch.where(hit[...,None],normal,-self.down)
            normal=torch.where(normal[...,2:3]<0,-normal,normal)
            ground=torch.where(hit[...,None],ground,position)
            use=(mode>=0)&hit.all(1)
            mode=torch.where(use,mode,-1)
            force=tensor(self.contact.data.net_normal_forces_w)
            normal_force=(force[:,self.sensor_wheels]*normal).sum(-1).clamp_min(0.)
            events,gap,slip,_,landed,normal_velocity=self.swing(
                mode,switch,normal_force,position,tensor(data.body_link_quat_w)[:,ids],
                tensor(data.body_link_lin_vel_w)[:,ids],tensor(data.body_link_ang_vel_w)[:,ids],
                ground,normal,dt)
            loaded=normal_force>=4.
            first_loaded=loaded&~self.last_loaded
            self.touchdown_speed.copy_(torch.where(first_loaded,-self.last_normal_velocity,self.touchdown_speed))
            self.last_loaded.copy_(loaded)
            impact=(((self.touchdown_speed-weights["landing_speed_free_m_s"])/.5).clamp(0,1).square()*landed).mean(1)
            self.last_normal_velocity.copy_(normal_velocity)
            nonwheel=(force[:,self.nonwheel].norm(dim=-1)>=weights["nonwheel_contact_force_n"]).any(1).float()
            q=tensor(data.joint_pos)[:,self.leg_ids]
            limits=(((self.soft_lower-q).clamp_min(0)+(q-self.soft_upper).clamp_min(0))/
                    self.limit_margin).square().clamp_max(1.).mean(1)
            rolling=(vx.abs()>=weights["rolling_command_min_m_s"])&~y&~z
            steady=rolling&(self.command_age>=weights["rolling_grace_s"])
            missing=steady[:,None]&(normal_force<3.)
            self.roll_missing.copy_(torch.where(missing,self.roll_missing+dt,0.))
            duration=weights["rolling_unload_s"]
            columns=self.cfg.scene.terrain.terrain_generator.num_cols
            slope=self.terrain.terrain_types/columns+.001>=.60
            allowed=duration*(1+.5*slope.float())
            rolling_cost=((self.roll_missing-allowed[:,None])/.12).clamp(0,1).mean(1)
            values=(weights["swing_event"]*events,weights["swing_overdue"]*dt*gap,
                    weights["loaded_lateral_slip"]*dt*slip,weights["landing_speed"]*impact,
                    weights["nonwheel_contact"]*dt*nonwheel,weights["joint_soft_limit"]*dt*limits,
                    weights["rolling_missing_wheel"]*dt*rolling_cost)
            with torch.no_grad():
                self.metric_sums.add_(torch.stack([v.mean() for v in values]+
                    [rolling.float().mean(),use.float().mean(),hit.float().mean(),
                     (self.trunk_time>=weights["body_contact_stop_s"]).float().mean(),
                     ((normal_force>=3.).all(1)&steady).float().sum()/steady.sum().clamp_min(1)]))
                self.metric_steps+=1
            return reward+values[0]-sum(values[1:])

    cfg=TaskCfg(rough=True,randomize=True,seed=args.seed)
    cfg.scene.robot.spawn.asset_path=str(args.asset.resolve(strict=True))
    cfg.scene.robot.spawn.usd_dir=str(args.out.parent/"usd"/args.out.name)
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")
    cfg.configure_terrain()
    print("UNIFIED_TERRAIN", {k:v.proportion for k,v in cfg.scene.terrain.terrain_generator.sub_terrains.items()},flush=True)
    env=Task(cfg)
    assert env.cfg.observation_space==58 and abs(env.step_dt-.02)<1e-8
    train(env,args,args.out)
except BaseException:
    traceback.print_exc()
    raise
finally:
    if env is not None: env.close()
    app.close()
