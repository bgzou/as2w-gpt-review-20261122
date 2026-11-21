import json
from pathlib import Path
import numpy as np
import torch
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass, instantiate
from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh
from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, WHEELS, tensor
from swing_reward import SwingReward
from domain import DomainState,set_contact_domains

def build_task(args):
    source=Path(__file__).resolve().parent
    config_path=Path(getattr(args,"reward_config",None) or (source/"reward_config.json"))
    weights=json.loads(config_path.read_text())

    @configclass
    class ContactSceneCfg(AS2WSceneCfg):
        contacts: ContactSensorCfg | None = None

    @configclass
    class TaskCfg(AS2WCfg):
        scene=ContactSceneCfg(num_envs=args.num_envs,env_spacing=2.5,replicate_physics=True)

    class Task(AS2WEnv):
        def _custom_reset_pose(self, ids, state, q):
            if not getattr(args, "stop_course", False):
                return
            if not hasattr(self, "field_poses"):
                table = json.loads((source / "t2_reset_poses.json").read_text())
                self.field_poses = q.new_tensor([row["q"] + row["quat_wxyz"] for row in table])
            rows = (ids.remainder(4) == 2).nonzero(as_tuple=True)[0]
            indices = torch.randint(len(self.field_poses), (len(rows),), device=q.device)
            pose = self.field_poses[indices]
            q[rows[:, None], self.leg_ids] = pose[:, :12]
            state[rows, 3:7] = pose[:, [13, 14, 15, 12]]  # Log wxyz -> Isaac native xyzw.

        def _sample_commands(self, ids):
            super()._sample_commands(ids)
            if not getattr(args, "stop_course", False):
                return
            # Half the environments alternate motion and zero every 2.5 s.
            selected = ids[ids.remainder(2) == 0]
            phase = self.episode_length_buf[selected] // round(2.5 / self.step_dt)
            mode = (selected // 2 + phase // 2).remainder(6)
            commands = self.command.new_tensor([
                [.30, 0, 0], [-.30, 0, 0], [0, .25, 0], [0, -.25, 0],
                [0, 0, .50], [0, 0, -.50],
            ])
            start_still = selected.remainder(4) == 2
            moving = phase.remainder(2) == start_still.long()
            self.command[selected] = commands[mode] * moving[:, None]

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
            self.swing=SwingReward(self.num_envs,self.device,json.loads((source/"calibration.json").read_text()),weights)
            self.last_command=torch.zeros_like(self.command)
            self.command_age=torch.zeros(self.num_envs,device=self.device)
            self.roll_missing=torch.zeros(self.num_envs,4,device=self.device)
            self.stop_unloaded_time=torch.zeros_like(self.roll_missing)
            self.trunk_time=torch.zeros_like(self.command_age)
            self.last_normal_velocity=torch.zeros_like(self.roll_missing)
            self.touchdown_speed=torch.zeros_like(self.roll_missing)
            self.last_loaded=torch.zeros_like(self.roll_missing,dtype=torch.bool)
            self.step_age=torch.zeros_like(self.roll_missing)
            # Lateral side-step gait: the left pair (FL+RL) and the right pair
            # (FR+RR) alternate. Diagonal pairs are wrong for pure vy motion and
            # left the rear-right wheel permanently planted.
            self.phase_pair_a=torch.tensor([True,False,True,False],device=self.device)
            self.phase_pair_b=torch.tensor([False,True,False,True],device=self.device)
            self.load_ema=torch.zeros_like(self.roll_missing)
            self.down=torch.tensor([0.,0.,-1.],device=self.device)
            lower,upper=tensor(self.robot.data.joint_pos_limits)[:,self.leg_ids].unbind(-1)
            margin=(upper-lower)*weights["soft_joint_limit_margin_fraction"]
            self.soft_lower,self.soft_upper=lower+margin,upper-margin
            self.limit_margin=margin.clamp_min(.01)
            self.metric_sums=torch.zeros(48,device=self.device)
            self.metric_steps=0
            self.executed_boundary=torch.zeros(self.num_envs,device=self.device)
            self.dr=DomainState(self,args.dr_level,args.seed)
            self.rollup_names=("swing_event","swing_coverage","loaded_lateral_slip","landing_speed",
                               "nonwheel_contact","joint_soft_limit","rolling_missing_wheel",
                               "rolling_samples","stepping_samples","terrain_hits","trunk_termination","rolling_support_fraction")+tuple(
                f"{kind}_{m}_{w}" for kind in ("event","support")
                for m in ("left","right","yaw_left","yaw_right") for w in ("FL","FR","RL","RR"))+(
                "executed_action_boundary_fraction","executed_abs_mean","executed_delta_mean","executed_signflip_fraction")

        def _reset_idx(self,ids):
            super()._reset_idx(ids)
            if hasattr(self,"swing"):
                ids=slice(None) if ids is None else ids
                self.swing.reset(ids)
                for value in (self.last_command,self.command_age,self.roll_missing,self.stop_unloaded_time,self.trunk_time,self.last_normal_velocity,self.touchdown_speed,self.last_loaded,self.step_age):
                    value[ids]=0
            if hasattr(self,"dr"):
                self.dr.reset(torch.arange(self.num_envs,device=self.device) if isinstance(ids,slice) else ids)

        def _get_observations(self):
            value=super()._get_observations()
            return self.dr.observations(value) if hasattr(self,"dr") else value

        def _sample_commands(self, ids):
            if getattr(self.cfg,"step_only",False):
                # Stepping-only task: no vx command exists, so rolling cannot
                # satisfy the command and the policy must lift wheels.
                n=len(ids)
                cmd=torch.zeros(n,3,device=self.device)
                kind=torch.randint(0,3,(n,),device=self.device)
                vy=(torch.rand(n,device=self.device)*2-1)*.45
                wz=(torch.rand(n,device=self.device)*2-1)*.80
                cmd[:,1]=torch.where(kind==1,vy,torch.zeros_like(vy))
                cmd[:,2]=torch.where(kind==2,wz,torch.zeros_like(wz))
                if getattr(self.cfg,"right_only",False):
                    # Concentrate on right translation, where the rear-right
                    # wheel has never lifted, while keeping some yaw samples.
                    vy_right=-(.15+torch.rand(n,device=self.device)*.30)
                    cmd[:,1]=torch.where(kind!=2,vy_right,torch.zeros_like(vy))
                    cmd[:,2]=torch.where(kind==2,wz,torch.zeros_like(wz))
                self.command[ids]=cmd
                return
            super()._sample_commands(ids)

        def _support_margin(self,xy,com):
            """Min signed edge distance from the COM projection to the triangle
            left when each wheel lifts; positive means the lift is statically safe."""
            idx=((1,2,3),(0,2,3),(0,1,3),(0,1,2))
            out=[]
            for k in range(4):
                tri=xy[:,idx[k],:]
                a,b,c=tri[:,0],tri[:,1],tri[:,2]
                area2=((b[:,0]-a[:,0])*(c[:,1]-a[:,1])-(b[:,1]-a[:,1])*(c[:,0]-a[:,0]))
                sign=torch.where(area2>=0,1.,-1.)
                best=None
                for p,q in ((a,b),(b,c),(c,a)):
                    e=q-p
                    d=sign*(e[:,0]*(com[:,1]-p[:,1])-e[:,1]*(com[:,0]-p[:,0]))/e.norm(dim=-1).clamp_min(1e-6)
                    best=d if best is None else torch.minimum(best,d)
                out.append(best)
            return torch.stack(out,-1)

        def _tracking_gate(self):
            floor=float(weights.get("step_gate_floor",1.))
            if floor>=1. or not hasattr(self,"step_age"):
                return 1.
            x,y,z=(self.command.abs()>1e-6).unbind(1)
            stepping=(~x&~z&(y.abs()>=.15))|(~x&~y&(z.abs()>=.35))
            recent=(self.step_age<float(weights.get("step_gate_window_s",1.))).all(1)
            gated=torch.where(recent,1.,floor)
            return torch.where(stepping,gated,1.)

        def _pre_physics_step(self,actions):
            # Fraction of executed commands at the tanh asymptote (|x|>=3.79).
            self.executed_boundary.copy_((actions.tanh().abs()>=.999).float().mean(1))
            super()._pre_physics_step(actions)
            self.dr.step()

        def _apply_action(self):
            if hasattr(self,"dr") and self.dr.level>=2:
                self.dr.apply()
            else:
                super()._apply_action()

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
            self.last_normal_force=normal_force
            self.load_ema.mul_(.98).add_(.02*normal_force)
            # Signed margin of the 3-wheel support polygon that remains when
            # wheel k lifts; negative means the COM projection is outside and
            # lifting k would tip the robot.
            self.support_margin_=self._support_margin(position[...,:2],tensor(data.root_pos_w)[:,:2])
            self.step_age.add_(dt)
            self.step_age.masked_fill_(self.swing.last_valid,0.)
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
            sub_terrains=self.cfg.scene.terrain.terrain_generator.sub_terrains
            plane_fraction=sub_terrains["plane"].proportion if "plane" in sub_terrains else 0.
            slope=self.terrain.terrain_types/columns+.001>=plane_fraction
            allowed=duration*(1+.5*slope.float())
            rolling_cost=((self.roll_missing-allowed[:,None])/.12).clamp(0,1).mean(1)
            values=(weights["swing_event"]*events,weights["swing_coverage"]*dt*gap,
                    weights["loaded_lateral_slip"]*dt*slip,weights["landing_speed"]*impact,
                    weights["nonwheel_contact"]*dt*nonwheel,weights["joint_soft_limit"]*dt*limits,
                    weights["rolling_missing_wheel"]*dt*rolling_cost)
            with torch.no_grad():
                executed=getattr(self,"executed",None)
                if executed is None:
                    act_abs=act_delta=act_flip=torch.zeros((),device=self.device)
                else:
                    # Field-jitter predictors requested in the 2026-11-19
                    # real-robot feedback: saturation level, per-step dither and
                    # sign-flip fraction of the executed action.
                    act_abs=executed.abs().mean()
                    act_delta=(executed-self.executed_prev).abs().mean()
                    act_flip=((executed*self.executed_prev)<0).float().mean()
                self.metric_sums.add_(torch.stack([v.mean() for v in values]+
                    [rolling.float().mean(),use.float().mean(),hit.float().mean(),
                     (self.trunk_time>=weights["body_contact_stop_s"]).float().mean(),
                     ((normal_force>=3.).all(1)&steady).float().sum()/steady.sum().clamp_min(1)] +
                    [(self.swing.last_valid[:,wi]&(mode==mi)).float().sum()/(mode==mi).sum().clamp_min(1)
                     for mi in range(4) for wi in range(4)] +
                    [((normal_force[:,wi]>=4.)&(mode==mi)).float().sum()/(mode==mi).sum().clamp_min(1)
                     for mi in range(4) for wi in range(4)]+[self.executed_boundary.mean(),
                     act_abs,act_delta,act_flip]))
                self.metric_steps+=1
            result = reward + values[0] - sum(values[1:])
            stability_weight = float(weights.get("stability_weight", 0.))
            if stability_weight:
                # Pay for lifts only when the remaining support triangle still
                # contains the COM; penalise lifts that would tip the robot.
                margin = self.support_margin_
                safe = self.swing.last_valid & (margin >= float(weights.get("stability_margin_m", .005)))
                unsafe = self.swing.last_valid & (margin < 0.)
                result = result + stability_weight * safe.sum(1).float()
                result = result - float(weights.get("unstable_penalty", 0.)) * unsafe.sum(1).float()
            hardest_weight = float(weights.get("hardest_lift_weight", 0.))
            if hardest_weight:
                # Extra credit when the most-loaded wheel (the one that never
                # lifts in practice) actually completes a valid lift.
                hardest = self.load_ema.argmax(1)
                lifted = self.swing.last_valid.gather(1, hardest[:, None]).squeeze(1).float()
                result = result + hardest_weight * lifted
            if getattr(self.cfg, "gait_phase", False):
                phase_weight = float(weights.get("phase_lift_weight", 0.))
                if phase_weight:
                    period = max(float(getattr(self.cfg, "gait_period_s", 1.0)), 1e-3)
                    phase = (self.episode_length_buf.float() * dt / period).remainder(1.0)
                    sched = torch.where((phase < .5)[:, None], self.phase_pair_a, self.phase_pair_b)
                    result = result + phase_weight * (self.swing.last_valid & sched).sum(1).float()
            light_load_weight = getattr(args, "rolling_light_load_weight", 0.)
            if light_load_weight:
                # Low load on any one wheel matters during pure rolling.
                deficit = ((4. - normal_force) / 4.).clamp(0., 1.)
                result = result - dt * light_load_weight * steady * deficit.amax(dim=1)
            if getattr(args, "stop_reward", False):
                settled_window = (~(x | y | z)) & (self.command_age >= .45)
                velocity = tensor(data.root_lin_vel_b)[:, :2].norm(dim=1)
                tilt = tensor(data.projected_gravity_b)[:, :2].norm(dim=1)
                drift_cost = ((velocity - .025).clamp_min(0) / .15).square().clamp_max(1.)
                tilt_cost = ((tilt - .12).clamp_min(0) / .20).square().clamp_max(1.)
                missing = settled_window[:, None] & (normal_force < 3.)
                self.stop_unloaded_time.copy_(torch.where(missing, self.stop_unloaded_time + dt, 0.))
                unloaded = (self.stop_unloaded_time >= .12).float().mean(1)
                return result - dt * settled_window * (.25 * drift_cost + .25 * tilt_cost + .30 * unloaded)
            return result

    cfg=TaskCfg(rough=True,randomize=args.dr_level>0,seed=args.seed,
                gait_phase=getattr(args,"gait_phase",False),
                gait_period_s=getattr(args,"gait_period_s",1.0),
                step_only=getattr(args,"step_only",False),
                right_only=getattr(args,"right_only",False))
    set_contact_domains(cfg,args.dr_level)
    from isaaclab.sim import UsdFileCfg
    asset = args.asset.resolve(strict=True)
    if asset.suffix != ".usd":
        raise ValueError("Run prepare_v2_usd.py and supply its .usd output")
    cfg.scene.robot.spawn = UsdFileCfg(usd_path=str(asset), activate_contact_sensors=True)
    cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")
    cfg.configure_terrain()
    friction = getattr(args, "ground_friction", None)
    if asset.name == "as2w-t2-v2-mu025.usd":
        if friction is not None and friction != .25:
            raise ValueError("The mu025 candidate requires ground friction 0.25")
        friction = .25
    if friction is not None:
        cfg.scene.terrain.physics_material.static_friction = friction
        cfg.scene.terrain.physics_material.dynamic_friction = friction
        cfg.scene.terrain.physics_material.friction_combine_mode = "min"
    if getattr(args,"terrain",None)=="flat":
        cfg.scene.terrain.terrain_generator.sub_terrains={"plane":cfg.scene.terrain.terrain_generator.sub_terrains["plane"]}
        cfg.scene.terrain.terrain_generator.sub_terrains["plane"].proportion=1.
    if getattr(args,"terrain",None)=="slope":
        # A long slope permits 4 s rollouts in either direction without
        # starting on the heightfield's flat top or leaving its tile.
        gen=cfg.scene.terrain.terrain_generator
        gen.size=(12.,12.)
        slope=gen.sub_terrains["slope"]
        slope.platform_width=.70
        slope.proportion=1.
        gen.sub_terrains={"slope":slope}
    print("UNIFIED_TERRAIN", {k:v.proportion for k,v in cfg.scene.terrain.terrain_generator.sub_terrains.items()},flush=True)
    cfg.scene.terrain.terrain_generator.curriculum=False
    env=Task(cfg)
    env.command_hold_s = getattr(args, "command_hold_s", 2.5)

    assert env.cfg.observation_space in (58,60) and abs(env.step_dt-.02)<1e-8
    return env
