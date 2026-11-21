"""Shared AS2W simulation with distinct policy and wheel-command contracts."""
from pathlib import Path
import math
import tomllib
import xml.etree.ElementTree as ET

import torch
import isaaclab.sim as sim
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, mdp
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from isaaclab.terrains import MeshPlaneTerrainCfg, MeshPyramidStairsTerrainCfg
from isaaclab.terrains import HfRandomUniformTerrainCfg, HfPyramidSlopedTerrainCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg

ROOT = Path(__file__).resolve().parents[1]
CONTROL = tomllib.loads((Path(__file__).with_name("control.toml")).read_text())["common"]
LEGS = [f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")]
WHEELS = [f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR")]
FRAME = 58
HISTORY = 6
RADIUS = 0.086
HEIGHT = 0.37
RESET_HEIGHT = 0.390

def tensor(value):
    return value.torch if hasattr(value, "torch") else value

@configclass
class DomainEvents:
    material = EventTermCfg(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot"), "static_friction_range": (.65, 1.1),
                "dynamic_friction_range": (.55, .65), "restitution_range": (0., .05),
                "num_buckets": 48},
    )
    mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot"), "mass_distribution_params": (.95, 1.05),
                "operation": "scale", "distribution": "uniform", "recompute_inertia": True},
    )
    com = EventTermCfg(
        func=mdp.randomize_rigid_body_com, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
                "com_range": {axis: (-.005, .005) for axis in ("x", "y", "z")}},
    )
    drive_gains = EventTermCfg(
        func=mdp.randomize_actuator_gains, mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot"), "stiffness_distribution_params": (.95, 1.05),
                "damping_distribution_params": (.95, 1.05), "operation": "scale"},
    )

@configclass
class AS2WSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg | None = None
    terrain: TerrainImporterCfg | None = None

@configclass
class AS2WCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 12.0
    action_space = 12
    observation_space = FRAME
    state_space = FRAME + 3
    him = False
    rough = True
    randomize = True
    sim = sim.SimulationCfg(
        dt=CONTROL["policy_dt_s"] / 4, render_interval=4,
        physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27), use_newton_actuators=False,
    )
    scene = AS2WSceneCfg(num_envs=64, env_spacing=2.5, replicate_physics=True)

    def __post_init__(self):
        source = ROOT / "assets/as2w/as2w.urdf"
        limits = {j.get("name"): j.find("limit").attrib for j in ET.parse(source).findall("joint")
                  if j.find("limit") is not None}
        self.scene.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim.UrdfFileCfg(asset_path=str(source), fix_base=False, merge_fixed_joints=True,
                                 usd_dir=str(ROOT / "assets/generated"), usd_file_name="as2w.usd",
                                 joint_drive=None),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0., 0., RESET_HEIGHT),
                joint_pos={".*_hip": 0., ".*_thigh": .8, ".*_calf": -1.6, ".*_foot": 0.},
            ),
            actuators={
                name: ImplicitActuatorCfg(
                    joint_names_expr=group, stiffness=kp, damping=kd,
                    effort_limit_sim={j: float(limits[j]["effort"]) for j in group},
                    velocity_limit_sim={j: float(limits[j]["velocity"]) for j in group},
                )
                for name, group, kp, kd in (("legs", LEGS, 55., 2.), ("wheels", WHEELS, 0., 2.))
            },
        )
        self.events = DomainEvents() if self.randomize else None
        if self.him:
            self.action_space = 16
            self.observation_space = FRAME * HISTORY

    def configure_terrain(self):
        terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                     num_envs=self.scene.num_envs,
                                     env_spacing=self.scene.env_spacing)
        if self.rough:
            terrain.terrain_type = "generator"
            terrain.max_init_terrain_level = 0
            terrain.terrain_generator = TerrainGeneratorCfg(
                seed=self.seed, curriculum=True, size=(8., 8.), num_rows=5, num_cols=max(8, math.ceil(self.scene.num_envs / 128)),
                sub_terrains={
                    "plane": MeshPlaneTerrainCfg(proportion=.60),
                    "slope": HfPyramidSlopedTerrainCfg(proportion=.40, slope_range=(0., .18), platform_width=2.5),
                },
            )
        self.scene.terrain = terrain


class AS2WEnv(DirectRLEnv):
    def __init__(self, cfg, **kwargs):
        super().__init__(cfg, **kwargs)
        names = self.robot.joint_names
        self.leg_ids = [names.index(name) for name in LEGS]
        self.wheel_ids = [names.index(name) for name in WHEELS]
        self.ids = self.leg_ids + self.wheel_ids
        self.command = torch.zeros(self.num_envs, 3, device=self.device)
        self.target = torch.zeros(self.num_envs, 16, device=self.device)
        self.previous = torch.zeros_like(self.target)
        self.previous_delta = torch.zeros_like(self.target)
        self.exercise = None
        self.history = torch.zeros(self.num_envs, HISTORY, FRAME, device=self.device)
        self.terminal_critic = torch.zeros(self.num_envs, FRAME + 3, device=self.device)
        self.alpha_leg, self.alpha_wheel = (
            -math.expm1(-2 * math.pi * CONTROL[key] * self.step_dt)
            for key in ("leg_target_cutoff_hz", "wheel_target_cutoff_hz")
        )

    def _setup_scene(self):
        self.robot = self.scene["robot"]
        self.terrain = self.scene.terrain

    def _pre_physics_step(self, actions):
        interval = round(2.5 / self.step_dt)
        env_ids = (self.episode_length_buf.remainder(interval) == 0).nonzero(as_tuple=True)[0]
        self._sample_commands(env_ids)
        if self.exercise is not None:
            self.command[:] = self.exercise
        actions = actions.clamp(-1., 1.)
        wheel = actions[:, 12:] * 12. if self.cfg.him else self.command[:, :1].expand(-1, 4) / RADIUS
        self.target[:, :12].lerp_(actions[:, :12] * .25, self.alpha_leg)
        self.target[:, 12:].lerp_(wheel.clamp(-12., 12.), self.alpha_wheel)
        if not self.cfg.him:
            zero = (wheel == 0) & (self.target[:, 12:].abs() < .02)
            self.target[:, 12:].masked_fill_(zero, 0.)

    def _apply_action(self):
        pos = tensor(self.robot.data.default_joint_pos)[:, self.leg_ids] + self.target[:, :12]
        self.robot.set_joint_position_target(pos, joint_ids=self.leg_ids)
        self.robot.set_joint_velocity_target(self.target[:, 12:], joint_ids=self.wheel_ids)

    def _frame(self):
        d = self.robot.data
        q = (tensor(d.joint_pos) - tensor(d.default_joint_pos))[:, self.ids]
        q[:, 12:] = 0.
        previous = torch.cat((self.target[:, :12] / .25, self.target[:, 12:] / 12.), -1)
        return torch.cat((self.command, tensor(d.root_ang_vel_b), tensor(d.projected_gravity_b),
                          q, tensor(d.joint_vel)[:, self.ids] * .05, previous,
                          torch.full_like(q[:, :1], HEIGHT)), -1)

    def _get_observations(self):
        frame = self._frame()
        critic = torch.cat((frame, tensor(self.robot.data.root_lin_vel_b)), -1)
        if self.cfg.him:
            old = self.history[:, :-1].clone()
            self.history[:, 1:] = torch.where((self.episode_length_buf == 0)[:, None, None],
                                               frame[:, None], old)
            self.history[:, 0] = frame
            frame = self.history.flatten(1).clone()
        return {"policy": frame, "critic": critic}

    def _get_dones(self):
        d = self.robot.data
        failed = (tensor(d.projected_gravity_b)[:, 2] > -.52)
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        if self.cfg.him:
            self.terminal_critic[:] = torch.cat((self._frame(), tensor(d.root_lin_vel_b)), -1)
        return failed, timeout

    def _get_rewards(self):
        d = self.robot.data
        v, w, g = map(tensor, (d.root_lin_vel_b, d.root_ang_vel_b, d.projected_gravity_b))
        xy_error = (v[:, :2] - self.command[:, :2]).square().sum(1)
        yaw_error = (w[:, 2] - self.command[:, 2]).square()
        q = (tensor(d.joint_pos) - tensor(d.default_joint_pos))[:, self.leg_ids]
        delta = self.target - self.previous
        jerk = delta - self.previous_delta
        self.previous.copy_(self.target)
        self.previous_delta.copy_(delta)
        still = self.command.abs().sum(1) < .03
        reward = 3 * (-xy_error / .25).exp() + 2 * (-yaw_error / .25).exp()
        reward -= .5 * v[:, 2].square() + .1 * w[:, :2].square().sum(1) + .5 * g[:, :2].square().sum(1)
        reward -= (.2 + still) * q.square().sum(1)
        reward -= .06 * (delta[:, :12].square() + jerk[:, :12].square()).sum(1)
        reward -= .0005 * delta[:, 12:].square().sum(1)
        reward -= still * .02 * tensor(d.joint_vel)[:, self.wheel_ids].square().sum(1)
        return self.step_dt * reward - .5 * self.reset_terminated

    def _sample_commands(self, ids):
        n = len(ids)
        cmd = (torch.rand(n, 3, device=self.device) * 2 - 1) * self.command.new_tensor([.8, .45, .8])
        mode = torch.randint(0, 5, (n,), device=self.device)
        cmd[mode == 0] = 0
        for axis in range(3):
            cmd[(mode >= 1) & (mode <= 3) & (mode != axis + 1), axis] = 0
        self.command[ids] = cmd

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        d = self.robot.data
        if self.cfg.rough:
            displacement = (tensor(d.root_pos_w)[env_ids, :2] - self.terrain.env_origins[env_ids, :2]).norm(dim=1)
            eligible = self.episode_length_buf[env_ids] > 50
            self.terrain.update_env_origins(env_ids, eligible & (displacement > 3.),
                                            eligible & (displacement < .5) & (self.command[env_ids].norm(dim=1) > .2))
        super()._reset_idx(env_ids)
        state = tensor(d.default_root_state)[env_ids].clone()
        state[:, :3] += self.terrain.env_origins[env_ids]
        q = tensor(d.default_joint_pos)[env_ids].clone()
        dq = tensor(d.default_joint_vel)[env_ids].clone()
        if self.cfg.randomize:
            q[:, self.leg_ids] += (torch.rand_like(q[:, self.leg_ids]) * 2 - 1) * .03
        self.robot.write_root_pose_to_sim(state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(q, dq, env_ids=env_ids)
        self.target[env_ids] = 0.
        self.target[env_ids[:, None], torch.arange(12, device=self.device)] = (
            q[:, self.leg_ids] - tensor(d.default_joint_pos)[env_ids][:, self.leg_ids]
        )
        self.previous[env_ids] = self.target[env_ids]
        self.previous_delta[env_ids] = 0.
        self._sample_commands(env_ids)
