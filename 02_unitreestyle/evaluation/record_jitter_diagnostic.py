import argparse
import json
import math
from pathlib import Path
import sys
import traceback

from isaaclab.app import AppLauncher

p=argparse.ArgumentParser()
p.add_argument("--mode", choices=["unitree_style","himloco"], required=True)
p.add_argument("--snapshot", type=Path, required=True)
p.add_argument("--terrain",choices=["flat","rough"],required=True)
p.add_argument("--leg-hz",type=float,default=10.)
args=p.parse_args()
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/"common"))
app=AppLauncher(headless=True).app
raw=None
try:
    import numpy as np
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, LEGS, WHEELS, tensor
    @configclass
    class EvalScene(AS2WSceneCfg):
        contacts: ContactSensorCfg | None = None
    @configclass
    class EvalCfg(AS2WCfg):
        scene=EvalScene(num_envs=48,env_spacing=3.,replicate_physics=True)
    cfg=EvalCfg(him=args.mode=="himloco",rough=args.terrain=="rough",randomize=False, seed=319)
    settings=json.loads((args.snapshot/"run.json").read_text())
    cfg.scene.robot.spawn.asset_path=str((root / settings["asset"]).resolve(strict=True))
    cfg.scene.robot.spawn.usd_dir=str(args.snapshot / (args.terrain+"_usd"))
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")
    cfg.configure_terrain()
    raw=AS2WEnv(cfg)
    if args.leg_hz!=10.:
        raw.alpha_leg=1.-math.exp(-2.*math.pi*args.leg_hz*raw.step_dt)
    weight=args.snapshot/"best.pt"
    if cfg.him:
        sys.path.insert(0,str(root/"upstream/HIMLoco/rsl_rl"))
        from rsl_rl.modules import HIMActorCritic
        model=HIMActorCritic(raw.cfg.observation_space,61,58,16).to(raw.device)
        model.load_state_dict(torch.load(weight,map_location=raw.device,weights_only=True)["model_state_dict"])
        model.eval()
        def policy(obs):
            return model.act_inference(obs["policy"])
    else:
        from tensordict import TensorDict
        from isaaclab_rl.rsl_rl import (RslRlVecEnvWrapper,create_rsl_rl_runner,
                                       check_rsl_rl_version,handle_deprecated_rsl_rl_cfg)
        from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg
        config=UnitreeGo2RoughPPORunnerCfg()
        config.device=raw.device
        config.num_steps_per_env=1
        config.obs_groups={"actor":["policy"],"critic":["critic"]}
        config=handle_deprecated_rsl_rl_cfg(config,check_rsl_rl_version())
        runner=create_rsl_rl_runner(RslRlVecEnvWrapper(raw,clip_actions=1.),config,log_dir=None)
        runner.load(str(weight))
        model=runner.get_inference_policy()
        def policy(obs):
            return model(TensorDict(obs,batch_size=[raw.num_envs],device=raw.device))
    if cfg.rough:
        raw.terrain.terrain_levels[:] = 2
        raw.terrain.env_origins[:] = raw.terrain.terrain_origins[2,raw.terrain.terrain_types]
        probs=np.cumsum([x.proportion for x in cfg.scene.terrain.terrain_generator.sub_terrains.values()])
        labels=[list(cfg.scene.terrain.terrain_generator.sub_terrains)[np.searchsorted(probs,int(x)/8+.001,side="right")] for x in raw.terrain.terrain_types.tolist()]
        command=torch.tensor([.60,0.,0.],device=raw.device).repeat(raw.num_envs,1)
    else:
        scenarios={
            "stand":(0.,0.,0.),"forward":(.60,0.,0.),"reverse":(-.60,0.,0.),
            "left":(0.,.30,0.),"right":(0.,-.30,0.),
            "yaw_left":(0.,0.,.60),"yaw_right":(0.,0.,-.60),
            "mixed":(.45,.20,.45),
        }
        labels=[label for label in scenarios for _ in range(6)]
        command=torch.tensor([scenarios[x] for x in labels],device=raw.device)
    if cfg.rough:
        from isaaclab.utils import instantiate
        terrain = instantiate(cfg.scene.terrain.terrain_generator, device=raw.device)
        i = labels.index("steps")
        center = raw.terrain.env_origins[i].detach().cpu().numpy()
        mesh = terrain.terrain_mesh
        keep = ((np.abs(mesh.triangles[:,:,:2] - center[:2]) <= 4.001).all((1,2)))
        tile = mesh.submesh([np.flatnonzero(keep)], append=True)
        assert len(tile.faces) > 0
        tile.apply_translation(-center)
        tile.export(args.snapshot/"steps.obj")
    d=raw.robot.data
    contact=raw.scene["contacts"]
    feet=[contact.body_names.index(name) for name in ("FL_foot","FR_foot","RL_foot","RR_foot")]
    nonfeet=[i for i in range(len(contact.body_names)) if i not in feet]
    wheels=[raw.robot.body_names.index(name) for name in WHEELS]
    wheel_data=[]
    targets=[]
    reference=tensor(d.default_joint_pos)[:,raw.leg_ids].clone()
    perturb=torch.Generator(device=raw.device).manual_seed(319)
    series={k:[] for k in ("vel","cmd","fall","tilt","leg_dq","wheel_target","air","nonwheel","dx")}
    poses, joint_q = [], []
    with torch.inference_mode():
        raw.exercise=torch.zeros_like(command)
        obs,_=raw.reset()
        q=tensor(d.joint_pos).clone()
        q[:,raw.leg_ids]+=(torch.rand((raw.num_envs,12),device=raw.device,generator=perturb)-.5)*.03
        raw.robot.write_joint_state_to_sim(q,torch.zeros_like(q))
        for i in range(50):
            obs,_,term,_,_=raw.step(policy(obs))
        origin=tensor(d.root_pos_w).clone()
        scene_origins=raw.terrain.env_origins.clone()
        raw.exercise=command
        # Provide the commanded velocity to the actor on the first commanded step.
        raw.command[:]=command
        obs=raw._get_observations()
        for i in range(300):
            action=policy(obs)
            obs,_,term,_,_=raw.step(action)
            targets.append(torch.stack((reference+action[:,:12].clamp(-1.,1.)*.25,
                                        reference+raw.target[:,:12]),1).cpu().numpy().copy())
            poses.append(torch.cat((tensor(d.root_pos_w)-scene_origins,
                                    tensor(d.root_quat_w)),1).detach().cpu().numpy().copy())
            joint_q.append(tensor(d.joint_pos)[:,raw.ids].detach().cpu().numpy().copy())
            v=tensor(d.root_lin_vel_b)
            w=tensor(d.root_ang_vel_b)
            g=tensor(d.projected_gravity_b)
            normals=tensor(contact.data.net_normal_forces_w)
            force=normals.norm(dim=-1)
            wheel_data.append(torch.cat((
                tensor(d.body_link_pos_w)[:,wheels]-scene_origins[:,None,:],
                tensor(d.body_link_quat_w)[:,wheels],
                tensor(d.body_link_lin_vel_w)[:,wheels],
                tensor(d.body_link_ang_vel_w)[:,wheels],
                normals[:,feet],
                tensor(d.joint_vel)[:,raw.wheel_ids,None],
            ), dim=-1).detach().cpu().numpy().copy())
            values=(torch.cat((v[:,:2],w[:,2:3]),1), command,term,
                    torch.rad2deg(torch.acos((-g[:,2]).clamp(-1,1))),
                    tensor(d.joint_vel)[:,raw.leg_ids].abs().mean(1),
                    raw.target[:,12:].abs().mean(1),
                    (force[:,feet]<3.).float().mean(1),
                    (force[:,nonfeet]>5.).any(dim=-1).float(),
                    tensor(d.root_pos_w)[:,0]-origin[:,0])
            for k,x in zip(series,values,strict=True):
                series[k].append(x.detach().cpu().numpy().copy())
    arrays={k:np.stack(v) for k,v in series.items()}
    assert all(np.isfinite(a).all() for a in arrays.values())
    alive=np.cumsum(arrays["fall"],axis=0)==0
    alive[:50]=False
    report={}
    for label in dict.fromkeys(labels):
        indices=np.flatnonzero(np.array(labels)==label)
        ok=alive[:,indices]
        dist=[]
        for i in indices:
            falls=np.flatnonzero(arrays["fall"][:,i])
            last=falls[0]-1 if len(falls) else len(arrays["fall"])-1
            dist.append(float(arrays["dx"][last,i]) if last>=0 else 0.)
        sel=lambda k: arrays[k][:,indices][ok]
        velocities,commands=sel("vel"),sel("cmd")
        report[label]={
          "n":len(indices),"survived":int(alive[-1,indices].sum()),
          "progress_2_1m":int(((np.array(dist)>=2.1)&alive[-1,indices]).sum()),
          "mean_dx_m":float(np.mean(dist)),
          "vel_mean":velocities.mean(axis=0).tolist() if len(velocities) else None,
          "mae":np.abs(velocities-commands).mean(axis=0).tolist() if len(velocities) else None,
          "tilt_p95_deg":float(np.quantile(sel("tilt"),.95)) if ok.any() else None,
          **{k:float(sel(k).mean()) if ok.any() else None for k in ("air","nonwheel","leg_dq","wheel_target")},
        }
    path=args.snapshot/(args.terrain+".json")
    path.write_text(json.dumps(report,indent=2)+"\n")
    np.savez_compressed(args.snapshot/(args.terrain+".npz"),labels=np.array(labels), pose=np.stack(poses), q=np.stack(joint_q),
                        joints=np.array(LEGS+WHEELS), wheel_data=np.stack(wheel_data), target_trace=np.stack(targets),**arrays)
    print("AS2W_EVAL_OK",path,flush=True)
except Exception:
    traceback.print_exc()
    raise
finally:
    if raw is not None:
        raw.close()
    app.close()
