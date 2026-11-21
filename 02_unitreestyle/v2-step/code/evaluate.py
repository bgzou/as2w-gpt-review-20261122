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
p.add_argument("--terrain",choices=["flat","slope"],required=True)
p.add_argument("--dr-level",type=int,choices=(0,1,2,3),required=True)
p.add_argument("--seed",type=int,default=319)
p.add_argument("--leg-hz",type=float,default=None)
p.add_argument("--stop-replay", action="store_true", help="Switch all trials to zero at 2.5 s")
p.add_argument("--gait-phase", action="store_true", help="60-dim observation with gait clock")
p.add_argument("--cmd-scale", type=float, default=1.0, help="scale every scenario command")
args=p.parse_args()
root=Path(__file__).resolve().parents[1]
app=AppLauncher(headless=True).app
raw=None
cfg=None
try:
    import numpy as np
    import torch
    from environment import WHEELS, LEGS, tensor
    from task import build_task
    from isaaclab.utils.warp import raycast_mesh
    settings=json.loads((args.snapshot/"run.json").read_text())
    args.asset=Path(settings["asset"])
    args.gait_phase=bool(getattr(args,"gait_phase",False)) or bool(settings.get("gait_phase",False))
    args.gait_period_s=float(settings.get("gait_period_s",1.0))
    args.out=args.snapshot/(args.terrain+f"-dr{args.dr_level}-state")
    args.num_envs=48
    raw=build_task(args)
    if args.leg_hz is not None:
        raw.alpha_leg=1.-math.exp(-2.*math.pi*args.leg_hz*raw.step_dt)
    weight=args.snapshot/"best.pt"
    if True:
        from tensordict import TensorDict
        from isaaclab_rl.rsl_rl import (RslRlVecEnvWrapper,create_rsl_rl_runner,
                                       check_rsl_rl_version,handle_deprecated_rsl_rl_cfg)
        from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg
        config=UnitreeGo2RoughPPORunnerCfg()
        config.device=raw.device
        config.num_steps_per_env=1
        config.obs_groups={"actor":["policy"],"critic":["critic"]}
        config=handle_deprecated_rsl_rl_cfg(config,check_rsl_rl_version())
        runner=create_rsl_rl_runner(RslRlVecEnvWrapper(raw),config,log_dir=None)
        runner.load(str(weight),load_cfg={"actor":True,"critic":True})
        model=runner.get_inference_policy()
        def policy(obs):
            return model(TensorDict(obs,batch_size=[raw.num_envs],device=raw.device))
    scenarios={
        "stand":(0.,0.,0.),"forward":(.60,0.,0.),"reverse":(-.60,0.,0.),
        "left":(0.,.30,0.),"right":(0.,-.30,0.),
        "yaw_left":(0.,0.,.60),"yaw_right":(0.,0.,-.60),"mixed":(.45,.20,.45),
    }
    if args.terrain=="slope":
        scenarios.update(forward=(.35,0.,0.),reverse=(-.35,0.,0.),mixed=(.28,.20,.45))
        cols=raw.cfg.scene.terrain.terrain_generator.num_cols
        raw.terrain.terrain_types[:]=torch.arange(raw.num_envs,device=raw.device)%cols
        raw.terrain.terrain_levels[:]=2
        raw.terrain.env_origins[:]=raw.terrain.terrain_origins[2,raw.terrain.terrain_types]
    labels=[label for label in scenarios for _ in range(6)]
    command=torch.tensor([scenarios[x] for x in labels],device=raw.device)*getattr(args,"cmd_scale",1.0)
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
    poses, joint_q, joint_dq, joint_applied_torque = [], [], [], []
    clearance, normal_load, slip, pushes = [], [], [], []
    sampled_drive_delay=None
    terrain_normal=None
    capture = args.dr_level == 0
    tape_names=("q_rad","dq_rad_s","gyro_rad_s","quat_wxyz","command_si","prior_target",
                "observation","raw_action","next_target","leg_q_target","wheel_dq_target","reset")
    tape = {k:[] for k in tape_names}
    with torch.inference_mode():
        raw.exercise=torch.zeros_like(command)
        obs,_=raw.reset()
        raw.command.copy_(raw.exercise)
        obs=raw._get_observations()
        if args.terrain=="slope":
            roots=tensor(d.root_state_w).clone()
            roots[:,0]=raw.terrain.env_origins[:,0]+2.85
            roots[:,1]=raw.terrain.env_origins[:,1]
            p=roots[:,:3].clone()
            p[:,2]=raw.terrain.env_origins[:,2]+1.0
            hits,_,normal,_=raycast_mesh(p,raw.down.expand_as(p),raw.mesh,max_dist=3,return_normal=True)
            assert torch.isfinite(hits).all() and torch.isfinite(normal).all()
            terrain_normal=normal.detach().cpu().numpy().copy()
            # Estimate ground grade at robot scale, since individual
            # heightfield triangles may be level under 5 mm quantization.
            sample=p.clone()
            grades=[]
            for axis in (0,1):
                sample[:]=p
                sample[:,axis]+=.30
                high,*_=raycast_mesh(sample,raw.down.expand_as(p),raw.mesh,max_dist=3)
                sample[:,axis]-=.60
                low,*_=raycast_mesh(sample,raw.down.expand_as(p),raw.mesh,max_dist=3)
                grades.append((high[:,2]-low[:,2])/.60)
            grade=torch.rad2deg(torch.atan(torch.stack(grades).norm(dim=0)))
            assert torch.isfinite(grade).all() and (grade>=2.).all(), grade
            measured_grade=grade.detach().cpu().numpy().tolist()
            roots[:,2]=hits[:,2]+.39
            by_trial=torch.arange(raw.num_envs,device=raw.device)%3
            yaw=torch.where(by_trial==0,0.,torch.where(by_trial==1,math.pi,math.pi/2))
            roots[:,3:7]=0.
            roots[:,5]=(yaw/2).sin()
            roots[:,6]=(yaw/2).cos()
            raw.robot.write_root_pose_to_sim(roots[:,:7])
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
        if args.dr_level>=2:
            sampled_drive_delay=raw.dr.delay.detach().cpu().numpy().astype(int).tolist()
        if args.dr_level==3:
            raw.dr.push_time.copy_(1.5+.5*torch.linspace(0,1,raw.num_envs,device=raw.device))
        for i in range(300 if args.terrain=="flat" else 200):
            if args.stop_replay and i == 125:
                command = torch.zeros_like(command)
                raw.exercise = command
                raw.command.copy_(command)
                obs = raw._get_observations()
            action=policy(obs)
            if args.dr_level==3:
                pushes.append((raw.dr.push_time<=raw.step_dt).detach().cpu().numpy().copy())
            if capture:
                pre=(tensor(d.joint_pos)[:,raw.ids],tensor(d.joint_vel)[:,raw.ids],
                     tensor(d.root_ang_vel_b),tensor(d.root_quat_w)[:,[3,0,1,2]],
                     command,raw.target,obs["policy"],action)
                for key,value in zip(tape_names[:8],pre,strict=True):
                    tape[key].append(value.detach().cpu().numpy().copy())
            obs,_,term,timeout,_=raw.step(action)
            if capture:
                post=(raw.target,reference+raw.target[:,:12],raw.target[:,12:],term|timeout)
                for key,value in zip(tape_names[8:],post,strict=True):
                    tape[key].append(value.detach().cpu().numpy().copy())
            clearance.append(raw.swing.last_clearance.detach().cpu().numpy().copy())
            normal_load.append(raw.last_normal_force.detach().cpu().numpy().copy())
            slip.append(raw.swing.last_slip.detach().cpu().numpy().copy())
            targets.append(torch.stack((reference+action[:,:12].tanh()*.25,
                                        reference+raw.target[:,:12]),1).cpu().numpy().copy())
            poses.append(torch.cat((tensor(d.root_pos_w)-scene_origins,
                                    tensor(d.root_quat_w)),1).detach().cpu().numpy().copy())
            joint_q.append(tensor(d.joint_pos)[:,raw.ids].detach().cpu().numpy().copy())
            joint_dq.append(tensor(d.joint_vel)[:,raw.ids].detach().cpu().numpy().copy())
            joint_applied_torque.append(tensor(d.applied_torque)[:,raw.ids].detach().cpu().numpy().copy())
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
    name = args.terrain + ("-stop" if args.stop_replay else "")
    if args.stop_replay:
        loads = np.stack(normal_load)
        positions = np.stack(poses)[..., :2]
        for label in report:
            mask = np.array(labels) == label
            window = slice(148, None)
            report[label]["stop"] = {
                "post_switch_survived": int(alive[-1, mask].sum()),
                "drift_from_switch_m": np.linalg.norm(positions[-1, mask] - positions[124, mask], axis=-1).tolist(),
                "settled_speed_p95_m_s": float(np.quantile(np.linalg.norm(arrays["vel"][window, mask, :2], axis=-1), .95)),
                "settled_tilt_p95_deg": float(np.quantile(arrays["tilt"][window, mask], .95)),
                "settled_four_wheel_fraction": float((loads[window, mask] >= 3.).all(axis=-1).mean()),
            }
    path=args.snapshot/(name+".json")
    path.write_text(json.dumps(report,indent=2)+"\n")
    replay_context=dict(seed=args.seed,terrain=args.terrain,randomization_level=args.dr_level,
                        period_s=raw.step_dt,labels=labels,
                        sourced_sensor_tape=capture,drive_delay_steps_by_trial=sampled_drive_delay,
                        slope_spawn_normal_world=terrain_normal.tolist() if terrain_normal is not None else None,
                        slope_spawn_grade_deg=measured_grade if terrain_normal is not None else None,
                        push_frames_by_trial=[np.flatnonzero(x).tolist() for x in np.stack(pushes).T] if pushes else None)
    context_name = "stop-replay-context.json" if args.stop_replay else "replay-context.json"
    (args.snapshot/context_name).write_text(json.dumps(replay_context,indent=2)+"\n")
    np.savez_compressed(args.snapshot/(name+".npz"),labels=np.array(labels), pose=np.stack(poses), q=np.stack(joint_q),
                        joints=np.array(LEGS+WHEELS), joint_dq_rad_s=np.stack(joint_dq),
                        isaac_actuator_applied_torque_estimate_nm=np.stack(joint_applied_torque),
                        wheel_data=np.stack(wheel_data), target_trace=np.stack(targets),clearance=np.stack(clearance),normal_load=np.stack(normal_load),lateral_slip=np.stack(slip),**arrays)
    if capture:
        np.savez_compressed(args.snapshot/(name+"-sensor-tape.npz"),dt_s=.02,
                            labels=np.array(labels),**{k:np.stack(v) for k,v in tape.items()})
    print("AS2W_EVAL_OK",path,flush=True)
except Exception:
    traceback.print_exc()
    raise
finally:
    if raw is not None:
        raw.close()
    app.close()
