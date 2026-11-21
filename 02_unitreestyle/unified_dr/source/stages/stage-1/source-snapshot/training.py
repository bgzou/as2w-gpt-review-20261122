import hashlib
import json
from pathlib import Path
import sys

import torch

from best_checkpoint import BestCheckpoint
from environment import FRAME, HISTORY, ROOT
from domain import SETTINGS


class HIMEnvironmentAdapter:
    """Adapt Isaac Lab transitions to the upstream HIM runner."""

    num_obs = FRAME * HISTORY
    num_one_step_obs = FRAME
    num_privileged_obs = FRAME + 3
    num_actions = 16

    def __init__(self, env):
        self.env = env
        self.num_envs = env.num_envs
        self.max_episode_length = env.max_episode_length

    def reset(self):
        self.obs, _ = self.env.reset()
        return self.get_observations(), self.get_privileged_observations()

    def get_observations(self):
        return self.obs["policy"]

    def get_privileged_observations(self):
        return self.obs["critic"]

    def step(self, actions):
        self.obs, reward, terminated, timeout, extras = self.env.step(actions)
        dones = terminated | timeout
        ids = dones.nonzero(as_tuple=True)[0]
        extras = {**extras, "time_outs": timeout & ~terminated}
        return (self.get_observations(), self.get_privileged_observations(),
                reward, dones, extras, ids, self.env.terminal_critic[ids])


def train(env, args, out):
    env.exercise = None
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    asset = Path(env.cfg.scene.robot.spawn.asset_path).resolve()
    (out / "run.json").write_text(json.dumps({
        "asset": str(asset), "asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
        "num_envs": env.num_envs, "rollout_steps": args.rollout_steps or (100 if env.cfg.him else 24),
        "policy_dt_s": env.step_dt, "seed": args.seed,
        "resume": str(args.resume), "dr_level": args.dr_level,
        "domain_config": SETTINGS, "base_update": args.resume_update,
    }, indent=2) + "\n")
    steps = args.rollout_steps or (100 if env.cfg.him else 24)
    print(f"AS2W_CONFIG envs={env.num_envs} rollout_steps={steps} iterations={args.iterations}", flush=True)
    if env.cfg.him:
        # This interpreter is dedicated to the HIM task. Its legacy rsl_rl
        # contains custom PPO, rollout storage, and successor-state training.
        sys.path.insert(0, str(ROOT / "upstream/HIMLoco/rsl_rl"))
        from rsl_rl.runners import HIMOnPolicyRunner

        cfg = {
            "policy": {"init_noise_std": 1.0, "activation": "elu",
                       "actor_hidden_dims": [512, 256, 128],
                       "critic_hidden_dims": [512, 256, 128]},
            "algorithm": {"value_loss_coef": 1., "use_clipped_value_loss": True,
                          "clip_param": .2, "entropy_coef": .01, "num_learning_epochs": 5,
                          "num_mini_batches": 4, "learning_rate": .001, "schedule": "adaptive",
                          "gamma": .99, "lam": .95, "desired_kl": .01, "max_grad_norm": 1.},
            "runner": {"policy_class_name": "HIMActorCritic", "algorithm_class_name": "HIMPPO",
                       "num_steps_per_env": steps, "save_interval": 100},
        }
        runner = HIMOnPolicyRunner(HIMEnvironmentAdapter(env), cfg, str(out), device=env.device)
    else:
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg, check_rsl_rl_version
        from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg

        cfg = UnitreeGo2RoughPPORunnerCfg()
        cfg.device = env.device
        cfg.seed = args.seed
        cfg.num_steps_per_env = steps
        cfg.obs_groups = {"actor": ["policy"], "critic": ["critic"]}
        cfg = handle_deprecated_rsl_rl_cfg(cfg, check_rsl_rl_version())
        cfg.actor.distribution_cfg.init_std=SETTINGS["optimization"]["fixed_policy_std"]
        cfg.actor.distribution_cfg.learn_std=False
        cfg.algorithm.entropy_coef=0.
        cfg.algorithm.learning_rate=SETTINGS["optimization"]["learning_rate"]
        cfg.algorithm.schedule="fixed"
        cfg.save_interval=100
        runner = create_rsl_rl_runner(RslRlVecEnvWrapper(env, clip_actions=1.), cfg, log_dir=str(out))
    if not env.cfg.him:
        resolved={
            "ppo_config":cfg.to_dict(),"physics":env.cfg.to_dict(),
            "actual_invocation":{"updates":args.iterations,"init_at_random_ep_len":False,
                                 "vec_env_action_clip":1.,"load_actor":True,"load_critic":True,
                                 "load_optimizer":False,"resume_update_index":args.resume_update},
            "policy_period_s":env.step_dt,
            "leg_alpha":env.alpha_leg,"wheel_alpha":env.alpha_wheel,
            "default_joint_position":env.robot.data.default_joint_pos.torch[0,env.ids].cpu().tolist()
                if hasattr(env.robot.data.default_joint_pos,"torch") else env.robot.data.default_joint_pos[0,env.ids].cpu().tolist(),
            "training_source":str(Path(__file__).parent),
        }
        (out/"resolved_config.json").write_text(json.dumps(resolved,indent=2,default=str)+"\n")
    if args.resume:
        runner.load(str(args.resume.resolve()),load_cfg={"actor":True,"critic":True})
        with torch.no_grad():
            runner.alg.actor.distribution.std_param.fill_(SETTINGS["optimization"]["fixed_policy_std"])
        if args.resume_update is not None:
            runner.current_learning_iteration = args.resume_update
        print(f"AS2W_RESUME {args.resume} runner_iteration={runner.current_learning_iteration}", flush=True)
    update=runner.alg.update
    completed=0
    def with_metrics(*a,**kw):
        nonlocal completed
        result=update(*a,**kw)
        completed+=1
        data=dict(zip(env.rollup_names,(env.metric_sums/max(1,env.metric_steps)).cpu().tolist()))
        assert all(torch.isfinite(env.metric_sums))
        env.metric_sums.zero_()
        env.metric_steps=0
        data["update"]=completed
        with (out/"reward-components.jsonl").open("a") as f:
            f.write(json.dumps(data,allow_nan=False)+"\n")
        if completed%250==0:
            tmp=out/f".update-{completed}.tmp"
            runner.save(str(tmp))
            tmp.replace(out/f"update-{completed}.pt")
        if completed in (1, 3) or completed%25==0:
            print("UNIFIED_REWARD",json.dumps(data),flush=True)
        return result
    runner.alg.update=with_metrics
    BestCheckpoint(env, runner, out, interval=3 if env.cfg.him else 10, previous=args.previous_best)
    runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
    runner.save(str(out/"endpoint.pt"))
    print("AS2W_TRAIN_SMOKE_OK", flush=True)
