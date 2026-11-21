import hashlib
import json
from pathlib import Path

import torch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg, check_rsl_rl_version
from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg
from best_checkpoint import BestCheckpoint
from policy import LIMIT

def train(env, args, out):
    out.mkdir(parents=True, exist_ok=False)
    env.exercise = None
    cfg = UnitreeGo2RoughPPORunnerCfg()
    cfg.device = env.device
    cfg.seed = 42
    cfg.num_steps_per_env = 48
    cfg.obs_groups = {"actor":["policy"], "critic":["critic"]}
    cfg = handle_deprecated_rsl_rl_cfg(cfg, check_rsl_rl_version())
    cfg.actor.distribution_cfg.init_std = .30
    cfg.actor.distribution_cfg.learn_std = False
    cfg.algorithm.entropy_coef = 0.
    cfg.algorithm.learning_rate = 1e-4
    cfg.algorithm.schedule = "fixed"
    cfg.save_interval = 100
    runner = create_rsl_rl_runner(RslRlVecEnvWrapper(env, clip_actions=None), cfg, log_dir=str(out))
    head = runner.alg.actor.mlp[-1]
    with torch.no_grad():
        head.weight.zero_()
        head.bias.zero_()
    runner.alg.critic.load_state_dict(env.base_ckpt["critic_state_dict"])
    digest = hashlib.sha256((args.root/"source/best.pt").read_bytes()).hexdigest()
    if args.resume:
        previous = torch.load(args.resume, map_location="cpu", weights_only=False)
        assert previous["infos"]["as2w_residual_v1"]["base_sha256"] == digest
        assert previous["infos"]["as2w_residual_v1"].get("profile","original") == args.profile
        runner.load(str(args.resume))
    runner.current_learning_iteration = args.offset
    std = runner.alg.actor.distribution.std_param
    assert not std.requires_grad and torch.allclose(std, torch.full_like(std,.3))
    info = {"profile":args.profile,"base_sha256":digest, "base_actor":env.base_ckpt["actor_state_dict"], "limit":LIMIT}
    save = runner.save
    def save_composite(path, infos=None):
        save(path, {**(infos or {}), "as2w_residual_v1":info})
    runner.save = save_composite
    config = json.loads((args.root/"source/run.json").read_text())
    config.update(profile=args.profile,policy_kind="as2w_residual_v1", num_envs=env.num_envs, rollout_steps=48,
                  offset=args.offset, iterations=args.iterations, std=.30, limit=LIMIT, base_sha256=digest)
    (out/"run.json").write_text(json.dumps(config,indent=2))
    BestCheckpoint(env,runner,out,interval=10)
    initial = head.weight.detach().clone()
    runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
    change = (head.weight-initial).abs().max().item()
    assert 0 < change < float("inf") and torch.isfinite(head.bias).all()
    runner.save(str(out/"endpoint.pt"))
    print("RESIDUAL_UPDATE_OK", change, flush=True)
