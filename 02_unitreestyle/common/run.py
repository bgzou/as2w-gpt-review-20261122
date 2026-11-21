import argparse
import hashlib
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=("unitree_style", "himloco"), required=True)
parser.add_argument("--num-envs", type=int, default=16)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--flat", action="store_true")
parser.add_argument("--smoke", action="store_true")
parser.add_argument("--iterations", type=int, default=0)
parser.add_argument("--out", type=Path)
parser.add_argument("--asset", type=Path, help="Versioned AS2W URDF candidate")
parser.add_argument("--rollout-steps", type=int)
parser.add_argument("--resume", type=Path)
parser.add_argument("--resume-update", type=int)
parser.add_argument("--previous-best", type=Path)
args = parser.parse_args()
if args.rollout_steps is not None and args.rollout_steps < 1:
    parser.error("--rollout-steps must be positive")
launcher = AppLauncher(headless=True)

try:
    import gymnasium as gym
    import torch
    from environment import AS2WCfg, AS2WEnv, ROOT
    for him, task_id in ((False, "AS2W-UnitreeStyle-Rough-v0"), (True, "AS2W-HIMLoco-Rough-v0")):
        gym.register(task_id, entry_point=AS2WEnv, disable_env_checker=True,
                     kwargs={"cfg": AS2WCfg(him=him)})
    task_id = "AS2W-HIMLoco-Rough-v0" if args.mode == "himloco" else "AS2W-UnitreeStyle-Rough-v0"
    cfg = AS2WCfg(him=args.mode == "himloco", rough=not args.flat, seed=args.seed, randomize=bool(args.iterations) or not args.smoke)
    cfg.scene.num_envs = args.num_envs
    if args.asset:
        asset = args.asset.resolve(strict=True)
        digest = hashlib.sha256(asset.read_bytes()).hexdigest()
        cfg.scene.robot.spawn.asset_path = str(asset)
        cfg.scene.robot.spawn.usd_dir = str(ROOT / "assets/generated_candidates" / digest / args.mode)
        print(f"AS2W_ASSET {asset} sha256={digest}", flush=True)
    cfg.configure_terrain()
    raw = gym.make(task_id, cfg=cfg).unwrapped
    try:
        if args.smoke:
            obs, _ = raw.reset()
            assert obs["policy"].shape == (args.num_envs, cfg.observation_space)
            assert abs(raw.step_dt - .02) < 1e-8
            actions = torch.zeros(args.num_envs, cfg.action_space, device=raw.device)
            for command in ((0.,0.,0.), (.3,0.,0.), (-.3,0.,0.), (0.,.2,0.), (0.,0.,.4)):
                raw.exercise = torch.tensor(command, device=raw.device)
                if cfg.him:
                    actions[:, 12:] = command[0]
                for _ in range(30):
                    obs, reward, terminated, timeout, _ = raw.step(actions)
                    assert all(x.isfinite().all() for x in (*obs.values(), reward, raw.target))
                if not cfg.him:
                    expected = command[0] / .086
                    assert torch.allclose(raw.target[:, 12:], torch.full_like(raw.target[:, 12:], expected), atol=.03)
                else:
                    assert torch.allclose(raw.target[:, 12:], actions[:, 12:] * 12., atol=.03)
            raw.episode_length_buf[:] = raw.max_episode_length - 1
            obs, reward, terminated, timeout, _ = raw.step(actions)
            assert timeout.all() and all(x.isfinite().all() for x in (*obs.values(), reward))
            assert raw.target[:, 12:].abs().max() == 0.
            if cfg.him:
                history = obs["policy"].reshape(args.num_envs, 6, 58)
                assert torch.allclose(history, history[:, :1].expand_as(history))
            print(f"AS2W_SMOKE_OK {task_id}, terrain={cfg.rough}, obs={cfg.observation_space}, actions={cfg.action_space}", flush=True)
        if args.iterations:
            from training import train
            train(raw, args, args.out or ROOT / args.mode / "logs" / "smoke")
    finally:
        raw.close()
except Exception:
    traceback.print_exc()
    launcher.app.close()
    raise SystemExit(1)
else:
    launcher.app.close()
