"""Keep the best sampled policy alongside normal runner checkpoints."""
from collections import deque
import json
import math
import shutil

import torch


class BestCheckpoint:
    def __init__(self, env, runner, out, interval, previous=None):
        self.env = env
        self.runner = runner
        self.out = out
        self.interval = interval
        self.returns = deque(maxlen=256)
        self.running = torch.zeros(env.num_envs, device=env.device)
        self.best = -math.inf
        self.updates = 0
        self.start_iteration = runner.current_learning_iteration
        if previous:
            info = json.loads((previous / "best.json").read_text())
            self.best = float(info["score"])
            shutil.copy2(previous / "best.pt", out / "best.pt")
            shutil.copy2(previous / "best.json", out / "best.json")
        self.original_step = env.step
        self.original_update = runner.alg.update
        env.step = self.step
        runner.alg.update = self.update

    def step(self, actions):
        observation, reward, terminated, timeout, extras = self.original_step(actions)
        self.running.add_(reward.view(-1))
        finished = terminated | timeout
        completed_returns = self.running[finished].detach().cpu().tolist()
        self.returns.extend(value for value in completed_returns if math.isfinite(value))
        self.running[finished] = 0.0
        return observation, reward, terminated, timeout, extras

    def update(self, *args, **kwargs):
        # Save before the update. The just-collected rollout used these weights.
        if len(self.returns) >= 64 and (self.best == -math.inf or self.updates % self.interval == 0):
            score = sum(self.returns) / len(self.returns)
            if score > self.best:
                temporary = self.out / ".best.pt.tmp"
                self.runner.save(str(temporary))
                temporary.replace(self.out / "best.pt")
                info = {"metric": "mean_recent_training_episode_return",
                        "episodes": len(self.returns), "score": score,
                        "completed_updates": self.start_iteration + self.updates}
                metadata = self.out / ".best.json.tmp"
                metadata.write_text(json.dumps(info, indent=2) + "\n")
                metadata.replace(self.out / "best.json")
                self.best = score
                print("AS2W_BEST", json.dumps(info), flush=True)
        result = self.original_update(*args, **kwargs)
        self.updates += 1
        if self.updates == 1 or self.updates % 10 == 0:
            print(f"AS2W_UPDATE {self.updates}", flush=True)
        return result
