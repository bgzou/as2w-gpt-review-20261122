"""Show PPO progress and recent TensorBoard scalar trends inside a terminal."""
import json
from pathlib import Path
import sys
import time
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

run = Path(sys.argv[1])
while True:
    stages = sorted((p for p in run.glob("u*/train") if (p/"run.json").exists()),
                    key=lambda p: int(p.parent.name[1:]))
    print("\033[2J\033[H", end="")
    if stages:
        stage = stages[-1]
        cfg = json.loads((stage/"run.json").read_text())
        events = EventAccumulator(str(stage)).Reload()
        tags = events.Tags()["scalars"]
        loss = events.Scalars("Loss/value") if "Loss/value" in tags else []
        if loss:
            completed = loss[-1].step + 1
            print(f"Completed PPO updates: {completed}   |   this stage: {completed-cfg['offset']}/{cfg['iterations']}")
            print("Latest event:",time.strftime("%F %T",time.localtime(loss[-1].wall_time)))
        print(stage)
        print("Recent 60 updates (each row scaled independently)")
        for tag in ("Train/mean_reward","Train/mean_episode_length","Loss/value","Loss/surrogate","Policy/mean_std","Perf/total_fps"):
            if tag in tags:
                v=[e.value for e in events.Scalars(tag)[-60:]]
                if v:
                    lo,hi=min(v),max(v)
                    line="".join("._-:=+*#%@"[max(0,min(9,round(9*(x-lo)/(hi-lo))))] if hi>lo else "-" for x in v)
                    print(f"{tag:26} {v[-1]:10.4g}  {line}")
    print("Refresh 10 s  |  detach: Ctrl-b then d",flush=True)
    time.sleep(10)
