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
parser.add_argument("--dr-level",type=int,choices=(1,2,3),required=True)
parser.add_argument("--resume",type=Path,required=True)
parser.add_argument("--resume-update",type=int,required=True)
args=parser.parse_args()
args.previous_best=None
app=AppLauncher(headless=True).app
env=None
try:
    from task import build_task
    from training import train
    env=build_task(args)
    train(env,args,args.out)
except BaseException:
    traceback.print_exc()
    raise
finally:
    if env is not None: env.close()
    app.close()
