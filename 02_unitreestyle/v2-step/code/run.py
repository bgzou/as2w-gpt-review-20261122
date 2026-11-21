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
parser.add_argument("--dr-level",type=int,choices=(0,1,2,3),required=True)
parser.add_argument("--resume",type=Path,required=True)
parser.add_argument("--resume-update",type=int,required=True)
parser.add_argument("--terrain", choices=("flat",), default="flat")
parser.add_argument("--stop-course", action="store_true")
parser.add_argument("--stop-reward", action="store_true")
parser.add_argument("--rolling-light-load-weight", type=float, default=0.)
parser.add_argument("--command-hold-s", type=float, choices=(2.5, 4.0, 5.0), default=2.5)
parser.add_argument("--reward-config", type=Path, default=None,
                    help="reward weights JSON; defaults to reward_config.json next to task.py")
parser.add_argument("--gait-phase", action="store_true",
                    help="add a sin/cos gait clock to the observation and enable phase-scheduled lift reward")
parser.add_argument("--gait-period-s", type=float, default=1.0,
                    help="gait clock period in seconds")
parser.add_argument("--step-only", action="store_true",
                    help="sample only vy/wz commands; vx rolling is excluded from this task")
parser.add_argument("--right-only", action="store_true",
                    help="with --step-only, bias sampling to right translation")
args=parser.parse_args()
if not 0. <= args.rolling_light_load_weight <= 1.:
    parser.error("--rolling-light-load-weight must be in [0, 1]")
if args.stop_reward and not args.stop_course:
    parser.error("--stop-reward requires --stop-course")
if args.stop_course and args.command_hold_s != 2.5:
    parser.error("--stop-course uses fixed 2.5 s intervals")
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
