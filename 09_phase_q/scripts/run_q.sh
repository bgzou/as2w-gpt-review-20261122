#!/usr/bin/env bash
set -euo pipefail
ROOT=/root/gpufree-data/as2w-cpro/phase_klm_20261122/phase_q_20261122
mkdir -p "$ROOT"/{code,params,results,logs,cache}
source /opt/isaaclab-venv/bin/activate
export ISAAC_PATH=/isaac-sim CARB_APP_PATH=/isaac-sim/kit EXP_PATH=/isaac-sim/apps
export PYTHONPATH="${PYTHONPATH:-}"
source /isaac-sim/setup_python_env.sh
export PYTHONPATH="/workspace/isaaclab/source/isaaclab:/workspace/isaaclab/source/isaaclab_physx:$PYTHONPATH"
export LD_LIBRARY_PATH="/isaac-sim/kit/python/lib:$LD_LIBRARY_PATH"
export LD_PRELOAD="/isaac-sim/kit/libcarb.so${LD_PRELOAD:+:$LD_PRELOAD}"
export OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=0 AS2W_USD_CACHE_ROOT=$ROOT/cache
TAG=$1
PARAMS=$ROOT/params/$TAG.json
RESULT=$ROOT/results/$TAG.json
test -f "$PARAMS"
test ! -e "$RESULT"
{
  date -u '+UTC %Y-%m-%dT%H:%M:%SZ'
  cat /workspace/isaaclab/VERSION
  cat /isaac-sim/VERSION
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
  python --version
  sha256sum "$PARAMS" "$ROOT/code/run_one_lock.py" "$ROOT/code/i_common_lock.py"
} > "$ROOT/logs/environment_$TAG.txt"
python -u "$ROOT/code/run_one_lock.py" --source-scripts "$ROOT/code" --params "$PARAMS" \
  > "$ROOT/logs/$TAG.log" 2>&1
test -s "$RESULT"
