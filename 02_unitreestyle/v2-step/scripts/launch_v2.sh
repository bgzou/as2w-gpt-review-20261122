#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-/root/as2w/preflight/isaac-python}"
: "${RESUME:?Set RESUME to an existing 58-observation, 12-action Unitree-style checkpoint}"
OUT="${OUT:-/root/gpufree-data/as2w-runs/unitree-flat-v2-$(date +%Y%m%d-%H%M%S)/train}"
ASSET="${ASSET:-$TASK_DIR/assets/as2w-t2-v2-mu025.usd}"
[[ -f "$RESUME" ]] || { printf 'RESUME not found: %s\n' "$RESUME" >&2; exit 1; }
[[ -f "$ASSET" ]] || { printf 'missing converted USD: %s\n' "$ASSET" >&2; exit 1; }
[[ ! -e "$OUT" ]] || { printf 'OUT already exists: %s\n' "$OUT" >&2; exit 1; }
export PYTHONUNBUFFERED=1
exec "$ISAAC_PYTHON" "$TASK_DIR/run.py" --out "$OUT" \
  --asset "$ASSET" --terrain flat \
  --num-envs "${NUM_ENVS:-8192}" --rollout-steps 48 --iterations 2000 \
  --dr-level 0 --seed 42 --resume "$RESUME" --resume-update 0
