#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-/root/as2w/preflight/isaac-python}"
: "${RESUME:?Set RESUME to an existing 58-observation, 12-action Unitree-style checkpoint}"
: "${OUT:?Set a new output directory for this stage}"
: "${LEVEL:?Set LEVEL to 0, 1, 2 or 3}"
: "${UPDATES:?Set UPDATES explicitly (use 1 for smoke)}"
case "$LEVEL" in 0|1|2|3) ;; *) echo "Invalid LEVEL: $LEVEL" >&2; exit 2 ;; esac
[[ "$UPDATES" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid UPDATES: $UPDATES" >&2; exit 2; }
ASSET="${ASSET:-$TASK_DIR/assets/as2w-t2-v2-mu025.usd}"
[[ -f "$RESUME" ]] || { printf 'RESUME not found: %s\n' "$RESUME" >&2; exit 1; }
[[ -f "$ASSET" ]] || { printf 'missing converted USD: %s\n' "$ASSET" >&2; exit 1; }
[[ ! -e "$OUT" ]] || { printf 'OUT already exists: %s\n' "$OUT" >&2; exit 1; }
export PYTHONUNBUFFERED=1
exec "$ISAAC_PYTHON" "$TASK_DIR/run.py" --out "$OUT" \
  --asset "$ASSET" --terrain flat \
  --num-envs "${NUM_ENVS:-8}" --rollout-steps 48 --iterations "$UPDATES" \
  --dr-level "$LEVEL" --seed "${SEED:-42}" --resume "$RESUME" --resume-update 0
