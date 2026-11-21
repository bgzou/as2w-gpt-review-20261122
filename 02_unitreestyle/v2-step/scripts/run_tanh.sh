#!/usr/bin/env bash
# Train in the tanh action-parameterization workspace.
# env: PARENT OUT UPDATES [NUM_ENVS] [ASSET]; extra args pass through to run.py
set -euo pipefail
cd /root/as2w/unitree-flat-v2-tanh
: "${PARENT:?}"; : "${OUT:?}"; : "${UPDATES:?}"
ASSET="${ASSET:-/root/as2w/unitree-flat-v2-tanh/generated-v2/as2w-t2-v2/as2w-t2-v2.usd}"
[[ -f "$PARENT" ]] || { echo "PARENT missing: $PARENT" >&2; exit 1; }
[[ -f "$ASSET" ]] || { echo "ASSET missing: $ASSET" >&2; exit 1; }
[[ ! -e "$OUT" ]] || { echo "OUT exists: $OUT" >&2; exit 1; }
exec /root/as2w/preflight/isaac-python run.py --out "$OUT" --asset "$ASSET" --terrain flat \
  --num-envs "${NUM_ENVS:-8192}" --rollout-steps 48 --iterations "$UPDATES" \
  --dr-level 0 --seed 42 --resume "$PARENT" --resume-update 0 "$@"
