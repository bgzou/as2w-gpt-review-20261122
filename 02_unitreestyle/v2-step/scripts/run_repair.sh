#!/usr/bin/env bash
# One-arm repair control from a fixed parent checkpoint.
#   ARM=hold   : 4 s command dwell (coverage time for vy/wz items)
#   ARM=load   : low-weight short-unload penalty on pure rolling (vx)
# env: ARM PARENT OUT UPDATES [NUM_ENVS] [ASSET]
set -euo pipefail
cd /root/as2w/unitree-flat-v2-dr
: "${ARM:?ARM=hold|load}"
: "${PARENT:?PARENT=/abs/path/model_x.pt}"
: "${OUT:?OUT=new output directory}"
: "${UPDATES:?UPDATES=n}"
ASSET="${ASSET:-/root/as2w/unitree-flat-v2-2000/generated-v2/as2w-t2-v2/as2w-t2-v2.usd}"
[[ -f "$PARENT" ]] || { echo "PARENT missing: $PARENT" >&2; exit 1; }
[[ -f "$ASSET" ]] || { echo "ASSET missing: $ASSET" >&2; exit 1; }
[[ ! -e "$OUT" ]] || { echo "OUT exists: $OUT" >&2; exit 1; }
extra=()
case "$ARM" in
  hold) extra+=(--command-hold-s 4.0) ;;
  load) extra+=(--rolling-light-load-weight 0.08) ;;
  *) echo "unknown ARM: $ARM" >&2; exit 2 ;;
esac
exec /root/as2w/preflight/isaac-python run.py --out "$OUT" --asset "$ASSET" --terrain flat \
  --num-envs "${NUM_ENVS:-8192}" --rollout-steps 48 --iterations "$UPDATES" \
  --dr-level 0 --seed 42 --resume "$PARENT" --resume-update 0 "${extra[@]}"
