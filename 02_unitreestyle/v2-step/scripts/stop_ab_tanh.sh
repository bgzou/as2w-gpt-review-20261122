#!/usr/bin/env bash
# Stop-reward A/B on the tanh line, sequential (single GPU job at a time).
# Parent is the tanh rescaled checkpoint: vx qualified (fwd 0.984 / rev 1.000),
# lateral stepping not qualified - documented deviation from the strict gate.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
PY=/root/as2w/preflight/isaac-python
ASSET=/root/as2w/unitree-flat-v2-tanh/generated-v2/as2w-t2-v2/as2w-t2-v2.usd
PARENT=/root/gpufree-data/as2w-runs/v2-tanh-init/best.pt
OUT=/root/gpufree-data/as2w-runs/v2-stop-ab-tanh
[[ ! -e "$OUT" ]] || { echo "OUT exists"; exit 1; }
mkdir -p "$OUT"
cp /root/gpufree-data/as2w-runs/v2-tanh-init/run.json "$OUT/parent-run.json"
echo "PARENT_SHA $(sha256sum "$PARENT" | cut -d' ' -f1)"
for arm in control reward; do
  opts=(--stop-course)
  if [[ "$arm" == reward ]]; then opts+=(--stop-reward); fi
  echo "[stop-ab] $arm start $(date -Is)"
  "$PY" run.py --out "$OUT/$arm" --asset "$ASSET" --terrain flat \
    --num-envs 4096 --rollout-steps 48 --iterations 600 --dr-level 0 --seed 42 \
    --resume "$PARENT" --resume-update 0 "${opts[@]}" >"$OUT/$arm.log" 2>&1
  echo "[stop-ab] $arm rc=$? $(date -Is)"
done
echo STOP_AB_DONE
