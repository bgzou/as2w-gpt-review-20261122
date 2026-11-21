#!/usr/bin/env bash
# Pre-flight code smokes for the tanh line (not stage results):
#   1) stop-course + stop reward path (reset poses, finite reward)
#   2) DR1 randomisation path (mass/COM/friction, reset)
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
PY=/root/as2w/preflight/isaac-python
ASSET=/root/as2w/unitree-flat-v2-tanh/generated-v2/as2w-t2-v2/as2w-t2-v2.usd
PARENT=/root/gpufree-data/as2w-runs/v2-tanh-init/best.pt
base=/root/gpufree-data/as2w-runs

rm -rf "$base/v2-tanh-stopsmoke" "$base/v2-tanh-dr1smoke"
echo "=== stop-course smoke"
$PY run.py --out "$base/v2-tanh-stopsmoke/train" --asset "$ASSET" --terrain flat \
  --num-envs 24 --rollout-steps 160 --iterations 1 --dr-level 0 --seed 42 \
  --resume "$PARENT" --resume-update 0 --stop-course --stop-reward \
  > /root/v2-tanh-stopsmoke.log 2>&1
echo "stop_smoke_rc=$?"
echo "=== dr1 smoke"
$PY run.py --out "$base/v2-tanh-dr1smoke/train" --asset "$ASSET" --terrain flat \
  --num-envs 8 --rollout-steps 48 --iterations 1 --dr-level 1 --seed 42 \
  --resume "$PARENT" --resume-update 0 \
  > /root/v2-tanh-dr1smoke.log 2>&1
echo "dr1_smoke_rc=$?"
echo SMOKES_DONE
