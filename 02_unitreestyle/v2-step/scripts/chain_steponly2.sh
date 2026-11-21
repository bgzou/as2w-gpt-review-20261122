#!/usr/bin/env bash
# Screen the step-only continuation.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-steponly1800/train
base=/root/gpufree-data/as2w-runs
for label in 400 800; do
  f=$src/model_$label.pt
  for _ in $(seq 1 360); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_out.sh "$f" "$base/v2-steponly2-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 480); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-steponly2-eval-1200" || echo "FAIL 1200"
fi
echo STEPONLY2_CHAIN_DONE
