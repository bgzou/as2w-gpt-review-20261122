#!/usr/bin/env bash
# Screen the stepup reward arm.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-stepup600/train
base=/root/gpufree-data/as2w-runs
for label in 100 200 400; do
  f=$src/model_$label.pt
  for _ in $(seq 1 360); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_out.sh "$f" "$base/v2-tanh-stepup-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 720); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-tanh-stepup-eval-600" || echo "FAIL 600"
else
  echo "no endpoint.pt"
fi
echo STEPUP_CHAIN_DONE
