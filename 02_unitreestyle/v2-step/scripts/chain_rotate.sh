#!/usr/bin/env bash
# Screen the rotate-credit arm.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-rotate400/train
base=/root/gpufree-data/as2w-runs
for label in 100 200; do
  f=$src/model_$label.pt
  for _ in $(seq 1 240); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_out.sh "$f" "$base/v2-tanh-rotate-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 480); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-tanh-rotate-eval-400" || echo "FAIL 400"
else
  echo "no endpoint.pt"
fi
echo ROTATE_CHAIN_DONE
