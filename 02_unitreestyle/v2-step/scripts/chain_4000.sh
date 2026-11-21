#!/usr/bin/env bash
# Screen the 4000-update tanh retrain: evaluate every 400 updates.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-4000/train
base=/root/gpufree-data/as2w-runs
for label in 400 800 1200 1600 2000 2400 2800 3200 3600; do
  f=$src/model_$label.pt
  for _ in $(seq 1 720); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_out.sh "$f" "$base/v2-tanh-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 1440); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-tanh-eval-4000" || echo "FAIL 4000"
else
  echo "no endpoint.pt"
fi
echo CHAIN4000_DONE
