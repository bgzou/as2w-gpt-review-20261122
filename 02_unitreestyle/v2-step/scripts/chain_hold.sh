#!/usr/bin/env bash
# Screen the 4 s-hold repair arm as checkpoints appear.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-hold4000/train
base=/root/gpufree-data/as2w-runs
for label in 200 400 600 1000 1500 2000 2500 3000 3500; do
  f=$src/model_$label.pt
  for _ in $(seq 1 720); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_out.sh "$f" "$base/v2-tanh-hold-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 1440); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-tanh-hold-eval-endpoint" || echo "FAIL endpoint"
else
  echo "no endpoint.pt"
fi
echo HOLD_CHAIN_DONE
