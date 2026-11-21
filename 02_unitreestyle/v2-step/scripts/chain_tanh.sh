#!/usr/bin/env bash
# Screen tanh-run checkpoints as they appear: 200, 400, endpoint.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-dr0-600/train
base=/root/gpufree-data/as2w-runs
for label in 200 400; do
  f=$src/model_$label.pt
  for _ in $(seq 1 200); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 3
  bash eval_out.sh "$f" "$base/v2-tanh-eval-$label" || echo "FAIL $label"
done
for _ in $(seq 1 240); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 3
  bash eval_out.sh "$src/endpoint.pt" "$base/v2-tanh-eval-endpoint600" || echo "FAIL endpoint"
else
  echo "no endpoint.pt"
fi
echo TANH_SCREEN_DONE
