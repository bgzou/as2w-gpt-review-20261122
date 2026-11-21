#!/usr/bin/env bash
# Screen the mu025 (friction-aligned) DR arm.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-mu025-600/train
base=/root/gpufree-data/as2w-runs
ASSET=/root/as2w/unitree-flat-v2-tanh/assets/as2w-t2-v2-mu025.usd
for label in 200 400; do
  f=$src/model_$label.pt
  for _ in $(seq 1 360); do [[ -s "$f" ]] && break; sleep 10; done
  [[ -s "$f" ]] || { echo "missing $f"; continue; }
  sleep 5
  bash eval_asset.sh "$f" "$base/mu025-eval-$label" "$ASSET" 0 || echo "FAIL $label"
done
for _ in $(seq 1 480); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ -s "$src/endpoint.pt" ]]; then
  sleep 5
  bash eval_asset.sh "$src/endpoint.pt" "$base/mu025-eval-600" "$ASSET" 0 || echo "FAIL 600"
fi
echo MU025_CHAIN_DONE
