#!/usr/bin/env bash
# Wait for the DR0 training process, then screen the tail checkpoints.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-dr
pid=2079591
src=/root/gpufree-data/as2w-runs/unitree-flat-v2-8192-2000/train
for label in 1800 1900; do
  bash eval_ckpt.sh "$src/model_$label.pt" "$label" "$pid" || echo "FAIL $label"
done
# endpoint.pt is written after training ends
while [[ ! -s "$src/endpoint.pt" ]]; do
  if ! kill -0 "$pid" 2>/dev/null; then
    if [[ ! -s "$src/endpoint.pt" ]]; then echo "no endpoint.pt after exit"; exit 1; fi
  fi
  sleep 10
done
sleep 3
bash eval_ckpt.sh "$src/endpoint.pt" endpoint || echo "FAIL endpoint"
echo SCREEN_DONE
