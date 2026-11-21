#!/usr/bin/env bash
# Evaluate both stop-A/B arms: fixed gait replay + stop replay.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
base=/root/gpufree-data/as2w-runs
for arm in control reward; do
  src=$base/v2-stop-ab-tanh/$arm
  for _ in $(seq 1 900); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
  if [[ ! -s "$src/endpoint.pt" ]]; then echo "no endpoint $arm"; continue; fi
  d=$base/v2-stop-ab-eval-$arm
  if [[ ! -e "$d" ]]; then
    bash eval_out.sh "$src/endpoint.pt" "$d" || { echo "FAIL eval $arm"; continue; }
  fi
  if [[ ! -s "$d/flat-stop.json" ]]; then
    echo "[stopab] stop replay $arm $(date -Is)"
    /root/as2w/preflight/isaac-python evaluate.py --mode unitree_style \
      --snapshot "$d" --terrain flat --dr-level 0 --stop-replay \
      >"$d/stop.log" 2>&1 || echo "FAIL stop $arm"
  fi
  echo "[stopab] $arm done $(date -Is)"
done
echo STOPAB_EVAL_DONE
