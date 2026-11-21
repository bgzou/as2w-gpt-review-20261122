#!/usr/bin/env bash
# Evaluate the DR1 stage: nominal fixed replay + gait gate.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-dr1-600/train
d=/root/gpufree-data/as2w-runs/v2-tanh-dr1-eval-600
for _ in $(seq 1 900); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ ! -s "$src/endpoint.pt" ]]; then echo "no dr1 endpoint"; exit 0; fi
[[ -e "$d" ]] || bash eval_out.sh "$src/endpoint.pt" "$d" || echo "FAIL dr1 eval"
echo DR1_EVAL_DONE
