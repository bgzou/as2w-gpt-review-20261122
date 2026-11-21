#!/usr/bin/env bash
# Evaluate the vxhold DR1 endpoint, then leave the result for the DR2 decision.
set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
src=/root/gpufree-data/as2w-runs/v2-tanh-dr1-vxhold/train
d=/root/gpufree-data/as2w-runs/v2-tanh-dr1-vxhold-eval-300
for _ in $(seq 1 240); do [[ -s "$src/endpoint.pt" ]] && break; sleep 10; done
if [[ ! -s "$src/endpoint.pt" ]]; then echo "no endpoint"; exit 0; fi
[[ -e "$d" ]] || bash eval_out.sh "$src/endpoint.pt" "$d" || echo "FAIL 300"
echo VXHOLD300_DONE
