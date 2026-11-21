#!/usr/bin/env bash
set -euo pipefail
root=$(realpath -e -- "$1")
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ -e "$root/smoke" || -e "$root/smoke.log" ]]; then
    printf 'Use a fresh run root for this smoke test.\n' >&2
    exit 2
fi
./preflight/isaac-python continuation1220/train.py --root "$root" --out "$root/smoke" \
    --iterations 1 --offset 1220 --weight .02 2>&1 | tee "$root/smoke.log"
grep -q AS2W_TRAIN_SMOKE_OK "$root/smoke.log"
