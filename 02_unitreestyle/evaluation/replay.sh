#!/usr/bin/env bash
set -euo pipefail
root=$(realpath -e -- "$1")
checkpoint=$(realpath -e -- "${2:-$root/source/best.pt}")
manifest=$(realpath -e -- "${3:-$root/source/run.json}")
output=$(realpath -m -- "${4:-$root/eval-1220}")
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -- "$output"
cp -- "$checkpoint" "$output/best.pt"
cp -- "$manifest" "$output/run.json"
./preflight/isaac-python evaluation/record_jitter_diagnostic.py \
    --mode unitree_style --snapshot "$output" --terrain flat --leg-hz 10 2>&1 | tee "$output/capture.log"
grep -q AS2W_EVAL_OK "$output/capture.log"
./preflight/analysis-python quality/summarize_contact.py "$output"
./preflight/analysis-python continuation1220/gate.py "$root/source" "$output"
