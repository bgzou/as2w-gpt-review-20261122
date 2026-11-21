#!/usr/bin/env bash
# Evaluate one checkpoint with the fixed flat replay and run the gait gate.
# usage: eval_ckpt.sh <ckpt_path> <label> [wait_pid]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ckpt=$(realpath "$1")
label=$2
pid=${3:-}
base=/root/gpufree-data/as2w-runs
if [[ -n "$pid" ]]; then
  while kill -0 "$pid" 2>/dev/null; do sleep 10; done
fi
d="$base/v2-baseline-eval-$label"
[[ ! -e "$d" ]] || { echo "exists: $d"; exit 0; }
mkdir "$d"
cp "$(dirname "$ckpt")/run.json" "$d/"
cp "$ckpt" "$d/best.pt"
sha256sum "$d/best.pt" > "$d/checkpoint.sha256"
/root/as2w/preflight/isaac-python evaluate.py --mode unitree_style \
  --snapshot "$d" --terrain flat --dr-level 0 >"$d/eval.log" 2>&1
/root/as2w/preflight/isaac-python gait_gate.py "$d/flat.npz"
