#!/usr/bin/env bash
set -eu
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
r=$1
base=$2
offset=1220
resume=()
for count in 100 400 $(seq 1 19 | sed 's/.*/500/'); do
    stage="$r/u$((offset+count))"
    mkdir "$stage"
    ./preflight/isaac-python residual1220/train.py --root "$base" --out "$stage/train" --offset "$offset" --iterations "$count" "${resume[@]}" >"$stage/train.log" 2>&1
    grep -q RESIDUAL_UPDATE_OK "$stage/train.log"
    for pick in endpoint best; do
        src="$stage/train/$pick.pt"; [[ -s "$src" ]] || continue
        d="$stage/$pick"; mkdir "$d"
        ln "$src" "$d/best.pt"
        cp "$stage/train/run.json" "$d/"
        ./preflight/isaac-python residual1220/evaluate.py --mode unitree_style --terrain flat --snapshot "$d" >"$d/eval.log" 2>&1
        grep -q AS2W_EVAL_OK "$d/eval.log"
        ./preflight/analysis-python quality/summarize_contact.py "$d" >"$d/summary.log"
        ./preflight/analysis-python continuation1220/gate.py "$base/source" "$d"
    done
    pick=$(./preflight/analysis-python residual1220/select_checkpoint.py "$stage" "$r") || exit 3
    resume=(--resume "$pick")
    offset=$((offset+count))
done
