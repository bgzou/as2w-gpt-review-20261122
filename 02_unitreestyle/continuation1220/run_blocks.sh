#!/usr/bin/env bash
set -euo pipefail
root=$(realpath -e -- "$1")
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ -e "$root/blocks" ]]; then
    printf "Use a new run root or resume a selected checkpoint explicitly.\n" >&2
    exit 2
fi
resume="$root/source/best.pt"
weight=.02
for i in 1 2 3 4 5; do
    from=$((1220+(i-1)*2000))
    end=$((from+2000))
    block="$root/blocks/u$end"
    mkdir -p "$block"
    printf 'AS2W_BLOCK_START %s %s %s\n' "$from" "$end" "$(date --iso-8601=seconds)"
    ./preflight/isaac-python continuation1220/train.py --root "$root" --out "$block/train" \
        --iterations 2000 --offset "$from" --resume "$resume" --weight "$weight" >"$block/train.log" 2>&1
    grep -q AS2W_TRAIN_SMOKE_OK "$block/train.log"
    for variant in endpoint training-best; do
        if [[ "$variant" == endpoint ]]; then
            ck="$block/train/model_$((end-1)).pt"
        else
            ck="$block/train/best.pt"
        fi
        if [[ ! -s "$ck" ]]; then continue; fi
        folder="$block/$variant"
        mkdir "$folder"
        cp "$ck" "$folder/best.pt"
        cp "$block/train/run.json" "$folder/"
        sha256sum "$folder/best.pt" >"$folder/checkpoint.sha256"
        ./preflight/isaac-python evaluation/record_jitter_diagnostic.py --mode unitree_style \
            --snapshot "$folder" --terrain flat --leg-hz 10 >"$folder/capture.log" 2>&1
        grep -q AS2W_EVAL_OK "$folder/capture.log"
        ./preflight/analysis-python quality/summarize_contact.py "$folder" >"$folder/summary.log"
        ./preflight/analysis-python continuation1220/gate.py "$root/source" "$folder" | tee "$folder/gate.log"
    done
    if ! resume=$(python3 - "$root" "$block" "$end" <<'PY'
import json,sys
from pathlib import Path
root,block=map(Path,sys.argv[1:3]); end=int(sys.argv[3])
r=[]
for name in ('endpoint','training-best'):
    d=block/name/'gate.json'
    if d.is_file():
        x=json.loads(d.read_text())
        # Prefer the current endpoint on ties.
        if x["gait_preserved"]:
            r.append((x["spikes_candidate"],name,d.parent/"best.pt",x["spikes_baseline"]))
if not r:
    print("No checkpoint passed the 1220 gait preservation checks.",file=sys.stderr)
    raise SystemExit(3)
score,name,path,original=min(r)
record={"stage_end":end,"selected":name,"checkpoint":str(path),"spikes":score,"baseline_spikes":original}
(block/"selection.json").write_text(json.dumps(record,indent=2)+"\n")
best=root/"validated-best.json"
if score<original and (not best.exists() or score<json.loads(best.read_text())["spikes"]):
    best.write_text(json.dumps(record,indent=2)+"\n")
print(path)
PY
    ); then
        echo "AS2W_HELD_FOR_GAIT_REVIEW $end"
        exit 3
    fi
    printf 'AS2W_BLOCK_GATE_OK %s %s\n' "$end" "$resume"
done
echo AS2W_10000_UPDATES_DONE
