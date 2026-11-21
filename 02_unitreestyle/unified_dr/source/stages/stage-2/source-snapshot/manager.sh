#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
r=$1
s=$(cat "$r/source-path.txt")
base=$(cat "$r/base-path.txt")
isaac=/root/as2w/preflight/isaac-python
asset=/root/as2w/calibration/candidates/load-v0/as2w-load-v0.urdf
ref=$r/reference-1220
expected=7c9740a95042bad7b73cadfda90bdc7c1873ac766afbb7c0e7e492367d45afb0
export PYTHONUNBUFFERED=1
exec 9>"$r/manager.lock"
flock -n 9
status() {
    printf '%s %s\n' "$(TZ=Asia/Shanghai date -Is)" "$*" | tee -a "$r/timeline.log" >"$r/current.tmp"
    mv "$r/current.tmp" "$r/current.txt"
}
trap 'code=$?; if (( code )); then status "ERROR exit=$code line=$LINENO"; fi' EXIT
test "$(sha256sum "$asset" | cut -d' ' -f1)" = "$expected"
prepare() {
    local dst="$1" ck="$2" iteration="$3"
    mkdir -p "$dst"
    ln -f "$ck" "$dst/checkpoint.pt" 2>/dev/null || cp -fp "$ck" "$dst/checkpoint.pt"
    cp "$(dirname "$ck")/run.json" "$dst/run.json"
    python3 -c 'import json,sys; print(json.dumps({"checkpoint":sys.argv[1],"runner_index":int(sys.argv[2])}))' "$ck" "$iteration" >"$dst/candidate.json"
}
replay() {
    local target="$1" terrain="$2" level="$3" seed="$4"
    local dst="$target/$terrain-dr$level"
    mkdir -p "$dst"
    if ! grep -q AS2W_EVAL_OK "$dst/run.log" 2>/dev/null || [[ ! -s "$dst/$terrain.npz" ]]; then
        ln -f "$target/checkpoint.pt" "$dst/best.pt"
        cp "$target/run.json" "$dst/run.json"
        status "REPLAY $dst seed=$seed"
        "$isaac" "$s/evaluate.py" --mode unitree_style --snapshot "$dst" --terrain "$terrain" --dr-level "$level" --seed "$seed" >"$dst/run.log" 2>&1
        grep -q AS2W_EVAL_OK "$dst/run.log"
    fi
}
gate() {
    local dst="$1" terrain="$2" level="$3" anchor="$4" seed="$5"
    replay "$dst" "$terrain" "$level" "$seed"
    python3 "$s/score.py" "$dst/$terrain-dr$level/$terrain.npz" "$anchor/$terrain-dr$level/$terrain.npz" "$dst/$terrain-dr$level/gate.json"
}
stage_updates=600
if [[ ! -f "$r/run-plan.txt" ]]; then
    printf '%s\n' "branch=model_1200.pt (1201 updates) after base crosses 1200" "stages=600/600/600" "training_envs=4096" "rollout=48" "nominal_seed=319" "stress_seed=719" "deadline=2026-11-20T09:00:00+08:00" >"$r/run-plan.txt"
fi
old=$(cat /root/as2w/continuation1220/latest.txt)
prepare "$ref" "$old/source/best.pt" 1219
for terrain in flat slope; do replay "$ref" "$terrain" 0 319; done
status "WAIT_BASE_1200 train_log=$base/train.log"
until python3 - "$base/train/model_1200.pt" <<'PY'
import sys,zipfile
with zipfile.ZipFile(sys.argv[1]) as ck:
    assert any(n.endswith("/data.pkl") for n in ck.namelist())
PY
do
    kill -0 "$(cat "$base/train.pid")" 2>/dev/null || { echo "Base runner exited before checkpoint"; exit 31; }
    sleep 30
done 2>/dev/null
branch=$r/branch-selection
for i in 1200 1150 1000; do
    if [[ -s "$base/train/model_$i.pt" ]]; then
        prepare "$branch/model-$i" "$base/train/model_$i.pt" "$i"
        for terrain in flat slope; do gate "$branch/model-$i" "$terrain" 0 "$ref" 319; done
    fi
done
labels=(model-1200)
for i in 1150 1000; do [[ ! -d "$branch/model-$i" ]] || labels+=("model-$i"); done
name=$(python3 "$s/select_checkpoint.py" "$branch" "${labels[@]}")
parent=$branch/$name
status "BRANCHED selected=$parent details=$branch/selection.json"
# Keep the base trainer untouched while each cumulative branch runs.
for level in 1 2 3; do
    stage=$r/stage-$level
    out=$stage/train
    mkdir -p "$stage"
    index=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["runner_index"])' "$parent/candidate.json")
    if [[ ! -s "$out/endpoint.pt" ]] || ! grep -q AS2W_TRAIN_SMOKE_OK "$stage/train.log"; then
        if [[ -d "$out" ]]; then mv "$out" "$stage/interrupted-$(date +%s)"; fi
        if [[ -f "$stage/train.log" ]]; then mv "$stage/train.log" "$stage/interrupted-$(date +%s).log"; fi
        status "TRAIN level=$level updates=$stage_updates parent=$parent"
        "$isaac" "$s/run.py" --out "$out" --asset "$asset" --num-envs 4096 --iterations "$stage_updates" --rollout-steps 48 --seed "$((42+level))" --dr-level "$level" --resume "$parent/checkpoint.pt" --resume-update "$index" >"$stage/train.log" 2>&1
        grep -q AS2W_TRAIN_SMOKE_OK "$stage/train.log"
    fi
    selection=$stage/selection
    prepare "$selection/incoming" "$parent/checkpoint.pt" "$index"
    for item in update-250 update-500 endpoint; do
        case "$item" in endpoint) local_update=$stage_updates ;; *) local_update=$(echo "$item" | cut -d- -f2) ;; esac
        prepare "$selection/$item" "$out/$item.pt" "$((index+local_update))"
    done
    for terrain in flat slope; do
        replay "$selection/incoming" "$terrain" "$level" 719
        for item in incoming endpoint update-500 update-250; do
            gate "$selection/$item" "$terrain" 0 "$ref" 319
            gate "$selection/$item" "$terrain" "$level" "$selection/incoming" 719
        done
    done
    name=$(python3 "$s/select_checkpoint.py" "$selection" endpoint update-500 update-250 incoming)
    parent=$selection/$name
    status "SELECTED level=$level selected=$parent details=$selection/selection.json"
done
cp "$r/stage-3/selection/selection.json" "$r/final-selection.json"
printf '%s\n' "$parent" >"$r/final-candidate.txt"
status "DR_FINISHED selection=$r/final-selection.json"
