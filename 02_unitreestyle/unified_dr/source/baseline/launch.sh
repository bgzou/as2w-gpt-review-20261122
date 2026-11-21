#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
r=$1
src=$2
asset=$3
for i in $(seq 1 48); do
    grep -q AS2W_TRAIN_SMOKE_OK "$r/verified-smoke.log" && break
    if ! kill -0 "$(cat "$r/verified-smoke.pid")" 2>/dev/null; then
        tail -30 "$r/verified-smoke.log"
        echo "FINAL_SMOKE_FAILED"
        exit 1
    fi
    sleep 5
done
grep -q AS2W_TRAIN_SMOKE_OK "$r/verified-smoke.log"
python3 - "$r" <<'PY'
import json,math,sys
from pathlib import Path
e=Path(sys.argv[1])/"verified-smoke/reward-components.jsonl"
updates=[json.loads(l) for l in e.read_text().splitlines()]
assert len(updates)==3
assert all(math.isfinite(v) for row in updates for v in row.values())
assert all(r["terrain_hits"]>.995 for r in updates)
assert all(r["stepping_samples"]>.05 and r["rolling_samples"]>.1 for r in updates)
assert any(r["rolling_missing_wheel"]>0 for r in updates)
assert any(r["loaded_lateral_slip"]>0 for r in updates)
PY
echo "UNIFIED_SMOKE_VERIFIED $(date -Is)"
export PYTHONUNBUFFERED=1
trap 'printf "%s\n" "$?" >"$r/exit-code"' EXIT
./preflight/isaac-python "$src/run.py" --out "$r/train" --asset "$asset" --num-envs 4096 --iterations 10000 --rollout-steps 48 --seed 42
