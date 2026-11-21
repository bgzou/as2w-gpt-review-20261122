#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
r=$1
base=$2
mkdir -p "$r"
./preflight/isaac-python residual1220/train.py --root "$base" --out "$r/smoke" --iterations 2 --envs 128 >"$r/smoke.log" 2>&1
grep -q RESIDUAL_UPDATE_OK "$r/smoke.log"
py=(./preflight/analysis-python)
"${py[@]}" residual1220/verify.py "$base/source/best.pt" "$r/smoke/endpoint.pt"
mkdir "$r/paired1220"
cp "$r/smoke/run.json" "$r/paired1220/"
"${py[@]}" - "$r" <<'PY'
import json,torch,sys
from pathlib import Path
p=Path(sys.argv[1])
d=torch.load(p/"smoke/endpoint.pt",map_location="cpu",weights_only=False)
for k in ("mlp.6.weight","mlp.6.bias"):
    d["actor_state_dict"][k].zero_()
d["infos"]["as2w_residual_v1"]["profile"]="original"
manifest=p/"paired1220/run.json"
cfg=json.loads(manifest.read_text())
cfg["profile"]="original"
manifest.write_text(json.dumps(cfg,indent=2))
torch.save(d,p/"paired1220/best.pt")
PY
d="$r/paired1220"
./preflight/isaac-python residual1220/evaluate.py --snapshot "$d" --mode unitree_style --terrain flat >"$d/eval.log" 2>&1
grep -q AS2W_EVAL_OK "$d/eval.log"
"${py[@]}" quality/summarize_contact.py "$d" >"$d/summary.log"
exec bash residual1220/run.sh "$r" "$base"
