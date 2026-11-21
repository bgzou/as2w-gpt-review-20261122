#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
: "${PARENT_EVAL:?Set PARENT_EVAL to the checkpoint evaluation directory}"
: "${OUT:?Set OUT to a new A/B results directory}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/root/as2w/preflight/isaac-python}"
PARENT_EVAL="$(realpath "$PARENT_EVAL")"
OUT="$(realpath -m "$OUT")"
[[ ! -e "$OUT" ]] || { echo "Output already exists: $OUT" >&2; exit 1; }
ASSET="$(python3 - "$PARENT_EVAL" <<'PY'
import json, sys, hashlib
from pathlib import Path
p = Path(sys.argv[1])
gate = json.loads((p / 'flat-gait-gate.json').read_text())
if gate.get('qualified') is not True:
    raise SystemExit(f"Parent failed gait gate: {gate.get('failures')}")
expected = (p / 'checkpoint.sha256').read_text().split()[0]
if hashlib.sha256((p / 'best.pt').read_bytes()).hexdigest() != expected:
    raise SystemExit("Parent checkpoint hash mismatch")
print(json.loads((p / 'run.json').read_text())['asset'])
PY
)"
mkdir -p "$OUT"
for arm in control reward; do
  options=(--stop-course)
  if [[ "$arm" == reward ]]; then options+=(--stop-reward); fi
  "$ISAAC_PYTHON" run.py --asset "$ASSET" --out "$OUT/$arm" \
    --num-envs "${NUM_ENVS:-8192}" --rollout-steps 48 --iterations "${UPDATES:-600}" \
    --terrain flat --dr-level 0 --seed 42 --resume "$PARENT_EVAL/best.pt" \
    --resume-update 0 "${options[@]}" >"$OUT/$arm.log" 2>&1
done
