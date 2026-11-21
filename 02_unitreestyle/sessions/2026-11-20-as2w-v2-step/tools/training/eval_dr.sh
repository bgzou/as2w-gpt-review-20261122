set -euo pipefail
# eval_dr.sh <ckpt> <snapshot-dir> <dr-level>
CKPT=$1; D=$2; DR=$3
cd /root/as2w/unitree-flat-v2-tanh
PY=/root/as2w/preflight/isaac-python
[[ ! -e "$D" ]] || { echo "exists $D"; exit 1; }
mkdir -p "$D"
cp "$CKPT" "$D/best.pt"
sha256sum "$D/best.pt" | awk '{print $1}' > "$D/checkpoint.sha256"
python3 - "$D" "$DR" <<'PY'
import json, sys, pathlib
d = pathlib.Path(sys.argv[1]); dr = int(sys.argv[2])
src = pathlib.Path("/root/gpufree-data/as2w-runs/v2-stephard600/train/run.json")
run = json.loads(src.read_text())
run["dr_level"] = dr
(d / "run.json").write_text(json.dumps(run, indent=2) + "\n")
print("run.json dr_level", dr)
PY
$PY evaluate.py --mode unitree_style --snapshot "$D" --terrain flat --dr-level "$DR" >"$D/eval.log" 2>&1
echo "eval_rc=$?"
$PY gait_gate.py "$D/flat.npz" || true
$PY stage_report.py --snapshot "$D" >/dev/null 2>&1 || true
echo "done $D"
