set -euo pipefail
# eval_asset.sh <ckpt> <snapshot-dir> <asset-usd> [dr-level]
CKPT=$1; D=$2; ASSET=$3; DR=${4:-0}
cd /root/as2w/unitree-flat-v2-tanh
PY=/root/as2w/preflight/isaac-python
[[ ! -e "$D" ]] || { echo "exists $D"; exit 1; }
mkdir -p "$D"
cp "$CKPT" "$D/best.pt"
sha256sum "$D/best.pt" | awk '{print $1}' > "$D/checkpoint.sha256"
python3 - "$D" "$DR" "$ASSET" <<'PY'
import hashlib, json, pathlib, sys
d = pathlib.Path(sys.argv[1]); dr = int(sys.argv[2]); asset = sys.argv[3]
run = json.loads(pathlib.Path("/root/gpufree-data/as2w-runs/v2-stephard600/train/run.json").read_text())
run["dr_level"] = dr
run["asset"] = asset
run["asset_sha256"] = hashlib.sha256(pathlib.Path(asset).read_bytes()).hexdigest()
(d / "run.json").write_text(json.dumps(run, indent=2) + "\n")
print("asset", asset, run["asset_sha256"][:12], "dr", dr)
PY
$PY evaluate.py --mode unitree_style --snapshot "$D" --terrain flat --dr-level "$DR" >"$D/eval.log" 2>&1
echo "eval_rc=$?"
$PY gait_gate.py "$D/flat.npz" >/dev/null 2>&1 || true
$PY stage_report.py --snapshot "$D" >/dev/null 2>&1 || true
ls "$D"/*.npz 2>/dev/null | xargs -r -n1 basename
echo done
