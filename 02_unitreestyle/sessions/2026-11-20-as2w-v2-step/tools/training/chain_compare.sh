set -uo pipefail
cd /root/as2w/unitree-flat-v2-tanh
SRC=/root/gpufree-data/as2w-runs
BASE=$SRC/v2-stephard600/train/endpoint.pt      # DR0 accepted weight
JIT=$SRC/v2-jitter600/train/model_500.pt        # DR2 + jitter weight

echo "=== 1/3 baseline @ DR2"
bash eval_dr.sh "$BASE" "$SRC/cmp-base-dr2" 2 || echo FAIL base-dr2
echo "=== 2/3 jitter @ DR0"
bash eval_dr.sh "$JIT" "$SRC/cmp-jit-dr0" 0 || echo FAIL jit-dr0
echo "=== 3/3 jitter @ DR2"
bash eval_dr.sh "$JIT" "$SRC/cmp-jit-dr2" 2 || echo FAIL jit-dr2

echo "=== metrics"
for d in v2-stephard-eval-600:base@DR0 cmp-base-dr2:base@DR2 cmp-jit-dr0:jit@DR0 cmp-jit-dr2:jit@DR2; do
  snap=${d%%:*}; label=${d##*:}
  echo "--- $label"
  /root/as2w/preflight/isaac-python jitter_metrics.py "$SRC/$snap" "$label" 2>&1 | tail -20
done
echo COMPARE_DONE
date
