#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
top=$PWD
isaac=$(command -v "$ISAAC_PYTHON")
asset=$top/simulation/assets/load-v0/as2w-load-v0.urdf
case "$1" in
  base)
    exec "$isaac" source/baseline/run.py --out "$2" --asset "$asset" --num-envs 4096 --iterations 10000 --rollout-steps 48 --seed 42
    ;;
  recorded-stage)
    index=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["base_update"])' "source/stages/stage-$2/run.json")
    exec "$isaac" "source/stages/stage-$2/source-snapshot/run.py" --out "$3" --asset "$asset" --num-envs 4096 --iterations 600 --rollout-steps 48 \
        --seed "$((42+$2))" --dr-level "$2" --resume "$top/model/stage-$2-initialization.pt" --resume-update "$index"
    ;;
  replay)
    dest=$(realpath -m "$3")
    mkdir -p "$dest"
    cp model/policy.pt "$dest/best.pt"
    python3 - "$asset" "$dest" <<'PY'
import json,sys
from pathlib import Path
dst=Path(sys.argv[2])
(dst/"run.json").write_text(json.dumps({"asset":sys.argv[1]})+"\n")
PY
    exec "$isaac" source/stages/evaluate.py --mode unitree_style --snapshot "$dest" --terrain "$2" --dr-level 0 --seed 319
    ;;
  stage)
    # stage LEVEL INITIAL_CHECKPOINT INITIAL_RUNNER_INDEX OUTPUT_DIR
    exec "$isaac" "source/stages/stage-$2/source-snapshot/run.py" --out "$5" --asset "$asset" --num-envs 4096 --iterations 600 --rollout-steps 48 \
        --seed "$((42+$2))" --dr-level "$2" --resume "$3" --resume-update "$4"
    ;;
  *)
    echo "reproduce.sh base DEST | replay flat|slope DEST | recorded-stage LEVEL DEST | stage LEVEL CHECKPOINT RUNNER_INDEX DEST" >&2
    exit 2
    ;;
esac
