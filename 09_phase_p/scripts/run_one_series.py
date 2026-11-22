#!/usr/bin/env python3
"""Phase P / P1-A runner: one PhysX case with a 20 Hz state/contact series."""
import argparse
import json
from pathlib import Path
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--source-scripts", type=Path, required=True)
parser.add_argument("--params", type=Path, required=True)
args = parser.parse_args()

params = json.loads(args.params.read_text(encoding="utf-8"))
required = {"tag", "prim", "init", "mode", "dt", "nstep", "kp", "kd",
            "friction", "outdir"}
if not required.issubset(params):
    raise SystemExit(f"Missing params: {sorted(required - set(params))}")

sys.path.insert(0, str(args.source_scripts.resolve()))
import i_common_series as i_common  # noqa: E402

i_common.worker(params)
