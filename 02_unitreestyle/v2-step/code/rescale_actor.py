#!/usr/bin/env python3
"""Rescale a clamp-trained actor so its pre-tanh output lands in the linear
region of tanh.

The DR0 actor was trained with `clamp(-1,1)` on the action; the clamp killed the
gradient at the rail, so the pre-clamp output grew without bound (measured
mean|a| = 17.6, 94.5% of dims at |a| >= 3.79). Feeding such an actor straight
into tanh keeps it saturated. Scaling the last layer (weight and bias) by `s`
maps the function to `s*f(x)`, which preserves the action ordering while
bringing |a| back to O(1).

Usage:
  python rescale_actor.py --source best.pt --out new.pt --scale 0.06
"""
import argparse, hashlib, json
from pathlib import Path

import torch


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scale", type=float, required=True)
    ap.add_argument("--note", default="")
    a = ap.parse_args()
    if not (0. < a.scale <= 1.):
        raise SystemExit("scale must be in (0, 1]")
    if a.out.exists():
        raise SystemExit(f"out exists: {a.out}")

    ckpt = torch.load(a.source, map_location="cpu", weights_only=False)
    actor = ckpt["actor_state_dict"]
    last_w = [k for k in actor if k.endswith("weight")][-1]
    last_b = [k for k in actor if k.endswith("bias")][-1]
    before = {"weight_absmean": float(actor[last_w].abs().mean()),
              "bias_absmean": float(actor[last_b].abs().mean())}
    actor[last_w].mul_(a.scale)
    actor[last_b].mul_(a.scale)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(ckpt, a.out)
    report = {
        "source": str(a.source), "source_sha256": sha256(a.source),
        "out": str(a.out), "out_sha256": sha256(a.out),
        "scale": a.scale, "layer": [last_w, last_b],
        "before": before, "note": a.note,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
