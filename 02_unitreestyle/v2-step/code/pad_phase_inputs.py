#!/usr/bin/env python3
"""Warm-start an existing 58-dim actor into the 60-dim gait-phase observation.

New observation dims are appended, so the first layer gains two input columns.
Zero-padding them reproduces the old policy exactly at initialisation (the new
inputs simply have no weight yet), which lets training continue from a known
behaviour instead of starting over.

Usage:
  python pad_phase_inputs.py --source best.pt --out padded.pt [--extra 2]
"""
import argparse, hashlib, json
from pathlib import Path

import torch


def pad(sd, extra, base):
    key = "mlp.0.weight"
    w = sd[key]
    if w.shape[1] >= base + extra:
        return False
    pad = torch.zeros(w.shape[0], base + extra - w.shape[1], dtype=w.dtype)
    sd[key] = torch.cat((w, pad), dim=1)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--extra", type=int, default=2)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit(f"out exists: {a.out}")
    ckpt = torch.load(a.source, map_location="cpu", weights_only=False)
    changed = {
        "actor": pad(ckpt["actor_state_dict"], a.extra, 58),
        "critic": pad(ckpt["critic_state_dict"], a.extra, 61),
    }
    a.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(ckpt, a.out)
    report = {"source": str(a.source),
              "source_sha256": hashlib.sha256(a.source.read_bytes()).hexdigest(),
              "out": str(a.out),
              "out_sha256": hashlib.sha256(a.out.read_bytes()).hexdigest(),
              "extra_inputs": a.extra, "padded": changed,
              "actor_in_features": int(ckpt["actor_state_dict"]["mlp.0.weight"].shape[1]),
              "critic_in_features": int(ckpt["critic_state_dict"]["mlp.0.weight"].shape[1])}
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
