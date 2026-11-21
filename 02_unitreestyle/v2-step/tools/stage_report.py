#!/usr/bin/env python3
"""Summarise one evaluated checkpoint: per-wheel support/lift, per-axis joint
motion and actuator torque, tracking, tilt, and the gait gate verdict.

Usage:
  python stage_report.py --snapshot <dir> [--out <prefix>]
The snapshot directory must contain flat.npz and (optionally) flat-gait-gate.json,
flat.json, checkpoint.sha256, run.json.
"""
import argparse, json
from pathlib import Path

import numpy as np

MODES = ("stand", "forward", "reverse", "left", "right", "yaw_left", "yaw_right", "mixed")
VX = ("forward", "reverse")
STEP = ("left", "right", "yaw_left", "yaw_right")


def load_json(p):
    p = Path(p)
    return json.loads(p.read_text()) if p.exists() else None


def events(load, clr, thresh_load=3.0, thresh_clr=0.015):
    """Count clearance events per wheel per trial (airborne then landed)."""
    air = (load < thresh_load) & (clr > thresh_clr)
    counts = []
    for t in range(air.shape[1]):
        state = air[:, t, :]
        for w in range(state.shape[1]):
            col = state[:, w]
            counts.append(int(np.count_nonzero(col[1:] & ~col[:-1])))
    return np.array(counts).reshape(air.shape[1], air.shape[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    snap = a.snapshot
    d = np.load(snap / "flat.npz", allow_pickle=False)
    labels = np.array([str(x) for x in d["labels"]])
    joints = [str(x) for x in d["joints"]]
    q = d["q"]
    tau = d.get("isaac_actuator_applied_torque_estimate_nm")
    load = d["normal_load"]
    clr = d["clearance"]
    vel = d["vel"]
    cmd = d["cmd"]
    tilt = d["tilt"]
    fall = d["fall"]
    gate = load_json(snap / "flat-gait-gate.json")
    flat = load_json(snap / "flat.json")
    sha = (snap / "checkpoint.sha256").read_text().split()[0] if (snap / "checkpoint.sha256").exists() else None
    run = load_json(snap / "run.json")

    out = {"snapshot": str(snap), "checkpoint_sha256": sha,
           "asset_sha256": (run or {}).get("asset_sha256"),
           "resume_sha256": (run or {}).get("resume_sha256"),
           "num_envs": (run or {}).get("num_envs"),
           "gate_qualified": (gate or {}).get("qualified"),
           "gate_failures": (gate or {}).get("failures"), "modes": {}}

    for mode in MODES:
        m = labels == mode
        if not m.any():
            continue
        L, C = load[50:, m], clr[50:, m]
        ev = events(L, C)
        V, K = vel[50:, m], cmd[50:, m]
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.nanmean(np.where(np.abs(K) > 1e-6, V / K, np.nan), axis=0)
        out["modes"][mode] = {
            "trials": int(m.sum()),
            "falls": int(fall[:, m].sum()),
            "four_wheel_support_min": float((L >= 3).all(axis=-1).mean(axis=0).min()),
            "min_lift_events_per_trial": int(ev.min()),
            "lift_events_per_wheel_min": ev.min(axis=0).tolist(),
            "per_wheel": {
                w: {"load_mean_n": float(L[:, :, k].mean()),
                    "unloaded_frac": float((L[:, :, k] < 3).mean()),
                    "clear15mm_frac": float((C[:, :, k] > .015).mean()),
                    "clear10mm_frac": float((C[:, :, k] > .010).mean()),
                    "clearance_p95_mm": float(np.quantile(C[:, :, k], .95) * 1000.),
                    "clearance_max_mm": float(C[:, :, k].max() * 1000.)}
                for k, w in enumerate(["FL", "FR", "RL", "RR"])},
            "wheels_with_2plus_events": int((ev.min(axis=0) >= 2).sum()),
            "tilt_p95_deg": float(np.quantile(tilt[50:, m], .95)),
            "tracking_ratio_min": float(np.nanmin(ratio)) if np.isfinite(ratio).any() else None,
        }

    if tau is not None:
        axes = {}
        for j, name in enumerate(joints):
            axes[name] = {"q_p2p_rad": float((q[:, :, j].max(0) - q[:, :, j].min(0)).max()),
                          "tau_abs_mean_nm": float(np.abs(tau[:, :, j]).mean()),
                          "tau_abs_p95_nm": float(np.quantile(np.abs(tau[:, :, j]), .95)),
                          "tau_abs_max_nm": float(np.abs(tau[:, :, j]).max())}
        out["joints"] = axes

    if flat:
        out["flat_report"] = {k: flat[k] for k in flat if k != "stop"}

    prefix = a.out or (snap / "stage-report")
    Path(str(prefix) + ".json").write_text(json.dumps(out, indent=2) + "\n")

    lines = [f"# Stage report: {snap.name}", "",
             f"- checkpoint sha256: `{sha}`",
             f"- asset sha256: `{out['asset_sha256']}`",
             f"- resume sha256: `{out['resume_sha256']}`",
             f"- gate: {'PASS' if out['gate_qualified'] else 'FAIL'} {out['gate_failures'] or ''}", "",
             "| mode | falls | 4-wheel support | lift events/wheel (min) | tilt p95 | tracking |",
             "|---|---|---|---|---|---|"]
    for mode, r in out["modes"].items():
        tr = r["tracking_ratio_min"]
        tr_txt = f"{tr:.3f}" if tr is not None else "n/a"
        lines.append(f"| {mode} | {r['falls']} | {r['four_wheel_support_min']:.3f} | "
                     f"{r['lift_events_per_wheel_min']} | {r['tilt_p95_deg']:.2f}° | "
                     f"{tr_txt} |")
    if tau is not None:
        lines += ["", "| joint | q p2p (rad) | mean \\|tau\\| (Nm) | p95 \\|tau\\| (Nm) | max \\|tau\\| (Nm) |",
                  "|---|---|---|---|---|"]
        for name, r in out["joints"].items():
            lines.append(f"| {name} | {r['q_p2p_rad']:.3f} | {r['tau_abs_mean_nm']:.2f} | "
                         f"{r['tau_abs_p95_nm']:.2f} | {r['tau_abs_max_nm']:.2f} |")
    Path(str(prefix) + ".md").write_text("\n".join(lines) + "\n")
    print(f"STAGE_REPORT_OK {prefix}.json {prefix}.md")


if __name__ == "__main__":
    main()
