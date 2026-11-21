#!/usr/bin/env python3
"""Compare a stop-reward A/B pair and check the gait regression.

Each arm directory must contain:
  flat-stop.json       (evaluate.py --stop-replay)
  flat-gait-gate.json  (gait_gate.py from a separate normal replay)

Usage:
  python stop_report.py --control DIR --reward DIR [--out prefix]
"""
import argparse, json
from pathlib import Path

MODES = ("stand", "forward", "reverse", "left", "right", "yaw_left", "yaw_right", "mixed")


def load(p):
    return json.loads(Path(p).read_text()) if Path(p).exists() else None


def stop_metrics(report):
    """Collapse per-mode stop metrics into gate-relevant numbers."""
    if not report:
        return None
    out = {}
    for mode in MODES:
        r = report.get(mode, {}).get("stop")
        if not r:
            continue
        out[mode] = {
            "survived": r["post_switch_survived"],
            "drift_max_m": max(r["drift_from_switch_m"]),
            "speed_p95_m_s": r["settled_speed_p95_m_s"],
            "tilt_p95_deg": r["settled_tilt_p95_deg"],
            "four_wheel": r["settled_four_wheel_fraction"],
        }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control", type=Path, required=True)
    ap.add_argument("--reward", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    c_stop = stop_metrics(load(a.control / "flat-stop.json"))
    r_stop = stop_metrics(load(a.reward / "flat-stop.json"))
    c_gate = load(a.control / "flat-gait-gate.json")
    r_gate = load(a.reward / "flat-gait-gate.json")
    if not c_stop or not r_stop:
        raise SystemExit("missing flat-stop.json in one arm")

    rows, verdicts = [], []
    drift_better = speed_better = False
    for mode in c_stop:
        c, r = c_stop[mode], r_stop.get(mode)
        if not r:
            continue
        rows.append((mode, c, r))
        drift_better |= r["drift_max_m"] < c["drift_max_m"]
        speed_better |= r["speed_p95_m_s"] < c["speed_p95_m_s"]

    out = {"control": str(a.control), "reward": str(a.reward),
           "gait_gate": {"control": (c_gate or {}).get("qualified"),
                         "reward": (r_gate or {}).get("qualified"),
                         "reward_failures": (r_gate or {}).get("failures")},
           "modes": {m: {"control": c, "reward": r} for m, c, r in rows},
           "verdict": {
               "stop_drift_improved": drift_better,
               "stop_speed_improved": speed_better,
               "gait_regression_passed": bool((r_gate or {}).get("qualified")),
               "acceptable": bool(drift_better and speed_better and (r_gate or {}).get("qualified")),
           }}
    prefix = a.out or (a.reward / "stop-ab-report")
    Path(str(prefix) + ".json").write_text(json.dumps(out, indent=2) + "\n")

    lines = ["# Stop-reward A/B", "",
             f"- control: `{a.control}`",
             f"- reward:  `{a.reward}`",
             f"- gait gate control: {(c_gate or {}).get('qualified')} / reward: {(r_gate or {}).get('qualified')}",
             f"- acceptable: **{out['verdict']['acceptable']}**", "",
             "| mode | drift c/r (m) | speed p95 c/r (m/s) | tilt p95 c/r (°) | 4-wheel c/r |",
             "|---|---|---|---|---|"]
    for mode, c, r in rows:
        lines.append(f"| {mode} | {c['drift_max_m']*100:.1f} / {r['drift_max_m']*100:.1f} | "
                     f"{c['speed_p95_m_s']:.3f} / {r['speed_p95_m_s']:.3f} | "
                     f"{c['tilt_p95_deg']:.2f} / {r['tilt_p95_deg']:.2f} | "
                     f"{c['four_wheel']:.3f} / {r['four_wheel']:.3f} |")
    if (r_gate or {}).get("failures"):
        lines += ["", "Reward-arm gait failures: " + "; ".join(r_gate["failures"])]
    Path(str(prefix) + ".md").write_text("\n".join(lines) + "\n")
    print(f"STOP_REPORT_OK {prefix}.json {prefix}.md acceptable={out['verdict']['acceptable']}")


if __name__ == "__main__":
    main()
