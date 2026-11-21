#!/usr/bin/env python3
"""Compare W_A/W_D against raw P55G2 states and separately labelled Fn inversion."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
RUN = (WORKSPACE / "部署参考" / "as2w-sim2real-rootcause-bundle-20261121"
       / "runs" / "as2w_v2_simreal_rootcause_20261119")
DEFAULT_OUT = Path(__file__).resolve().parents[1] / "results" / "wa_candidate_reaudit.json"


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def rounded(x):
    return np.round(np.asarray(x, dtype=float), 4).tolist()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=RUN)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    i5_file = args.run / "tables" / "I5_GOW_CONTACT_VARIANTS.json"
    g2_file = args.run / "tables" / "G2_CONTACT_RECHECK.json"
    npz_file = args.run / "qc" / "a1_timelines" / "P55G2.npz"
    i5 = json.loads(i5_file.read_text(encoding="utf-8"))
    g2 = json.loads(g2_file.read_text(encoding="utf-8"))
    with np.load(npz_file) as trace:
        mask = (trace["t"] >= 8.0) & (trace["t"] < 19.0)
        hip_q = trace["q"][mask, :4].mean(0)
        hip_tau_est = trace["tau"][mask, :4].mean(0)
        pitch_deg = np.rad2deg(trace["pitch"][mask].mean())
    fn_inverted = np.array(g2["P55G2"]["variants"]["analytic"]["Fn"])
    report = {
        "inputs": [{"path": str(p), "sha256": sha256(p)} for p in (npz_file, g2_file, i5_file)],
        "real_feedback": {
            "hip_q_rad": rounded(hip_q), "hip_tau_est_Nm": rounded(hip_tau_est),
            "pitch_deg": round(float(pitch_deg), 4),
        },
        "model_inversion_not_independent_measurement": {
            "Fn_N": rounded(fn_inverted),
            "source": "G2 P55G2 analytic contact inversion; no four-wheel force sensor",
        },
        "simulation_variants": {},
    }
    for variant in ("W_A", "W_D"):
        row = next(r for r in i5["rows"] if r["variant"] == variant)
        q = np.asarray(row["hip_q"])
        tau = np.asarray(row["hip_tau_Nm"])
        fn = np.asarray(row["Fn"])
        report["simulation_variants"][variant] = {
            "pitch_deg": row["pitch_deg"],
            "pitch_minus_real_deg": round(row["pitch_deg"] - float(pitch_deg), 4),
            "hip_q_rad": rounded(q), "hip_q_minus_real_rad": rounded(q - hip_q),
            "hip_tau_Nm": rounded(tau),
            "hip_tau_minus_tau_est_Nm": rounded(tau - hip_tau_est),
            "Fn_simulated_N": rounded(fn),
            "Fn_sim_minus_model_inversion_N": rounded(fn - fn_inverted),
            "Fn_comparison_limit": "Reference Fn is a model inversion, not direct real measurement.",
            "contact_points_per_wheel_simulated": row["n_contacts_per_wheel"],
            "settle_ok": row["settle_ok"],
        }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
