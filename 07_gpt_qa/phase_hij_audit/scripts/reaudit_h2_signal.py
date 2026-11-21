#!/usr/bin/env python3
"""Quantify whether archived H2 steady windows can identify dynamic lag/bandwidth.

Reads archived 500 Hz NPZ traces and writes a separate diagnostic JSON. A
correlation peak or spectral coherence is descriptive, not motor transfer
function identification: tau_est is a firmware estimate and excitation is
uncontrolled.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
QC = (WORKSPACE / "部署参考" / "as2w-sim2real-rootcause-bundle-20261121"
      / "runs" / "as2w_v2_simreal_rootcause_20261119" / "qc" / "a1_timelines")
OUT = Path(__file__).resolve().parents[1] / "results" / "h2_signal_reaudit.json"
SESSIONS = {
    "OG": ((17, 120), 200.0, 4.0, [-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6, -1.2, -1.2, -1.2, -1.2]),
    "OS": ((0, 60), 200.0, 4.0, [-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6, -1.2, -1.2, -1.2, -1.2]),
    "P55G1": ((10, 30), 55.0, 2.0, [0.0]*4 + [0.8]*4 + [-1.6]*4),
    "P55G2": ((8, 19), 55.0, 2.0, [0.0]*4 + [0.8]*4 + [-1.6]*4),
    "P55G3": ((7, 19), 55.0, 2.0, [0.0]*4 + [0.8]*4 + [-1.6]*4),
    "P55S": ((25, 90), 55.0, 2.0, [0.0]*4 + [0.8]*4 + [-1.6]*4),
    "P85DS": ((48, 100), 85.0, 3.2, [0.0]*4 + [0.8]*4 + [-1.6]*4),
}
HIPS = ("FL_hip", "FR_hip", "RL_hip", "RR_hip")


def correlation(x, y):
    x = x - x.mean()
    y = y - y.mean()
    den = np.linalg.norm(x) * np.linalg.norm(y)
    return float(x @ y / den) if den > 1e-12 else None


def cross_lag(x, y, max_lag):
    out = []
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            c = correlation(x[-lag:], y[:lag])
        elif lag > 0:
            c = correlation(x[:-lag], y[lag:])
        else:
            c = correlation(x, y)
        out.append((lag, c))
    valid = [(lag, c) for lag, c in out if c is not None]
    return max(valid, key=lambda v: v[1]) if valid else (None, None)


def spectral(x, y, fs, nfft=2048):
    if len(x) < nfft:
        return None
    step = nfft // 2
    win = np.hanning(nfft)
    xx, yy, xy = 0.0, 0.0, 0.0j
    count = 0
    for start in range(0, len(x) - nfft + 1, step):
        xs = x[start:start+nfft]
        ys = y[start:start+nfft]
        fx = np.fft.rfft((xs - xs.mean()) * win)
        fy = np.fft.rfft((ys - ys.mean()) * win)
        xx = xx + np.abs(fx)**2
        yy = yy + np.abs(fy)**2
        xy = xy + np.conj(fx) * fy
        count += 1
    xx, yy, xy = xx/count, yy/count, xy/count
    freqs = np.fft.rfftfreq(nfft, 1/fs)
    coherence = np.abs(xy)**2 / np.maximum(xx*yy, 1e-24)
    result = {}
    for lo, hi in ((0.5, 5), (5, 20), (20, 100)):
        band = (freqs >= lo) & (freqs < hi)
        name = f"{lo:g}-{hi:g}Hz"
        result[name] = {
            "pd_band_energy_fraction": float(xx[band].sum()/max(xx[freqs >= 0.5].sum(), 1e-24)),
            "est_band_energy_fraction": float(yy[band].sum()/max(yy[freqs >= 0.5].sum(), 1e-24)),
            "median_coherence": float(np.median(coherence[band])),
        }
    return {"segments": count, "bands": result}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--qc", type=Path, default=QC)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    result = {
        "interpretation_limit": (
            "All windows are nominal static holds. Low correlation or a lag peak does not by itself identify "
            "motor bandwidth; tau_est is firmware-estimated, quantized and its filter is unknown."
        ),
        "lag_convention": "positive lag means tau_est later than tau_PD",
        "sessions": {},
    }
    for tag, (window, kp, kd, targets) in SESSIONS.items():
        path = args.qc / f"{tag}.npz"
        with np.load(path) as data:
            t = data["t"]
            mask = (t >= window[0]) & (t < window[1])
            q, dq, tau_est = data["q"][mask], data["dq"][mask], data["tau"][mask]
            dt = float(np.median(np.diff(t[mask])))
        tau_pd = kp*(np.asarray(targets)[None, :] - q) - kd*dq
        joints = {}
        for index, name in enumerate(HIPS):
            x, y = tau_pd[:, index], tau_est[:, index]
            unique = np.unique(y)
            steps = np.diff(unique)
            peak_lag, peak_corr = cross_lag(x, y, max_lag=10)
            diff_lag, diff_corr = cross_lag(np.diff(x), np.diff(y), max_lag=10)
            joints[name] = {
                "tau_pd_mean_Nm": float(x.mean()),
                "tau_est_mean_Nm": float(y.mean()),
                "tau_pd_std_Nm": float(x.std()),
                "tau_est_std_Nm": float(y.std()),
                "residual_rms_Nm": float(np.sqrt(np.mean((y-x)**2))),
                "tau_est_unique_levels": len(unique),
                "tau_est_min_level_step_Nm": float(steps[steps > 1e-8].min()) if np.any(steps > 1e-8) else None,
                "corr_zero": correlation(x, y),
                "corr_peak_plus_minus_20ms": peak_corr,
                "peak_lag_ms": 1000*dt*peak_lag if peak_lag is not None else None,
                "difference_corr_peak": diff_corr,
                "difference_peak_lag_ms": 1000*dt*diff_lag if diff_lag is not None else None,
                "spectrum": spectral(x, y, 1/dt),
            }
        result["sessions"][tag] = {
            "source": str(path), "source_sha256": sha256(path),
            "window_s": window, "samples": len(q),
            "sample_dt_s": dt,
            "residual_rms_all_12_Nm": float(np.sqrt(np.mean((tau_est - tau_pd)**2))),
            "hip": joints,
        }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for tag in ("P55G2", "P55S", "OG", "OS"):
        vals = result["sessions"][tag]["hip"]
        print(tag, [(name, round(v["tau_pd_std_Nm"], 4),
                     round(v["tau_est_std_Nm"], 4), v["tau_est_unique_levels"],
                     round(v["corr_zero"], 3), v["peak_lag_ms"])
                    for name, v in vals.items()])
    print("wrote", args.out)


if __name__ == "__main__":
    main()
