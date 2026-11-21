"""Generate the session plots + small CSV extracts for the GitHub archive."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BASE = Path(r"X:\Items\as2w\.stage58")
STAGE = BASE / "gh-stage-20261120"
PLOTS = STAGE / "plots"
DATA = STAGE / "data"
ANAL = STAGE / "analysis"
for p in (PLOTS, DATA, ANAL):
    p.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ = 149


def load_field(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.startswith("DEBUG_POLICY,"):
                continue
            p = line.rstrip("\n").split(",")
            if p[2].upper() != "WALK":
                continue
            try:
                rows.append({
                    "t": float(p[1]),
                    "cmd": [float(x) for x in p[5:8]],
                    "gyro": [float(x) for x in p[15:18]],
                    "quat": [float(x) for x in p[18:22]],
                    "q": [float(x) for x in p[25:41]],
                    "dq": [float(x) for x in p[41:57]],
                    "tgt": [float(p[REQ + 4 * i]) for i in range(16)],
                })
            except (ValueError, IndexError):
                continue
    return rows


def roll_pitch(quat):
    w, x, y, z = quat
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def series(rows):
    t = np.array([(r["t"] - rows[0]["t"]) / 1e9 for r in rows])
    cmd = np.array([r["cmd"] for r in rows])
    gyro = np.array([r["gyro"] for r in rows])
    q = np.array([r["q"][:12] for r in rows]) - DEFAULT
    dq = np.array([r["dq"][:12] for r in rows])
    tgt = np.array([r["tgt"][:12] for r in rows]) - DEFAULT
    rp = np.array([roll_pitch(r["quat"]) for r in rows])
    return t, cmd, gyro, q, dq, tgt, rp


def metrics(rows):
    t, cmd, gyro, q, dq, tgt, rp = series(rows)
    return {
        "p2p": float(np.mean(tgt.max(0) - tgt.min(0))),
        "delta": float(np.mean(np.abs(np.diff(tgt, axis=0)))),
        "flip": float(np.mean((tgt[1:] * tgt[:-1]) < 0)),
        "track": float(np.mean(np.abs(tgt - q))),
        "pitch_mean": float(rp[:, 1].mean()), "pitch_min": float(rp[:, 1].min()), "pitch_max": float(rp[:, 1].max()),
        "roll_mean": float(rp[:, 0].mean()), "roll_min": float(rp[:, 0].min()), "roll_max": float(rp[:, 0].max()),
        "tau_hip": [float(np.abs(55 * (tgt[:, i] - q[:, i]) - 2 * dq[:, i]).mean()) for i in (0, 3, 6, 9)],
        "t": t, "tgt": tgt, "q": q, "rp": rp, "cmd": cmd, "gyro": gyro,
    }


field = {
    "zeroB_old": metrics(load_field(BASE / "field20261120" / "debug_policy.csv")),
    "zeroB_mu025": metrics(load_field(BASE / "field-mu025" / "zerob.csv")),
    "joy_old": metrics(load_field(BASE / "field20261120c.csv")),
    "joy_mu025": metrics(load_field(BASE / "field-mu025" / "lateral2.csv")),
}

# 鈹€鈹€ P1: field zero-B comparison (target p2p, flip, tracking, roll/pitch) 鈹€鈹€鈹€鈹€鈹€
fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
labels = ["jitter(0.5) ", "mu025(0.25)"]
mp = [field["zeroB_old"]["p2p"], field["zeroB_mu025"]["p2p"]]
fl = [field["zeroB_old"]["flip"] * 100, field["zeroB_mu025"]["flip"] * 100]
ax = axes[0]
xx = np.arange(2)
ax.bar(xx - 0.18, mp, 0.36, label="leg target p2p [rad]", color="#3b7dd8")
ax2 = ax.twinx()
ax2.bar(xx + 0.18, fl, 0.36, label="sign-flip [%]", color="#e0703a")
ax.set_xticks(xx); ax.set_xticklabels(labels)
ax.set_ylabel("p2p [rad]"); ax2.set_ylabel("sign-flip [%]")
ax.set_title("zero-command: dither (lower is better)")
ax = axes[1]
for key, c in (("zeroB_old", "#3b7dd8"), ("zeroB_mu025", "#e0703a")):
    m = field[key]
    ax.errorbar([0, 1], [m["roll_mean"], m["pitch_mean"]],
                yerr=[[m["roll_mean"] - m["roll_min"], m["pitch_mean"] - m["pitch_min"]],
                      [m["roll_max"] - m["roll_mean"], m["pitch_max"] - m["pitch_mean"]]],
                fmt="o", capsize=4, label=key, color=c)
ax.set_xticks([0, 1]); ax.set_xticklabels(["roll", "pitch"])
ax.set_ylabel("deg (mean, min..max)")
ax.set_title("body attitude on hardware")
ax.legend(fontsize=8)
fig.savefig(PLOTS / "p1-field-zeroB-weights.png"); plt.close(fig)

# 鈹€鈹€ P2: joystick yaw tracking time series 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
fig, axes = plt.subplots(2, 1, figsize=(11, 5), sharex=False)
for ax, key, title in ((axes[0], "joy_old", "jitter(0.5) weight 鈥?commanded vz vs measured yaw rate"),
                       (axes[1], "joy_mu025", "mu025(0.25) weight 鈥?commanded vz vs measured yaw rate")):
    m = field[key]
    ax.plot(m["t"], m["cmd"][:, 2], color="#3b7dd8", lw=1, label="commanded wz [rad/s]")
    ax.plot(m["t"], m["gyro"][:, 2], color="#e0703a", lw=0.8, alpha=0.85, label="measured yaw rate")
    ax.set_ylabel("rad/s"); ax.set_xlabel("t [s]"); ax.set_title(title, fontsize=9)
    ax.legend(fontsize=8, loc="upper right")
fig.savefig(PLOTS / "p2-field-yaw-tracking.png"); plt.close(fig)

# 鈹€鈹€ P3: signed slopes comparison 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
slopes = json.loads((BASE / "field-mu025" / "yaw-lateral-slopes.json").read_text())
fig, axes = plt.subplots(1, 2, figsize=(9, 3.2))
names = list(slopes)
ax = axes[0]
ax.bar(names, [slopes[k]["yaw_slope"] for k in names], color=["#3b7dd8", "#e0703a"])
ax.set_title("yaw tracking slope (measured/commanded)"); ax.set_ylim(0, 0.3)
for i, k in enumerate(names):
    ax.text(i, slopes[k]["yaw_slope"] + 0.005, f'{slopes[k]["yaw_slope"]:.3f}', ha="center", fontsize=8)
ax = axes[1]
ax.bar(names, [slopes[k]["lateral_slope"] for k in names], color=["#3b7dd8", "#e0703a"])
ax.axhline(0, color="k", lw=0.8)
ax.set_title("lateral accel slope (noisy proxy)")
for i, k in enumerate(names):
    v = slopes[k]["lateral_slope"]
    ax.text(i, v + (0.05 if v >= 0 else -0.12), f"{v:.2f}", ha="center", fontsize=8)
fig.savefig(PLOTS / "p3-field-yaw-lateral-slopes.png"); plt.close(fig)

# 鈹€鈹€ P4: static load vs friction (the "unmodelled" finding) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
sp = BASE / "staticprobe"


def hip_load(name):
    d = np.load(sp / f"{name}.npz", allow_pickle=False)
    pd = d["samples"][..., 24:36]
    labs = d["labels"]
    sel = [i for i, l in enumerate(labs) if l != "nominal"]
    return float(np.abs(pd[:, sel][..., [0, 3]]).mean())  # front hips


fig, ax = plt.subplots(figsize=(7.5, 3.4))
labels = ["mu=0.5\nstatic 55/2", "mu=0.25\nstatic 55/2", "mu=0.5\nstiff kp200", "mu=0.25\nstiff kp200", "hardware\nT2 stand"]
vals = [0.39, 5.37, hip_load("orig-kp200-field"), hip_load("mu025-kp200-field"), 11.5]
cols = ["#9aa5b1", "#4b8fd8", "#9aa5b1", "#e0703a", "#2f9e6e"]
bars = ax.bar(labels, vals, color=cols)
for b, v in zip(bars, vals):
    ax.text(b.get_x() + b.get_width() / 2, v + 0.2, f"{v:.2f}", ha="center", fontsize=8)
ax.set_ylabel("front-hip load / PD torque [Nm]")
ax.set_title("sim-to-real hip load: ground/tire friction is the missing term")
fig.savefig(PLOTS / "p4-sim-torque-vs-friction.png"); plt.close(fig)

# 鈹€鈹€ P5: mass / COM sweep 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cases = [("mu025 base", "mu025-kp200-field"), ("+1kg", "mu025-mass1-field"),
         ("+2kg", "mu025-mass2-field"), ("+3kg", "mu025-mass3-field"),
         ("COM x-10", "mu025-comxb10-kp200"), ("COM x-20", "mu025-comxb20-kp200"),
         ("COM x-30", "mu025-comxb30-kp200"), ("COM x-40", "mu025-comxb40-kp200")]
fig, ax = plt.subplots(figsize=(9, 3.2))
vals = [hip_load(n) for _, n in cases]
ax.bar([c for c, _ in cases], vals, color="#4b8fd8")
for i, v in enumerate(vals):
    ax.text(i, v + 0.15, f"{v:.1f}", ha="center", fontsize=7.5)
ax.axhline(11.5, color="#2f9e6e", ls="--", lw=1, label="hardware front hip 11-12 Nm")
ax.set_ylabel("front-hip load [Nm]"); ax.legend(fontsize=8)
ax.set_title("negative controls: base mass and COM shift do not explain the gap")
fig.savefig(PLOTS / "p5-negative-controls.png"); plt.close(fig)

# 鈹€鈹€ P6: mu025 training curves 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
log = None
for cand in (BASE / "jit_src" / "train_reward_components.jsonl",
             BASE / "jit_src" / "train_reward_components.jsonl"):
    if cand.exists():
        log = cand
        break
if log:
    rows = [json.loads(l) for l in log.open()]
    upd = np.array([r["update"] for r in rows])
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.4))
    ax = axes[0]
    for k, c in (("executed_signflip_fraction", "#e0703a"), ("executed_delta_mean", "#3b7dd8")):
        if k in rows[0]:
            ax.plot(upd, [r[k] for r in rows], label=k, color=c, lw=1)
    ax.set_xlabel("update"); ax.set_title("jitter arm: executed-action dither metrics"); ax.legend(fontsize=8)
    ax = axes[1]
    for w, c in (("FL", "#3b7dd8"), ("FR", "#e0703a"), ("RL", "#2f9e6e"), ("RR", "#9b59b6")):
        k = f"event_right_{w}"
        if k in rows[0]:
            ax.plot(upd, [r[k] for r in rows], label=f"right {w}", color=c, lw=1)
    ax.set_xlabel("update"); ax.set_title("right-direction lift events per wheel"); ax.legend(fontsize=8)
    fig.savefig(PLOTS / "p6-training-curves-jitter.png"); plt.close(fig)

# 鈹€鈹€ P7: gate matrix across checkpoints 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def gate_of(path):
    rep = json.loads(Path(path).read_text())
    return {m: rep["modes"][m]["wheels_with_2plus_events"] for m in ("left", "yaw_left", "yaw_right", "right")}


mat = []
labels = []
for label, path in (("base 0.5", BASE / "pkg_src" / "stage-report.json"),
                    ("mu025 @200", BASE / "mu025" / "e200-stage-report.json"),
                    ("mu025 @400", BASE / "mu025" / "e400-stage-report.json"),
                    ("mu025 @600", BASE / "mu025" / "e600-stage-report.json")):
    if Path(path).exists():
        g = gate_of(path)
        mat.append([g[m] for m in ("left", "yaw_left", "yaw_right", "right")])
        labels.append(label)
if mat:
    fig, ax = plt.subplots(figsize=(6.5, 2.8))
    im = ax.imshow(mat, cmap="RdYlGn", vmin=0, vmax=4)
    ax.set_xticks(range(4)); ax.set_xticklabels(["left", "yaw_left", "yaw_right", "right"])
    ax.set_yticks(range(len(labels))); ax.set_yticklabels(labels)
    for i in range(len(mat)):
        for j in range(4):
            ax.text(j, i, f"{mat[i][j]}/4", ha="center", va="center", fontsize=9)
    ax.set_title("wheels with >=2 lift events (gate)")
    fig.colorbar(im, ax=ax, shrink=0.8)
    fig.savefig(PLOTS / "p7-gate-matrix.png"); plt.close(fig)
    (DATA / "gate-matrix.csv").write_text(
        "checkpoint,left,yaw_left,yaw_right,right\n" +
        "\n".join(f"{l},{','.join(map(str, r))}" for l, r in zip(labels, mat)) + "\n",
        encoding="utf-8")

# 鈹€鈹€ CSV extracts (downsampled, git-friendly) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
for key in ("zeroB_old", "zeroB_mu025", "joy_old", "joy_mu025"):
    m = field[key]
    step = max(1, len(m["t"]) // 400)
    out = DATA / f"field-{key}-downsampled.csv"
    with out.open("w", encoding="utf-8") as f:
        f.write("t_s,cmd_vx,cmd_vy,cmd_wz,gyro_x,gyro_y,gyro_z,roll_deg,pitch_deg,"
                + ",".join(f"tgt_{i}" for i in range(12)) + "\n")
        for i in range(0, len(m["t"]), step):
            f.write(f'{m["t"][i]:.2f},' + ",".join(f"{v:.4f}" for v in m["cmd"][i])
                    + "," + ",".join(f"{v:.4f}" for v in m["gyro"][i])
                    + f',{m["rp"][i][0]:.3f},{m["rp"][i][1]:.3f},'
                    + ",".join(f"{v:.4f}" for v in m["tgt"][i]) + "\n")

# metrics summary json
summary = {k: {kk: vv for kk, vv in v.items() if not isinstance(vv, np.ndarray)} for k, v in field.items()}
(ANAL / "field-metrics-summary.json").write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")

# training log downsample
if log:
    rows = [json.loads(l) for l in log.open()]
    keys = [k for k in ("update", "swing_event", "executed_signflip_fraction",
                        "executed_delta_mean", "executed_abs_mean",
                        "event_left_RR", "event_right_RR", "event_right_RL") if k in rows[0]]
    with (DATA / "training-jitter-downsampled.csv").open("w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for r in rows[::5]:
            f.write(",".join(str(r[k]) for k in keys) + "\n")

print("plots:", sorted(p.name for p in PLOTS.glob("*.png")))
print("data:", sorted(p.name for p in DATA.glob("*")))

