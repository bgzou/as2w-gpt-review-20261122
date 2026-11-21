import numpy as np
from pathlib import Path

base = Path(r"X:\Items\as2w\.stage58\staticprobe")
rows = [
    ("orig (mu 0.5)", "orig"),
    ("mu025 (mu 0.25)", "mu025"),
    ("mu025 + joint fric 0.5", "mu025-fric0.5"),
    ("mu025 + joint fric 1.5", "mu025-fric1.5"),
    ("mu025 + joint fric 3.0", "mu025-fric3.0"),
]
print(f"{'case':26s} {'hip|pd|':>8s} {'all|pd|':>8s} {'FL_hip':>7s} {'FR_hip':>7s} {'RL_hip':>7s} {'RR_hip':>7s}")
for label, name in rows:
    d = np.load(base / f"{name}.npz", allow_pickle=False)
    pd = d["samples"][..., 24:36]
    hips = np.abs(pd[..., [0, 3, 6, 9]]).mean()
    print(f"{label:26s} {hips:8.2f} {np.abs(pd).mean():8.2f} "
          f"{np.abs(pd[...,0]).mean():7.2f} {np.abs(pd[...,3]).mean():7.2f} "
          f"{np.abs(pd[...,6]).mean():7.2f} {np.abs(pd[...,9]).mean():7.2f}")
print()
print("hardware reference: static held pose front hip 11-12 Nm (2026-11-19 T2 session);")
print("hardware policy run (2026-11-20 zero-B): hip |tau| 4-6 Nm, calf 6-9 Nm")
