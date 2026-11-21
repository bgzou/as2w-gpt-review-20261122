import numpy as np
from pathlib import Path

base = Path(r"X:\Items\as2w\.stage58\staticprobe")
cases = [
    ("mu025 base", "mu025-kp200-field"),
    ("COM x -10mm", "mu025-comxb10-kp200"),
    ("COM x -20mm", "mu025-comxb20-kp200"),
    ("COM x -30mm", "mu025-comxb30-kp200"),
    ("COM x -40mm", "mu025-comxb40-kp200"),
    ("orig mu0.5", "orig-kp200-field"),
]
print("stiff hold kp=200, PD torque ~ load (Nm)")
print(f"{'case':14s} {'front_hip':>9s} {'rear_hip':>8s} {'calf':>7s}")
for label, name in cases:
    p = base / f"{name}.npz"
    if not p.exists():
        print(f"{label:14s}  (missing)")
        continue
    d = np.load(p, allow_pickle=False)
    s = d["samples"]
    pd = s[..., 24:36]
    labs = d["labels"]
    sel = [i for i, l in enumerate(labs) if l != "nominal"]
    fh = np.abs(pd[:, sel][..., [0, 3]]).mean()
    rh = np.abs(pd[:, sel][..., [6, 9]]).mean()
    calf = np.abs(pd[:, sel][..., [2, 5, 8, 11]]).mean()
    print(f"{label:14s} {fh:9.2f} {rh:8.2f} {calf:7.2f}")
print()
print("hardware T2 fixed stand: front hip 11-12 Nm, rear hip 13-15.2 Nm")
