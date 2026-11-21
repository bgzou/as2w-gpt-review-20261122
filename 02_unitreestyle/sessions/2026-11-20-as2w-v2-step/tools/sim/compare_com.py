import numpy as np
from pathlib import Path

base = Path(r"X:\Items\as2w\.stage58\staticprobe")
cases = [
    ("mu025 (base)", "mu025-field"),
    ("mu025 COM x +20mm", "mu025-comxf20-field"),
    ("mu025 COM x +40mm", "mu025-comxf40-field"),
    ("mu025 COM x +60mm", "mu025-comxf60-field"),
    ("mu025 COM y +40mm", "mu025-comf40-field"),
    ("mu025 COM y -30mm", "mu025-comb30-field"),
]
print(f"{'case':22s} {'front_hip':>9s} {'rear_hip':>8s} {'all_hip':>8s} {'calf':>7s} {'all':>6s}")
for label, name in cases:
    p = base / f"{name}.npz"
    if not p.exists():
        print(f"{label:22s}  (missing)")
        continue
    d = np.load(p, allow_pickle=False)
    pd = d["samples"][..., 24:36]
    labs = d["labels"]
    sel = [i for i, l in enumerate(labs) if l != "nominal"]
    fh = np.abs(pd[:, sel][..., [0, 3]]).mean()
    rh = np.abs(pd[:, sel][..., [6, 9]]).mean()
    ah = np.abs(pd[:, sel][..., [0, 3, 6, 9]]).mean()
    calf = np.abs(pd[:, sel][..., [2, 5, 8, 11]]).mean()
    allj = np.abs(pd[:, sel]).mean()
    print(f"{label:22s} {fh:9.2f} {rh:8.2f} {ah:8.2f} {calf:7.2f} {allj:6.2f}")
print()
print("hardware T2 fixed stand: front hip 11-12 Nm, rear hip 13-15.2 Nm, calf static err 0.26-0.28 rad")
print("2026-11-19 MuJoCo candidate: front hip 5.6-6.7 Nm")
