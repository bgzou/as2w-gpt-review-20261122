"""Does the policy produce its 'left' or 'right' gait for each cmd_vy sign?

Sim reference (jitter weight, fixed replay):
  vy>0 (left)  -> all four wheels lift (FL/FR/RL/RR >= 2 lifts, clearance 27-36 mm)
  vy<0 (right) -> rear-right wheel NEVER lifts (clearance ~-3 mm)

If the hardware log shows the right-command signature (RR flat) for cmd_vy>0,
the command sign reaching the policy is inverted relative to the trained task.
"""
import json
from pathlib import Path

import numpy as np

LOG = Path(r"X:\Items\as2w\.stage58\field20261120c.csv")
DEFAULT = np.array([0.0, 0.8, -1.6] * 4)
REQ = 149

frames = []
with LOG.open("r", encoding="utf-8", errors="replace") as f:
    for line in f:
        if not line.startswith("DEBUG_POLICY,"):
            continue
        p = line.rstrip("\n").split(",")
        if p[2].upper() != "WALK":
            continue
        try:
            cmd = np.array([float(x) for x in p[5:8]])
            req = np.array([float(p[REQ + 4 * i]) for i in range(16)])[:12]
        except (ValueError, IndexError):
            continue
        frames.append((cmd[1], req))

arr = np.array([[c] + list(t) for c, t in frames])
cmdv, tgt = arr[:, 0], arr[:, 1:]

res = {}
for label, mask in (("cmd_vy>+0.03", cmdv > 0.03), ("cmd_vy<-0.03", cmdv < -0.03),
                    ("cmd_vy~0", np.abs(cmdv) <= 0.03)):
    if not mask.any():
        res[label] = "no frames"
        continue
    seg = tgt[mask]
    p2p = seg.max(0) - seg.min(0)
    res[label] = {
        "frames": int(mask.sum()),
        "per_leg_p2p_rad": {k: round(float(v), 3) for k, v in
                            zip(("FL", "FR", "RL", "RR"), p2p[0::3])},
        "per_leg_absmax_rad": {k: round(float(v), 3) for k, v in
                               zip(("FL", "FR", "RL", "RR"), np.abs(seg).max(0)[0::3])},
    }
print(json.dumps(res, indent=1))

# also: is there any correlation between cmd_vy and the RR joint motion?
out = Path(r"X:\Items\as2w\.stage58\field20261120c-gait-by-vysign.json")
out.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
print("wrote", out)
