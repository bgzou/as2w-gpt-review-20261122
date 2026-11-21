import numpy as np
from pathlib import Path

tape = Path(r"X:\Items\as2w\.stage58\jit_src\sensor_tape.npz")
out = Path(r"X:\Items\as2w\.stage58\fixture-jit.txt")
N = 300
TRIAL = 0

d = np.load(tape, allow_pickle=False)
quat = d["quat_wxyz"][:N, TRIAL]      # wxyz
gyro = d["gyro_rad_s"][:N, TRIAL]     # body-frame
q = d["q_rad"][:N, TRIAL]             # 16 policy order
dq = d["dq_rad_s"][:N, TRIAL]
cmd = d["command_si"][:N, TRIAL]

lines = []
for i in range(N):
    vals = [f"{i*0.02:.6f}"]
    vals += [f"{v:.9g}" for v in quat[i]]
    vals += [f"{v:.9g}" for v in gyro[i]]
    vals += [f"{v:.9g}" for v in q[i]]
    vals += [f"{v:.9g}" for v in dq[i]]
    vals += [f"{v:.9g}" for v in cmd[i]]
    lines.append(" ".join(vals))
out.write_text("\n".join(lines) + "\n", encoding="ascii")
print("wrote", out, "rows", len(lines), "bytes", out.stat().st_size)

# also dump the same sensor sequence as npz for the local reference run
np.savez(out.with_suffix(".npz"), quat=quat, gyro=gyro, q=q, dq=dq, cmd=cmd,
         obs_ref=d["observation"][:N, TRIAL], raw_ref=d["raw_action"][:N, TRIAL],
         prior=d["prior_target"][:N, TRIAL], nxt=d["next_target"][:N, TRIAL])
print("dumped reference arrays")
