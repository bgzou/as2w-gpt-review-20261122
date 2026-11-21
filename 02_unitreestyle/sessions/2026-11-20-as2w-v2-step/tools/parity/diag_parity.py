import sys

import numpy as np

sys.path.insert(0, "/root/as2w/pkgcheck2/runtime")
import as2w_runtime as rt

obs_cpp = []
for line in open("/root/as2w/parity/replay_out.txt"):
    p = line.strip().split(",")
    if p and p[0] == "UNITREE58_OBS":
        obs_cpp.append([float(x) for x in p[2:]])
obs_cpp = np.array(obs_cpp)

d = np.load("/root/as2w/parity/fixture-jit.npz")
quat, gyro, q, dq, cmd, prior = d["quat"], d["gyro"], d["q"], d["dq"], d["cmd"], d["prior"]


def pg(qq):
    w, x, y, z = qq
    return np.array([-(2 * (x * z - w * y)), -(2 * (y * z + w * x)), -(1 - 2 * (x * x + y * y))])


blocks = {"cmd": (0, 3), "gyro": (3, 6), "grav": (6, 9), "q": (9, 25), "dq": (25, 41), "prev": (41, 57), "h": (57, 58)}
for i in (0, 1, 2):
    o = rt.build_obs(cmd[i], gyro[i], pg(quat[i]), q[i], dq[i], prior[i])
    print(f"--- frame {i}")
    for name, (a, b) in blocks.items():
        e = np.abs(obs_cpp[i][a:b] - o[a:b])
        print(f"  {name:5s} max_err {e.max():.6f}  argmax {a+int(e.argmax())}  cpp={obs_cpp[i][a:b][:4]} py={o[a:b][:4]}")
    # full diff per index
    diff = np.abs(obs_cpp[i] - o)
    idx = np.argsort(diff)[::-1][:6]
    print("  top diffs:", [(int(j), round(float(diff[j]), 4)) for j in idx])
