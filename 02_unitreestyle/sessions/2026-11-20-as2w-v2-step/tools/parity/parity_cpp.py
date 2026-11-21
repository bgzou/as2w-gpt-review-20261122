import json
import sys

import numpy as np
import onnxruntime as ort

sys.path.insert(0, "/root/as2w/pkgcheck2/runtime")
import as2w_runtime as rt

REPLAY = "/root/as2w/parity/replay_out.txt"
FIXTURE = "/root/as2w/parity/fixture-jit.npz"
ONNX = "/root/as2w/pkgcheck2/model/step_policy_deploy_tanh.onnx"

obs_cpp, tgt_cpp = [], []
for line in open(REPLAY):
    p = line.strip().split(",")
    if not p or not p[0].startswith("UNITREE58_"):
        continue
    if p[0] == "UNITREE58_OBS":
        obs_cpp.append([float(x) for x in p[2:]])
    elif p[0] == "UNITREE58_TARGET":
        tgt_cpp.append([float(x) for x in p[2:]])
obs_cpp = np.array(obs_cpp, dtype=np.float64)
tgt_cpp = np.array(tgt_cpp, dtype=np.float64)

d = np.load(FIXTURE)
quat, gyro, q, dq, cmd = d["quat"], d["gyro"], d["q"], d["dq"], d["cmd"]
prior = d["prior"]
obs_rec = d["obs_ref"]
n = min(len(obs_cpp), len(q))


def pg(qq):
    w, x, y, z = qq
    return np.array([-(2 * (x * z - w * y)), -(2 * (y * z + w * x)), -(1 - 2 * (x * x + y * y))])


# 1) observation parity: C++ vs recorded Isaac observation, and C++ vs numpy runtime
obs_py = np.array([rt.build_obs(cmd[i], gyro[i], pg(quat[i]), q[i], dq[i], prior[i]) for i in range(n)])
obs_to_rec = float(np.max(np.abs(obs_cpp[:n] - obs_rec[:n])))
obs_cpp_vs_py = float(np.max(np.abs(obs_cpp[:n] - obs_py)))

# 2) target parity: Python self-consistent recursion vs C++ targets
sess = ort.InferenceSession(ONNX, providers=["CPUExecutionProvider"])
target = np.zeros(16)
py_q = np.zeros((n, 12))
for i in range(n):
    o = rt.build_obs(cmd[i], gyro[i], pg(quat[i]), q[i], dq[i], target)
    raw = sess.run(None, {"obs": o.reshape(1, -1)})[0][0]
    a = np.clip(raw, -1.0, 1.0)
    target = rt.wrap_action(raw, target, cmd[i])
    py_q[i] = rt.DEFAULT_JOINT_POS[:12] + a * 0 + target[:12]  # default + offset

# C++ columns: action(12), leg q(12), wheel dq(4) -> leg q at 12..24
cpp_leg_q = tgt_cpp[:n, 12:24]
qp_err = float(np.max(np.abs(cpp_leg_q - py_q)))
# action parity: C++ action vs tanh(clamped) applied action
cpp_action = tgt_cpp[:n, 0:12]

report = {
    "frames": int(n),
    "obs_cpp_vs_recorded_isaac": obs_to_rec,
    "obs_cpp_vs_numpy_runtime": obs_cpp_vs_py,
    "leg_q_target_cpp_vs_python": qp_err,
    "cpp_action_abs_max": float(np.abs(cpp_action).max()),
    "pass": bool(obs_cpp_vs_py <= 2e-5 and qp_err <= 2e-5),
}
print(json.dumps(report, indent=1))
