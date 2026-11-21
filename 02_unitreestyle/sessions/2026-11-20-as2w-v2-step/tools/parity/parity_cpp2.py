import json
import sys

import numpy as np
import onnxruntime as ort

sys.path.insert(0, "/root/as2w/pkgcheck2/runtime")
import as2w_runtime as rt

obs_cpp, tgt_cpp = [], []
for line in open("/root/as2w/parity/replay_out.txt"):
    p = line.strip().split(",")
    if not p or not p[0].startswith("UNITREE58_"):
        continue
    if p[0] == "UNITREE58_OBS":
        obs_cpp.append([float(x) for x in p[2:]])
    elif p[0] == "UNITREE58_TARGET":
        tgt_cpp.append([float(x) for x in p[2:]])
obs_cpp = np.array(obs_cpp)
tgt_cpp = np.array(tgt_cpp)

d = np.load("/root/as2w/parity/fixture-jit.npz")
quat, gyro, q, dq, cmd = d["quat"], d["gyro"], d["q"], d["dq"], d["cmd"]
n = min(len(obs_cpp), len(q))


def pg(qq):
    w, x, y, z = qq
    return np.array([-(2 * (x * z - w * y)), -(2 * (y * z + w * x)), -(1 - 2 * (x * x + y * y))])


def wrap_exec(executed, target, command):
    """Training wrapper but taking the already-squashed action (tanh-baked graph)."""
    target = target.copy()
    target[:12] += (executed * rt.LEG_SCALE - target[:12]) * rt.ALPHA_LEG
    wheel = np.clip(np.full(4, command[0] / rt.RADIUS), -rt.WHEEL_LIMIT, rt.WHEEL_LIMIT)
    target[12:] += (wheel - target[12:]) * rt.ALPHA_WHEEL
    if abs(command[0]) < 1e-9:
        target[12:][np.abs(target[12:]) < 0.02] = 0.0
    return target


sess = ort.InferenceSession("/root/as2w/pkgcheck2/model/step_policy_deploy_tanh.onnx",
                            providers=["CPUExecutionProvider"])
target = np.zeros(16)
obs_err = 0.0
q_err = 0.0
w_err = 0.0
worst = -1
for i in range(n):
    o = rt.build_obs(cmd[i], gyro[i], pg(quat[i]), q[i], dq[i], target)
    e = float(np.max(np.abs(o - obs_cpp[i])))
    if e > obs_err:
        obs_err, worst = e, i
    raw = sess.run(None, {"obs": o.reshape(1, -1)})[0][0]
    executed = np.clip(raw, -1.0, 1.0)
    target = wrap_exec(executed, target, cmd[i])
    q_err = max(q_err, float(np.max(np.abs((rt.DEFAULT_JOINT_POS[:12] + target[:12]) - tgt_cpp[i, 12:24]))))
    w_err = max(w_err, float(np.max(np.abs(target[12:] - tgt_cpp[i, 24:28]))))

report = {
    "frames": int(n),
    "obs_cpp_vs_python_max_abs": obs_err,
    "obs_worst_frame": worst,
    "leg_q_target_cpp_vs_python": q_err,
    "wheel_target_cpp_vs_python": w_err,
    "tolerance": 2e-5,
    "pass": bool(obs_err <= 2e-5 and q_err <= 2e-5 and w_err <= 2e-5),
}
print(json.dumps(report, indent=1))

