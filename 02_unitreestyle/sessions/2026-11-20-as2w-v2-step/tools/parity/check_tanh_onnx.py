import sys, json
from pathlib import Path
import numpy as np

sys.path.insert(0, "/root/as2w/pkgcheck/runtime")
import as2w_runtime as rt
import onnxruntime as ort

tape = Path("/root/as2w/pkgcheck/replay/sensor_tape.npz")
onnx = Path("/root/as2w/calibration/unitree58/v2step_hard_tanh.onnx")
sess = ort.InferenceSession(str(onnx), providers=["CPUExecutionProvider"])
d = np.load(tape, allow_pickle=False)
obs = d["observation"].reshape(-1, rt.OBS_DIM).astype(np.float32)
raw = d["raw_action"].reshape(-1, rt.ACT_DIM)
expect = np.tanh(raw)
out = sess.run(None, {"obs": obs})[0]
err = float(np.max(np.abs(out - expect)))
print(json.dumps({"samples": int(obs.shape[0]),
                  "tanh_onnx_vs_tanh_recorded_max_abs_err": err,
                  "pass": err <= 1e-5}, indent=1))
