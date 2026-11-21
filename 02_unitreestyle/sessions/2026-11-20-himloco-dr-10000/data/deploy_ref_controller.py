"""Reference deployment controller for the AS2-W HIMLoco policy.

This mirrors the training environment exactly (observation layout, target
smoothing, phase clock) and runs the exported ONNX policy. It is a reference
implementation for integration/verification on the robot PC - not a safety
layer. All limits below must be re-checked against the real LowCmd/LowState
contract before publishing.

Contract (from common/environment.py + quality_environment.py):
  frame(58) = [cmd(3) | gyro(3) | gravity(3) | q-default(16) | dq*0.05(16)
               | applied_leg/0.25(12) | applied_wheel/12(4) | 0.37(1)]
  QualityEnv appends [sin(phase*2pi)*strength, cos(phase*2pi)*strength] -> 60
  history = 6 newest-first frames -> 360
  policy.onnx: 360 -> 16 actions
  action[:12] -> target_leg lerp toward action*0.25 (alpha_leg)
  action[12:] -> target_wheel lerp toward clamp(action*12, +-12) (alpha_wheel)
"""
import argparse
import json
import math

import numpy as np

DT = 0.02
LEG_CUTOFF_HZ = 10.0
WHEEL_CUTOFF_HZ = 5.0
ALPHA_LEG = 1.0 - math.exp(-2.0 * math.pi * LEG_CUTOFF_HZ * DT)
ALPHA_WHEEL = 1.0 - math.exp(-2.0 * math.pi * WHEEL_CUTOFF_HZ * DT)
HEIGHT = 0.37
FRAME = 60
HISTORY = 6


def gait_frequency(vx, vy, wz):
    speed = abs(vy) + 0.25 * abs(wz)
    return min(max(1.25 + 0.95 * speed, 1.25), 1.75)


def step_strength(vy, wz):
    return min(max((abs(vy) + 0.28 * abs(wz) - 0.04) / 0.16, 0.0), 1.0)


class Controller:
    def __init__(self, policy_path, default_joint_pos):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name
        assert np.asarray(default_joint_pos).size == 16
        self.default = np.asarray(default_joint_pos, dtype=np.float32)
        self.history = np.zeros((HISTORY, FRAME), dtype=np.float32)
        self.target_leg = np.zeros(12, dtype=np.float32)
        self.target_wheel = np.zeros(4, dtype=np.float32)
        self.phase = 0.0

    def obs(self, cmd, gyro, gravity, q, dq):
        """q/dq are 16-dim (12 legs then 4 wheels), in the LEGS+WHEELS order."""
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        gyro = np.asarray(gyro, dtype=np.float32)
        gravity = np.asarray(gravity, dtype=np.float32)
        delta = q - self.default
        delta[12:] = 0.0
        frame = np.concatenate([
            cmd, gyro, gravity, delta, dq * 0.05,
            self.target_leg / 0.25, self.target_wheel / 12.0,
            np.array([HEIGHT], dtype=np.float32)])
        self.phase = (self.phase + DT * gait_frequency(*cmd)) % 1.0
        strength = step_strength(cmd[1], cmd[2])
        phi = self.phase * 2.0 * math.pi
        frame = np.concatenate([frame,
                                np.array([math.sin(phi) * strength, math.cos(phi) * strength],
                                         dtype=np.float32)])
        self.history = np.roll(self.history, 1, axis=0)
        self.history[0] = frame
        return self.history.reshape(-1)[None]

    def act(self, obs_history):
        action = self.sess.run(None, {self.input_name: obs_history})[0][0]
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        self.target_leg += (action[:12] * 0.25 - self.target_leg) * ALPHA_LEG
        wheel = np.clip(action[12:] * 12.0, -12.0, 12.0)
        self.target_wheel += (wheel - self.target_wheel) * ALPHA_WHEEL
        joint_pos_target = self.default[:12] + self.target_leg
        return {"leg_position": joint_pos_target.tolist(),
                "wheel_velocity": self.target_wheel.tolist()}


def selftest(policy_path):
    import onnxruntime as ort
    sess = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
    shape = sess.get_inputs()[0].shape
    out = sess.get_outputs()[0].shape
    ctl = Controller(policy_path, np.zeros(16))
    rng = np.random.default_rng(0)
    worst = 0.0
    for i in range(50):
        cmd = np.array([0.6, 0.0, 0.0], dtype=np.float32)
        gyro = rng.normal(0, .1, 3).astype(np.float32)
        gravity = np.array([0., 0., -1.], dtype=np.float32)
        q = np.zeros(16, dtype=np.float32)
        dq = np.zeros(16, dtype=np.float32)
        obs = ctl.obs(cmd, gyro, gravity, q, dq)
        assert obs.shape == (1, 360), obs.shape
        out_cmd = ctl.act(obs)
        worst = max(worst, max(abs(v) for v in out_cmd["wheel_velocity"]))
        assert len(out_cmd["leg_position"]) == 12 and len(out_cmd["wheel_velocity"]) == 4
    print(json.dumps({"onnx_input": shape, "onnx_output": out,
                      "alpha_leg": round(ALPHA_LEG, 4), "alpha_wheel": round(ALPHA_WHEEL, 4),
                      "wheel_velocity_max_after_50_steps": round(worst, 3),
                      "selftest": "ok"}, indent=1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", required=True)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest(a.policy)
