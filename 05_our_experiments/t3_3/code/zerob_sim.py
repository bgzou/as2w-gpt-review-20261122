#!/usr/bin/env python3
"""T3-3: HIMLoco model_18200 zero-B, same-contract MuJoCo run (server side).

Contract used (must match the DEPLOYED one, not deploy_ref_controller.py):
  * obs frame 60 = [cmd3 | gyro3 | gravity3 | q-default(16, wheels zeroed) | dq*0.05(16)
                    | applied_leg/0.25(12) | applied_wheel/12(4) | 0.37 | sin/cos phase*strength]
  * history 6 newest-first -> 360 ; ONNX 360 -> 16 pre-tanh
  * bounded = tanh(raw) ; physical = leg_gain(=0.6 flat) * bounded
  * target_leg += (0.25*physical - target_leg) * alpha_leg
  * wheel_cmd = clamp((vx - wz*side)/R + 2*tanh(raw[12:]), +-12) ; target_wheel low-passed
  * handoff: first 100 policy steps (2 s) applied = (k+1)/100 * target, command forced 0
  * PD: legs position kp55 kv2, wheels velocity kv2 (from the MJCF actuators)

Protocol: 0.5 s precondition at the default pose -> 2 s handoff (zero command) -> hold until
`--seconds` total. Records per policy step: pitch, base z, per-leg (cmd-actual), wheel cmd/dq,
per-foot normal force. Prints the rear-leg mean delta over the last 3 s, next to the field
values (RL_thigh +0.237, RR_thigh +0.239, RL_calf +0.336, RR_calf +0.331 rad).

Exit 0 on a completed run.
"""
import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

DT_POLICY = 0.02
ALPHA_LEG = 1.0 - math.exp(-2.0 * math.pi * 10.0 * DT_POLICY)
ALPHA_WHEEL = 1.0 - math.exp(-2.0 * math.pi * 5.0 * DT_POLICY)
LEG_SCALE = 0.25
LEG_GAIN = np.array([0.6, 0.6, 0.6] * 4)
WHEEL_GAIN = 2.0
WHEEL_CLAMP = 12.0
WHEEL_R = 0.086
WHEEL_SIDE = 0.1654
HEIGHT = 0.37
HANDOFF_STEPS = 100
LEGS = ["FL", "FR", "RL", "RR"]
POLICY_JOINTS = [f"{l}_{p}" for l in LEGS for p in ("hip", "thigh", "calf")] + [f"{l}_foot" for l in LEGS]
DEFAULT = np.array([0.0, 0.8, -1.6] * 4 + [0.0] * 4, dtype=float)
FIELD_REAR = {"RL_thigh": 0.237, "RR_thigh": 0.239, "RL_calf": 0.336, "RR_calf": 0.331}


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def gait_frequency(vx, vy, wz):
    return min(max(1.25 + 0.95 * (abs(vy) + 0.25 * abs(wz)), 1.25), 1.75)


def step_strength(vy, wz):
    return min(max((abs(vy) + 0.28 * abs(wz) - 0.04) / 0.16, 0.0), 1.0)


class Contract:
    def __init__(self, onnx_path, default):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.default = np.asarray(default, dtype=float)
        self.history = np.zeros((6, 60))
        self.target_leg = np.zeros(12)
        self.target_wheel = np.zeros(4)
        self.applied_leg = np.zeros(12)
        self.applied_wheel = np.zeros(4)
        self.phase = 0.0

    def step(self, cmd, gyro, gravity, q16, dq16, handoff_count):
        delta = np.asarray(q16, float) - self.default
        delta[12:] = 0.0
        frame = np.concatenate([cmd, gyro, gravity, delta, np.asarray(dq16, float) * 0.05,
                                self.applied_leg / LEG_SCALE, self.applied_wheel / WHEEL_CLAMP,
                                [HEIGHT]])
        self.phase = (self.phase + DT_POLICY * gait_frequency(*cmd)) % 1.0
        st = step_strength(cmd[1], cmd[2])
        phi = self.phase * 2 * math.pi
        frame = np.concatenate([frame, [math.sin(phi) * st, math.cos(phi) * st]])
        self.history = np.roll(self.history, 1, axis=0)
        self.history[0] = frame
        raw = self.sess.run(None, {self.in_name: self.history.reshape(-1)[None].astype(np.float32)})[0][0].astype(float)

        bounded = np.tanh(raw)
        physical = LEG_GAIN * bounded[:12]
        self.target_leg += (LEG_SCALE * physical - self.target_leg) * ALPHA_LEG
        ff = (cmd[0] - cmd[2] * WHEEL_SIDE) / WHEEL_R
        wheel_cmd = np.clip(ff + WHEEL_GAIN * np.tanh(raw[12:]), -WHEEL_CLAMP, WHEEL_CLAMP)
        self.target_wheel += (wheel_cmd - self.target_wheel) * ALPHA_WHEEL

        if handoff_count < HANDOFF_STEPS:
            ramp = (handoff_count + 1) / HANDOFF_STEPS
        else:
            ramp = 1.0
        self.applied_leg = ramp * self.target_leg
        self.applied_wheel = ramp * self.target_wheel
        return raw, ramp

    def joint_targets(self):
        return self.default[:12] + self.applied_leg, self.applied_wheel.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=Path, required=True)
    ap.add_argument("--policy", type=Path, required=True)
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--precondition-s", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=9.7)
    ap.add_argument("--z0", type=float, default=0.33)
    # single-variable overrides (ablation); defaults keep the delivered MJCF values
    ap.add_argument("--base-mass", type=float, default=None, help="override base_link mass [kg]")
    ap.add_argument("--base-com-x", type=float, default=None, help="override base_link inertial pos x [m]")
    ap.add_argument("--leg-kp", type=float, default=None)
    ap.add_argument("--leg-kd", type=float, default=None)
    ap.add_argument("--init-pose", choices=["default", "factory"], default="default")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    import mujoco
    for p in (args.scene, args.policy, args.input):
        if not p.is_file():
            print(f"ERROR: missing {p}", file=sys.stderr)
            return 2
    pkg = json.loads(args.input.read_text(encoding="utf-8"))
    init = pkg["initial_state"]
    quat = init["quat_wxyz"]

    m = mujoco.MjModel.from_xml_path(str(args.scene))
    overrides = {}
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if args.base_mass is not None:
        overrides["base_mass_kg_before"] = float(m.body_mass[bid])
        m.body_mass[bid] = float(args.base_mass)
        overrides["base_mass_kg_after"] = float(m.body_mass[bid])
    if args.base_com_x is not None:
        overrides["base_com_x_before"] = float(m.body_ipos[bid][0])
        m.body_ipos[bid][0] = float(args.base_com_x)
        overrides["base_com_x_after"] = float(m.body_ipos[bid][0])
    if args.leg_kp is not None or args.leg_kd is not None:
        before = {}
        for n in POLICY_JOINTS[:12]:
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"a_{n}")
            before[n] = [float(m.actuator_gainprm[a][0]), float(-m.actuator_biasprm[a][2])]
            kp = args.leg_kp if args.leg_kp is not None else before[n][0]
            kv = args.leg_kd if args.leg_kd is not None else before[n][1]
            m.actuator_gainprm[a][0] = kp
            m.actuator_biasprm[a][0] = 0.0
            m.actuator_biasprm[a][1] = -kp
            m.actuator_biasprm[a][2] = -kv
        overrides["leg_gain_before"] = before
        overrides["leg_gain_after"] = {"kp": args.leg_kp, "kd": args.leg_kd}
    dec = int(round(DT_POLICY / m.opt.timestep))
    if abs(dec * m.opt.timestep - DT_POLICY) > 1e-9:
        print(f"ERROR: timestep {m.opt.timestep} not commensurate with {DT_POLICY}", file=sys.stderr)
        return 2
    d = mujoco.MjData(m)
    qa = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in POLICY_JOINTS}
    da = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in POLICY_JOINTS}
    aid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"a_{n}") for n in POLICY_JOINTS}
    foot_geoms = {}
    for l in LEGS:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{l}_foot")
        foot_geoms[l] = set(range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]))

    mujoco.mj_resetData(m, d)
    d.qpos[0:3] = [0.0, 0.0, args.z0]
    d.qpos[3:7] = quat
    init_pose = DEFAULT.copy()
    if args.init_pose == "factory":
        q16_pkg = pkg["initial_state"]["q16_policy_order"]
        init_pose = DEFAULT.copy(); init_pose[:12] = np.array(q16_pkg[:12], dtype=float)
    for i, n in enumerate(POLICY_JOINTS):
        d.qpos[qa[n]] = init_pose[i]
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)

    def gravity_gyro():
        w, x, y, z = d.qpos[3:7]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return R.T @ np.array([0.0, 0.0, -1.0]), R.T @ d.qvel[3:6]

    # precondition: hold the INITIAL pose with PD so contact settles before the policy starts
    for _ in range(int(round(args.precondition_s / DT_POLICY))):
        for i, n in enumerate(POLICY_JOINTS[:12]):
            d.ctrl[aid[n]] = init_pose[i]
        for i, n in enumerate(POLICY_JOINTS[12:]):
            d.ctrl[aid[n]] = 0.0
        for _ in range(dec):
            mujoco.mj_step(m, d)

    ctl = Contract(args.policy, DEFAULT)
    cmd = np.array([0.0, 0.0, 0.0])
    n_steps = int(round(args.seconds / DT_POLICY))
    rows = []
    for k in range(n_steps):
        q16 = np.array([d.qpos[qa[n]] for n in POLICY_JOINTS])
        dq16 = np.array([d.qvel[da[n]] for n in POLICY_JOINTS])
        grav, gyro = gravity_gyro()
        raw, ramp = ctl.step(cmd, gyro, grav, q16, dq16, k)
        leg_target, wheel_target = ctl.joint_targets()
        for i, n in enumerate(POLICY_JOINTS[:12]):
            d.ctrl[aid[n]] = leg_target[i]
        for i, n in enumerate(POLICY_JOINTS[12:]):
            d.ctrl[aid[n]] = wheel_target[i]
        for _ in range(dec):
            mujoco.mj_step(m, d)

        w, x, y, z = d.qpos[3:7]
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        fn = {l: 0.0 for l in LEGS}
        for ci in range(d.ncon):
            con = d.contact[ci]
            f6 = np.zeros(6)
            mujoco.mj_contactForce(m, d, ci, f6)
            for l, geoms in foot_geoms.items():
                if con.geom1 in geoms or con.geom2 in geoms:
                    fn[l] += float(abs(f6[0]))
        q_act = np.array([d.qpos[qa[n]] for n in POLICY_JOINTS[:12]])
        delta_cmd = leg_target - q_act
        rows.append({
            "step": k, "t_s": round((k + 1) * DT_POLICY, 3), "handoff": k < HANDOFF_STEPS,
            "ramp": round(ramp, 4), "pitch_deg": round(pitch, 4), "base_z_m": round(float(d.qpos[2]), 5),
            "leg_cmd": [round(float(v), 5) for v in leg_target],
            "leg_act": [round(float(v), 5) for v in q_act],
            "cmd_minus_actual": [round(float(v), 5) for v in delta_cmd],
            "wheel_cmd": [round(float(v), 4) for v in wheel_target],
            "wheel_dq": [round(float(d.qvel[da[n]]), 4) for n in POLICY_JOINTS[12:]],
            "fn_N": {l: round(v, 2) for l, v in fn.items()},
            "raw_abs_max": round(float(np.abs(raw).max()), 4),
        })

    tail = [r for r in rows if r["t_s"] >= rows[-1]["t_s"] - 3.0]
    idx = {n: i for i, n in enumerate(POLICY_JOINTS[:12])}
    rear = {n: float(np.mean([r["cmd_minus_actual"][idx[n]] for r in tail])) for n in FIELD_REAR}
    res = {
        "task": "T3-3 HIMLoco model_18200 zero-B, same-contract MuJoCo",
        "generator": Path(__file__).name,
        "generator_sha256": sha256(Path(__file__)),
        "mujoco_version": mujoco.__version__,
        "inputs": {"scene": {"path": str(args.scene), "sha256": sha256(args.scene)},
                   "policy": {"path": str(args.policy), "sha256": sha256(args.policy)},
                   "zeroB_package": {"path": str(args.input), "sha256": sha256(args.input)}},
        "protocol": {"precondition_s": args.precondition_s, "handoff_s": HANDOFF_STEPS * DT_POLICY,
                     "total_s": args.seconds, "z0": args.z0, "dt_policy": DT_POLICY,
                     "decimation": dec, "command": [0.0, 0.0, 0.0],
                     "init_pose": args.init_pose, "tag": args.tag,
                     "overrides": overrides},
        "model_mass_total_kg": float(sum(m.body_mass)),
        "base_link_mass_kg": float(m.body_mass[bid]),
        "base_link_com_m": [float(v) for v in m.body_ipos[bid]],
        "contract": {"alpha_leg": ALPHA_LEG, "alpha_wheel": ALPHA_WHEEL, "leg_gain": LEG_GAIN.tolist(),
                     "leg_scale": LEG_SCALE, "wheel_gain": WHEEL_GAIN, "wheel_r": WHEEL_R,
                     "wheel_side": WHEEL_SIDE, "handoff_steps": HANDOFF_STEPS},
        "tail_window_s": 3.0,
        "rear_leg_cmd_minus_actual_tail_mean_rad": rear,
        "rear_leg_cmd_minus_actual_tail_mean_field_rad": FIELD_REAR,
        "pitch_final_deg": rows[-1]["pitch_deg"],
        "pitch_first_deg": rows[0]["pitch_deg"],
        "base_z_final_m": rows[-1]["base_z_m"],
        "base_z_min_m": min(r["base_z_m"] for r in rows),
        "fn_min_tail": {l: min(r["fn_N"][l] for r in tail) for l in LEGS},
        "rows": rows,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({"rear_tail_mean_sim": rear, "rear_field": FIELD_REAR,
                      "pitch_final": res["pitch_final_deg"], "base_z_min": res["base_z_min_m"],
                      "out": str(args.out)}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

