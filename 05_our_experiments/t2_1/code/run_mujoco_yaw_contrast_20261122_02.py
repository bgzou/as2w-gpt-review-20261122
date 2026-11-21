#!/usr/bin/env python3
"""R2000A T2-1: MuJoCo yaw contrast, full composite vs base-only (vs residual-only).

Runs the DELIVERED runtime contract inside MuJoCo at the training cadence
(dt 0.005 x 4 substeps = 0.020 s policy step) for identical initial state and identical
commands, but swaps the action composition:

    full           : clip( clip(base_raw) + gate*0.24*tanh(res) )   (delivered behaviour)
    base_only      : clip(base_raw)                                (no residual, no gate)
    residual_only  : clip(gate*0.24*tanh(res))                     (diagnostic, optional)

Everything else (obs build, 10/5 Hz low-pass, wheel analytic target + deadband, kp/kd,
settle, reset) comes from the delivered `runtime/contract.py`. Only the composition differs,
so a difference in fall time is attributable to the residual/gate path.

The official package is never written to: the scene XML, contract.py and weights.npz are
COPIED into --work-dir (new), and the copy's XML points its meshdir at the read-only package
meshes dir. The script refuses to use an existing --work-dir or an existing output file.

Recorded per policy step (0.02 s): roll/pitch/yaw [deg], base height, body velocity,
wheel joint velocities, per-wheel normal contact force, and the action internals (pre-clip
max |value|, per-frame clip hits, gate).

Exit 0 when every requested variant/scenario completed and produced finite series.
"""
import argparse
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True

SCENARIOS = {
    "STAND": (0.0, 0.0, 0.0),
    "YAWL": (0.0, 0.0, 0.60),
    "YAWR": (0.0, 0.0, -0.60),
    "MIX": (0.45, 0.20, 0.45),
}
LEGS = ["FL", "FR", "RL", "RR"]
JOINT_ORDER = [f"{leg}_{p}" for leg in LEGS for p in ("hip", "thigh", "calf")] + [f"{leg}_foot" for leg in LEGS]
RESET_HEIGHT = 0.390
KP_LEG, KD_LEG, KD_WHEEL = 55.0, 2.0, 2.0
SETTLE_STEPS = 50
FALL_TILT_DEG = 60.0
FALL_Z_M = 0.15


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def prepare_workdir(pkg, work):
    if work.exists():
        print(f"REFUSING: work dir exists: {work}", file=sys.stderr)
        sys.exit(2)
    (work / "runtime").mkdir(parents=True)
    (work / "model").mkdir(parents=True)
    shutil.copy2(pkg / "runtime" / "contract.py", work / "runtime" / "contract.py")
    shutil.copy2(pkg / "model" / "weights.npz", work / "model" / "weights.npz")
    xml = (pkg / "simulation" / "mujoco" / "as2w_scene.xml").read_text(encoding="utf-8", errors="replace")
    meshdir = str((pkg / "simulation" / "mujoco" / "meshes").resolve())
    patched = xml.replace('meshdir="meshes"', f'meshdir="{meshdir}"')
    if patched == xml:
        print("WARNING: meshdir not patched (pattern not found)", file=sys.stderr)
    (work / "as2w_scene.xml").write_text(patched, encoding="utf-8")
    return {
        "contract": sha256_file(work / "runtime" / "contract.py"),
        "weights": sha256_file(work / "model" / "weights.npz"),
        "mjcf_patched": sha256_file(work / "as2w_scene.xml"),
        "meshdir": meshdir,
    }


class Instrumented:
    """Wrap a composition mode around the contract's own MLP weights."""

    def __init__(self, contract, mode):
        self.c = contract
        self.mode = mode
        self.policy = contract.NumpyPolicy()
        self.last = {}

    @staticmethod
    def _mlp(x, layers):
        for i, (W, b) in enumerate(layers):
            x = x @ W.T + b
            if i < len(layers) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return x

    def raw_actions(self, obs58):
        x = obs58[None, :]
        base_raw = self._mlp(x, self.policy.base)[0]
        base_clip = np.clip(base_raw, -1.0, 1.0)
        res = np.tanh(self._mlp(x, self.policy.residual)[0])
        gate = min(1.0, max(abs(float(obs58[1])) / self.c.RESIDUAL_GATE_VY,
                            abs(float(obs58[2])) / self.c.RESIDUAL_GATE_WZ))
        if self.mode == "full":
            pre = base_clip + gate * self.c.LIMIT * res
        elif self.mode == "base_only":
            pre = base_clip.copy()
        elif self.mode == "residual_only":
            pre = gate * self.c.LIMIT * res
        else:
            raise ValueError(self.mode)
        post = np.clip(pre, -1.0, 1.0)
        self.last = {
            "base_raw_max_abs": float(np.abs(base_raw).max()),
            "base_clip_hits": int(np.sum(np.abs(base_raw) > 1.0)),
            "gate": float(gate),
            "composite_pre_max_abs": float(np.abs(pre).max()),
            "composite_pre_hits": int(np.sum(np.abs(pre) > 1.0)),
        }
        return post


def make_model(mujoco, xml_path, base_mass=None, base_com_x=None, leg_kp=None, leg_kd=None):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    m.opt.timestep = 0.005
    assert abs(4 * m.opt.timestep - 0.020) < 1e-12, "policy cadence mismatch"
    for name in JOINT_ORDER[:12]:
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        kp = KP_LEG if leg_kp is None else leg_kp
        kd = KD_LEG if leg_kd is None else leg_kd
        m.actuator_gainprm[a, 0] = kp
        m.actuator_biasprm[a, 1] = -kp
        m.actuator_biasprm[a, 2] = -kd
    for name in JOINT_ORDER[12:]:
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        m.actuator_gainprm[a, 0] = KD_WHEEL
        m.actuator_biasprm[a, 1] = 0.0
        m.actuator_biasprm[a, 2] = -KD_WHEEL
    overrides = {}
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_mass is not None:
        overrides["base_mass_before"] = float(m.body_mass[bid])
        m.body_mass[bid] = float(base_mass)
        overrides["base_mass_after"] = float(m.body_mass[bid])
    if base_com_x is not None:
        overrides["base_com_x_before"] = float(m.body_ipos[bid][0])
        m.body_ipos[bid][0] = float(base_com_x)
        overrides["base_com_x_after"] = float(m.body_ipos[bid][0])
    if leg_kp is not None or leg_kd is not None:
        overrides["leg_gain_after"] = {"kp": leg_kp, "kd": leg_kd}
    return m, overrides


def run_one(mujoco, m, contract, mode, cmd3, steps):
    d = mujoco.MjData(m)
    qa = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_ORDER}
    da = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_ORDER}
    aid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_act") for n in JOINT_ORDER}
    foot_geoms = {}
    for leg in LEGS:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_foot")
        adr, num = m.body_geomadr[b], m.body_geomnum[b]
        foot_geoms[leg] = set(range(adr, adr + num))

    mujoco.mj_resetData(m, d)
    d.qpos[0:3] = [0.0, 0.0, RESET_HEIGHT]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i, n in enumerate(JOINT_ORDER):
        d.qpos[qa[n]] = contract.DEFAULT_JOINT_POS[i]
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)

    rt = contract.Runtime()
    rt.policy = Instrumented(contract, mode)
    leg_q0 = np.array([d.qpos[qa[n]] for n in JOINT_ORDER[:12]])
    rt.reset(leg_q0)
    zero = (0.0, 0.0, 0.0)

    def frame(command):
        q16 = np.array([d.qpos[qa[n]] for n in JOINT_ORDER])
        dq16 = np.array([d.qvel[da[n]] for n in JOINT_ORDER])
        grav_b, ang_b = contract.sensor_frame_to_gravity_angvel(d.qpos[3:7], d.qvel[3:6])
        leg_pos, wheel_vel = rt.step(command, q16, dq16, ang_b, grav_b)
        internals = dict(rt.policy.last)
        for i, n in enumerate(JOINT_ORDER[:12]):
            d.ctrl[aid[n]] = leg_pos[i]
        for i, n in enumerate(JOINT_ORDER[12:]):
            d.ctrl[aid[n]] = wheel_vel[i]
        for _ in range(4):
            mujoco.mj_step(m, d)
        return internals

    for _ in range(SETTLE_STEPS):
        frame(zero)

    cmd = tuple(map(float, cmd3))
    series, fell_at = [], None
    for i in range(steps):
        internals = frame(cmd)
        w, x, y, z = d.qpos[3:7]
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
        R = contract.quat_to_rot_wxyz(d.qpos[3:7])
        vb = R.T @ d.qvel[0:3]
        wb = R.T @ d.qvel[3:6]
        fn = {leg: 0.0 for leg in LEGS}
        for ci in range(d.ncon):
            con = d.contact[ci]
            f6 = np.zeros(6)
            mujoco.mj_contactForce(m, d, ci, f6)
            for leg, geoms in foot_geoms.items():
                if con.geom1 in geoms or con.geom2 in geoms:
                    fn[leg] += float(abs(f6[0]))
        series.append({
            "t_s": round((i + 1) * 0.020, 3),
            "roll_deg": round(roll, 4), "pitch_deg": round(pitch, 4), "yaw_deg": round(yaw, 4),
            "tilt_deg": round(tilt, 4), "base_z_m": round(float(d.qpos[2]), 5),
            "vx_body": round(float(vb[0]), 5), "vy_body": round(float(vb[1]), 5),
            "wz_body": round(float(wb[2]), 5),
            "wheel_dq": [round(float(d.qvel[da[n]]), 4) for n in JOINT_ORDER[12:]],
            "fn_N": {k: round(v, 2) for k, v in fn.items()},
            "act": {k: (round(v, 5) if isinstance(v, float) else v) for k, v in internals.items()},
        })
        if fell_at is None and (tilt > FALL_TILT_DEG or d.qpos[2] < FALL_Z_M):
            fell_at = round((i + 1) * 0.020, 3)
            break

    arr = lambda k: np.array([s[k] for s in series], dtype=float)  # noqa: E731
    return {
        "mode": mode, "command": list(cmd), "steps_done": len(series),
        "fell": fell_at is not None, "fell_at_s": fell_at,
        "tilt_max_deg": float(arr("tilt_deg").max()),
        "pitch_final_deg": float(arr("pitch_deg")[-1]),
        "roll_final_deg": float(arr("roll_deg")[-1]),
        "yaw_final_deg": float(arr("yaw_deg")[-1]),
        "yaw_rate_mean_radps": float(arr("wz_body").mean()),
        "base_z_min_m": float(arr("base_z_m").min()),
        "fn_max_N": {leg: float(max((s["fn_N"][leg] for s in series), default=0.0)) for leg in LEGS},
        "base_raw_frame_any_clip_frac": float(np.mean([s["act"]["base_clip_hits"] > 0 for s in series])),
        "composite_pre_frame_any_clip_frac": float(np.mean([s["act"]["composite_pre_hits"] > 0 for s in series])),
        "gate_mean": float(np.mean([s["act"]["gate"] for s in series])),
        "base_raw_max_abs": float(max(s["act"]["base_raw_max_abs"] for s in series)),
        "series": series,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--package-root", type=Path, required=True)
    ap.add_argument("--work-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--variants", default="full,base_only")
    ap.add_argument("--scenarios", default="STAND,YAWL,YAWR,MIX")
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--base-mass", type=float, default=None)
    ap.add_argument("--base-com-x", type=float, default=None)
    ap.add_argument("--leg-kp", type=float, default=None)
    ap.add_argument("--leg-kd", type=float, default=None)
    ap.add_argument("--extra-scenarios", default="",
                    help="semicolon list name:vx:vy:wz added to SCENARIOS")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    pkg = args.package_root.resolve()
    if args.output.exists():
        print(f"REFUSING: output exists: {args.output}", file=sys.stderr)
        return 2
    import mujoco
    shas = prepare_workdir(pkg, args.work_dir)
    sys.path.insert(0, str(args.work_dir / "runtime"))
    import contract  # from the copied work dir
    m, override_info = make_model(mujoco, args.work_dir / "as2w_scene.xml",
                                  base_mass=args.base_mass, base_com_x=args.base_com_x,
                                  leg_kp=args.leg_kp, leg_kd=args.leg_kd)

    variants = [v.strip() for v in args.variants.split(",") if v.strip()]
    scen = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    if args.extra_scenarios:
        for spec in args.extra_scenarios.split(";"):
            spec = spec.strip()
            if not spec:
                continue
            name, vx, vy, wz = spec.split(":")
            SCENARIOS[name] = (float(vx), float(vy), float(wz))
            if name not in scen:
                scen.append(name)
    results = {}
    for v in variants:
        results[v] = {}
        for s in scen:
            r = run_one(mujoco, m, contract, v, SCENARIOS[s], args.steps)
            results[v][s] = r
            status = f"FELL@{r['fell_at_s']:.2f}s" if r["fell"] else "survived"
            print(f"{v:14s} {s:6s} {status:14s} tilt_max {r['tilt_max_deg']:6.2f} "
                  f"z_min {r['base_z_min_m']:.3f} yaw {r['yaw_final_deg']:+8.2f} "
                  f"wz_mean {r['yaw_rate_mean_radps']:+.3f}", flush=True)

    out = {
        "task": "T2-1 R2000A MuJoCo yaw contrast: full vs base-only",
        "generator": Path(__file__).name,
        "generator_sha256": sha256_file(Path(__file__)),
        "mujoco_version": mujoco.__version__,
        "package_root": str(pkg),
        "work_dir": str(args.work_dir.resolve()),
        "copied_inputs_sha256": shas,
        "official_asset_sha256": {
            "mjcf": sha256_file(pkg / "simulation" / "mujoco" / "as2w_scene.xml"),
            "urdf": sha256_file(pkg / "simulation" / "urdf" / "as2w-load-v0.urdf"),
        },
        "cadence": {"physics_dt_s": 0.005, "decimation": 4, "policy_dt_s": 0.020,
                    "settle_steps_zero_cmd": SETTLE_STEPS, "steps": args.steps},
        "fall_gate": {"tilt_deg_gt": FALL_TILT_DEG, "base_z_m_lt": FALL_Z_M},
        "note": ("Composition variants share the delivered obs/low-pass/wheel/PD code; only "
                 "the action composition differs. MuJoCo scene base mass/COM differ from the "
                 "training URDF (see r2000a_urdf_mjcf_asset_ledger_20261122_01.json)"),
        "variants": variants,
        "scenarios": scen,
        "overrides": override_info,
        "tag": args.tag,
        "results": results,
    }
    issues = []
    for v in variants:
        for s in scen:
            r = results[v][s]
            if not r["series"]:
                issues.append(f"{v}/{s}: empty series")
    out["consistency_issues"] = issues
    out["verdict"] = "COMPLETE" if not issues else "INCOMPLETE"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({"verdict": out["verdict"], "issues": issues}))
    return 0 if not issues else 1


if __name__ == "__main__":
    sys.exit(main())


