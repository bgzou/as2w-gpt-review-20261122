"""Targets for the optional raised posture, calibrated against the 1220 flat replay."""
from pathlib import Path

import numpy as np

CASES = ("stand", "forward", "reverse", "left", "right", "yaw_left", "yaw_right", "mixed")
BODY_RISE = .010
SWING_RISE = .010
PROFILE = "raised-v1"


def set_posture(cfg):
    # Both the actuator reference and policy joint-offset observations use this pose.
    cfg.scene.robot.init_state.joint_pos = dict(cfg.scene.robot.init_state.joint_pos)
    cfg.scene.robot.init_state.joint_pos.update({".*_thigh": .755, ".*_calf": -1.51})


def references(path):
    """Median actual base height and phase-aligned swing wheel-link height on flat ground."""
    with np.load(Path(path) / "flat.npz") as a:
        valid = ~np.maximum.accumulate(a["fall"], axis=0)
        valid[:50] = False
        labels = a["labels"]
        wheel = a["wheel_data"]
        body = [np.median(a["pose"][..., 2][valid & (labels == case)]) for case in CASES]
        zref = np.zeros((4, 128, 4), np.float32)
        swing = np.zeros_like(zref)
        grid = np.arange(128) / 128
        for case, (label, anchor) in enumerate(zip(CASES[3:7], (1, 1, 3, 2))):
            heights, air = [], []
            for j in np.flatnonzero(labels == label):
                supported = wheel[:, j, :, 15] >= 3.
                hits = [k for k in range(50, len(wheel)-2) if valid[k+2, j]
                        and not supported[k-3:k, anchor].any() and supported[k:k+3, anchor].all()]
                for start, end in zip(hits, hits[1:]):
                    if not 18 <= end-start <= 55:
                        continue
                    phase = np.linspace(0, 1, end-start+1)
                    heights.append([np.interp(grid, phase, w) for w in wheel[start:end+1, j, :, 2].T])
                    air.append([np.interp(grid, phase, ~w) for w in supported[start:end+1, :].T])
            if not heights:
                raise ValueError(f"No complete anchor cycles for {label}")
            zref[case] = np.median(heights, axis=0).T
            probability = np.mean(air, axis=0).T
            x = np.clip((probability-.5)/.35, 0, 1)
            swing[case] = x**3 * (10 - 15*x + 6*x*x)
    return np.asarray(body, np.float32), zref, swing


def penalty(env, code, phase, phase_ok, plane):
    import torch
    from environment import tensor
    vx, vy, wz = env.command.unbind(1)
    x, y, w = vx.abs()>1e-6, vy.abs()>1e-6, wz.abs()>1e-6
    mode = torch.zeros_like(code)
    mode = torch.where(x, 1+(vx<0).long(), mode)
    mode = torch.where(y, 3+(vy<0).long(), mode)
    mode = torch.where(w, 5+(wz<0).long(), mode)
    mode = torch.where(x.long()+y.long()+w.long()>1, 7, mode)
    ground = env.terrain.env_origins[:, 2]
    body = tensor(env.robot.data.root_pos_w)[:, 2]-ground
    body_cost = ((body-env.body_ref[mode]-BODY_RISE)/.02).square().clamp_max(4.) * plane
    case = code.clamp_min(0)
    index = phase.long()
    nxt = (index+1) % 128
    frac = (phase-index)[:, None]
    weight = env.swing_ref[case,index].lerp(env.swing_ref[case,nxt],frac)
    target = env.wheel_ref[case,index].lerp(env.wheel_ref[case,nxt],frac) + SWING_RISE*weight
    wheel = tensor(env.robot.data.body_link_pos_w)[:,env.wheel_body_ids,2]-ground[:,None]
    cost = (((wheel-target)/.015).square().clamp_max(4.)*weight).mean(1)
    return .15*body_cost + .20*cost*phase_ok
