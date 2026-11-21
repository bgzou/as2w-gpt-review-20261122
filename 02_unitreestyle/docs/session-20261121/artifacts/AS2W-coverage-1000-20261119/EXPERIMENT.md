# Four-wheel coverage correction, 1000 additional updates

{
  "parent": "/root/gpufree-data/as2w-runs/unitree-unified-reward-20261119-010208/train/model_1000.pt",
  "parent_sha256": "8e9f2e0fae9701853ca6b21c03b8e325c2a24733642f08d27969e08144e7dbce",
  "additional_updates": 1000,
  "num_envs": 4096,
  "rollout_steps": 48,
  "seed": 42,
  "terrains": {
    "plane": 0.6,
    "slope": 0.4
  },
  "domain_level": 0,
  "policy_dt_s": 0.02,
  "leg_filter_hz": 10,
  "optimizer": {
    "lr": 0.0001,
    "schedule": "fixed",
    "fixed_std": 0.4,
    "entropy": 0.0,
    "load_optimizer": false
  },
  "reward_changes": [
    "all four wheels eligible in all four pure stepping directions",
    "uniform event validity: 0.06-0.80 s, clearance 0.015-0.15 m",
    "no baseline cadence or phase reference",
    "per-wheel cap 2 rewarded events per 2 s window",
    "per-wheel coverage gap penalty ramps after 2 s, coefficient 0.04",
    "original tracking, posture, smoothing, loaded slip, touchdown speed and vx-only rolling support preserved"
  ],
  "checkpoint_policy": "best training return checked every update; immutable snapshots every 100 updates; endpoint always saved; no automatic qualified-best promotion",
  "limitations": [
    "not a strict reward-only comparison against original baseline: fixed exploration std, optimizer reset and DR0 differ",
    "executed action boundary fraction is not deterministic actor-mean saturation",
    "coverage windows and event durations are provisional engineering limits, not a measured target cadence"
  ],
  "status": "starting"
}

Acceptance must inspect deterministic joint-angle and per-wheel load/clearance traces for stand, forward, reverse, left/right and yaw left/right on flat and slopes. Do not use 1220 cadence or old gap calibration to pass a candidate. Training-return best is a candidate only. Baseline and HIM jobs are not stopped by this experiment.
