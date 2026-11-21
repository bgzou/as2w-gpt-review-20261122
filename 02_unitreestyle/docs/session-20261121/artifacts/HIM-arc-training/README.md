# HIM arc training
Commands use vy=0. Tasks are stand, forward/reverse rolling and left/right arcs.
Flat stage uses speed 0.15–0.35 m/s, expanding to 0.7 m/s over 2000 updates.
Curvature expands from 0.3–0.8 to 0.3–1.5 inverse metres. yaw=speed*curvature.

HIM estimator and PPO architecture are retained, with fresh parameters and a zero actor output head.
PPO samples Gaussian latent actions, stores those latent actions in rollout storage, and computes
likelihood ratios in latent space. Environment tanh maps legs to +/-0.15 rad and wheel corrections
to +/-2 rad/s around (vx-wz*y)/0.086. Fixed std=0.3, entropy coefficient=0.
This avoids needing a tanh inverse or Jacobian in the latent-variable PPO objective.

Rewards retain the common velocity, posture, target differences, wheel speed and termination terms.
Additional per-second costs in arc_environment.py include calibrated joint posture, tilt dead zone,
wheel load deficit, height error, contact-point lateral slip, actual joint speed and latent excess.
Static reference was measured for 5 seconds at zero residual on flat ground. See static_reference.json.
Stance is weighted smoothly by exp(-vx^2/0.01-wz^2/0.01).
Phase forcing and pure lateral tasks are removed.

Training uses 4096 environments x48 steps, 2000-update blocks up to 40000.
Each block evaluates both return-best and endpoint. Flat validation includes joint angles,
wheel forces, sustained unloading, tilt and vx/wz tracking. Pure vy is not a task.
Only passing candidates enter validated-best.json. A pass promotes flat to randomized,
then randomized to rough training. Endpoint training state is continued between blocks.
The initial flat 128-env two-update smoke completed. All 6 stand trials passed.
36/48 motion trials failed, primarily forward and arc pitch and tracking.
This is a working training setup, with motion quality still to be learned.

Run from /root/as2w:
bash him_arc/run_stages.sh NEW_OUTPUT_DIRECTORY
Outputs and active path are recorded in him_arc/latest.txt.
