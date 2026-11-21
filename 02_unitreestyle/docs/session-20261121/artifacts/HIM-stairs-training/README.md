# AS2W HIM single-step training
Start checkpoint: HIM arc accepted update 5836.
First course: single 5 cm step, leading edge x=0.7 m, platform ends x=2.7 m.
Command: vx=0.25 m/s, vy=wz=0, followed by stop after the four wheels clear the edge.
4096 environments, 48 rollout steps, 2000 additional updates. New PPO optimizer,
retained actor/critic/estimator parameters, fixed latent std 0.3.
Initial leg action map matches the accepted policy. Thigh/calf range increases
from +/-0.15 to +/-0.25 rad over 1000 updates; hip range stays +/-0.15 rad.
The action remains tanh-bounded, with latent likelihood evaluated by PPO.
Reward adds per-wheel first supported landing (0.5 total for four wheels),
stall penalty, local ground-relative body height, relaxed crossing posture,
non-wheel collisions and insufficient support. Terrain geometry defines landing.
Completion requires all four verified landings, wheels on the platform, >=3 supported,
stable gravity/angular velocity and 0.5 seconds continuous dwell.
Only small-scale runtime validation is complete. Smoke success is in the run directory.
The current task trains ascent of one low step. Multi-step, descent, randomized
terrain and flat-regression acceptance must be added before claiming the full W1W_2 course.
Launch from /root/as2w with bash him_stairs/launch.sh RUN_DIRECTORY.
RUN_DIRECTORY must contain seed.txt pointing to the accepted checkpoint.
The final endpoint and return-best are both evaluated, with separate success records.
