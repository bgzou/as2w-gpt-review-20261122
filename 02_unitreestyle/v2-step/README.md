# AS2-W v2 step-only line (58 obs / 12 action)

This folder is the training recipe and tooling behind the `as2w-v2-step-20261120`
weight: a Unitree-style flat-ground policy that steps while translating laterally
(`vy`) or turning (`wz`), with forward/backward rolling (`vx`) removed from the
task on purpose.

It is a separate line from the `common/` + `preflight/` Unitree-style baseline in
the repository root; the two share the 58-D observation / 12-D action contract
but differ in the action squash, the command distribution and the reward terms
listed below.

## What changed versus the 10 Hz clamp baseline

| Area | Baseline | This line |
|---|---|---|
| Action squash | `clamp(raw, -1, 1)` | **`tanh(raw)`** — clamp had zero gradient at the rail; 94.5% of action dims sat on the boundary |
| Warm start | train from scratch | rescale the actor head by 0.06 (`code/rescale_actor.py`) before continuing, so the tanh policy starts from the same behaviour |
| Commands | `vx`/`vy`/`wz` | **`vx` removed** (`--step-only`): 20% zero, 40% `vy ±0.45`, 40% `wz ±0.80`, held 2.5 s |
| Leg filter | 10 Hz | **5 Hz** (`configs/control.toml`), matching the deployment variant |
| Reward | tracking only | coverage window, per-wheel credit, rear-wheel coverage ×3, hardest-wheel weight, stop-reward terms; see `configs/reward_config*.json` |
| Domain randomization | none | optional DR1/DR2/DR3 (`code/domain.py`): friction, mass, COM, gains; sensor bias/noise; 1-tick delay; pushes; **motor jitter** (per-step target dither + per-joint gain jitter) |

## Training

```bash
# one continuation block on top of a parent checkpoint
PARENT=/path/to/parent.pt OUT=/path/to/run UPDATES=600 NUM_ENVS=4096 \
  bash scripts/run_tanh.sh --step-only \
  --reward-config configs/reward_config_hard.json
```

`run_tanh.sh` also honours `ASSET=` (USD path) and `DR=` level. The evaluation
sweeps are the `scripts/chain_*.sh` files: each waits for `model_200/400/...`,
runs `eval_out.sh` (fixed replay + gait gate + stop replay) and writes a
per-stage report.

## Results of the delivered weight

Fixed replay, Isaac Lab / PhysX, seed 319, six trials per direction, zero falls:

| Direction | Four-wheel support | Wheels with ≥2 lifts ≥15 mm | Tracking | Tilt P95 |
|---|---|---|---|---|
| left (vy +0.30) | 0.080 | **4/4** | 1.196 | 2.9° |
| yaw_left (wz +0.60) | 0.096 | **4/4** | 1.209 | 2.1° |
| yaw_right (wz −0.60) | 0.244 | **4/4** | 1.075 | 3.7° |
| right (vy −0.30) | 0.124 | 3/4 (RR never lifts) | 0.233 | 5.3° |

Stop replay: drift 3.1–14.5 cm, steady-window speed P95 0.034–0.070 m/s.

Weights in `model/`:

- `step_policy.pt` — training checkpoint, SHA256 `45b36ee789e1ac50692b8bedf16c4a7754095dfbae1fe9eca438682756a1e53c`
- `step_policy.onnx` — exported actor (16 kS), SHA256 `04c86e7c3c6068b9316e30ababe67610aeef28ccb4ddc84a9eb62ab97ed16a5d`

## Known limitation: right translation

The rear-right wheel never lifts when translating right. The support triangle
after lifting RR is FL+FR+RL, and the FR–RL edge passes through the COM
projection, so the static margin is zero. Fifteen training arms (coverage
penalties, hardest-wheel weighting, diagonal and lateral phase clocks, right-only
curriculum, half command amplitude) plus a simulated 15 mm-left COM shift all
failed to produce a single RR lift event. Treat the right direction as a
skating side-step and cap `|vy|` low in deployment.

## `vx` is out of scope here

The rolling policy lives in the repository baseline (`baseline/best.pt`); this
line deliberately never samples `vx` so the legs cannot learn to stay planted
while the wheels roll.

## File map

| Path | Contents |
|---|---|
| `code/` | environment, task/rewards, DR, training loop, evaluation, gate, reporting |
| `configs/` | control contract (`control.toml`), calibration, DR config, reward configs |
| `scripts/` | run/eval/chain shell drivers used on the training server |
| `tools/` | stage report and stop-metric reporters |
| `results/` | gate, stop metrics and MuJoCo replay of the delivered weight |
| `model/` | delivered PyTorch and ONNX weights |

Paths inside the shell scripts are the training server's (`/root/as2w/...`);
they document the exact runs and are not required to be valid elsewhere.
