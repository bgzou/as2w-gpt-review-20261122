# Stage report: v2-tanh-eval-endpoint

- checkpoint sha256: `c404f20e56400344c94db6fea8624df2d660b6b46c916c67a3981b6acc7dc0f5`
- asset sha256: `ebb087194b2b90d27878532f0f0e3f5824e44932fdafc4c87f8f1c2bcc5b0688`
- resume sha256: `None`
- gate: FAIL ['left: fewer than two events on a wheel in a trial', 'right: fewer than two events on a wheel in a trial', 'yaw_left: fewer than two events on a wheel in a trial', 'yaw_right: fewer than two events on a wheel in a trial']

| mode | falls | 4-wheel support | lift events/wheel (min) | tilt p95 | tracking |
|---|---|---|---|---|---|
| stand | 0 | 1.000 | [0, 0, 0, 0] | 0.57° | n/a |
| forward | 0 | 0.984 | [0, 0, 0, 0] | 2.51° | 0.951 |
| reverse | 0 | 1.000 | [0, 0, 0, 0] | 1.87° | 0.945 |
| left | 0 | 0.204 | [0, 0, 8, 0] | 5.07° | 0.878 |
| right | 0 | 0.628 | [0, 0, 0, 0] | 2.40° | 0.789 |
| yaw_left | 0 | 0.724 | [0, 0, 0, 8] | 3.12° | 0.834 |
| yaw_right | 0 | 0.636 | [0, 0, 8, 0] | 5.68° | 0.931 |
| mixed | 0 | 0.476 | [0, 0, 1, 0] | 5.62° | 0.854 |

| joint | q p2p (rad) | mean \|tau\| (Nm) | p95 \|tau\| (Nm) | max \|tau\| (Nm) |
|---|---|---|---|---|
| FL_hip | 0.332 | 9.29 | 16.62 | 20.88 |
| FL_thigh | 0.630 | 5.35 | 13.10 | 35.67 |
| FL_calf | 0.394 | 8.50 | 15.53 | 19.36 |
| FR_hip | 0.350 | 9.10 | 14.69 | 17.71 |
| FR_thigh | 0.582 | 4.88 | 15.93 | 31.62 |
| FR_calf | 0.376 | 7.45 | 13.73 | 18.12 |
| RL_hip | 0.359 | 7.45 | 12.40 | 19.96 |
| RL_thigh | 0.256 | 2.90 | 5.78 | 9.54 |
| RL_calf | 0.290 | 7.46 | 16.93 | 21.85 |
| RR_hip | 0.356 | 7.41 | 14.24 | 18.65 |
| RR_thigh | 0.263 | 2.87 | 5.39 | 8.78 |
| RR_calf | 0.313 | 7.83 | 16.57 | 19.35 |
| FL_foot | 12.550 | 0.92 | 2.82 | 15.00 |
| FR_foot | 12.530 | 0.94 | 2.68 | 15.00 |
| RL_foot | 12.548 | 0.59 | 2.12 | 15.00 |
| RR_foot | 12.534 | 0.62 | 2.22 | 15.00 |
