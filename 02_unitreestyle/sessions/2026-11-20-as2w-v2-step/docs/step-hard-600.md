# Stage report: v2-stephard-eval-600

- checkpoint sha256: `45b36ee789e1ac50692b8bedf16c4a7754095dfbae1fe9eca438682756a1e53c`
- asset sha256: `ebb087194b2b90d27878532f0f0e3f5824e44932fdafc4c87f8f1c2bcc5b0688`
- resume sha256: `aa41bbe8b31fcd65a37a7540bc47e937ea37f7ef48163adb945e4cb80c069e69`
- gate: FAIL ['forward: four-wheel rolling support below 90%', 'reverse: four-wheel rolling support below 90%', 'right: survival or tracking', 'right: fewer than two events on a wheel in a trial']

| mode | falls | 4-wheel support | lift events/wheel (min) | tilt p95 | tracking |
|---|---|---|---|---|---|
| stand | 0 | 1.000 | [0, 0, 0, 0] | 3.17° | n/a |
| forward | 0 | 0.760 | [0, 0, 0, 0] | 5.03° | 0.945 |
| reverse | 0 | 0.864 | [0, 0, 0, 0] | 2.23° | 0.918 |
| left | 0 | 0.080 | [13, 14, 14, 13] | 2.86° | 1.196 |
| right | 0 | 0.124 | [19, 17, 19, 0] | 5.33° | 0.233 |
| yaw_left | 0 | 0.096 | [15, 15, 15, 14] | 2.13° | 1.209 |
| yaw_right | 0 | 0.244 | [12, 13, 13, 11] | 3.73° | 1.075 |
| mixed | 0 | 0.024 | [11, 13, 14, 0] | 2.67° | 0.868 |

| joint | q p2p (rad) | mean \|tau\| (Nm) | p95 \|tau\| (Nm) | max \|tau\| (Nm) |
|---|---|---|---|---|
| FL_hip | 0.408 | 5.75 | 11.48 | 19.86 |
| FL_thigh | 0.311 | 5.21 | 14.91 | 22.70 |
| FL_calf | 0.421 | 6.50 | 16.34 | 24.05 |
| FR_hip | 0.384 | 5.34 | 10.28 | 15.90 |
| FR_thigh | 0.337 | 5.55 | 14.84 | 19.15 |
| FR_calf | 0.385 | 9.64 | 21.78 | 30.74 |
| RL_hip | 0.352 | 6.42 | 12.83 | 17.51 |
| RL_thigh | 0.338 | 2.71 | 6.70 | 13.74 |
| RL_calf | 0.304 | 8.95 | 18.71 | 28.13 |
| RR_hip | 0.335 | 4.54 | 10.41 | 15.82 |
| RR_thigh | 0.265 | 3.03 | 6.86 | 13.52 |
| RR_calf | 0.305 | 7.74 | 17.57 | 23.63 |
| FL_foot | 12.554 | 0.78 | 2.75 | 15.00 |
| FR_foot | 12.554 | 0.91 | 3.56 | 15.00 |
| RL_foot | 12.543 | 0.70 | 2.56 | 15.00 |
| RR_foot | 12.559 | 0.59 | 2.20 | 14.21 |
