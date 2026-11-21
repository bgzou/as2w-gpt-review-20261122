# Stop-reward A/B

- control: `/root/gpufree-data/as2w-runs/v2-stop-ab-eval-control`
- reward:  `/root/gpufree-data/as2w-runs/v2-stop-ab-eval-reward`
- gait gate control: False / reward: False
- acceptable: **False**

| mode | drift c/r (m) | speed p95 c/r (m/s) | tilt p95 c/r (°) | 4-wheel c/r |
|---|---|---|---|---|
| stand | 1.3 / 1.0 | 0.009 / 0.011 | 0.94 / 1.53 | 0.999 / 0.996 |
| forward | 5.8 / 5.1 | 0.010 / 0.015 | 0.98 / 2.31 | 1.000 / 0.997 |
| reverse | 7.0 / 7.1 | 0.019 / 0.015 | 3.43 / 1.75 | 1.000 / 0.999 |
| left | 3.1 / 3.2 | 0.010 / 0.013 | 4.00 / 1.05 | 1.000 / 0.992 |
| right | 2.4 / 1.8 | 0.010 / 0.012 | 3.25 / 1.94 | 1.000 / 1.000 |
| yaw_left | 2.8 / 1.1 | 0.010 / 0.009 | 1.52 / 1.22 | 1.000 / 1.000 |
| yaw_right | 2.4 / 2.2 | 0.010 / 0.012 | 1.23 / 2.78 | 1.000 / 1.000 |
| mixed | 6.4 / 8.0 | 0.025 / 0.022 | 2.43 / 3.40 | 0.996 / 0.995 |

Reward-arm gait failures: forward: four-wheel rolling support below 90%; left: fewer than two events on a wheel in a trial; right: fewer than two events on a wheel in a trial; yaw_left: fewer than two events on a wheel in a trial; yaw_right: fewer than two events on a wheel in a trial
