# 基础训练第 2001 次更新的固定平地回放

评估对象为 model_2000.pt。使用与第 1001 次更新相同的固定平地回放入口，仍与冻结的第 1220 轮参考作步态保留比较。

| 命令 | 存活 | 第 1001 次更新四轮支撑 | 第 2001 次更新四轮支撑 | 当前 FL/FR/RL/RR 抬轮事件 |
| --- | ---: | ---: | ---: | --- |
| stand | 6/6 | 100.00% | 99.87% | [0, 0, 0, 0] |
| forward | 6/6 | 97.47% | 97.13% | [0, 0, 0, 0] |
| reverse | 6/6 | 84.00% | 71.73% | [0, 0, 0, 0] |
| left | 6/6 | 4.20% | 11.20% | [42, 22, 43, 0] |
| right | 6/6 | 12.87% | 26.27% | [31, 18, 0, 26] |
| yaw_left | 6/6 | 56.80% | 57.13% | [19, 0, 0, 49] |
| yaw_right | 6/6 | 53.73% | 57.27% | [0, 8, 24, 0] |
| mixed | 6/6 | 26.93% | 23.67% | [11, 20, 39, 7] |

48/48 试次存活，完整步态验收未通过。倒车四轮支撑占比从 84.00% 降至 71.73%，还出现侧移抬轮、节律、关节活动幅度，以及转向相位残差失败项。当前结果不足以登记为通过验收的最佳权重。

失败项如下。

- reverse: loss of four-wheel stance
- left: FR lifts
- left: cadence
- left: reduced active joint excursion
- right: increased measured phase residual
- right: reduced active joint excursion
- yaw_left: cadence
- yaw_left: increased measured phase residual
- yaw_left: reduced active joint excursion
- yaw_right: increased measured phase residual
- yaw_right: reduced active joint excursion
- mixed: FL lifts
- mixed: FR lifts
- reverse: wheel duty

基础训练继续至 10000 次更新的计划正在执行，域随机化第二阶段同时推进。此处评估的是基础训练分支，域随机化候选继续使用其平地与斜坡、名义与压力回放的统一筛选协议。
