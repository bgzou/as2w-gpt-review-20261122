# 第 1001 次 PPO 更新的固定平地回放

基础训练 model_1000.pt 已完成固定回放。沿用旧固定平地回放协议与第 1220 轮参考对照。各命令 6 个试次，共 48 个，全部存活，完整步态验收未通过。

| 命令 | 四轮支撑占比 | FL/FR/RL/RR 抬轮事件 |
| --- | ---: | --- |
| stand | 100.00% | [0, 0, 0, 0] |
| forward | 97.47% | [0, 0, 0, 0] |
| reverse | 84.00% | [0, 0, 1, 0] |
| left | 4.20% | [34, 27, 37, 6] |
| right | 12.87% | [18, 32, 2, 26] |
| yaw_left | 56.80% | [35, 0, 0, 40] |
| yaw_right | 53.73% | [0, 19, 23, 0] |
| mixed | 26.93% | [3, 7, 36, 8] |

失败项如下。

- reverse: loss of four-wheel stance
- left: reduced active joint excursion
- right: tracking axis 1
- right: cadence
- right: reduced active joint excursion
- yaw_left: increased measured phase residual
- yaw_right: increased measured phase residual
- yaw_right: reduced active joint excursion
- mixed: tracking axis 0
- mixed: FL lifts
- mixed: FR lifts

本回放与第 601 次更新使用的域随机化评估器为两个评估入口。直接比较时需先核对命令值、出生点和参考口径。此处结论来自本协议的固定参考比较。

yaw_left 的实测关节相位残差 RMS 从 0.550° 变为 0.694°。

yaw_right 的实测关节相位残差 RMS 从 0.579° 变为 0.738°。

基础任务继续训练到 10000 次更新。跨过第 1200 次更新后的分支按平地与斜坡统一回放协议比较相邻候选。
