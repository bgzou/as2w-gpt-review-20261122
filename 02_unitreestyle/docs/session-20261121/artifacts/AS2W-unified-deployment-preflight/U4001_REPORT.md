# 基础 model_4000 验收

记录时间：2026-11-19T07:26:38.002093+08:00。
checkpoint 索引 4000，已完成更新数 4001。48/48 试次存活，完整步态验收未通过。

前进四轮支撑率 91.47%，倒车 66.07%；第 3000 索引权重对应为 95.20% 与 88.80%。左移 FL/FR/RL/RR 累计抬轮次数为 32/32/45/5，左转为 18/0/0/27。存活和速度跟踪不能替代四轮踏步及滚动承重验收。

失败项：
- forward: loss of four-wheel stance
- reverse: loss of four-wheel stance
- left: reduced active joint excursion
- right: increased measured phase residual
- right: reduced active joint excursion
- yaw_left: cadence
- yaw_left: increased measured phase residual
- yaw_left: reduced active joint excursion
- yaw_right: cadence
- yaw_right: increased measured phase residual
- yaw_right: reduced active joint excursion
- mixed: tracking axis 1
- mixed: FL lifts
- mixed: RR lifts
- mixed: wheel duty
- reverse: wheel duty

基础训练继续至 10000 轮。此权重不登记为合格最佳，不替换域随机化交付包中的研究候选。数据协议为原 intent-audit，与 DR 回放协议不同。
