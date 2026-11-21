# 视频说明

## `isaac-consolidate-18200.mp4` — Isaac Lab 回放，已验证

model_18200 的 8 工况回放（Stand / Forward / Reverse / 4 组弧线 / Fast left），前视 + 侧视双机位，**8 段全部无 fall**，与 48/48 存活一致。约 48 秒、6.1 MB。

## `mujoco-validated-20261121.mp4` — MuJoCo 第二仿真器，2026-11-21 修复后

站立 / 前进 0.30 / 弧线 (0.30, wz 0.30) / 后退 −0.30，各 5–7 s，640×480、25 fps、约 0.6 MB。

修复内容与实测数据见 [`../analysis/mujoco-sim2sim.md`](../analysis/mujoco-sim2sim.md)。要点：

- 站立高度 0.333–0.384 m，全程无翻倒
- 前进/后退速率跟踪 92% / 85%
- **偏航跟踪偏弱（7 s 只转 24°，理论约 120°）**，转向不作为能力证据

## `mujoco-draft-unvalidated.mp4` — 历史草稿，已废弃

2026-11-20 首次尝试的产物，三个缺陷未修复（显式轮速力矩环、ctrl 索引用错、绝对角重复叠加），机器人翻滚。**保留仅作对照，不要引用。**
