# 训练合同逐项审计（him_arc ↔ 部署）

对比对象：训练侧 `him_arc/{quality_environment,arc_environment,dr_environment}.py` + `common/environment.py`；部署侧 `data/himloco_controller.cpp` + `data/deploy.yaml`。

| 合同项 | 训练 | 部署 | 结论 |
|---|---|---|---|
| 观测 60 维布局 | `quality_environment._frame` | 同序拼接 | ✅ |
| 历史 6 帧、新帧在前、首帧填满整栈 | `_get_observations` | `history_valid_` 分支 | ✅（Python 参考曾错，已修） |
| 相位时钟 | `phase += dt*gait_frequency(cmd)`，sin/cos·strength | `phase_clock: true` | ✅ |
| 重力投影 | `projected_gravity_b` | 四元数同式 | ✅ |
| 轮位置置零 | `q[:,12:]=0` | obs[21..24]=0 | ✅ |
| 动作映射 | `tanh → 0.6 腿增益 → 0.25`；轮 `(vx−wz·side)/R+2·tanh` | 同 | ✅（早期文档写错） |
| 腿/轮 lerp | α_leg 0.7154 / α_wheel 0.4665 | 同（`-expm1(-2πf·dt)`） | ✅ |
| **100 步零命令交接 + applied 线性斜坡** | `dr_environment._pre_physics_step`（`w=(k+1)/100`，command 强制 0） | **本次新增 `handoff_steps: 100`**，obs 的 applied 用斜坡值 | ✅ 此前缺失 |
| PD | 腿 55/2、轮 0/2 | deploy.yaml | ✅ |
| 默认关节角 | `[0,0.8,−1.6]×4` | deploy.yaml | ✅ |
| HEIGHT | 0.37 | `obs_height` | ✅ |
| 陀螺/关节偏置、观测噪声 | `randomize=False` 时为 0 | 无（等价） | ✅ |
| 质量/质心/摩擦随机化 | ±0.5 kg、±10 mm、0.25×(0.8–1.2) | 名义固定 | ✅ 分布内中心 |
| 0–1 步执行延迟 | consolidate 见过 delay [0,1] | 部署 0 延迟 | ⚠️ 只覆盖一档 |
| 相位初值 | 每回合随机 | 固定 0 | ✅ 分布内 |
| ONNX 契约 | — | `obs[N,360]→out[N,16]` float32，输入名 `obs` | ✅ 控制器启动校验 |
| `ckpt["iter"]` | 停在 16800（仅训练结束才更新） | 只用文件名 | ✅ 已记录 |

## 本次审计抓到并修复的纰漏

**缺少 100 步零命令交接斜坡**：控制器原先第一步就下发 lerp 后的目标（约 0.1 rad），训练只给 1%。补齐后 replay 的 `MAX_LEG_STEP` 从 **0.1206 → 0.0582 rad**，且 C++↔Python parity 仍为 1.19e-07。

## 已知差异（非缺陷，需知情）

1. 1 步延迟档未在部署复现（训练见过 0/1 两档）
2. 部署固定名义质量/质心/摩擦，极端组合未覆盖
3. 相位初值固定 0（训练为随机初值）
4. 实机 PD 在重力下的静差（后腿 0.33 rad）训练侧不存在——这是 **sim2real 的植物差异**，不是合同不一致
