# 2026-11-20 AS2-W HIMLoco DR（model_18200）实机首测会话

本会话目标：把 HIMLoco 平地域随机化（10k 轮五阶段）的最新完整权重 **`model_18200`** 部署到 AS2-W 实机，按 G0–G8 门槛做首测，并记录全部离线/实机数据。

> 结论先行：**离线链路全部通过（ONNX↔torch 2.38e-06、C++↔Python 1.19e-07、CPRO ARM64 CTest 8/8、offline audit PASS）；实机完成"固定站 + 零速策略闭环"，但推杆驱动发现后腿 PD 撑不住（小腿 cmd−actual 0.33 rad），静态重力补偿方案导致前倾/嗡鸣，已回退。MuJoCo 第二仿真器 2026-11-21 修复后通过（站立/前进/后退可用，转向偏弱）。**

## 关键结果

| 项 | 结果 |
|---|---|
| 部署权重 | `model_18200.pt`（consolidate 阶段；训练在 19688/20400 被终止后另起 `motor_jitter`） |
| ONNX | `policy.onnx`，输入 `obs[N,360]` → `out[N,16]`，sha `02352011…78bf` |
| ONNX↔torch（真实 env 观测，60 步） | max 动作差 **2.38e-06** |
| C++↔Python（120 帧夹具） | 腿 **1.19e-07 rad** / 轮 **5.96e-07 rad/s** |
| CPRO ARM64 | 构建 rc=0、CTest **8/8**、离线 audit **PASS** |
| 实机只读 | LowState ~1060 Hz、`mode_pr=0`、`mode_machine=2`、16 轴有反馈、`motorstate_or=0x0`、`warning_or=0x0`、电池 48.5 V |
| 实机零速闭环 | A 固定站 → B 交接（pose_error 0.202 / quiet_dq 0.043）→ PolicyWalk 零速 → 自动 Brake → ReturnStand → X，**0 安全告警** |
| 实机推杆 | 反向 −0.30 m/s 可达、静差大；掉头/横移未测 |
| 后腿下沉 | RL/RR calf `cmd−actual` **+0.33 rad**（约 3–4 cm） |
| 静态重力补偿 | 后腿改善但整体前倾 + 力矩嗡鸣，**已回退为 0** |
| MuJoCo 第二仿真器（11-21 修复） | 站立 z 0.333–0.384、前进 0.276（92%）、后退 −0.254（85%）、无翻倒；**转向仅 24°/7 s，偏弱** |

## 目录

| 目录 | 内容 |
|---|---|
| `docs/` | 部署契约、训练合同逐项审计、实机首测准备单（G0–G8） |
| `analysis/` | 时间线、下沉逐轴分析、能力矩阵、三级 parity 链 |
| `data/` | ONNX、四份 deploy.yaml、控制器源码、export/parity 元数据 |
| `plots/` | 下沉柱状图、能力矩阵图、关节角曲线（前/弧/站） |
| `field/` | CPRO 只读日志（LowState/保护流/模式/服务）、零速发布端日志 |
| `videos/` | Isaac 回放（已验证）、MuJoCo 修复后回放（2026-11-21，已验证）、历史草稿（已废弃） |

## 未完成 / 下一步

1. 后腿下沉的机械侧排查（减速器背隙、电压、结构松动）
2. 策略相位 kp 55→65 的实机对比（`data/deploy-drive-kp65.yaml` 已备好，未执行）
3. MuJoCo 转向对齐（轮地摩擦 0.62 与轮速伺服带宽），当前转向只跟踪 20%
4. `motor_jitter` 阶段新权重产出后再评估

详细时间线见 [`analysis/session-timeline.md`](analysis/session-timeline.md)。
