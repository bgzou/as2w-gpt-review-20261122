# model_18200 实机部署契约与脚本

**权重**：`/root/gpufree-data/as2w-runs/him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt`
**导出包**：`/root/as2w/deploy/model_18200/`
**导出校验**：ONNX vs PyTorch 在**真实观测**上 60 步最大动作差 **2.4e-06**（`parity.json`）

---

## 1. 交付文件

| 文件 | 形状 | SHA256 | 用途 |
|---|---|---|---|
| `policy.onnx` | 360 → 16 | `02352011169cbf1dc3acfe3b698de27b98a32b6ad56c6063118c3d6e999678bf` | **部署用**（融合 estimator+actor） |
| `estimator.onnx` | 360 → 19 | `5d32959b38cda5d81d1bdd52334bd3f0a767eb646fa21e083e98297b1f11cb0d` | 拆分调试（含 latents L2 归一化） |
| `actor.onnx` | 79 → 16 | `50de80a8cf0040a2034777029f6146ad231c1910f5079c386813a4865722cdb8` | 拆分调试 |
| `export.json` | — | — | 导出元数据 |
| `parity.json` | — | — | 真实观测 parity 结果 |
| `deploy_ref_controller.py` | — | — | 参考控制器（观测构造 + 推理 + 目标映射） |

> **`ckpt["iter"]` 不可信**：本 run 的所有 checkpoint 里 `iter` 字段都停在 16800（runner 只在训练结束时更新它）。权重版本必须用**文件名**（`model_18200.pt`）判定。

---

## 2. 观测契约（60 维/帧，6 帧历史 = 360）

| 索引 | 内容 | 说明 |
|---|---|---|
| 0:3 | `command` (vx, vy, wz) | 训练分布 vx ±0.8、vy ±0.45、wz ±0.8 |
| 3:6 | `root_ang_vel_b` (rad/s) | 机身角速度 |
| 6:9 | `projected_gravity_b` | IMU 重力在机体系投影，模 1 |
| 9:25 | `q − default_joint_pos` (16) | 顺序 `FL/FR/RL/RR × hip,thigh,calf` + 4 轮；**轮位置强制为 0** |
| 25:41 | `joint_vel × 0.05` (16) | 同上顺序 |
| 41:53 | `applied_leg / 0.25` (12) | **实际施加**的腿目标（不是策略原始输出） |
| 53:57 | `applied_wheel / 12` (4) | 实际施加的轮速目标 |
| 57 | `0.37` | 固定机身高度常数 |
| 58 | `sin(phase·2π)·strength` | 相位时钟 |
| 59 | `cos(phase·2π)·strength` | 相位时钟 |

- `phase += 0.02 × gait_frequency(cmd)`，`gait_frequency = clamp(1.25 + 0.95·(|vy| + 0.25·|wz|), 1.25, 1.75)`，取模 1
- `strength = clamp((|vy| + 0.28·|wz| − 0.04)/0.16, 0, 1)`
- 历史缓冲：**新帧在前**（`history[0]` 为当前帧），展平顺序 `history.reshape(-1)`
- 训练时的 `gyro_bias / joint_bias / noise` 在评测/部署时为 0

---

## 3. 动作契约（16 维）

> 2026-11-20 修正：真正的映射在 `arc_environment._pre_physics_step` 里——策略输出先 **tanh**，腿再乘 **0.6 增益**，轮带 **(vx−wz·side)/R** 前馈。早期版本按 `QualityEnv` 单独推导成“clamp + ×12”，是错的；C++ 控制器原本就按 ArcEnv 实现，是对的。

```text
bounded = tanh(policy_out)                       # 16
leg_phys = 0.6 * bounded[:12]                    # 腿增益 0.6（平地层）
wheel_phys = clamp((vx − wz*side)/R + 2*bounded[12:], ±12)   # side = [+.1654,-.1654,+.1654,-.1654]
target_leg   ← lerp(target_leg,   0.25 * leg_phys, α_leg)     # => ±0.15 rad
target_wheel ← lerp(target_wheel, wheel_phys,      α_wheel)
```

- `α_leg = 1 − exp(−2π·10·0.02) = 0.7154`（腿目标 10 Hz 低通）
- `α_wheel = 1 − exp(−2π·5·0.02) = 0.4665`（轮目标 5 Hz 低通）
- **PD 增益**：腿 `kp=55, kd=2`，轮 `kp=0, kd=2`（轮为速度控制）
- **控制周期 20 ms（50 Hz）**，与训练一致，不允许改频率而不重算 α 与 phase

---

## 4. 部署状态机（建议）

```
Passive → FixedStand(A 段, kp55) → 保持 100 步零命令交接 → PolicyWalk → Brake → Passive
```

1. **Zero-command handoff**：进入 PolicyWalk 后先发 100 个控制周期的零命令（vx=vy=wz=0），这与训练时 `handoff_steps=100` 完全一致；期间 `applied` 从 0 缓入
2. 交接结束时 `phase` 可任意初值（训练为随机初始化），建议置 0 后自然演进
3. 退出：先发零命令 0.5 s，再切 Brake/Passive

---

## 5. 安全门限（部署前逐项确认）

| 项 | 阈值 | 动作 |
|---|---|---|
| 姿态 | `gravity_b.z > −0.52`（约 31° 倾角） | 立即切 Passive |
| 腿目标 | 必须落在 URDF 限幅内（髋 ±0.785、前大腿 [−2.0, 3.492]、后大腿 [−1.3, 4.19]、小腿 [−2.768, −0.813]） | 越界则钳位并告警 |
| 轮速目标 | ±12 rad/s | 钳位 |
| 机身高度 | 相对站姿 < 0.28 m 或 > 0.50 m | 切 Passive |
| 通信 | LowState 超时 > 50 ms | 切 Passive |
| 急停 | 摇杆/按键 | 立即 Passive，不发新目标 |

---

## 6. 已知限制（必须写进部署风险）

1. **sim2real 静差未解决**：保持实测 T2 姿态时仿真俯仰比实机多约 7.5°，前髋力矩约为实机一半（3.7–5.1 vs 11–12 Nm）。本策略在该静差下训练，上机前需先用小步速、低增益试跑。
2. **接触合同是 0.25 + min**：若实机轮胎/地面材质不是这一档，硬件与训练不一致，性能未知。
3. **`ckpt["iter"]` 停在 16800**，不要用它判断版本。
4. **观测里的 41:57 是“施加目标”**，不是策略输出；如果部署侧错用原始动作，观测分布会偏。
5. 训练分布含 ±0.5 kg / ±10 mm 质心随机化，实机若超出该范围（例如加装电池/支架）需重新训练。

---

## 7. 快速自检（服务器上已跑通）

```bash
cd /root/as2w
./preflight/isaac-python deploy/model_18200/deploy_ref_controller.py \
    --policy deploy/model_18200/policy.onnx --selftest
# 期望：onnx_input [batch,360]，onnx_output [batch,16]，
#       alpha_leg 0.7154，alpha_wheel 0.4665，selftest ok

./preflight/isaac-python him_arc/deploy_parity_check.py \
    --ckpt /root/gpufree-data/as2w-runs/him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt \
    --onnx deploy/model_18200/policy.onnx \
    --asset /root/as2w/unitree-flat-v2-dr/assets/as2w-t2-v2-mu025.usd \
    --steps 60 --out deploy/model_18200/parity.json
# 期望：max_abs_action_diff ≈ 2e-06，parity_ok true
```

---

## 8. 与机器人接口对接

`deploy_ref_controller.py` 的 `Controller.obs(cmd, gyro, gravity, q, dq)` 与 `Controller.act(obs)` 就是全部契约：

- `q/dq`：16 维，顺序 `FL_hip, FL_thigh, FL_calf, FR_hip, …, RR_calf, FL_foot, FR_foot, RL_foot, RR_foot`
- `gravity`：`R^T·(0,0,−1)`，由 IMU 四元数计算
- 输出 `leg_position`（12 个绝对关节角，rad）与 `wheel_velocity`（4 个 rad/s）

C++ 侧只需要复刻这三段：帧拼接、`lerp` 平滑、`sin/cos` 相位。
