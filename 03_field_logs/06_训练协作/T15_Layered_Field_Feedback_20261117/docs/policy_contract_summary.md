# 部署侧实测合同摘要（T15 真机链，供训练侧对照）

部署 C++ 链已按交付合同逐帧 parity（≤3.7e-07），以下为真机上实际生效的完整合同。任何与训练意图不符处请指出。

## 观测（65 维）
- 0:3 命令 (vx−0.4, vy, wz)**×8**（AS2W_COMMAND_GAIN=8，reproduce.sh 实值）；命令**无滤波直入**（训练语义阶跃命令）
- 3:6 IMU 机体角速度直读；6:9 重力 Rᵀ[0,0,−1]（站立 z=−1）；9 = 0.4
- 10:22 腿角 − 默认角 (hip 0, thigh 0.8, calf −1.6)（init_state.joint_pos，三方一致后采用）
- 22:26 轮角 wrap ±2π（**double 精度**——float32 在大轮角下有 4e-6 舍入，parity 修正项）
- 26:42 16 关节速度；42:58 上一动作 = **有效 14 维的 16 通道语义展开**（legs 原值 + K/16·(tanh(u_c)+tanh(u_y)·side)）
- 58:60 相位（0.62s，**double 常数**——float32 π 在长回合漂移，parity 修正项）
- 60:64 零速轮差/0.2 clip ±3；64 全零命令标志；**was_zero=False 初始化**（episode 首帧捕获，训练语义）
- IMU 角速度：**按训练语义直读**（交付 MuJoCo 参考脚本的 qvel[3:6] 双旋转偏差未采用，armD 同款决定）

## 动作与包装
- 腿 = default + 0.35·action[:12]，**无 clip**（训练无 clip 配置；关节限位由硬件/守卫兜底）
- 轮通道 12/13 丢弃，由 LayeredWheelController 覆写：u_common=arctanh(vx_served/(rK))（1 m/s² slew、0.99 限幅）；
  u_yaw 闭环积分 kp3/deadband 0.004/teacher 表×1.17 钳位/零命令 0.75 衰减/250 步看门狗/**与 wz_cmd 反号**
- 四轮目标 K·tanh(u_c)+K·tanh(u_y)·side，K=4，side=(+1,−1,+1,−1)，clip ±98.47
- prev 反馈 = 控制器覆写后的有效动作（与训练 expand 分布一致）

## 增益（关键分歧点）
| 段 | 腿 kp/kd | 轮 | 说明 |
|---|---|---|---|
| 训练合同 | 55/2 | kp0/kd2 | 交付 config；真机结果：PolicyWalk 段后腿塌 0.24–0.33 rad、拖地（见 sessions 1–2） |
| STIFF 候选（部署侧独立修订） | **85/3.2**（仅 B 后策略段；A 段仍 55/2；交接渐变） | 同 | 塌 0.036–0.057、零拖地、三次验证；**非训练合同，训练侧更新后作废** |

## 入口/退出
- 入口 = 趴下→A（FixStand 插值 3s→固定 PD 站 T2）→B（gate：pose_error≤0.35/wheel_dq/upright≥0.85/quiet）→
  2s 零速交接→PolicyWalk（30s cap，LY→vx、RX→wz，包络 deploy.yaml 0.2/0.12，CLI 只可收窄）
- 退出 = A 刹车回站 / Y 趴下 / X 紧急阻尼 / cap 自动刹；松杆 = 零命令（合同原生驻停，无锁存包装）
- 命令不进发布端 slew（训练无命令滤波；控制器内部 vx 有 1 m/s² 平滑）

## 已知缺口（沿用交付 + 真机新证）
1. PhysX 前进误差 20–49%（交付已知；真机未以速度精度为判据）
2. **真机站姿塌陷/拖地（新增，本包核心）**——55/2 合同的静态平衡问题
3. **策略层无抗扰（新增）**——教师腿动作恒零，BC 无负载反馈行为
4. watchdog 5s hold-last（交付已知；真机全程人值守急停，未触发）
5. gyro 滤波：训练合同无滤波；交付“10–20Hz 硬件推荐”为未验证选项，部署侧 load_config 拒绝非零值
