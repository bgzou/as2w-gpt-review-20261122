# 11 Phase H：执行器力矩真实性审计（LowCmd/LowState 源码 + 日志重析 + 力矩回放）

日期：2026-11-21 起。任务书：《AS2-W Sim2Real 根因诊断 Phase H/I/J（2026-11-21）》。
约束遵守：无 RL、无实机、无正式资产修改；本文件全部结论可溯源到列出的源码行、原始日志或可复算数据。
证据标签沿用六类。G2 勘误后的接触几何结论为准（见 10 文档）。

---

## H1 LowCmd/LowState 源码审计

### H1.1 审计范围与资料可用性

| 对象 | 位置 | 可用性 |
|---|---|---|
| 实机发布端源码 `as2w_publish.cpp` | `部署参考/cpp_deploy-as2w/deploy/robots/as2w/common/`（上游维护服务器） | **本仓库无副本**；其行为经 V2 包 `docs/00_控制接口与模式审计.md` §2 的逐行线号引用 + 本轮对其产物的逐帧复算交叉核验（`SOURCE_CONFIRMED(线号引用)` + `RAW_DATA_CONFIRMED(产物复算)`） |
| C++ SDK2 IDL | `/home/robot/z00959834/robogauge_5090/cpp_stack/unitree_sdk2/include/unitree/idl/{hg,go2}/` | 本轮逐文件读出 |
| Python SDK2（unitree_sdk2py） | `/home/robot/z00959834/robogauge_5090/CPRO代码/thirdparty/unitree_sdk2_python/` | 参考链；AS2-W 实机不经它 |
| 本仓库 AS2-W C++ 部署链 | `deliveries/as2w-deploy/`（r152_controller.cpp/.hpp、replay.cpp、shadow.cpp、main.cpp、sdk2_shadow.cpp、policy.cpp） | 本轮逐文件读出 |
| 本仓库 Python 部署链 | `rob2nav/deployment/unitree.py` 等 | Go2 参考链，读出 |
| 实机录制链产物 | V2 包 recordings/（low_capture_fast.log、lowcmd_capture.log、debug_lowcmd.csv、low_protection.log、publisher.log） | 本轮抽样复算 |

**重要更正（相对本轮任务书预设）**：AS2-W 实机会话的录制/发布链 **不经本仓库 `rob2nav/deployment/`，也不经 deliveries/ 的 r152 部署二进制**；七会话由上游服务器维护的 `as2w_publish.cpp`（hipdiag 构建宏）在实机 CPRO 上产生。本仓库交付链只作为"同族合同"旁证。以下 §H1.2/H1.3 区分三条链。

### H1.2 IDL 判定：AS2-W 实机走 HG IDL（不是 go2）

判定证据（`RAW_DATA_CONFIRMED` + `SOURCE_CONFIRMED`）：

1. LOW_SAMPLE schema 实录 35 电机槽（`m0_q`…`m34_temp`，P55G2 `low_capture_fast.log` schema 行）。
2. C++ SDK `unitree/idl/hg/LowState_.hpp:33`：`std::array<::unitree_hg::msg::dds_::MotorState_, 35> motor_state_`；而 `go2/LowState_.hpp:37` 为 `array<..., 20>`。
3. V2 00 文档 §1.2：官方 `hardware_contract.md` 记录话题类型 `unitree_hg::LowCmd_`。
4. Python 链（`rob2nav/deployment/unitree.py`）订的是 `unitree_go` LowState_（20 槽、有 `foot_force/lost/power_v`）——那是 Go2 参考链，字段集与 AS2-W 实机 HG 流**不同**，不得混用。

HG IDL 字段实况（C++ SDK 逐行读出，`SOURCE_CONFIRMED`；IDL 由 Cyclone DDS 从 .idl 生成、**无任何语义注释**）：

- `hg/MotorState_.hpp`（L27–L34）：`uint8 mode_; float q_, dq_, ddq_, tau_est_; int16 temperature_[2]; float vol_; uint32 sensor_[2]; uint32 motorstate_; uint32 reserve_[4]`
- `hg/MotorCmd_.hpp`：`uint8 mode_; float q_, dq_, tau_, kp_, kd_; uint32 reserve_`
- 录制链只取 `q/dq/ddq/tau(=tau_est)/mode/temp` 子集；`vol_/sensor_/motorstate_` 仅进入保护流（`low_protection.log`，`motorstate_or`/`warning_or` 聚合位）。

### H1.3 命令侧：七个会话的完整命令合同（谁填什么）

本轮从 `debug_lowcmd.csv`（500 Hz 发布端自记录，per_contract_axis×16：(slot,mode,q,dq,kp,kd,tau)）与 `lowcmd_capture.log`（100 Hz 只读探针，官方模式）直接复算，`RAW_DATA_CONFIRMED`：

| 会话 | 控制器 | 腿合同 | 轮合同 | tau_ff | q_des |
|---|---|---|---|---|---|
| P55G1/G2/G3/P55S | as2w_hipdiag_publish（FixedStand 段） | mode=1, kp=55, kd=2 | mode=1, kp=0, kd=2, dq_des=0 | **0** | T2 冻结：hip 0 / thigh 0.8 / calf −1.6 |
| P85DS | 同上（p85d 构建） | kp=85, kd=3.2 | 同上 | **0** | 同 T2 |
| OG/OS（ai_stand_up 段） | 官方固件 | mode=1, kp=200, kd=4 | mode=1, kp=0, kd=0.5, dq=0 | **0** | 冻结：hip ±0.15 / thigh 0.6 / calf −1.2（100 s 漂移 0.0000 rad） |
| OG 起身插值段（14.25→18.2 s） | 官方固件 | kp=200/kd=4 | 同上 | 0 | 从实测趴下姿态插值到站姿 |
| （参照）ai_free_walk 探针 | 官方固件 | kp=40, kd=1 | kp=0, kd=1, 小 dq | 0 | 持续微调（单关节跨度 ≤0.13 rad） |
| （参照）ai_damping | 官方固件 | kp=0, kd=8 | kp=0, kd=0.5 | 0 | q_des=0（不承载信息） |

P55G2 逐轴复核（本轮）：FixedStand 段（state 2，6 476 行）腿 12 轴 `kp uniq={55}、kd uniq={2}、tau={0}、q_des std=0.00000`；轮 4 轴 `kp=0/kd=2/q=0/dq=0/tau=0`。**没有任何会话使用力矩前馈。**

对任务书核心问题"q/dq/kp/kd/tau_ff 如何进驱动器"的回答：

- 发布端每 2 ms（500 Hz）打包 HG LowCmd：腿 = `{mode:1, q:q_des_raw, dq:0, kp, kd, tau:0}`，轮 = `{mode:1, q:0, dq:0, kp:0, kd, tau:0}`，`q_raw = sign*target + zero`（轴标定映射），CRC32 后发 `rt/lowcmd`。
- **PD 乘法 (kp·e_q + kd·e_dq) 不在发布端发生**——发布端只送目标和增益，PD 在驱动器固件内环执行。因此 `tau_out = KP_firmware·(q_des−q) + KD_firmware·(dq_des−dq) + tau_ff(=0)`，其中 KP/KD 的执行（频率、滤波、限幅、电流环整定）全部在厂商固件内，`UNKNOWN`。
- 命令侧限幅只存在于发布端软件安全层（FieldSafetyProfile：腿 tau warn35/hard50 Nm、腿 dq warn5/hard8 rad/s、目标步进 warn0.2/hard0.35 rad、warn 5 帧→SoftBrake、hard 3 帧→EmergencyDamping），它们触发的是**发布端自己刹车**，不是修改命令值；驱动器固件限幅（电流/峰值/连续/温度/电压降额）`UNKNOWN`，且 LOW_SAMPLE 无电压字段，日志无法核查电压降额。

### H1.4 状态侧字段真值表

| 数据（npz/日志字段） | 真实来源 | 物理含义 | 单位 | 独立测量？ | 可信度 |
|---|---|---|---|---|---|
| `q` (motor_state.q) | 驱动器编码器经固件折算 | 关节角（与命令同坐标空间） | rad | 否（固件折算含零位/减速比） | 高：PD 残差自洽 + 悬空下垂方向物理锚定（00 §2/§4） |
| `dq`/`ddq` | 固件差分/估计 | 关节角速度/加速度 | rad/s, rad/s² | 否 | 高（dq）；ddq 未在本轮分析使用 |
| `tau` (motor_state.tau_est) | **厂商固件估计**（算法 UNKNOWN：电流估计 or 观测器；输出侧/电机侧 UNKNOWN；减速比/传动效率补偿 UNKNOWN） | 关节力矩估计 | Nm | **否** | 中：四条间接证据（下） |
| `temp` (motor_state.temperature) | 电机温度传感 | °C | 是（传感） | 高 |
| `mode_pr/mode_machine` | 固件状态字 | 发布/整机模式 | 枚举 | 是 | 高 |
| `quat/gyro` | 主板 IMU | 姿态/角速度 | — | 是 | 高 |
| q_des/kp/kd/tau_ff | 发布端自记录（debug_lowcmd/lowcmd_capture） | 实际发出的命令 | rad, Nm/rad, Nm·s/rad, Nm | 是（自记录） | 高 |

**tau_est 间接证据链**（支撑"关节输出侧 Nm、静载下可用"的结论，`RAW_DATA_CONFIRMED`/`SIM_RUN_CONFIRMED`）：

1. 静态 PD 自洽：逐关节 `mean(tau − kp·(tgt−q))` = 0.03–0.27 Nm（7 会话，00 §4）；OG kp200 下 hip 残差 0.042/0.135/−0.123/−0.034 Nm ≪ 力矩本身 4.3–5.8 Nm——若 tau_est 在别的坐标空间或含大偏置，该残差不可能这么小。
2. 悬空重力对账：tau_est 悬空窗口均值与 MuJoCo 腿节自重模型残差 0.02–0.46 Nm（Phase B）——tau_est 包含真实重力负载，量纲为关节输出 Nm。
3. 增益缩放律：悬空 kp=85 误差比实测 0.604–0.687 vs 理论 55/85=0.647（V2 报告 §0）——误差确实由 kp·e 决定。
4. 官方/自定义两族独立控制器（200/4、40/1、55/2、85/3.2）下 tau_est 行为全部可预测。

**但以下仍 `UNKNOWN`**：tau_est 算法（电流→力矩标定、是否含摩擦/传动效率补偿）、带宽/滤波、饱和状态下 tau_est 是否削顶（本轮七会话 |tau| 最大 ~21 Nm ≪ 发布端 soft 限 35，未见削顶平台，但驱动器固件限幅高于此未见证据）、动态（非静态）下 tau_est 滞后量。

### H1.5 与本仓库交付合同的对照（防混淆说明）

- `rob2nav/deployment/unitree.py`（Go2 链）有两种 wire mode：'torque'（哨兵值 `q=2.146e9, dq=16000, kp=kd=0, tau=命令`）与 'hybrid_pd'（全字段）；校验 kp≤80/kd≤8——**该链未用于 AS2-W 实机会话**，仅作接口语义参考。
- `deliveries/as2w-deploy/.../r152_controller.cpp:158` 实机候选合同：腿 `{q=clamp(def+0.35a, lo, hi), dq=0, kp=55, kd=2}`、轮 `{q=0, dq=clamp(16a), kp=0, kd=2}`——与 hipdiag P55 实机合同同族（55/2），但那是 R177 策略部署，非七会话产生者。
- 任务书禁止的"按变量名推断语义"点逐一落实：`mode` 实测取值仅见 0（未控）与 1（受控）两态，其余取值含义 `UNKNOWN`；`sensor_[2]`、`motorstate_` 完整位表 `UNKNOWN`（保护流只聚合出 `motorstate_or`）。

### H1.6 厂家资料清单（需向厂商索取，索取前对应项维持 UNKNOWN）

1. HG `MotorCmd.tau` 与 kp/kd 项的叠加语义（加法前馈？mode 依赖？）及 mode 枚举完整表。
2. `tau_est` 算法：电流估计 or 扰动观测器、标定曲线、减速比/传动效率补偿、滤波/带宽。
3. 驱动器电流/力矩限幅（峰值 vs 连续曲线）与温度/电压降额策略。
4. 固件 PD 内环执行频率、命令保持（100/500 Hz 到达内环的重采样/插值）、电流环整定。
5. `sensor_[2]`/`motorstate_` 位定义、`vol_` 语义（单电机电压？）与正常范围。
6. HG 35 槽的槽位-物理关节官方对照表（当前映射是现场标定 `r152_measured.map`，非厂商提供）。

### H1.7 对 H2/H3 的可行性判定

| 后续需求 | 可行性 | 依据 |
|---|---|---|
| H2 逐帧重构 tau_PD = kp·(q_des−q)+kd·(dq_des−q̇)+tau_ff | **可行** | q_des/kp/kd/tau_ff 全部有记录：500 Hz `debug_lowcmd`（自定义会话）、100 Hz `lowcmd_capture`（官方会话）；且窗口内合同恒定（std=0，本轮复算），亦可用合同常数 |
| H2 饱和/削顶检测 | **部分可行** | tau_est 序列 500 Hz 可查平台/削顶；电压降额不可检（LOW_SAMPLE 无电压字段）→ UNKNOWN 项 |
| H3 模式 B（LowCmd 闭环回放） | **可行且已有** | q_des 恒定 ⇒ 等价于 P55/P85 位置伺服合同直接进 MuJoCo/PhysX 位置 PD —— 即 Phase D `d1_closed_loop_matrix` 已跑的配置（本轮 H3 补 P55G2 逐帧行为对比） |
| H3 模式 C（tau_est 开环回放） | **仅敏感性** | tau_est 是固件内环**执行后**的估计（含静摩擦/传动损耗/电流环滞后）；把它开环注入仿真等于把估计误差与传动损耗一起注入。H1 判定：只做敏感性对照，不作为复现基准（任务书 §H3 前置条件满足方式=保守解释） |
| H3 模式 D（tau_PD 开环回放） | **可行** | kp/kd/q_des 已知 ⇒ tau_PD 序列可离线构造；预期上界=模式 B 的开环展开，用于暴露"固件内环 ≠ 理想 PD"的差 |

---

## H2 实机日志重新分析（tau_PD vs tau_est，七会话）

### H2.1 字段完整性核查（先于任何计算）

| 需求 | 记录源 | 结论 |
|---|---|---|
| q/dq/ddq/tau/mode/temp（35 槽） | `low_capture_fast.log` LOW_SAMPLE 500 Hz | **在**（`RAW_DATA_CONFIRMED`，schema 行） |
| q_des/kp/kd/tau_ff（自定义会话） | `debug_lowcmd.csv` 500 Hz per_contract_axis×16 + `lowcmd_capture.log` 100 Hz | **在**；窗口内恒定（H1.3 复算 std=0） |
| 电压/电流 | 无对应 schema 字段 | **缺** → 电压降额/电流上限核查不可能，`UNKNOWN` |
| 基座位置 | npz 无（仅 IMU 姿态） | 已知限制（Phase A 起），间隙为相对量 |

脚本：`runs/…/scripts/h2_torque_pd_analysis.py`；逐帧 `tau_PD = kp·(tgt−q) + kd·(0−dq) + 0`，合同取 H1.3 表（P55=55/2、P85DS=85/3.2、OG/OS=200/4，T 族目标冻结）。数据：`tables/H2_TORQUE_PD_CONSISTENCY.{csv,json}`（84 关节行）、`H2_SESSION_TAU_STATS.csv`、`H2_WHEEL_TAU_CHANNEL.csv`（24 行）。

### H2.2 总体结果（`RAW_DATA_CONFIRMED`，本轮复算）

| 会话 | 窗口 | n | resid RMS (Nm) | resid max (Nm) | \|tau_est\| max (Nm) | 温度范围 °C |
|---|---|---|---|---|---|---|
| OG | 17–120 | 51500 | 0.122 | 0.33 | 7.8 | 35–46 |
| OS | 0–60 | 29991 | 0.120 | 0.30 | 2.6 | 36–49 |
| P55G1 | 10–30 | 10000 | 0.126 | **1.65** | 15.9 | 36–48 |
| P55G2 | 8–19 | 5500 | 0.134 | 0.35 | 15.3 | 34–46 |
| P55G3 | 7–19 | 6000 | 0.126 | 0.31 | 15.3 | 35–47 |
| P55S | 25–90 | 32500 | 0.119 | 0.30 | 6.3 | 33–47 |
| P85DS | 48–100 | 26000 | 0.120 | 0.37 | 3.8 | 34–49 |

七会话残差 RMS 全部 0.119–0.134 Nm。P55G1 的 1.65 Nm 单帧极值集中在 calf（该窗含小扰动瞬态，resid_mean 仍 −0.21…+0.07 Nm），非饱和。

### H2.3 发现 1：静态一致性极好，无饱和/削顶/降额迹象

- hip 级：`resid_mean = est − PD` 介于 −0.161…+0.158 Nm，作用在 |τ| 高达 15.2 Nm 上（P55G2 RL −15.15 vs PD −15.29 Nm）。
- **无削顶**：全部窗口 |tau_est| max = 15.9 Nm ≪ 发布端 soft 限 35 / hard 限 50 Nm；|tau| 距窗口极值 <1% 的帧占比在地面会话 ≈0.98–1.00——这是**静态恒定**（τ 波动 <1%）而非限幅平台（限幅平台的特征是分布上界被截断，此处无）。
- **无热降额迹象**：温度 33–49 °C 平稳；残差-温度相关 |ρ|≤0.08、残差-时间相关多数 |ρ|≤0.07（例外：P55G2 FL_hip +0.28、P55G3 FL_hip −0.21，量级 ≤0.1 Nm 漂移）。

### H2.4 发现 2：悬空 vs 地面的动态跟踪差异（执行通道带宽的间接证据）

| 场景 | hip corr(PD,est) | 拟合 slope | R² |
|---|---|---|---|
| P55S 悬空 | **0.974–0.997** | **0.966–0.996** | 0.948–0.994 |
| P55G1–G3 地面 | 0.18–0.85 | 0.19–0.88 | 0.03–0.72 |
| OG/OS/P85DS | 0.22–0.44 / 0.38–0.62 | 0.15–0.38 / 0.34–0.56 | 0.04–0.19 / 0.14–0.38 |

悬空下 tau_est 与理想 PD 逐帧几乎重合（执行链透明）；地面下逐帧相关骤降但**静态残差不变**（仍 ~0.12 Nm RMS）。物理解释：地面接触激励使 q/dq 含高频抖动，理想 PD（kp=55）把这些抖动放大为大幅高频需矩，而 tau_est（固件估计，含滤波/电流环带宽）与实机电流响应跟不满瞬时需要——即 **tau_est 通道带宽 < 理想 PD 带宽**。这是唯一的执行器非理想性动态证据；它不改变静态力矩账本（H4 归因按静态口径），但给 H3 模式 C/D 定义了预期差。

### H2.5 发现 3：系统性 per-joint 常数偏置 ~±0.1–0.2 Nm（与力矩符号无关）

跨会话稳定的 hip 偏置模式（`RAW_DATA_CONFIRMED`）：前腿 FL −0.02…−0.12、FR −0.06…−0.16；后腿 RL +0.06…+0.15、RR +0.04…+0.13 Nm——**前负后正，与各关节力矩符号无关**（FL τ<0 时 −0.03、FR τ>0 时 −0.16 同号偏置）→ 执行器/标定常数项（静摩擦方向或零位），非增益误差。量级进入静力误差预算（0.1–0.2 Nm/关节）。

### H2.6 发现 4：轮通道几乎零载荷且零输出

轮（kp=0/kd=2 或官方 kd=0.5，dq_des=0）在七个窗口：|dq| ≤ 0.016 rad/s（静止），|tau_est| ≤ 0.4 Nm（P55G1 瞬态），典型 ±0.02 Nm 常数项 + −kd·dq 项可忽略（corr −0.045…+0.80 不定）。**实机静止站立时轮驱动器不输出主动力矩，地面切向力由轮传动静摩擦/轴承承担**——与仿真轮 velocity-actuated 合同（静止时 τ≈0）定性一致；但"传动能承担多大切向力不出力"（静摩擦容量）`UNKNOWN`，列厂家清单。轮侧无任何受限迹象。

### H2.7 小结（对接任务书 H4 判定选项）

| 审计项 | 判定 |
|---|---|
| 饱和/削顶（tau 序列平台、|τ| 上界截断） | **未发现**（≤15.9 Nm ≪ 软/硬限幅；无平台） |
| 位置误差异常增大 | 未发现（窗口 e_q 稳定，P55G2 hip 0.20–0.24 rad 为接触负载下的稳态静差，与 G2 反演自洽） |
| 温度/电压同步降额 | 温度：未发现（33–49 °C 平稳、无相关）；电压：**无法判断**（无字段，UNKNOWN） |
| 前后腿差异 | 有（常级 ±0.1–0.2 Nm 偏置，前负后正），量级不构成"受限" |
| tau_est 与 tau_PD 静态自洽 | 是（0.12 Nm RMS），tau_est 可作静态关节力矩真值使用（误差预算 0.12–0.3 Nm） |
| tau_est 高负载可靠性 | 12–16 Nm 区间仍与 PD 自洽（P55G×）；>16 Nm 区间无数据 → `UNKNOWN`（须更高负载实验或厂家数据） |

**H2 结论：执行器力矩通道在七个会话的全部负载范围内未发现受限迹象；已知的执行器非理想性（±0.1–0.2 Nm 常数偏置 + 动态带宽差异）合计不足以解释 G3 残余的 sim-real 力矩缺口（前腿 hip 6.2 Nm、后腿 2.8 Nm）。** 执行器问题从"不能排除"降级为"有强证据基本排除（静态口径）"。

---

## H3 力矩时间序列回放实验（MuJoCo，candidate 副本）

### H3.1 设置

脚本：`runs/…/scripts/h3_torque_replay.py`；模型：G3 candidate 副本 `M_cylwheel`（地面，freejoint）与 `M_cylwheel_susp`（无地面、无 freejoint = 固定基座）；数据：P55S 窗 (25,90)（悬空阶段参照）与 P55G2 窗 (8,19)（地面阶段参照），500 Hz，初始位形取各参照窗首帧，基座高度按 G3 口径（最低轮点离地 2 mm）。

**方法学警告（本轮踩坑，须与既有记忆条目同类）：交付 MJCF 自带 position/velocity 执行器块（kp=60/90、kv=8，ctrl=0 时把所有腿关节拉向 q=0）。用 `qfrc_applied` 开环/闭环驱动时必须先 `actuator_gainprm/biasprm 清零`（G3 的 f2 settle 合同正确做过），否则隐藏执行器与显式 PD 叠加——首次运行的"平衡点" thigh=0.379 恰等于 55·0.8/(55+60)+阻尼，即两套位置弹簧的折中，是假平衡。** 修正后全部 7 组 run 重跑（`SIM_RUN_CONFIRMED`）。

模式（H1.7 判定的可行性范围内）：

- **A**：闭环显式 PD（kp=55/kd=2，T2 冻结，轮 −kd·dq），settled 窗取最后 5 s。
- **B**：同 A，但合同逐帧取自 debug_lowcmd（H1.3 已证 std=0）。**B≡A 逐位成立**（`SIM_RUN_CONFIRMED`），验证读取管线，也证明"命令逐帧回放"与"合同常数"无差别。
- **C**：开环注入实机 tau_est 时间序列（敏感性口径，H1.7）。
- **D**：开环注入 tau_PD(t)=kp(T2−q_real(t))+kd(0−dq_real(t))。

### H3.2 结果（`SIM_RUN_CONFIRMED`，`tables/H3_TORQUE_REPLAY.{csv,json}`）

| 阶段-模式 | 参照 | 类型 | q 误差 RMS (rad) | q 误差 max (rad) | tau 误差 RMS (Nm) | 末帧漂移 (rad) |
|---|---|---|---|---|---|---|
| S-A 闭环 PD | P55S | settled vs 实机均值 | **0.0096** | 0.022 | 0.52 | 0.025 |
| S-C tau_est 开环 | P55S | 轨迹跟踪 | 0.349 | 0.81 | — | 0.74 |
| S-D tau_PD 开环 | P55S | 轨迹跟踪 | 0.344 | 0.81 | — | 0.77 |
| G-A 闭环 PD | P55G2 | settled vs 实机均值 | 0.075 | 0.132 | 4.07 | 0.128 |
| G-B 命令回放 | P55G2 | 同 A | 0.075 | 0.132 | 4.07 | 0.128 |
| G-C tau_est 开环 | P55G2 | 轨迹跟踪 | 1.61 | 3.56 | — | 3.49 |
| G-D tau_PD 开环 | P55G2 | 轨迹跟踪 | 1.61 | 3.56 | — | 3.49 |

分关节 hip 摘要（G-A，settled − 实机均值）：FL −0.131 / FR +0.132 / RL −0.112 / RR +0.109 rad——即 MuJoCo 落在 G3 已知的"前腿偏低"分支（前腿 0.084 vs 实测 0.214）；hip tau 差 ±6.0–7.2 Nm——G3 残余缺口原样复现。

### H3.3 解读

1. **悬空固定基座闭环复现实机到 0.01 rad 级**（S-A：q RMS 0.0096 rad、tau RMS 0.52 Nm，全部 12 腿关节）。执行器合同（HG 位模 PD 55/2，理想化）+ MuJoCo 重力模型 = 悬空物理的完整复现——执行器建模充分。
2. **开环注入必然积分漂移，且 C≈D**：S 阶段 65 s 开环漂 0.35 rad RMS（大腿/小腿主导，hip 仅 0.05–0.07 rad RMS）；C 与 D 的漂移逐位几乎相同（0.349 vs 0.344）——因为 tau_est ≈ tau_PD（H2 残差 0.12 Nm），两者开环积分后的发散轨迹重合。**这直接证明 tau_est 记录的不是"缺了什么力矩"的记录，它就是实机 PD 产生的力矩本身**；开环发散是"无反馈积分 + 0.1 Nm 级偏置 + 腿摩擦建模差"的必然结果，不构成执行器异常证据。
3. **地面闭环复现精度受 G3 已知分支限制**（G-A：前腿 hip q −0.13、tau −6.0…−7.2 Nm），与 G3 G 场景结果一致——缺口在接触/负载分配层，不在执行器层。G-B 逐位= G-A 再次确认命令面无逐帧信息可挖。
4. **地面开环回放（G-C/D）大幅发散（1.6 rad RMS、3.5 max）为预期物理**：自由基座倒立摆类平衡在无反馈开环力矩下不稳定（实机也是靠闭环节制平衡的）；G-C≈G-D 同样说明回放发散不来自 tau_est 缺陷。
5. 基座高度不确定性声明：P55G2 npz 无基座位置，地面 run 初始高度由 FK+2 mm 间隙归一（G2/G3 口径）；settled 统计对此不敏感（G3 已证 init 无关）。

### H3.4 H3 结论

| 任务书判定选项 | 本轮证据 |
|---|---|
| 已发现受限迹象 | **无**：悬空闭环 0.01 rad 复现；开环发散可全由已知 0.1 Nm 级偏置+无反馈积分+摩擦建模解释 |
| 未发现受限 | **是**（0–16 Nm 负载域内） |
| 无法判断 | >16 Nm 域（无数据）、电压降额（无字段）、固件限幅/降额内部（厂家黑箱）、tau_est 动态带宽定量值 |

**H3 最终结论：执行器力矩通道（HG 位模 PD + tau_est 回读）建模为"理想 PD + 0.1–0.3 Nm 级误差预算"是充分的；sim-real 残余缺口（地面 hip q 0.11–0.13 rad、tau 6–7 Nm）不在执行器层。**

---

## H4 Phase H 总结论

1. **执行器问题从候选根因中基本排除（静态口径，0–16 Nm 负载域）**。证据三角：H1 源码审计（合同干净、无前馈、全窗口恒定）+ H2 静态自洽（0.119–0.134 Nm RMS、无饱和/削顶/热漂移、前后腿仅 ±0.1–0.2 Nm 常数偏置）+ H3 悬空闭环复现（0.0096 rad RMS）。
2. 已固化的执行器误差预算（进所有后续误差分析）：静态 tau 通道 0.12 Nm RMS（P95 ≤0.35，瞬态 1.65）；per-joint 常数偏置 ±0.1–0.2 Nm；q 通道 <0.01 rad。
3. 已知 UNKNOWN（厂家清单 H1.6）：tau_est 算法内部、固件限幅/降额、动态带宽定量、>16 Nm 行为、电压降额。在拿到厂家资料或更高负载实验前，上述结论限定在已测负载域。
4. **残余 sim-real 缺口的归因进一步收窄**：G-A 复现的分关节缺口（前腿 hip q −0.131/tau −7.2、后腿 −0.112/−6.0 Nm）与 H2 执行器误差预算相差 20–60×，缺口必然来自接触/负载分配/参数层——交由 Phase I（PhysX 分支诊断）与 Phase J（根因对照）处理。

### 产物索引（Phase H）

| 文件 | 内容 |
|---|---|
| 本文 | H1 源码审计 + H2 日志重析 + H3 回放 + H4 结论 |
| `scripts/h2_torque_pd_analysis.py` + `tables/H2_TORQUE_PD_CONSISTENCY.{csv,json}`、`H2_SESSION_TAU_STATS.csv`、`H2_WHEEL_TAU_CHANNEL.csv` | 7 会话逐帧 tau_PD vs tau_est（84 关节行）+ 轮通道（24 行） |
| `scripts/h3_torque_replay.py` + `tables/H3_TORQUE_REPLAY.{csv,json}` | 7 组回放 run（S/G × A/B/C/D） |
