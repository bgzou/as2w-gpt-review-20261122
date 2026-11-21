# H0：源数据与控制合同确认

- 日期：2026-11-19
- 主机：`robot-motion`；仓库：`/home/robot/z00945176/Rob2Nav`，git HEAD `97488b10ffbb5898ca6607ffd7c33941ed610607`（main）
- 证据标记：本页所有"已核验"条目均为 `RAW_DATA_CONFIRMED`（直接从文件/SHA/npz 读出）或 `MODEL_CALCULATION`；无臆测。

## 1. 实机数据源

### 1.1 主数据包（已在本服务器定位并校验）

| 项 | 值 |
|---|---|
| 路径 | `runs/as2w-r4000-realcal-wheelrl-20261114/data/SimDynamicsValidation_AS2W_20261114.zip` |
| SHA256 | `1ed1ed5e82b0308f31547f4227efd9a368b7acefcdae0e412964426b81753e3a`——**与任务书逐字一致** `RAW_DATA_CONFIRMED` |
| 已解压 | 同目录 `extracted/SimDynamicsValidation_AS2W_20261114/`（20 clip + raw + tools + docs） |
| clip 列表 | 3×`*_fixedstand_hold`、3×`*_fixsquat`、3×`*_fixstand_rise`、3×`*_brake_return`、3×`*_policy_walk`/walk 四分段、`*_handoff_zero`、`*_prone_passive`、`factory_stand`——与任务书 §H1.1 建议清单全覆盖 |
| 复用先行工作 | 同 run 目录已有 `REAL_ZIP_MANIFEST_AND_QC.md`（QC 通过：20/20 完整、500Hz 一致、无 NaN）与 `ACTUATOR_SEMANTICS_AND_UNIT_AUDIT.md`——本任务复用其结论并抽查关键样本 |

**四髋逐帧复核包（`AS2W_四髋PD实机原始数据复核_20261119.zip` / `AS2W_HIP_PD_ALL_CLIPS.csv`）在本服务器未找到，标记 `SOURCE_UNAVAILABLE`。** 但该包只是对同一 clip.npz 的四髋子集复核；本任务直接从 clip.npz 全量重算（更完整，含 12 腿关节），不依赖该 CSV，不伪造"已读过"。

### 1.2 clip 合同（从包内 docs/02 与 npz 实读，`RAW_DATA_CONFIRMED`）

- 16 轴 contract 序（**part-major**）：`0-3 hip(FL,FR,RL,RR)、4-7 thigh、8-11 calf、12-15 wheel`；`joint_names` 逐项映射为准，不假设 0..15 与 Isaac/MuJoCo 一致。
- 符号：`*`（无后缀）字段已按模型坐标统一（`model = sign*raw`）；hip `sign=-1`，其余 +1。**分析不再翻转**。
- 腿＝位置模式（`cmd_q/kp/kd`，`cmd_tau=0`：无前馈）；轮＝速度模式（`cmd_q=0, kp=0, kd=2, cmd_dq` 为目标）。
- 采样：500 Hz（factory_stand 100 Hz）；`st_offset_ms = st_tick − cmd_tick`，最大 ±7 ms、99% ≤1 ms，**这是采样对齐误差，不是执行器延迟**。
- `cmd_tick` 可能重复，积分/求导一律用 `cmd_t_s`。
- `st_tau` 是**驱动器估计力矩**，非独立力传感；quat 为 w,x,y,z body→world。
- 状态枚举：Passive/FixStand/FixedStand/PolicyHandoff/PolicyWalk/Brake/ReturnStand/FixSquat/EmergencyDamping；`*_brake_return` 后的 FixedStand 目标是冻结帧，不能当 T2 静态测试。
- 首行可能是状态切换边界帧，纯段分析丢首行。
- **关键合同陷阱（docs/02 §2.3）**：行走段 hip `cmd_q` 自带 ±0.26~0.38 rad 准静态偏置、实测在 0 附近摆动——同刻可能同号也可能反号，是真实控制偏置，**不是符号错误**。`factory_stand` 实测髋 +0.13/−0.12/+0.12/−0.13 印证 sign=−1 正确。

### 1.3 已知实机基线（复核报告，供 H1 复算对照）

- 静态 T2 保持：pitch −6.0~−6.5°、后小腿误差 ~0.30 rad、`st_tau` 16–17 Nm（三会话 std 0.016 rad——稳定可复现）；`factory_stand`：pitch +0.33°、后小腿 `st_tau` 5–7 Nm。
- 数据盲区：**无基座线速度、无基座高度、无足端/接触力**——这三项是 H3/H4 的推断项，不得当作已测。
- PD 重建（四髋）：`tau_PD = kp*(cmd_q−st_q)+kd*(cmd_dq−st_dq)+cmd_tau`，FixedStand MAE ~0.090 Nm、PolicyWalk MAE ~0.095 Nm——控制一致性已验证，本任务抽验。

## 2. 仿真资产

### 2.1 PhysX / Isaac Lab（当前训练链）

| 项 | 值 |
|---|---|
| USD | `training/as2w-backup/as2w-current/runs/r280-isaac-flat-preflight/kit3-as2w.usd/as2w/as2w.usda` |
| URDF | `/home/robot/z00945176/as2w_recovery/assets/as2w/urdf/as2w.urdf`（sha256 `05da6240…7d2c`） |
| 执行器 | 腿 ImplicitActuator kp55/kd2（hip/thigh/calf 同 kp），轮 foot kp0/kd2，effort/velocity 限幅取 URDF |
| 关节限位 | hip ±0.785（effort 60）、thigh 前(−2,3.492)/后(−1.3,4.19)、calf(−2.7677,−0.8133)（effort 90）、轮力矩限 15 |
| profile=baseline | 无 DR；combined/expanded 才有质量(0.95,1.05)/增益(0.9,1.1)/摩擦(0.8,1.2)随机化 |

### 2.2 MuJoCo（同源 recovery 资产）

| 项 | 值 |
|---|---|
| 主 MJCF | `deliveries/as2w-t15-layered-delivery-20261117/simulation/mujoco/as2w_scene.xml`（sha `89ea64da…6950`，model=as2w_recovery，mesh STL 车轮+显式 cylinder 碰撞） |
| 备用 | `training/as2w-backup/as2w-current/assets/local-step-diagnostic/as2w_flat.xml`（model=AS2W，17 body） |
| 轮碰撞 | `type="cylinder" size="0.086 0.026"`（半径 0.086 m、半宽 0.026 m，轴线 Y）；轮体质量 0.595 kg |
| 质量 | base 10.08 + 4×(hip 0.75 + thigh 1.41 + calf 1.1 + wheel 0.595) = **25.50 kg**；base COM 前移 +0.035 m |
| 摩擦 | 轮/地 `friction="1.0 0.05 0.001"`、condim=4；关节默认 damping 0.1/armature 0.01/frictionloss 0.05 |
| 引擎 | mujoco 3.12.0（`isaac_env/venv312`），无独立 GPU 占用需求 |
| PhysX 对照 | PhysX 轮凸包/STL vs MuJoCo 圆柱支撑——历史报告线索，H5 用同命令量化 |

### 2.3 Python 环境

- 分析（纯 numpy/scipy/matplotlib，无 pandas）：`/home/robot/miniconda3/bin/python3`
- Isaac/PhysX：`isaac_env/venv312`（经既有 `run_isaac.sh` 包装，GPU 5090）
- MuJoCo：`isaac_env/venv312`（mujoco 3.12.0）

## 3. 可比性等级判定

| 数据 | vs 当前链 | 等级 | 理由 |
|---|---|---|---|
| `*_fixedstand_hold` / `*_fixstand_rise` / `*_fixsquat`（FixStand 家族） | PhysX 仿真 | **MATCHED**（物理/控制合同层面） | 腿位置模式 55/2 + 轮速度模式与 Isaac actuator 合同一致；静态基座速度≈0 盲区假设成立 |
| `*_prone_passive` | — | **PARTIALLY_MATCHED** | kp=0 只能验证重力/摩擦静力学方向，不能推外部载荷为零 |
| `112630_policy_walk_30s`、`114335_walk_00~120` | 当前策略 | **DIFFERENT_CONTRACT** | R4000 测试链 16D 腿轮控制；当前训练为 12D 腿 Actor+固定轮映射。只作"同命令重放动力学"用途，不作当前策略同步实机 rollout |
| `factory_stand` | — | PARTIALLY_MATCHED | 仅姿态/力矩参考（T1），无 LowCmd |
| `*_brake_return` 尾段 FixedStand | — | DIFFERENT_CONTRACT | 目标为冻结策略帧，非 T2 |

## 4. H0 结论

1. 主数据包本机在位、SHA 匹配、20 clip 全、QC 先行报告可复用；四髋复核 zip 缺失（`SOURCE_UNAVAILABLE`），由 clip.npz 全量重算替代。
2. clip 合同逐项读出，最大风险点（行走 hip 命令偏置、brake_return 冻结帧、st_offset 语义）已登记。
3. PhysX 与 MuJoCo 同源资产均在位且参数齐全，可支撑 H2/H3/H5 的双引擎实验。
4. 可比性：FixStand 家族 MATCHED（本诊断主对象），行走段 DIFFERENT_CONTRACT（限用开环重放）。

下一步：H1 全关节实机证据分析（不依赖 MuJoCo/PhysX，立即执行）。
