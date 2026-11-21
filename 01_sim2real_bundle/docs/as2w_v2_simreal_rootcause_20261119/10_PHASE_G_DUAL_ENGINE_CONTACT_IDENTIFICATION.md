# 10 Phase G：双引擎接触辨识、实机轮姿态 FK 与接触点算法审计

日期：2026-11-20/21
脚本：`runs/as2w_v2_simreal_rootcause_20261119/scripts/g2_wheel_pose_fk.py`、`g2b_contact_inversion_recheck.py`
数据：`tables/G2_WHEEL_POSE_FK.{csv,json}`、`tables/G2_CONTACT_RECHECK.{csv,json}`
约束遵守：无重训、无实机控制、无正式资产修改（FK/反演均只读，MJCF 用既有交付副本加载）。
G1（Go2-W 参考）见 `09_PHASE_G_GO2W_REFERENCE_AUDIT.md`，本文只含 G2–G4。

证据标签沿用：`RAW_DATA_CONFIRMED / SOURCE_CONFIRMED / SIM_RUN_CONFIRMED / MODEL_CALCULATION / INFERENCE / UNKNOWN`。

---

## G2. 实机轮姿态 FK 重建与接触点算法审计

### G2.1 方法

- 7 会话全部采用 Phase B/C 审定窗口（地面 OG (17,120)、P55G1 (10,30)、P55G2 (8,19)、P55G3 (7,19)；悬空 P55S (25,90)、P85DS (48,100)、OS (0,60)），窗口均值 q(12)+基座四元数经 C0 交付 MJCF 做完整运动学链 FK（hip→thigh→calf→foot，不用 hip q 近似轮倾角）。
- 轮心 c = xpos[foot] + R_foot@[0, ±0.0546, 0]（真轮心，含 54.6 mm 侧偏）；轮轴 a = R_foot@[0,1,0]；轮平面倾角 = asin(|a_z|)。
- 倾斜有限宽圆柱（r=0.086，半宽 0.026）最低点解析式：
  `p = c − w·sign(a_z)·a + r·(a·a_z − ẑ)/√(1−a_z²)`，`min_z = c_z − w·|a_z| − r·√(1−a_z²)`
  ——已用暴力曲面采样逐位验证（20 001 采样点，解析=暴力，`SIM_RUN_CONFIRMED`）。
- 基座 z 以最低轮点贴地归一（npz 无基座位置，间隙为相对量并已声明）。

### G2.2 发现 1：f1_audit_phaseC.py 的「tilt 感知接触点」存在几何方向 bug（本轮最重要发现）

- f1 tilt 点 = `xpos[foot] + R_body@[0, ±0.0546, −0.086]`。但 foot body 坐标系被 thigh+calf 绕 y 的关节角合计旋转了 **−67°…−86°**（P55G2 RL：thigh 0.704 + calf −1.875 ≈ −67°），所以 `R_body@[0,0,−0.086]` 指向世界系**近乎水平**（body z 世界向量 ≈ [−0.95, −0.12, +0.27]），落在轮缘前/后侧而非轮底。
- 正确解析点与 f1 tilt 点距离 **58–106 mm**（P55G2：前 92–94、后 106 mm），其中 z 向偏差 48–67 mm。`SIM_RUN_CONFIRMED`（`G2_WHEEL_POSE_FK.csv` `dist_analytic_vs_f1_mm`）。
- 结论：V2 复核文档 §4.3 的 tilt_C0/tilt_C1 行与 §2.2/§2.3 的推论（Fn 32/32/94/95、后轮离地 15–19 mm、μ_req 1.21）**全部是接触点方向 bug 的伪影**。已对该文档做勘误（见文末勘误记录）。
- 注意：54.6 mm 侧偏本身不受此 bug 影响（沿轮轴方向，Ry 旋转下不变），因此 Phase D 的 Δτ=2.1–4.0 Nm 结论方向保留，且由正确接触点重算得到印证（下）。

### G2.3 发现 2：正确接触几何下「四轮同时接地」成立、μ_req ≤0.4、Fn 回到 ~47/48/80/80 N

用正确解析接触点重跑 Phase C 反演（`g2b_contact_inversion_recheck.py`，18 方程非加权最小二乘，与 f1 同口径）：

| P55G2 接触点变体 | Fn (FL/FR/RL/RR) | Fy | 轮高 mm | μ_req | leg 残差 RMS |
|---|---|---|---|---|---|
| naive（旧口径） | 47.4/48.1/80.2/80.0 | +24.1/−24.2/+18.6/−18.5 | 1.5/5.5/0/0.5 | 0.51/0.50/0.23/0.23 | 1.48 Nm |
| f1_tilt（bug，=文档§4.3 tilt_C1） | 32.0/31.8/94.0/94.6 | +33.1/−33.6/+1.3/−0.9 | 0/6.1/18.0/18.6 | 1.13/1.21/0.19/0.20 | 3.98 Nm |
| **analytic（正确）** | **48.2/48.4/79.5/79.5** | +17.6/−17.8/+5.7/−5.5 | **0/4.3/1.5/2.1** | **0.37/0.37/0.07/0.07** | **1.23 Nm** |
| analytic dy±5 mm | Fn 变 ≤0.15 N | Fy 变 ≤1.5 N | ±0.3 mm | — | ±0.03 Nm |
| analytic dz±5 mm | Fn 变 ≤0.1 N | Fy 变 ≤0.9 N | ±0.1 mm | — | ±0.01 Nm |

- **四轮离地间隙 0–4.3 mm**（P55G2；其余会话 0–8.8 mm）——实测位形下四轮同时接地与水平地面假设自洽，V2 §2.2「后轮离地 15–19 mm」推翻。`MODEL_CALCULATION`+`SIM_RUN_CONFIRMED`。
- **μ_req 最大 0.37（前轮）**：μ≤0.8 重新可行，V2 §2.3 的 1.21 是伪影。`MODEL_CALCULATION`。
- **正确接触点 leg 残差最小**（1.23 vs naive 1.48 vs buggy 3.98 Nm）——三口径中解析点最优，间接支持其正确性。
- 四个地面会话一致（P55G1 Fn 50.4/50.7/77.8/76.7 μ≤0.30；G3 49.6/49.4/78.3/78.2 μ≤0.37；OG 62.1/70.7/68.8/53.5 μ≤0.05）。Fn 对接触点模型的真实摆幅 ~1–4 N（naive↔analytic），不是 15 N；接触点 ±5 mm 扰动只动 Fn ≤0.15 N——Phase C 反演对接触点误差的真实敏感度远低于 buggy 口径的估计。
- **fixed-λ 轮心杠杆重算**（naive→analytic，P55G2）：hip Δτ = −2.30/+2.30/−4.99/+4.87 Nm，与理论 Δr_y×Fn（54.6 mm × 48–80 N = 2.6–4.4 Nm）一致且略大（含 Fn 重分布）。Phase D 结论（必要不充分、闭环放大到 ~7.6 Nm）保留并加强。`MODEL_CALCULATION`。

### G2.4 发现 3：实测轮轴倾角与 hip q 的定量关系（「不能拿 hip q 当倾角」的量化）

| 会话 | hip q (deg) | 轮平面倾角 (deg) | 差异 |
|---|---|---|---|
| P55G2 FL/RL | +12.3/+15.9 | +12.1/+15.7 | 0.2°（基座姿态抵消一部分） |
| OG FL/FR | −7.4/+6.9 | +8.3/+6.1 | 最大 0.9° |
| P55G2 全部 | ±12.1–15.9 | 12.1–15.8 | ≤0.5° |

轮轴 z 分量 a_z=0.209/0.215/0.271/0.273（P55G2）。在此姿态范围内轮倾角≈hip q（差 ≤0.9°，由基座 roll/pitch 贡献），但这是**数值巧合级**而非恒等式——OG 的 FL（hip −0.129）轮倾是 +8.3°，符号都不同。任何用 hip q 直接当倾角的脚本都会在 OG 类姿态出错。`MODEL_CALCULATION`。

### G2.5 悬空会话轮姿态（参考）

OS/P55S/P85DS 窗口轮倾 1.3–10.5°（hip q −0.16…+0.06），FK 已入 `G2_WHEEL_POSE_FK.csv`；悬空无地面约束，仅作链上几何参考。

### G2.6 对 Phase C 结论的修订汇总

| 原 V2 复核结论 | G2 修订 |
|---|---|
| Fn 对接触点摆幅 15 N / 30%（§2.1/§4.3） | 摆幅实为 ~1–4 N（正确口径）；15 N 是 tilt bug 伪影 |
| 后轮模型离地 15–19 mm、四轮同接不成立（§2.2） | **推翻**：正确间隙 0–4.3 mm，四轮同接成立 |
| tilt_C1 下 μ_req=1.21、μ≤0.8 不可行（§2.3） | **推翻**：μ_req ≤0.37（P55G2 前轮） |
| Phase C Fn 数值仅为「方向参考」 | Fn ≈47–50/77–80 N（前/后）反演方向+量级双双回升可信度（leg 残差最小口径） |
| Phase D Δτ=2.1–4.0 Nm | 保留（侧偏不受 bug 影响；正确口径重算 −2.3/+2.3/−5.0/+4.9） |

Fn/Fy 仍是 18 参数 LS 的**模型反演量**（非测量），其「待称重验证」地位不变；但已知的算法性误差源已消除两个（接触点方向、离地间隙假设）。

---

## G3. 双引擎 × 轮碰撞基元 × 初始条件矩阵

脚本：`scripts/g3_mujoco_matrix.py`、`g3_physx_matrix.py`、`g3_digest.py`；数据：`tables/G3_ENGINE_PRIMITIVE_MATRIX.{csv,json}`（36 行 = 2 引擎 × 3 场景 × 2 基元 × 3 初始条件）。

### G3.1 统一条件与方法修正

- 两引擎统一：轮 r 0.086、宽 52 mm、轮心侧偏 ±0.0546、轮距 0.44、质量/惯量统一为交付模型（base 10.08 kg / CoM 0.035002）、重力、PD 目标、初始高度（最低轮点离地 2 mm）。
- **修正 1（MuJoCo）**：原 `A_C1_base` 的视觉轮 mesh 同时可碰撞（f2 的“圆柱基线”实为圆柱+凸包 mesh 双碰撞）。本轮拆为 `M_cylwheel`（视觉 mesh 设 contype=0，仅圆柱）与 `M_meshwheel`（删圆柱，仅 STL mesh；MuJoCo 3.12 mesh 碰撞=凸包）。
- **修正 2（PhysX）**：IsaacLab 本构建中 `InitialStateCfg.joint_pos` 被静默忽略（腿从直腿开始）→ 改为 `sim.reset()` 后显式写关节位置，18 个 PhysX run 全部重跑。这是 PhysX 首次出现“初始条件敏感性”的原因。
- PhysX 圆柱碰撞由导入器生成真 USD cylinder（NewtonCollisionAPI），非 capsule/mesh 近似；PhysX 每轮接触点数 `UNKNOWN`（ContactSensor 只给合力）。
- 本矩阵中 PhysX 的 `bind_physics_material` 因 instanced prim 未绑定成功（日志明列），摩擦取默认 1.0；Phase E/f3b 已证静态对摩擦不敏感，不影响分支结论。

### G3.2 结果（G 场景 = 实测 P55 目标、kp55/kd2；GO = 高增益 kp200/kd4）

| 引擎 | 场景 | 基元 | init | hip τ (Nm) | hip q | pitch° | Fn (N) |
|---|---|---|---|---|---|---|---|
| MuJoCo | G | cyl | real | −4.61/4.64/−11.57/11.60 | 0.084/−0.084/0.210/−0.211 | −6.37 | 49.4/49.6/77.9/78.1 |
| MuJoCo | G | mesh | real | −5.59/5.62/−12.39/12.41 | 0.102/−0.102/0.225/−0.226 | −6.63 | 49.2/49.4/78.1/78.3 |
| MuJoCo | G | cyl | nearflat/crouch | −4.59…−4.54 | 0.083–0.084 | −6.38/−6.48 | 同量级 |
| MuJoCo | G | mesh | nearflat/crouch | −5.57…−5.53 | 0.101–0.102 | −6.67/−6.74 | 同量级 |
| MuJoCo | GO | cyl | real/nearflat | −6.25/6.28/−5.80/5.83 | −0.119…0.121 | +0.08 | 65.1/65.3/62.2/62.4 |
| MuJoCo | GO | mesh | real/nearflat | −5.10/5.13/−4.73/4.75 | −0.124…0.126 | +0.07 | 同量级 |
| PhysX | G | mesh | real | −9.84/10.29/−10.63/10.36 | 0.179/−0.187/0.193/−0.188 | **+1.91** | 60.3/52.5/67.3/76.9 |
| PhysX | G | mesh | nearflat | 2.12/0.79/−1.13/−2.00 | ≈0 | +2.07 | 61.5/53.5/69.8/84.4 |
| PhysX | G | mesh | crouch | 1.88/−2.26/−0.44/0.98 | ≈0 | +3.42 | 58.2/54.2/77.6/79.7 |
| PhysX | G | cyl | real | −7.49/7.79/−11.08/10.90 | 0.136/−0.142/0.201/−0.198 | **+4.01** | 53.3/52.6/74.0/75.7 |
| PhysX | G | cyl | nearflat/crouch | ≈0 | ≈0 | +1.81/+4.60 | — |
| PhysX | GO | mesh | real/nearflat/crouch | −5.2…+16.3 | −0.18…0.18 | −0.77/+0.09/+0.27 | — |
| PhysX | GO | cyl | real/nearflat/crouch | −4.1…+21.9 | −0.21…0.21 | −1.33/+0.03/+0.42 | — |

完整 36 行见 CSV。悬空 S 控制：两引擎 × 两基元 × 三 init 全部收敛到同一姿态 hip q=[−0.052, 0.053, −0.053, 0.053]、τ=±2.88–2.92 Nm，与实机 P55S 窗口均值一致 → **腿链/重力通道无问题，分歧只在地面接触**。`SIM_RUN_CONFIRMED`。

### G3.3 分支敏感性（同一配置三个初始条件的落点极差）

| 引擎 | 场景 | 基元 | pitch 极差 | hip q 极差 | 同一分支 |
|---|---|---|---|---|---|
| MuJoCo | G | cyl / mesh | 0.11° / 0.11° | 0.001 / 0.001 | **是** |
| MuJoCo | GO | cyl / mesh | 0.63° / 0.61° | 0.066 / 0.057 | 是（crouch 翻倒除外） |
| PhysX | G | cyl / mesh | **2.79° / 1.51°** | **0.210 / 0.228** | **否** |
| PhysX | GO | cyl / mesh | **1.75° / 1.04°** | **0.169 / 0.107** | **否** |

- MuJoCo：单一吸引子，初始条件几乎不影响落点（GO 的 crouch 是病态初值，仅 2 轮接地翻倒，`settle_ok=False`）。
- PhysX：**多个近平平衡分支**，初始条件决定落点；G mesh 的 real 落 +1.91°、nearflat +2.07°、crouch +3.42°，hip 力矩从 ±10 变到 ≈0。`SIM_RUN_CONFIRMED`。

### G3.4 引擎差（同基元/场景/init 的 MuJoCo−PhysX pitch）

- G 场景：cyl real −10.4°、nearflat −8.2°、crouch −11.1°；mesh real −8.5°、nearflat −8.7°、crouch −10.2°。
- GO 场景：差 ≤1.4°（两者都在近平分支）。
- **基元本身不换分支**：MuJoCo 圆柱/mesh 都落在实测族分支（−6.4°），mesh 只把 pitch 再压 0.26–0.30°、hip q +0.02；PhysX 圆柱/mesh 都停在近平正 pitch（+1.8…+4.6°），互差 1–2°，从未到 −6.1°。
- **接触点数**：MuJoCo 稳定浅穿透下圆柱与凸包 mesh 每轮均 1 个接触点；深穿透探针圆柱 2 点、mesh 1 点。mesh 接触点比圆柱更靠外侧（G real FL y 0.248 vs 0.225、RL 0.273 vs 0.262），因为 STL 凸包最低点在轮缘外沿。`SIM_RUN_CONFIRMED`。

### G3.5 G3 结论

1. **轮碰撞基元是二阶效应**，不能把任一引擎推到另一分支：MuJoCo 内 mesh vs 圆柱 ≤1 Nm / 0.3°；PhysX 内 ≤2°。
2. **分支由引擎（及其求解器/接触流形）决定，不由基元决定**：MuJoCo 唯一分支≈实测；PhysX 多分支全在近平侧。
3. **初始条件只在 PhysX 选择分支**（极差 2.8°/0.23 rad），MuJoCo 无此敏感性。
4. 因此“PhysX 复现不了实机”的剩余原因不是轮几何基元，而是 **PhysX 的接触流形/求解器把系统稳定在近平平衡分支**；MuJoCo 的 G 场景（kp55）已能落在实测族分支（pitch −6.4° vs 实测 −6.13°，后腿 hip q 0.210–0.226 vs 实测 0.278/0.276 略低，前腿 0.084–0.102 vs 实测 0.214/0.216 明显低）。
5. **残余力矩缺口仍在**：即便最接近的 MuJoCo mesh，hip τ=−5.6/+5.6/−12.4/+12.4，实测 −11.8/+11.7/−15.2/+15.2，前腿差 6.2 Nm、后腿差 2.8 Nm；PhysX 最好（G mesh real）前 9.8、后 10.6，量级更接近但 pitch 分支错。**没有任何“引擎×基元×init”组合同时复现 pitch 与四个 hip 力矩**（`SIM_RUN_CONFIRMED`）。

---

## G5. 实机测量 SOP

见 `07_PHASE_G_REAL_MEASUREMENT_SOP.md`（11 项）与 `08_PHASE_G_MEASUREMENT_TEMPLATE.csv`。要点：两姿态（P55 T2 / OG）、四轮独立称重+CoM 倾斜拟合、接触间隙、轮轴倾角、轮心/轮距、轮胎宽径、接触斑、空载/加载压缩、实际关节角（非 q_des）、IMU 姿态、重复性；四平台共面 ≤±1 mm、备用支撑不承重、单秤不得分时直读（须声明初测）。

---

## G6. 终局九问直答

**Q1 两引擎的接触模型异同？**
相同：都用刚性轮+平面、库仑摩擦、单点/少点接触；静态下都对摩擦系数、condim、compliant 刚度、dt 不敏感（≤0.4 Nm，f2/f3/f3b）。
不同（`SOURCE_CONFIRMED`+`SIM_RUN_CONFIRMED`）：MuJoCo 用圆柱/凸包 mesh + solref/solimp 软约束、pyramidal 锥（交付 MJCF condim=4）；PhysX 用 STL 三角网格或导入圆柱 + contact_offset/compliant spring、默认摩擦 1.0。Go2-W 官方 MJCF 则用 condim=6+椭圆锥+impratio=100（G1）——AS2-W 交付 MJCF 未采用。
**关键差异不在参数而在接触流形与求解器**：同一套几何/质量/PD 下，MuJoCo 收敛到唯一实测族分支，PhysX 收敛到多个近平分支（G3）。

**Q2 可迁移的方法（Go2-W）？**
可迁移的是**结构性做法**，不是参数：①轮地接触按“轮接触”建模（condim=6 含扭转/滚转、椭圆锥、高 impratio）作为候选变体；②mesh 轮与圆柱轮必须区分口径（Go2-W 官方自身两套表示宽差 7 mm）；③轮力矩低上限+速度型驱动（与 AS2-W 一致）。不可迁移：所有 PD/DR/摩擦数值（质量 19.3 vs 26 kg、轮距 0.386 vs 0.44）。DR 范围与 3 个 GitHub 训练仓库/arXiv 论文内容 `UNKNOWN`（企业网络不可达，本地无副本）。

**Q3 hip 姿态造成多大轮轴倾斜？**
P55 T2 实测位形下轮平面倾角 FL/FR/RL/RR = 12.1/12.4/15.7/15.8°（a_z 0.209/−0.215/0.271/−0.273），与 hip q（12.3/12.3/15.9/15.8°）差 ≤0.5°；OG 位形差最大 0.9° 且 FL 符号不同（hip −7.4° vs 轮倾 +8.3°）。结论：**本组姿态内轮倾≈hip q 是数值巧合**，不能当恒等式；G2 已用完整 FK 取代该近似。`MODEL_CALCULATION`。

**Q4 Phase C 接触点误差源？**
主误差源是 **f1 的 tilt 接触点方向 bug**（foot body 被 thigh+calf 旋转 −67°…−86°，`R@[0,0,−0.086]` 指向轮缘前后而非轮底，偏差 58–106 mm）。修正后：四轮离地 0–4.3 mm（四轮同接成立）、μ_req ≤0.37（非 1.21）、Fn 48.2/48.4/79.5/79.5 N、leg 残差三口径最小（1.23 Nm）。剩余误差源：单点接触假设（真实轮有接触斑/多接触点）、轮胎柔顺 `UNKNOWN`、基座无位置（间隙为相对量）、真实 μ `UNKNOWN`。`MODEL_CALCULATION`+`SIM_RUN_CONFIRMED`。

**Q5 引擎差异来源（几何/流形/求解器）？**
排序：**求解器/接触流形（主）> 几何基元（次）> 参数（可忽略）**。证据：同一几何/质量/PD，MuJoCo 唯一分支≈实测、PhysX 多近平分支（G3）；圆柱↔mesh 在两引擎内都只动 ≤1–2°/≤1 Nm；摩擦/刚度/dt/condim 静态 ≤0.4 Nm（Phase E）。PhysX 唯一能移动分支的已知旋钮是 contact_offset 0.001（0.65 Nm，仍不够）。`SIM_RUN_CONFIRMED`。

**Q6 同一套物理参数能否双引擎复现实机？**
**不能**。以交付质量/CoM/几何/PD 为同一套 θ_physical：MuJoCo G 复现 pitch（−6.4 vs −6.13°）但前腿 hip q/力矩偏低；PhysX 任何基元/init 都不复现 pitch 分支（+1.8…+4.6°）。没有任何组合同时复现 pitch 与四个 hip 力矩。因此“同一套参数双引擎复现”在当前求解器差异下不成立；需先解决 PhysX 分支选择（或明确以 MuJoCo 为辨识基准、PhysX 只做训练环境并单独标定）。`SIM_RUN_CONFIRMED`。

**Q7 哪些参数须等实机？**
必须实测才能定：整备质量（26.0 vs 27.5 kg）、base CoM（0.035 vs 0.020 m）、真实轮胎 μ、轮胎垂向柔顺/压缩、接触斑尺寸、轮轴实际倾角与轮心位置（验证 FK）、hip 机械零位/传动摩擦（仅需工作载荷下的附加摩擦，空载扫角无信息）。可由仿真/反演先定的：轮几何标称值、PD 合同、重力项。`INFERENCE`+`UNKNOWN`。

**Q8 是否具备修改正式资产的条件？**
**不具备**。质量/CoM 未称重、轮胎柔顺/μ 未测、PhysX 分支未定；此时改正式 URDF/MJCF/Isaac 资产会把未验证假设固化。前置条件：G5 的 M2（四轮称重+CoM）与 M3–M8（轮几何/接触斑/压缩）完成，且 G3 的 PhysX 分支问题有处置结论。当前只允许 candidate 副本（本轮全部在 runs/ 下）。`INFERENCE`。

**Q9 是否需要下轮 RL？**
**否（暂不启动）**。理由：①接触模型尚未定（PhysX 分支错、轮胎柔顺/μ `UNKNOWN`），重训会继承错误接触；②G2 表明连“接触点在哪”此前都算错了，此时重训的收益无法归因；③MuJoCo 已能复现实测族分支，说明问题在接触/求解器而非策略容量。建议顺序：实机 M2–M8 → 用实测 θ_physical 重跑 G3/G4 → 再决定是否重训。`INFERENCE`。

---

## 附：Phase G 产物索引

| 文件 | 内容 |
|---|---|
| `09_PHASE_G_GO2W_REFERENCE_AUDIT.md` + `tables/GO2W_AS2W_COMPARISON.csv` | G1 Go2-W 审计（29 行对照） |
| `scripts/g2_wheel_pose_fk.py` + `tables/G2_WHEEL_POSE_FK.{csv,json}` | G2 七会话轮姿态 FK、倾角、间隙、接触点三口径差 |
| `scripts/g2b_contact_inversion_recheck.py` + `tables/G2_CONTACT_RECHECK.{csv,json}` | G2 修正接触点的 Phase C 反演重算 |
| `scripts/g3_mujoco_matrix.py` / `g3_physx_matrix.py` / `g3_digest.py` + `tables/G3_ENGINE_PRIMITIVE_MATRIX.{csv,json}` | G3 36 行双引擎矩阵 |
| `07_PHASE_G_REAL_MEASUREMENT_SOP.md` + `08_PHASE_G_MEASUREMENT_TEMPLATE.csv` | G5 实机 SOP 与模板 |
| 本文 | G2/G3/G4/G6 汇总与九问 |

---

## G4. 参数分层与辨识顺序（敏感性分析；无称重数据，不宣布辨识完成）

### G4.1 三层参数定义

| 层 | 内容 | 两引擎共享 | 已知敏感度 |
|---|---|---|---|
| **θ_physical** | 轮几何（r 0.086 / 宽 52 mm / 侧偏 54.6 mm / 轮距 0.44）、质量、CoM、轮胎柔顺（UNKNOWN）、真实 μ（UNKNOWN） | 是 | 侧偏 54.6 mm → hip Δτ 2.3–5.0 Nm（G2 §G2.3）；质量/CoM（load-v0）→ MuJoCo 6.0 Nm（Phase E）；轮胎柔顺 UNKNOWN |
| **θ_mujoco_solver** | solref/solimp、condim、cone、impratio、dt | 否 | solref → 2.0 Nm；condim/摩擦 → <0.2 Nm；dt → 0.3 Nm（Phase E 消融） |
| **θ_physx_solver** | contact_offset、compliant stiffness/damping、friction 材质、dt | 否 | contact_offset 0.001 → 0.65 Nm（分支移动）；摩擦/compliant/dt → ≤0.3 Nm（Phase E+f3b） |

### G4.2 辨识顺序（按敏感度×可测性降序）

1. **接触几何**（侧偏/轮倾/轮心）：敏感度最大（2.3–5.0 Nm/腿），且 SOP M3–M6 可低成本实测 → 先行。
2. **质量/CoM**：6.0 Nm（load-v0 口径），四轮称重（M2）直接定 → 第二。
3. **轮胎柔顺**：UNKNOWN 占位（压缩/接触斑 M7–M8），量级待测；>5 mm 压缩才值得进模型。
4. **接触本构/求解器**：两引擎各自兜底项，敏感度 ≤2.0 Nm（solref）≤0.65 Nm（PhysX offset），在前三层定之前调它们=为匹配而调参，禁止。G3 已证轮基元（圆柱/mesh）是二阶效应（≤1–2°、≤1 Nm），真正决定分支的是 PhysX 接触流形/求解器（多分支、init 敏感）——这一层应作为“环境标定项”单列，不能与 θ_physical 混调。
5. **θ_mujoco_solver 内参考项**：Go2-W 官方 MJCF 用 condim=6+elliptic cone+impratio=100（G1），作为候选变体测试，非默认改动；Phase E 已证 condim 静态不敏感（<0.4 Nm），预期贡献小、但椭圆锥未测（UNKNOWN，G3 后若仍有缺口再测）。

### G4.3 多指标归一化误差函数（对拍目标定义）

对每个候选 θ 组合、每场景 e（S/G/GO），双引擎分别得静态平衡解，与实机窗口均值比对归一化误差：

`E(θ) = Σ_e [ w_τ·RMS(τ_hip−τ̂_hip)/τ̂_scale + w_q·RMS(q−q̂)/q̂_scale + w_p·|pitch−p̂|/p̂_scale + w_F·RMS(Fn−F̂n)/F̂_scale ]`

- 实机锚点：P55G2 窗口 τ̂_hip=−11.81/+11.70/−15.15/+15.19、q̂（G2 表）、pitcĥ=−6.13°、F̂n=48.2/48.4/79.5/79.5（G2 反演，权 w_F 低——反演量非测量）。
- 比例：w_F < w_τ = w_q（Fn 是反演量）；S/G/GO 同权。
- **在拿到称重（M2）前，只报告敏感性（各层参数对 E 的偏导量级），不宣布任何 θ 辨识完成**——G2 已示范：接触点口径一错，Fn/μ_req/离地全部翻面。

### G4.4 当前已知敏感性汇总表（证据标签）

| 参数（层） | 引擎 | 对 hip 静态力矩敏感度 | 证据 |
|---|---|---|---|
| 轮心侧偏 54.6 mm（θ_physical） | 双 | fixed-λ 2.3/2.3/5.0/4.9 Nm（P55G2） | `MODEL_CALCULATION`（G2 §G2.3） |
| 接触点 ±5 mm dy/dz（θ_physical） | 双 | Fn ≤0.15 N、Fy ≤1.5 N、leg 残差 ≤0.03 Nm | `MODEL_CALCULATION`（g2b） |
| load-v0 质量/CoM（θ_physical） | MuJoCo | 6.0 Nm、pitch −12° | `SIM_RUN_CONFIRMED`（Phase E）；PhysX ≤0.15 Nm（分支不同） |
| solref 硬度（θ_mujoco_solver） | MuJoCo | 2.0 Nm | `SIM_RUN_CONFIRMED`（f2） |
| contact_offset 0.001（θ_physx_solver） | PhysX | 0.65 Nm（pitch +1.4→−0.78°） | `SIM_RUN_CONFIRMED`（f3） |
| 摩擦系数/compliant 刚度/condim/dt | 双 | ≤0.4 Nm（静态） | `SIM_RUN_CONFIRMED`（f2/f3/f3b） |
| 轮碰撞基元 圆柱 vs mesh | 双 | MuJoCo ≤1 Nm / 0.3°；PhysX ≤2°（不换分支） | `SIM_RUN_CONFIRMED`（G3） |
| 初始条件→平衡分支 | 双 | MuJoCo 无敏感（极差 0.11°）；PhysX 极差 2.8°/0.23 rad（多分支） | `SIM_RUN_CONFIRMED`（G3） |


## 勘误记录

- 2026-11-21：发现 `f1_audit_phaseC.py` tilt 接触点方向 bug（§G2.2），据此修订《AS2W_悬空负载诊断V2_PhaseA-F复核与下一步计划.md》§2.2/§2.3/§4.3 相关表述（详见该文档头部勘误框）。
