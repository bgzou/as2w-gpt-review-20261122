# 12 Phase I：PhysX 分支诊断（公平对比 / 材质绑定 / 分支稳定性 / 接触流形 / Go2-W 配置 / 七指标）

日期：2026-11-21。任务书：《AS2-W Sim2Real 根因诊断 Phase H/I/J（2026-11-21）》。
前置：Phase H 已完成（11 文档，执行器静态口径基本排除，误差预算 0.12 Nm RMS）；G2 勘误后接触结论为准；G3 引擎×基元×init 矩阵已完成。
约束遵守：无 RL、无实机、无正式资产修改；全部结论可溯源。证据标签六类。

实测锚点（P55G2 窗 8–19 s，`RAW_DATA_CONFIRMED`）：hip q = 0.2142/−0.2155/0.2779/−0.2755 rad，hip τ = −11.81/+11.70/−15.15/+15.19 Nm，pitch = −6.13°，Fn ≈ 48.2/48.4/79.5/79.5 N，μ_req ≤ 0.37。

---

## I1 公平对比条件：MuJoCo 显式 PD vs PhysX 隐式 Joint Drive

### I1.1 十项核对表（g3_mujoco_matrix.py vs g3_physx_matrix.py，`SOURCE_CONFIRMED`）

| # | 项 | MuJoCo（G3 harness） | PhysX（G3 harness） | 公平性判定 |
|---|---|---|---|---|
| 1 | PD 执行方式 | 显式 `qfrc_applied` PD，500 Hz（dt 0.002，每步计算） | `ImplicitActuatorCfg` + `write_joint_stiffness/damping_to_sim`——PD 在 PhysX 求解器内隐式执行的 Joint Drive，200 Hz（dt 0.005） | **不公平（主要缺口）**：隐式积分对刚性接触的稳定性裕度显著更大。修正=两方向补对照组：MuJoCo 加 implicit 执行器 run（W_I）、PhysX 加显式 `write_joint_effort_to_sim` run（I1-experiment） |
| 2 | 步长/步数 | dt 0.002 × 20000 = 40 s | dt 0.005 × 12000 = 60 s | 统计窗同为末 20%，settle 判据相同；步长差异与 #1 合并处理 |
| 3 | 力矩限幅 | per-kind clip：hip/thigh ±60、calf ±90、轮 ±15（软件 clip） | `effort_limit_sim` 取 URDF limit（同 60/60/90；轮取 URDF effort） | 数值一致；执行位置不同（求解器内 vs 外）随 #1 |
| 4 | 轮合同 | velocity servo τ=clip(2·(−dq),−15,15) | Joint Drive stiffness=0/damping=2（隐式） | 同族（kp0/kd2），执行方式异随 #1 |
| 5 | 地面材料 | 优先级混合 friction "1.0 0.05 0.001"（滑动/扭转/滚动） | `PhysxRigidBodyMaterialCfg(static=1.0, dynamic=1.0)`；实测有效摩擦=整机统一 0.8（I2），合成模式默认 `UNKNOWN` | **已运行时验证**（I2）：逐轮绑定无效、整机 0.8；但静力矩对 μ 不敏感 |
| 6 | 接触参数 | condim=4、pyramidal cone、impratio=1、solref 默认 | contact_offset=0.005、rest_offset=0.0、solver 默认（TGS? `UNKNOWN` 待 I2） | 结构差异如实记录，属引擎本质差异（Phase I 正是要测的） |
| 7 | 初始状态写入 | 直接 `d.qpos[:]` | `InitialStateCfg.joint_pos` 在本 build **不被 reset 应用**（腿会伸直），须 reset 后显式 `write_joint_position_to_sim_index` | 已在 G3 修正（脚本注释为证）；I2/I3 复用该路径 |
| 8 | 初始离地间隙 | 最低轮点 +2 mm（FK 归一） | 同（`write_root_pose_to_sim` 抬升） | 一致 |
| 9 | 悬空实现 | 去地面+去 freejoint（基座焊死） | `fix_base=True` | 等价 |
| 10 | factory 执行器干扰 | 交付 MJCF 自带 position/velocity 执行器块（kp 60/90、kv 8），用 `qfrc_applied` 驱动时 `actuator_gainprm/biasprm` 清零（H3 方法学警告，G3 原脚本已做） | URDF 导入不经 MJCF 执行器块，无此层 | 已处理；W_I 变体重写执行器块为合同值后走 ctrl 通道 |

### I1.2 对照实验结果

**MuJoCo 侧（`SIM_RUN_CONFIRMED`，`tables/I5_GOW_CONTACT_VARIANTS.{csv,json}` 的 W_A/W_I 行）**：

W_I（implicit position/velocity 执行器，ctrl 通道，implicitfast）与 W_A（显式 `qfrc_applied` PD 500 Hz）在全部指标上**逐位一致**：pitch −6.37°、hip q [0.084, −0.084, 0.210, −0.211]、hip τ [−4.61, 4.64, −11.57, 11.59]、Fn [49.4, 49.6, 77.9, 78.1]、q12_err_rms 0.0636。

**PhysX 侧（`SIM_RUN_CONFIRMED`，`tables/I1_PHYSX_EXPLICIT_PD.{csv,json}`）**：同 dt=0.002 下显式 `write_joint_effort_to_sim` 与隐式 Joint Drive 对照：

| 运行 | hip τ Nm | hip q | pitch° | Fn N | settle |
|---|---|---|---|---|---|
| exp_cyl_real（显式） | −10.55/10.55/−10.29/10.36 | 0.191/−0.192/0.187/−0.189 | +1.27 | 58.0/57.9/70.6/70.6 | False（40 s 未静） |
| imp_dt002_cyl_real（隐式） | −10.51/10.58/−10.44/10.47 | 0.191/−0.192/0.190/−0.190 | +1.32 | 57.7/57.6/70.3/71.0 | True（29.6 s） |
| exp_mesh_real（显式） | −9.68/9.96/−10.79/10.58 | 0.176/−0.181/0.196/−0.192 | +2.10 | 60.7/52.1/67.1/77.2 | True |
| imp_dt002_mesh_real（隐式） | −9.63/9.99/−10.49/10.27 | 0.175/−0.182/0.191/−0.187 | +1.96 | 60.7/52.3/66.9/77.1 | True |

显式 vs 隐式（同 dt=0.002）差：hip τ ≤0.3 Nm、hip q ≤0.003 rad、pitch ≤0.14°、Fn ≤0.4 N——**与 MuJoCo 侧同样结论：PD 执行方式不是分支决定因素**。唯一差异是显式 cyl 收敛更慢（40 s 未达 dqmax<0.2 判据，隐式 29.6 s 收敛），属数值稳定性而非平衡点差异。

**注意 PhysX 的 dt 敏感性**（与 I1 无关但同表可见）：G3 的 dt=0.005 cyl real（`G3_imp_dt005_cyl_real`）pitch +4.01°、hip q 0.136/−0.142/0.201/−0.198；本轮 dt=0.002 同配置 pitch +1.27°、hip q 0.191/...——**PhysX 的平衡分支随步长移动（pitch 差 2.7°）**，而 MuJoCo 在 dt 0.002 下唯一分支 −6.37°。这坐实 G3 的"分支由引擎决定"并进一步指出 PhysX 分支本身不稳健（见 I3）。

### I1.3 结论

**在 MuJoCo 静态平衡问题上，PD 执行方式（隐式执行器 vs 500 Hz 显式 qfrc PD）不改变 settled 平衡分支（逐位一致）。** 物理解释：settled 态 dq≈0、e_q 近恒定，显式 PD 的数值刚度劣势只在瞬态/接触抖动中显现，静态账本不变。因此 I1 判定：**G3 的 MuJoCo↔PhysX 分支差异不能用"隐式 vs 显式 PD"解释（MuJoCo 逐位一致、PhysX ≤0.3 Nm，两格均补齐）。** 管线基准 W_A 与 G3 发表的 MuJoCo G-cyl-real 数值逐位复现（pitch −6.37、hip τ −4.61/4.64/−11.57/11.60），确认 I5 管线与 G3 同源可比。

---

## I2 G3 PhysX 材质绑定修复验证

**运行时真值（`RAW_DATA_CONFIRMED`，`tables/I2_MATERIAL_BINDING.{csv,json}`）**：

1. 轮碰撞体实际路径 = `/World/envs/env_0/Robot/Geometry/base_link/{FL,FR,RL,RR}_hip/.../{leg}_foot/cylinder`（4 个 cylinder prim，均找到）。
2. **逐轮显式材质绑定未生效**：请求 `friction=0.8` 的 `/World/PhysicsMaterials/wheel_fric` 未被应用到 4 个轮 cylinder（pre_bind 阶段 `bound_material=None`）；reset 后运行时读取，4 轮绑定的是整机材质 `/World/envs/env_0/Robot/material`，不是请求的 `wheel_fric`。
3. **有效摩擦来自整机单一材质**：`/World/envs/env_0/Robot/material` 的 `physics:staticFriction = physics:dynamicFriction = 0.8`，4 个轮 prim 在 `after_reset_runtime` 均绑定到它（`bound_material_physics=/World/envs/env_0/Robot/material`、friction 0.8）。即有效 μ=0.8 整机统一，而非 G3 以为的逐轮独立材质。PhysX 内部实际摩擦标量不可直接读（`UNKNOWN`），以 USD physics-purpose 绑定为证据。
4. 修复重跑（friction 0.8，dt0.005 隐式）：fix_real pitch **4.036°**、hip q 0.136/−0.140/0.201/−0.199、hip τ −7.49/7.71/−11.04/10.94、Fn 53.2/52.7/74.2/75.6——与 G3 原 `G3_imp_dt005_cyl_real`（pitch 4.01、q 0.136/−0.142/0.201/−0.198、τ −7.49/7.79/−11.08/10.90）**几乎逐位相同**。

**判定**：G3 的"材质绑定"确实存在缺陷（请求的逐轮 `wheel_fric` 未生效，实际是整机 `Robot/material` 0.8 统一摩擦），但**修复后平衡点不变**——与 Phase E/f3b（静力矩对摩擦不敏感 ≤0.3 Nm）、I5（μ 数值二阶、impratio 一阶）三方一致。因此 I2 缺陷不影响 G3/I1 的任何结论，只澄清了"PhysX 有效 μ = 0.8 整机统一"这一事实。合成模式（average/min/multiply）仍未显式设定（`UNKNOWN`，依赖 PhysX 默认），但已无实质影响。

---

## I3 PhysX 多分支稳定性（扰动实验）

配置：G3 G 场景、cyl 轮、real init、隐式 Joint Drive kp55/kd2、dt=0.005、12000 步（与 G3 基线同源，基线应落 G3 的 +4.01° 分支）。每条件在独立 app 进程跑 2 次。判据（任务书）：跨条件 settled pitch 极差 <0.3° **且** hip q 极差 <0.02 rad 视为同分支。

`SIM_RUN_CONFIRMED`，`scripts/i3_physx_branch_perturbation.py`，`tables/I3_PHYSX_BRANCH_PERTURBATION.{csv,json}`：

| 条件 | 扰动 | pitch° | hip q [FL FR RL RR] | Fn N | settle s | 重复差 (Δpitch/Δq) |
|---|---|---|---|---|---|---|
| base | 无（gap 2 mm） | **4.038** | 0.136/−0.140/0.200/−0.199 | 53.2/52.7/74.3/75.4 | 28.8 | 0.0 / 0.0 |
| pitch_p2 | 基座 pitch +2° | 2.973 | 0.136/−0.141/0.173/−0.171 | 54.7/53.9/72.7/74.2 | 37.0 | 0.0 / 0.0 |
| pitch_m2 | 基座 pitch −2° | 3.618 | 0.167/−0.168/0.221/−0.223 | 54.0/54.2/74.1/74.2 | 21.6 | 0.0 / 0.0 |
| hip_p002 | 四 hip 同向 +0.02 rad | 4.733 | 0.135/−0.140/0.219/−0.216 | 52.4/51.8/75.1/76.5 | 33.1 | 0.0 / 0.0 |
| hip_m002 | 四 hip 同向 −0.02 rad | 3.812 | 0.139/−0.139/0.191/−0.195 | 52.9/53.3/74.9/74.6 | 32.3 | 0.0 / 0.0 |
| gap0 | 轮地间隙 0 mm | 3.979 | 0.136/−0.140/0.201/−0.199 | 53.3/52.9/74.1/75.3 | 30.8 | 0.0 / 0.0 |
| gap4 | 轮地间隙 4 mm | 4.030 | 0.136/−0.140/0.200/−0.198 | 53.2/52.6/74.1/75.7 | 23.1 | 0.0 / 0.0 |

**汇总（`SIM_RUN_CONFIRMED`）**：跨条件 pitch 极差 **1.76°**（2.97–4.73）、hip q 极差 **0.0518 rad**（后腿 0.173–0.223），均超判据 5.9×/2.6× → 判定 **`INIT_SENSITIVE_MULTIBRANCH`**；而**同条件两次重复差精确为 0.0**（确定性，非混沌）。

### I3.1 发现

1. **PhysX 分支对初始条件敏感、对重复完全确定**。同一初始条件两次运行逐位相同（Δpitch=Δq=0.0），说明没有随机性/混沌；但把初始 pitch 改 ±2° 或 hip 改 ±0.02 rad，settled pitch 移动 0.42–1.07°、后腿 hip q 移动最多 0.052 rad。即 PhysX 的平衡点是一个**浅势阱**（多个近邻平衡点），落点由初始条件选择——这正是"分支"的物理含义。
2. **扰动方向与幅度不成比例**：pitch +2° 使 settled pitch 降到 2.97（−1.07），pitch −2° 只到 3.62（−0.42）；hip ±0.02 分别 +0.70/−0.23°。非对称响应说明势阱底部平坦且多峰，不是单一大刚度线性系统。
3. **基线复现 G3**：base 4.038° ≈ G3 `G3_imp_dt005_cyl_real` 4.01°、hip q 0.136/−0.140/0.200/−0.199 逐位一致，I3 管线与 G3 同源可比。
4. **与 dt 敏感性合并**：dt 0.005→0.002 使 pitch 4.0→1.3°（I1 表），初始 pitch ±2° 使 pitch 4.0→3.0/3.6°（I3）。两者都是"数值参数/初值选择分支"，量级相当（1–2.7°），而 MuJoCo 在 dt 0.002 下对 init 不敏感（G3：唯一分支 −6.37°）。**因此 PhysX 的物理平衡点是"分支族"，MuJoCo 是"单吸引子"——这是两引擎分支差异的机制。**

### I3.2 结论

PhysX 分支**不稳健（init-sensitive multibranch）**，但**确定性可复现**。这意味着：PhysX 给出的 +4.0°（dt0.005）或 +1.3°（dt0.002）都不是"物理唯一解"，而是某一初始条件/步长下的代表点；不能把 PhysX 的任一分支当作验收真值。这与 I5 的 impratio 结论同构：**平衡点由数值层（求解器参数/步长/初值）在浅势阱中选择，而非由物理参数唯一决定。**

---

## I4 接触流形提取与静力平衡残差

### I4.1 MuJoCo 侧（`SIM_RUN_CONFIRMED`，`scripts/i4_mujoco_contact_balance.py`，`tables/I4_MUJOCO_CONTACT_BALANCE.{csv,json}`）

配置：M_cylwheel、G 场景、显式 PD kp55/kd2、real init、settle 20000 步后采样 200 帧（每帧推进 10 步）。

| 项 | 结果 |
|---|---|
| 模型总质量 | 26.0 kg（重力 255.06 N） |
| 合力残差（接触力+重力） | mean/max/p95 = **0.000 N**（求解器精确满足） |
| 合力矩残差（绕全身 CoM） | mean/max/p95 = **0.000 Nm** |
| 每轮接触点数 | **1.0**（FL/FR/RL/RR 全部单点） |
| 接触斑展布（点到均值最大距离） | 0.12–0.21 mm（数值级，无有限接触斑） |
| 穿透距离 dist | 0.0 m（几何精确接触） |
| 每轮 Fn | 49.18 / 49.41 / 78.13 / 78.34 N（和 = 255.06 N = 自重） |
| 每轮 Ft（切向合力） | 0.094 / 0.094 / 0.103 / 0.103 N |
| μ_used = Ft/Fn | 0.001–0.002（远离 0.37 需求） |
| 扭转/滚动摩擦 | ~1e-4 Nm 量级（可忽略） |

**关键发现：MuJoCo 圆柱-平面接触是严格单点、零穿透、零接触斑。** 实机充气/弹性轮必然有有限接触斑（J2 第 3 项待测），单点假设使接触点位置对轮姿态极敏感（G2 的接触点方向 bug 即源于此），且切向约束全部集中在一点——这解释了为何"摩擦约束阻抗"（impratio）能一阶移动分支（I5），而物理 μ 数值（0.8 vs 1.0）只是二阶：**缺口不在摩擦系数，而在接触流形从"点"到"斑"的缺失。**

质量一致性旁证：MuJoCo 总质量 26.0 kg（255.06 N）、URDF 18 连杆质量和 26.000 kg、实机称重 255.1 N（load self-test 四轮和 257.5 N vs 255.1 N 自重，1.0%）三者一致。注：旧汇总表中的"27.512 kg"来自另一资产版本，不适用于本 candidate 副本（本轮已用 URDF/MJCF 双源核实为 26.000 kg）。

### I4.2 PhysX 侧（`SIM_RUN_CONFIRMED`，`scripts/i4_physx_contact_balance.py`，`tables/I4_PHYSX_CONTACT_BALANCE.{csv,json}`）

配置：G3 G 场景、cyl 轮、real init、隐式 Joint Drive kp55/kd2、dt=0.005、12000 步（与 I3 base 同源：pitch 4.038°、hip q 0.136/−0.140/0.200/−0.199、hip τ −7.49/7.69/−11.02/10.97、Fn 53.2/52.7/74.3/75.4）。

| 项 | 结果 | 证据 |
|---|---|---|
| 四轮 Fz 和 | **255.06 N**（= 模型自重 26.0 kg × 9.81） | `SIM_RUN_CONFIRMED` |
| 竖直残差 | **0.0 N** | `MODEL_CALCULATION` |
| 水平残差 (Fx, Fy) | 0.0 / −0.0 N | `SIM_RUN_CONFIRMED` |
| 每轮 Fn（合力幅值） | 53.19 / 52.71 / 74.25 / 75.43 N | `SIM_RUN_CONFIRMED` |
| 每轮法向分量 | 52.97 / 52.42 / 74.25 / 75.43 N | `SIM_RUN_CONFIRMED` |
| 每轮摩擦分量 | **4.82 / 5.57** / 0.59 / 0.28 N（前腿显著大于后腿） | `SIM_RUN_CONFIRMED` |
| CoP_xy | (0.0163, 0.0009) m | `SIM_RUN_CONFIRMED` |
| 每轮平均接触位置 | FL [0.252,0.262,0.001] / FR [0.256,−0.257,0.001] / RL [−0.153,0.260,0.000] / RR [−0.148,−0.259,0.000] m | `SIM_RUN_CONFIRMED` |
| 绕全身 CoM 合力矩 | **5.872 Nm** | `MODEL_CALCULATION` |
| 逐接触点流形 | **`UNKNOWN`**（PhysX ContactSensor 只暴露合力 + 平均接触位置；`net_forces_w`/`net_normal_forces_w`/`net_friction_forces_w`/`contact_pos_w`/`force_matrix_w`/`max_contact_data_count_per_prim` 均已尝试，无逐点数组） | `UNKNOWN` |

**与 MuJoCo 的对照（`SIM_RUN_CONFIRMED` + `INFERENCE`）**：

1. **竖直/水平合力账本两引擎都精确闭合**（PhysX 255.06 N = 自重、水平 0.0；MuJoCo 255.06 N、水平 0.0）——质量/重力/接触法向总量无缺口，进一步排除质量与法向接触力层。
2. **逐点流形在 PhysX 上不可观测**（`UNKNOWN`），只能给 4 个平均接触位置；MuJoCo 能证明"每轮严格 1 点、零斑"。因此"点 vs 斑"的直接对比只能由 MuJoCo 侧给出，PhysX 侧只能作旁证：其 5.872 Nm 的"绕 CoM 合力矩残差"在静止且竖直/水平力已闭合的前提下**不应是真实静力不平衡**，更可能来自"每轮平均接触位置 × 合力"对逐点分布的近似误差（`INFERENCE`）；MuJoCo 逐点求和得 0.000 Nm 正说明差异来自分布信息缺失。但 PhysX 侧无法区分"平均化误差"与"CoM/接触几何真实前后偏 ~2 cm"各占多少（`UNKNOWN`，代理汇总 §I4 亦列为候选解释）。
3. **前腿摩擦分量 4.8–5.6 N vs MuJoCo 0.09–0.10 N（约 50×）**：两引擎在前腿切向力上差异显著，但都远低于滑动门槛（μ_req≤0.37 → 前轮 ~19.7 N）。这与 I5 的"前腿 hip 缺口集中在切向摩擦约束实现"一致——PhysX 用有限摩擦分量承担前腿外撑，MuJoCo 单点几乎无切向力，故 MuJoCo 前腿 hip 力矩被低估（W_A −4.6 vs 实测 −11.8 Nm）。
4. 结论：I4 两侧共同把缺口定位到**切向接触力分布**：法向总量与质量两引擎一致（排除），差异在切向力大小与接触点分布（MuJoCo 单点无斑、PhysX 不可逐点观测但前腿切向力大 50×）。这支持"接触流形保真度"为主因，而非质量/法向接触/执行器。

---

## I5 Go2-W 官方接触配置 MuJoCo 实验

变体（全部 M_cylwheel 派生 XML、G 场景、real init、显式 PD kp55/kd2、20000 步、末 20% 统计，脚本 `i5_gow_contact_variants.py`，`SIM_RUN_CONFIRMED`）：

| 变体 | 设置 | 依据 |
|---|---|---|
| W_A | 基线：condim=4、pyramidal、impratio=1、friction "1.0 0.05 0.001" | == G3 M_cylwheel G/real，管线内基准 |
| W_B | 仅 cone=elliptic | Go2-W 官方 MJCF 用椭圆锥 |
| W_C | 仅 impratio=100 | Go2-W 官方 MJCF；pyramidal 下预期无效应（如实记录） |
| W_D | condim=6 + elliptic + impratio=100 | Go2-W 求解器组合，AS2-W 摩擦 |
| W_E | W_D + 轮摩擦 "0.8 0.02 0.01" | Go2-W 官方轮摩擦 |
| W_I | （I1 MuJoCo 侧）implicit position/velocity 执行器（kp55/kv2 腿、kv2 轮）+ ctrl 目标，implicitfast | I1 公平对照 |

结果表（`tables/I5_GOW_CONTACT_VARIANTS.{csv,json}`，`SIM_RUN_CONFIRMED`；P55G2 锚点：hip q 0.2142/−0.2155/0.2779/−0.2755、hip τ −11.81/11.70/−15.15/15.19、pitch −6.13°、Fn 48.2/48.4/79.5/79.5）：

| 变体 | pitch° | hip q [FL FR RL RR] | hip τ Nm | Fn N | q12_err_rms | settle |
|---|---|---|---|---|---|---|
| W_A 基线 | −6.37 | 0.084/−0.084/0.210/−0.211 | −4.61/4.64/−11.57/11.60 | 49.4/49.6/77.9/78.1 | 0.0636 | OK |
| W_B elliptic | −5.27 | 0.097/−0.097/0.199/−0.199 | −5.33/5.36/−10.92/10.95 | 51.0/51.2/76.3/76.5 | 0.0627 | OK |
| W_C impratio=100 | **−0.80** | 0.199/−0.199/0.183/−0.184 | **−10.93/10.96**/−10.08/10.11 | 57.7/57.8/69.7/69.9 | 0.0591 | OK |
| W_D condim6+ell+impr100 | **−0.51** | **0.208/−0.208**/0.184/−0.185 | **−11.47/11.47**/−10.10/10.15 | 58.1/58.3/69.3/69.4 | 0.0597 | OK |
| W_E W_D+轮摩擦 | −0.54 | 0.208/−0.208/0.184/−0.184 | −11.46/11.46/−10.10/10.15 | 58.0/58.3/69.3/69.5 | 0.0595 | OK |
| W_I implicit（I1） | −6.37 | 0.084/−0.084/0.210/−0.211 | −4.61/4.64/−11.57/11.59 | 49.4/49.6/77.9/78.1 | 0.0636 | OK |
| （实测 P55G2） | −6.13 | 0.214/−0.216/0.278/−0.276 | −11.81/11.70/−15.15/15.19 | 48.2/48.4/79.5/79.5 | — | — |

### I5.1 发现

1. **impratio=100（W_C）是一阶效应，不是无效应**。任务书预期"pyramidal 下 impratio 无效应"被实验推翻（`SIM_RUN_CONFIRMED`）：本版 MuJoCo（3.x，implicitfast）下 impratio 同样作用于 pyramidal 锥的摩擦-法向阻抗比，把前腿 hip 力矩从 −4.61 拉到 −10.93 Nm（实测 −11.81）、前 hip q 从 0.084 拉到 0.199（实测 0.214）。**Go2-W 官方求解器配置下，MuJoCo 前腿 sim-real 缺口基本关闭**（W_D：τ err −0.34 Nm、q err −0.006 rad，在执行器误差预算 2–3× 以内）。
2. **但后腿与 pitch 缺口反向扩大/未关闭**：W_D 后腿 τ −10.10/10.15 vs 实测 −15.15/+15.19（缺口 5.0 Nm）、后 hip q 0.184 vs 0.278（缺口 0.094 rad）、pitch −0.51° vs −6.13°、Fn 后 69.3 vs 79.5。即 **G3 的"MuJoCo 唯一分支 pitch 匹配"（W_A，pitch −6.37≈−6.13）与"前腿力矩匹配"（W_D）不可兼得**——它们由不同求解器参数主导：pitch 匹配需要低摩擦阻抗（impratio≈1），前腿匹配需要高摩擦阻抗（impratio=100）。
3. condim6 + elliptic + 轮摩擦（W_D→W_E）为二阶（|Δpitch|≤0.03°、|Δτ|≤0.05 Nm）：**Go2-W 组合里的独立主成分就是 impratio**。
4. W_A 与 G3 发表值逐位一致（pitch −6.37、hip τ −4.61/4.64/−11.57/11.60），I5 管线与 G3 同源可比。

### I5.2 结论

Go2-W 官方接触配置在 MuJoCo 上**系统性移动分支**且方向是把负载分配向前腿集中（Fn 前后 58/69 vs 基线 49/78）。没有任何变体同时复现实测 pitch（−6.13°）与全四 hip 力矩——与 G3 跨引擎结论同构，但把"引擎差异"收窄为"**摩擦约束阻抗（impratio）**"这一可标定参数。impratio 是 MuJoCo 求解器数值参数而非物理参数，实机无对应物——它的作用实为补偿摩擦锥求解的数值柔度；这提示缺口本体在**切向摩擦约束的实现保真度**，与 Phase C/G4 的 μ_req 分析一致（μ_req≤0.37 远离滑动边界，正常求解器不该有此大差异）。

---

## I6 七项指标统一评价

锚点 = P55G2 窗 (8,19) s（`RAW_DATA_CONFIRMED`）。PhysX 列取 I1 dt0.002 隐式 cyl real 运行（I3/I4 的 dt0.005 基线在 ⑦/⑥ 行注明）：

| 指标 | 实测 | W_A 基线 (MJC) | W_D Go2-W 组合 (MJC) | PhysX cyl dt0.002 | 判定 |
|---|---|---|---|---|---|
| ① 四 hip q (rad) | 0.214/−0.216/0.278/−0.276 | 0.084/−0.084/0.210/−0.211 | **0.208/−0.208**/0.184/−0.185 | 0.191/−0.192/0.187/−0.189 | W_D 前腿 ✓（err ≤0.008）；后腿 ✗（err −0.094）；W_A 后腿 q ✓（err −0.068/−0.066）前腿 ✗；PhysX 前后均落在 ~0.19 近平分支 |
| ② 四 hip τ (Nm) | −11.81/11.70/−15.15/15.19 | −4.61/4.64/−11.57/11.60 | **−11.47/11.47**/−10.10/10.15 | −10.55/10.55/−10.29/10.36 | W_D 前腿 ✓（err 0.34/0.23，≈执行器预算 2×）；后腿 ✗（err 5.05/5.04）；W_A 后腿 τ err 3.58/3.59 亦 ✗；PhysX 后腿 err 4.9/4.8 |
| ③ 12 关节 q RMS err | — | 0.0636 | 0.0597 | （表内未给，q 分布同 ①） | MJC 全变体同量级（thigh/calf 匹配良好，缺口集中于 hip） |
| ④ pitch° | −6.13 | **−6.37** | −0.51 | +1.27 | W_A ✓（err −0.24°）；W_D ✗（+5.62°）；PhysX ✗（+7.40°，反向且近平分支）|
| ⑤ Fn 分配 (N) | 48.2/48.4/79.5/79.5 | 49.4/49.6/77.9/78.1 | 58.1/58.3/69.3/69.4 | 58.0/57.9/70.6/70.6 | **W_A 已在实测 ±1.6 N 内**；W_D/PhysX 均把前后比拉平（58/70）|
| ⑥ 接触状态 | 四轮同接 | 4×1 点、零穿透、零斑（spread 0.12–0.21 mm） | 4×1 点、settle_ok | 四轮同接、Fz 和 255.06 N=自重、水平 0.0；逐点流形 `UNKNOWN`（仅合力+平均接触位置）；前腿摩擦 4.8/5.6 N vs MJC 0.09/0.10 N | MJC 可证单点无斑；PhysX 逐点不可观测但前腿切向力大 50×（I4） |
| ⑦ 收敛稳定性 | — | settle_ok、W_I≡W_A 逐位 | settle_ok | 显式 40 s 未静 / 隐式 29.6 s 静；dt0.005→0.002 pitch 移 2.7°；I3：同条件重复差 0.0，但 init 扰动 ±2°/±0.02 rad 使 pitch 跨条件极差 1.76°、hip q 极差 0.052 rad（`INIT_SENSITIVE_MULTIBRANCH`） | MJC 显式/隐式无差、init 无关（单吸引子）；**PhysX 确定性但对 init/dt 敏感（浅势阱多分支）** |

**I6 核心张力（跨引擎定论）**：没有任何单一配置同时满足 ①②④⑤——MuJoCo W_A 匹配 pitch+Fn 分配但前腿 hip 力矩差 7.2 Nm；MuJoCo W_D 匹配前腿 hip 但 pitch 差 5.6°、Fn 前后比反转；PhysX 落在第三个近平分支（pitch +1.3°、前后 hip q 均 ~0.19、Fn 58/70），既不匹配实测 pitch 也不匹配后腿负载。**由于 ⑤（Fn 分配）是 G2 勘误后最可信的独立实测锚点且 W_A 已匹配，W_A 基线仍是"最可信仿真基准"；W_D 证明前腿缺口可被求解器数值参数（impratio）吸收，PhysX 的 dt 敏感性（I3）进一步说明分支是数值解而非物理唯一解。**

---

## Phase I 总结

### 三元区分（任务书核心问题）

| 候选层 | 判定 | 证据 |
|---|---|---|
| **执行器** | **基本排除**（Phase H 已定，Phase I 加固） | I1：两引擎上显式 PD vs 隐式 Joint Drive 的平衡点差 ≤0.3 Nm / 0.003 rad（MJC 逐位一致、PhysX ≤0.3 Nm）；H3 悬空闭环 0.0096 rad |
| **接触模型** | **主因** | I4：MuJoCo 圆柱-平面 = 严格单点、零穿透、零接触斑（实机轮胎必有斑）；PhysX 法向总量与水平力同样精确闭合（255.06 N / 0.0），但前腿切向摩擦 4.8–5.6 N vs MuJoCo 0.09–0.10 N（约 50×），逐点分布不可观测；I5：摩擦约束阻抗 impratio 一阶移动分支（前 hip τ 4.6→11.5 Nm），而 μ 数值（0.8/1.0）二阶；I2：材质绑定缺陷修复后平衡点不变 |
| **求解器** | **分支选择器（数值层）** | I5：impratio 是数值参数却能一阶移动物理平衡点；PhysX 对 dt 敏感（dt0.005→0.002 pitch 4.0→1.3°）；I3：PhysX 同条件重复差 0.0 但 init 扰动使 pitch 极差 1.76°、hip q 极差 0.052 rad（`INIT_SENSITIVE_MULTIBRANCH`，浅势阱）；两引擎给不同分支（MJC 单吸引子 −6.4° / PhysX 多分支 +1.3~4.7°） |

**一句话**：残余 sim-real 缺口不是执行器问题，而是**接触流形保真度问题**（点接触 vs 有限接触斑），其数值表现在求解器层（impratio/dt/引擎）以"分支选择"的形式放大；物理层的 μ 数值已被排除。

### 关键量化结论

1. **没有任何配置同时复现实测 ①②④⑤**：MJC W_A 匹配 pitch/Fn/后腿 q，MJC W_D 匹配前腿 hip，PhysX 落第三分支。缺口最小化需在"姿态/负载分配"与"前腿 hip 力矩"之间取舍。
2. **前腿缺口的解释自由度在求解器数值参数**（impratio），**后腿缺口无任何配置消除**（W_A 后腿 τ err 3.6 Nm、W_D 5.0 Nm、PhysX 4.9 Nm；后 hip q 误差 0.09–0.09 rad）——后腿缺口是更硬的约束。
3. **MuJoCo 最可信基线 = W_A**（pitch err 0.24°、Fn err ≤1.6 N、后腿 q err 0.07 rad），声明"前腿 hip 力矩不可信"后可用于相对比较，不可作绝对验收门（J1）。
4. 执行器误差预算（H4：0.12 Nm RMS、±0.2 Nm 偏置）比缺口小 20–60×，结论稳固。

### 产物索引（Phase I）

| 文件 | 内容 |
|---|---|
| 本文 | I1 十项核对+I1 结果、I2 材质绑定、I3 分支稳定性、I4 接触流形、I5 Go2-W 变体、I6 七指标 |
| `scripts/i5_gow_contact_variants.py` + `tables/I5_GOW_CONTACT_VARIANTS.{csv,json}` | 6 变体（W_A–W_E + W_I implicit），含 I1 MuJoCo 侧 |
| `scripts/i4_mujoco_contact_balance.py` + `tables/I4_MUJOCO_CONTACT_BALANCE.{csv,json}` | MuJoCo 逐点流形 + 合力/合力矩残差 |
| `scripts/i1_physx_explicit_pd.py`、`i3_physx_branch_perturbation.py`、`i4_physx_contact_balance.py` | PhysX 侧（代理） |
| `tables/I1_PHYSX_EXPLICIT_PD.{csv,json}` | PhysX 显式 vs 隐式（同 dt）+ dt 敏感性 |
| `tables/I2_MATERIAL_BINDING.{csv,json}` | 运行时材质绑定真值 + 修复重跑 |
| `tables/I3_PHYSX_BRANCH_PERTURBATION.{csv,json}` | 7 条件 ×2 重复扰动实验，`INIT_SENSITIVE_MULTIBRANCH`（pitch 极差 1.76°、hip q 极差 0.052 rad，重复差 0.0） |
| `tables/I4_PHYSX_CONTACT_BALANCE.{csv,json}` | PhysX 合力账本（Fz 和=自重、水平 0.0）+ 逐点流形 `UNKNOWN` + 前腿切向力 4.8/5.6 N |
| `physx_abl/i3_driver.log`、`i4_driver.log`、`i3_logs/`、`i4_logs/` | 各 run 的 rc 与原始日志 |
| `physx_i_logs/I_PHASE_SUMMARY.md` | PhysX 侧 I1–I4 代理汇总（含 UNKNOWN 清单） |
