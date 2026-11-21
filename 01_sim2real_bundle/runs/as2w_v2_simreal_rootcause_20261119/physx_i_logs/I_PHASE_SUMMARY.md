# Phase I（PhysX 侧）实验总结

日期：2026-11-21
脚本：`scripts/i_common.py`、`i1_physx_explicit_pd.py`、`i2_material_runtime_check.py`、`i3_physx_branch_perturbation.py`、`i4_physx_contact_balance.py`
数据：`tables/I1_PHYSX_EXPLICIT_PD.{csv,json}`、`I2_MATERIAL_BINDING.{csv,json}`、`I3_PHYSX_BRANCH_PERTURBATION.{csv,json}`、`I4_PHYSX_CONTACT_BALANCE.{csv,json}`
运行目录：`physx_abl/i{1,2,3,4}_runs/`、`physx_abl/i{1,2,3,4}_logs/`
约束遵守：全部在 `runs/` 下 candidate 副本上；无 RL、无实机、未改 deliveries/ 正式资产。每个 run 独立 app 进程（`OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=1 LD_LIBRARY_PATH=/home/robot/miniconda3/lib`）。
证据标签沿用六类：`RAW_DATA_CONFIRMED / SOURCE_CONFIRMED / SIM_RUN_CONFIRMED / MODEL_CALCULATION / INFERENCE / UNKNOWN`。

---

## I1-explicit：PhysX 显式 PD 对照组

**做了什么**：把 leg/wheel 的 drive stiffness/damping 全写 0，每控制步（dt=0.002，500 Hz，20000 步）读 q/dq 算 `tau=kp*(tgt−q)+kd*(0−dq)`（kp55/kd2），经 `set_joint_effort_target_index`（PhysX `set_dof_actuation_forces`）注入；轮 `tau=clip(−2*dq,±15)`；限幅 hip/thigh 60、calf 90。另加同 dt=0.002 的隐式 PD 对照（隔离 dt 混淆）。G 场景 × {cyl,mesh} × {real,nearflat} + 2 个同 dt 隐式对照 = 6 run。

**关键数字**（pitch° / hip q / hip τ Nm）：

| 运行 | pitch | hip q | hip τ |
|---|---|---|---|
| explicit cyl real | **1.266** | 0.191/−0.192/0.187/−0.189 | −10.55/10.55/−10.29/10.36 |
| implicit dt0.002 cyl real | **1.317** | 0.191/−0.192/0.190/−0.190 | −10.51/10.58/−10.44/10.47 |
| G3 implicit dt0.005 cyl real | **4.01** | 0.136/−0.142/0.201/−0.198 | −7.49/7.79/−11.08/10.90 |
| explicit mesh real | **2.098** | 0.176/−0.181/0.196/−0.192 | −9.68/9.96/−10.79/10.58 |
| implicit dt0.002 mesh real | **1.955** | 0.175/−0.181/0.191/−0.187 | −9.6/10.0/−10.5/10.3 |
| G3 implicit dt0.005 mesh real | **1.91** | 0.179/−0.187/0.193/−0.188 | −9.84/10.29/−10.63/10.36 |
| explicit cyl nearflat | 1.499 | ≈0 | ≈0 |
| explicit mesh nearflat | 1.903 | ≈0 | ≈0 |

**结论**（`SIM_RUN_CONFIRMED`）：
1. **显式 vs 隐式在同 dt=0.002 下几乎完全一致**（cyl real 1.266 vs 1.317；mesh real 2.098 vs 1.955），仍是同一近平正 pitch 分支。隐式 Joint Drive 不是分支差异来源。
2. **G3 与 I1 的差异主要来自 dt**：cyl real 由 dt0.005 的 4.01° 降到 dt0.002 的 1.3°；mesh real 基本不变（1.91→1.96）。即 cyl 基元对时间步更敏感。
3. 任何组合都停在近平正 pitch（+1.3…+2.1°），**从未到实测 −6.1° 族**。
4. 显式 cyl real 的 `settle_ok=False` 是判据严格所致：后腿 RL/RR_calf 有间歇 ~0.48 rad/s 速度尖峰（settled 均值仅 0.059、位置稳定），属轻度接触颤振；隐式同 dt 无此尖峰。`SIM_RUN_CONFIRMED`。

---

## I2：材质绑定运行时验证 + 修正后重跑

**做了什么**：
1. `--check`：以 friction=0.8 spawn，**不 step**，在 runtime USD stage 读取材质 prim 与轮碰撞 prim 的 physics-purpose 绑定。
2. 修正：首个 `sim.reset()` 后（此时碰撞 prim 才有 CollisionAPI）用 `UsdShade.MaterialBindingAPI.Bind(..., materialPurpose='physics')` 直接绑定到 4 个轮碰撞 prim，再 `sim.reset()` 重新 cook；重跑 G real/nearflat。

**关键证据**（`SOURCE_CONFIRMED` + `SIM_RUN_CONFIRMED`）：
- 运行时存在材质 prim `/World/envs/env_0/Robot/material`，其 `physics:staticFriction=physics:dynamicFriction=0.8`；
- reset 前 4 个轮碰撞 prim（`.../FL_foot/cylinder` 等）**`bound_material=None`**，请求的 `wheel_fric` 未生效；`sim.reset()` 后运行时读取，4 轮均绑定到整机材质 `/World/envs/env_0/Robot/material`（static=dynamic=0.8），即**有效 μ=0.8 整机统一**，而非 G3 以为的逐轮独立材质。**G3 的已知缺陷被闭环复现**。PhysX 内部实际摩擦标量不可直接读（`UNKNOWN`）。
- 修正后 4 个轮碰撞 prim 均绑定到 `/World/PhysicsMaterials/wheel_fric`，运行时读取 static/dynamic=0.8（`post_reset` 证据链已写入 json）。
- 重跑：real pitch **4.036**（G3 为 4.01），nearflat **1.694**（G3 1.81）。**分支不变**，与 Phase E/f3b「静态对摩擦不敏感（≤0.3 Nm）」一致。`SIM_RUN_CONFIRMED`。
- 注：`sim_utils.bind_physics_material` 在本构建返回 False（即使 prim 已有 CollisionAPI），故改用直接 USD 绑定；PhysX 内部实际摩擦标量不可直接读，以 USD physics-purpose 绑定为证据。`UNKNOWN`（PhysX 内部摩擦数值）。

---

## I3：PhysX 分支稳定性扰动研究

**做了什么**：基线 = G3 G 场景 real init、cyl、隐式 kp55/kd2、dt0.005、12000 步。每条件独立 app 进程跑 2 次。7 条件 = 14 run。

**结果**（2 次重复的均值；两次 run 完全一致）：

| 条件 | pitch° | hip q | 两次重复 Δpitch / Δhipq |
|---|---|---|---|
| base | **4.038** | 0.136/−0.140/0.200/−0.199 | 0.0 / 0.0 |
| pitch +2° | 2.973 | 0.136/−0.141/0.173/−0.171 | 0.0 / 0.0 |
| pitch −2° | 3.618 | 0.167/−0.168/0.221/−0.223 | 0.0 / 0.0 |
| hip +0.02 rad | 4.733 | 0.135/−0.140/0.219/−0.216 | 0.0 / 0.0 |
| hip −0.02 rad | 3.812 | 0.139/−0.139/0.191/−0.195 | 0.0 / 0.0 |
| gap 0 mm | 3.979 | 0.136/−0.140/0.201/−0.199 | 0.0 / 0.0 |
| gap 4 mm | 4.030 | 0.136/−0.140/0.200/−0.198 | 0.0 / 0.0 |

**判定**（`SIM_RUN_CONFIRMED`）：
- 跨条件 pitch 极差 **1.76°**（>0.3°），hip q 极差 **0.052 rad**（>0.02）→ **INIT_SENSITIVE_MULTIBRANCH（初值敏感多分支）**，非单一稳定吸引子。
- 同一条件两次 run **完全一致（Δ=0）** → PhysX 本身确定性；分叉由初始条件选择，不是随机噪声。
- 轮地间隙 0/2/4 mm 几乎不动分支（3.98/4.04/4.03°），**间隙不敏感**；pitch 基座与 hip 初值显著移动分支（hip 单调：−0.02→3.81、0→4.04、+0.02→4.73）。
- 所有扰动都留在近平正 pitch（2.97…4.73°），**没有任何扰动把 pitch 推到 −6° 族**。

---

## I4-PhysX：接触信息提取与静力平衡残差

**做了什么**：基线 run（G/cyl/real/隐式 kp55/kd2/dt0.005/12000 步），ContactSensor 开 `track_contact_points=True`、`track_friction_forces=True`、`filter_prim_paths_expr=['/World/ground']`、`max_contact_data_count_per_prim=64`；在 settled 窗口统计。

**关键数字**（`SIM_RUN_CONFIRMED` / `MODEL_CALCULATION`）：
- ΣFz = **255.06 N**，模型重量 26.0 kg×9.81 = 255.06 N，垂直残差 **0.0 N**。
- ΣFx = 0.0、ΣFy = −0.0 N（水平残差 0）。
- 每轮合力 Fn = 53.19/52.71/74.25/75.43 N（FL/FR/RL/RR）。
- 每轮法向力 = 52.97/52.42/74.25/75.43 N；每轮摩擦力 = 4.82/5.57/0.59/0.28 N。
- 轮平均接触点（world m）：FL[0.252,0.262,0.001]、FR[0.256,−0.257,0.001]、RL[−0.153,0.260,0.000]、RR[−0.148,−0.259,0.000]。
- CoP = **(0.0163, 0.0009) m**（Fz 加权）。
- 绕 CoM 力矩残差 |Σ(r−com)×F| = **5.872 Nm**；绕世界原点 = 5.873 Nm。
- **逐接触点（单点位置/法向/力）：UNKNOWN**。PhysX ContactSensor 只暴露合力 + 每 body 的平均接触点（`contact_pos_w`），无逐点数组。已尝试 `net_forces_w`、`net_normal_forces_w`、`net_friction_forces_w`、`contact_pos_w`、`force_matrix_w`（需 filter）、`max_contact_data_count_per_prim`。

**解读**（`MODEL_CALCULATION` / `INFERENCE`）：
- 力平衡完美（ΣF=重量、水平为 0），说明 ContactSensor 合力通道可信。
- 力矩残差 5.87 Nm ≈ 255 N × 23 mm：前轮接触 x≈+0.25、后轮 x≈−0.15，而整机 CoM x≈+0.035，CoP x=0.016，**CoM 与接触 CoP 在前后向偏约 1.9–2.3 cm**。这是模型几何/质量口径下的静力矩不平衡，可能是近平正 pitch 分支的一个贡献项。**注意**：`contact_pos_w` 是每轮接触点的平均位置，非真实力线作用点，5.87 Nm 含该平均化偏差，故仅作候选解释，不作定论。

---

## 总体结论

1. **隐式 Joint Drive 不是 PhysX 分支差异的根因**：同 dt 下显式 PD 与隐式 PD 落点几乎相同（I1）。G3 与 I1 的差异来自 dt（cyl 尤甚）。
2. **PhysX 仍是初值敏感多分支、且全部停在近平正 pitch**：小扰动即可在 2.97…4.73° 间切换分支，同条件完全可重复（I3）；无任何扰动到 −6° 族。
3. **G3 材质绑定缺陷已闭环证实并修正**：spawn 材质存在但未绑定到轮碰撞 prim（默认 1.0）；直接 physics-purpose 绑定后 4 轮均为 0.8，但分支不变（I2），与摩擦不敏感结论一致。
4. **接触合力静力平衡自洽**（ΣF=重量、水平 0），但存在 ~5.9 Nm 力矩残差，指向模型 CoM 与接触 CoP 的前后向 ~2 cm 偏差（候选，非定论）（I4）。逐接触点数据 PhysX 侧不可得。

## UNKNOWN 项
- PhysX 内部实际摩擦标量（仅能读 USD physics-purpose 绑定）。
- 逐接触点的位置/法向/力（ContactSensor 不暴露）。
- 5.87 Nm 力矩残差中「平均接触点偏差」与「真实 CoM/接触几何不平衡」各占多少。
- PhysX 圆柱碰撞导入器的精确几何（G3 已标 UNKNOWN，本轮未改变）。
