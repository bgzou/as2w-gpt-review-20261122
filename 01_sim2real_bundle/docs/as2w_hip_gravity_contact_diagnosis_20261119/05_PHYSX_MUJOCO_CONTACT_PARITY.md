# H5：PhysX ↔ MuJoCo 接触一致性交叉检查

- 日期：2026-11-19
- 脚本：`runs/as2w_hip_gravity_contact_diagnosis_20261119/scripts/h5_physx_mujoco_parity.py`
- 输出：`PHYSX_MUJOCO_PARITY_CHECKS.csv`
- 对照材料：交付 MJCF（MuJoCo）vs `training/as2w-backup/as2w-current` Isaac/URDF 资产链（PhysX 侧源 URDF `as2w.urdf` + `as2w_isaac_flat_env_all.py` + 先前 `runs/as2w-layered-mujoco-validation-20261117/reports/12_physx_mujoco_comparison.md`）
- 引擎级 PhysX 复跑：**本次未重跑**（H2/H3 静态分解需要带接触力读出的定制实验，Isaac 端重跑成本高且不改变本发现的性质）；以配置级审计 + MJCF 侧代数复算 + 既有 PhysX 矩阵证据交叉，逐项标注来源。`BLOCKED_WITH_REASON`（引擎级）/ `SIM_RUN_CONFIRMED`（MJCF 侧复算）

## 1. 重大发现：MuJoCo 轮碰撞体整体错位 54.6 mm（碰撞轮距比真实窄 25%）

**事实链（全部实测读出，`SIM_RUN_CONFIRMED` × `RAW_DATA_CONFIRMED` 资产读出）**：

| 项 | 值 | 来源 |
|---|---|---|
| MJCF 轮碰撞 geom 局部位置 | `(0, 0, 0)`（在 foot 关节原点），cylinder r=0.086 半宽 0.026 | 运行时 `geom_pos` 读出 |
| URDF/STL 真实轮心（foot 关节系） | FL/RL **+0.0546 m**（y 跨 0.0287–0.0805）、FR/RR **−0.0546 m**（完美镜像） | 四个 `*_foot.STL` 二进制解析（15198 三角面） |
| 真实轮半径（STL 径向 max） | 0.086 m（与 MJCF cylinder r 一致） | 同上 |
| MuJoCo 碰撞轮距 | **0.3308 m**（foot 关节 y=±0.1654，cylinder 在原点） | 运行时 `xpos` 读出 |
| 真实轮心轮距 | **0.4400 m** | 0.3308 + 2×0.0546 |
| 控制器合同 track 常量 | 0.4376 m | controller 源码（12 报告） |

**MJCF 的轮碰撞 cylinder 完全落在真实轮面（y 0.029–0.0805）的内侧，与真实轮不重叠**——MuJoCo 场景里 visual mesh（STL，正确位置）与碰撞 cylinder（错位）不一致；PhysX 侧 URDF collision 用的是同一 STL（convex hull），接触发生在真实轮面。

**定量影响**：

1. **支撑轮距**：MuJoCo 0.331 m vs 真实 0.440 m——**侧向支撑基础窄 25%**。所有 MuJoCo 闭环的侧倾（roll）稳定性、侧向接触动力学、hip（abduction）扰动响应与 PhysX/实机系统性不同。`SIM_RUN_CONFIRMED`
2. **单位力敏感性**：竖直 Fn=62 N 作用在真实轮心 vs 关节原点，该腿 hip 力矩差 **3.38 Nm**（`mj_jac` J^T 投影，`MODEL_CALCULATION`）。即 MuJoCo 的碰撞错位使"法向支撑产生的 hip 力矩"被系统性低估。
3. **最小范数静态解复算（C6 反事实，接触点移到真实轮心）**：hip 组 0.65/−0.67/1.58/−1.51 vs 名义 0.68/−0.72/1.65/−1.58——**静态最小范数几乎不变**（求解器用切向自由度吸收了错位）。这再次印证 H2 纪律：最小范数解对接触几何细节不敏感，**该错位的危害在闭环动力学（轮距窄 25%），不在静态账本**；hip 静态残差的归因（侧向夹持/传动偏置）不受影响。

**结论**：交付 MJCF 存在一处**资产级错误**——轮碰撞 cylinder 未加 `pos="0 ±0.0546 0"` 偏移。它不改变 H3/H4 的 hip 残差归因（静态杠杆小），但使所有 MuJoCo 闭环的侧向/侧倾行为与 PhysX/实机不可比，且使法向力-髋力矩通路偏低 3.4 Nm/轮。**任何后续 MuJoCo 闭环复现前必须先修此 geom 偏移。** `SUPPORTED`（资产错误存在性 + 量级）

## 2. 其余 parity 项（配置级审计，引用 12 报告 + 本次读出）

| 项 | PhysX（Isaac，as2w-current） | MuJoCo（交付 MJCF） | parity |
|---|---|---|---|
| 腿 Kp/Kd | 55 / **2**（`as2w_isaac_flat_env_all.py:221`，`ImplicitActuatorCfg damping=2.`） | 代码级 `set_gains(55,2)` 覆写；**MJCF actuator 自带 60/90、kd 2/3、ctrlrange [0,0]（不用）** | ✓（代码级）/ ✗（裸 MJCF 直接加载会错） |
| 轮控制 | JointVelocityAction scale 16，stiffness 0 damping 2，effort 15 | MJCF velocity kv=8 forcerange ±15 | ✗（kv 8 vs 2；闭环必须脚本覆写） |
| 轮碰撞几何 | URDF collision=foot.STL（真实轮心 ±0.0546，r=0.086） | cylinder r=0.086 @ 关节原点（**错位 §1**） | **✗（§1 重大）** |
| 轮半径/质量 | 0.086 m / 0.595 kg（STL+URDF inertial） | 同（cylinder r、body inertial 读出） | ✓ |
| 地面摩擦 | PhysX implicit contact + terrain 材质；expanded profile 开 DR | plane friction 1.0/0.05/0.001、condim 4、solimp/solref 记录于 03 | ✗（引擎固有差异，12 报告已声明，不混称等价） |
| obs/控制器/50 Hz | 逐项一致（12 报告 §1 表，按名索引） | 同 | ✓ |

## 3. 既有 PhysX 证据（引用，不重跑）

`12_physx_mujoco_comparison.md`（2026-11-17）：同控制器逻辑 Sim2Sim，26 组矩阵 22 转向工况双引擎同带通过；前进速度 PhysX 19–79% vs MuJoCo 2–17%（expanded DR/延迟 vs 干净物理 + 接触模型差异）；SIM2SIM_PASS_CONDITIONAL。该证据与本诊断相容：**轮距错位属于 MuJoCo 侧资产问题，会加剧而非消除两引擎闭环差异**；实机 hip 负载在两个引擎的闭环里都没有被建模（PhysX 也无 10-13 Nm 级 hip 静态偏置——其 55/2 PD 在干净支撑下同样趋向"几何正确"位姿）。

## 4. H5 结论

1. **资产错误清单新增一项（最高优先）**：MJCF 轮碰撞 cylinder 缺 ±0.0546 m 横向偏移 → 碰撞轮距 0.331 vs 真实 0.440 m。修法一行（每 wheel geom 加 `pos`），修后 MuJoCo 侧向动力学与 URDF/PhysX/实机方可对齐。`SUPPORTED`
2. 该错位**不改变** H3/H4 的 hip 残差归因排序（侧向夹持 vs 传动偏置仍为唯二嫌疑、不可区分）；静态最小范数复算证明其静态杠杆 ≤0.07 Nm。
3. 裸 MJCF actuator（60/90/kv8/ctrlrange0）与实机及 Isaac 端 55/2 合同不一致——复现必须脚本覆写（已在 H2-H4 全部实验中执行）。
4. 引擎级 PhysX 复跑标记 `BLOCKED_WITH_REASON`；既有 PhysX 矩阵证据 + 配置审计已足以支撑上述 parity 结论。

下一步 H6：最终诊断、七问直答、最小补测建议（新增：修 MJCF geom 偏移后重跑 Layer B 作为闭环复验项）。
