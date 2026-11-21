# 09 Phase G：Go2-W 开源项目交叉参考审计（G1）

日期：2026-11-20/21
产物：`runs/as2w_v2_simreal_rootcause_20261119/tables/GO2W_AS2W_COMPARISON.csv`
约束遵守：未启动重训、未触实机、未改任何正式资产；本审计只读。

## 0. 数据源声明（先读）

任务书给出的 5 个外链（zhaozijie2022/LocoWheeledLegged、ShengqianChen/DreamWaQ_Go2W、ShengqianChen/go2w_rl_gym、unitreerobotics/unitree_mujoco、arXiv 2606.21387）在企业网络下**均不可达**（直连超时；WebFetch 被 host_not_allowlisted 拦截）。因此：

- **unitreerobotics/unitree_mujoco** 与 **unitree_rl_lab / LeggedSkillDeploy** 用工作站上的本地 vendored 官方副本替代审计（源码级，等价于上游内容快照）：
  - `Rob2Nav-upstreams/LeggedSkillDeploy/robot_description/mjcf/go2w/go2w.xml`（官方 MJCF）+ `assets/wheel.stl`
  - `Rob2Nav-upstreams/LeggedSkillDeploy/robot_description/urdf/src/go2w_description/urdf/go2w_description.urdf`
  - `Rob2Nav-upstreams/LeggedSkillDeploy/policy/issacgym/go2w/go2w_himloco/config.yaml`
  - `Rob2Nav-upstreams/unitree_rl_lab/deploy/robots/go2w/config/config.yaml` + `source/.../assets/robots/unitree.py`（UNITREE_GO2W_CFG）
  - `~z00959834/robogauge_5090/cpp_stack/unitree_mujoco/unitree_robots/go2w/`（交叉校验副本，与上面副本一致）
- **LocoWheeledLegged / DreamWaQ_Go2W / go2w_rl_gym 三个 GitHub 训练仓库与 arXiv 2606.21387 论文**：本地无对应副本，内容一律标 `UNKNOWN`，不做任何猜测。CSV 中对应行已落 UNKNOWN。

## 1. Go2-W 官方双表示：MJCF 与 URDF 自己就不一致（最重要发现）

| 项 | 官方 MJCF（unitree_mujoco） | 官方 URDF（go2w_description） |
|---|---|---|
| 轮碰撞基元 | **mesh** `wheel.stl`（r=0.0859，径向宽 0.0518 m） | **cylinder**（r=0.086，长 0.045 m） |
| 侧向偏移 | mesh 几何中心 y=+0.0508（geom quat 180°绕 y 翻转后） | cylinder origin xyz=(0, 0.05, 0) |
| condim | **6**（foot class，priority=1，margin=0.001） | n/a（PhysX 侧由引擎决定） |
| 摩擦 | "0.8 0.02 0.01"（滑动/扭转/滚转） | 无材质作者化 |
| 求解器 | cone=**elliptic**，impratio=**100** | n/a |
| 默认 geom（非轮） | friction 0.4、condim=1、margin 0.001 | — |

`SOURCE_CONFIRMED`。**含义**：Go2-W 官方自己就存在"MuJoCo 用网格轮 + URDF 用圆柱轮"的双表示分歧，轮宽 52 mm vs 45 mm 差 7 mm。这与 AS2-W 当前面对的"Isaac STL 网格轮 vs MuJoCo 圆柱轮"是**同一类结构性风险**，且说明该分歧是轮足机器人资产普遍痛点，不是 AS2-W 独有 bug。Go2-W 的处置方式可以作为先例：官方 MJCF 给轮用 **condim=6 + 椭圆锥 + 高 impratio + 扭转/滚转摩擦**，即承认"轮地接触不是普通足底接触"。

## 2. 几何对照（逐项）

| 项 | Go2-W | AS2-W | 备注 |
|---|---|---|---|
| 轮半径 | 0.0859（mesh）/0.086（URDF） | 0.086 | 一致 |
| 轮侧向偏移 | +0.050 m | +0.0546 m | 同一构造模式（腿链 y 偏移 + 偏心轮） |
| 轮距 | 0.386 m | 0.44 m | AS2-W 宽 54 mm |
| hip 偏移 | ±0.0465 | — | — |
| thigh 偏移 | ±0.0955 | ±0.1054 | — |
| 训练目标质量 | ~19.3 kg（URDF 惯量和） | 26.0（官方）/27.5（load-v0，待称重） | 质量/CoM 不确定是 AS2-W 特有问题 |
| 默认站姿 | [0, 0.8, −1.5, 0]×4 | [0, 0.8, −1.6]×4 + 轮 | 几乎同一站姿 |

`SOURCE_CONFIRMED`。

## 3. 执行器与控制对照

| 项 | Go2-W | AS2-W |
|---|---|---|
| 轮电机力矩上限 | MJCF motor ctrlrange ±15；URDF effort 23.7；IsaacLab IdealPD effort 23.5 | MuJoCo 轮力矩上限 15 Nm（速度伺服，阻尼 2.0）； Isaac 轮 effort 15 |
| 腿 PD（部署） | fixed_kp 40 / kd 1；rl_kp 20 / kd 0.5 | G: kp 55/kd 2；GO: kp 200/kd 4 |
| 轮控制方式 | action_scale 轮=5.0（速度型） | 轮速度伺服（阻尼 2.0） |
| 力矩总限 | 33.5 Nm | hip/thigh 60、calf 90 |
| dt | 0.005（decimation 4） | MuJoCo 0.002 / Isaac 0.005 |

`SOURCE_CONFIRMED`。两者都把**轮力矩上限显著压低于腿**（15 vs 60–90），且都以速度/阻尼方式驱动轮——这印证 AS2-W 的轮驱动合同与业界同类一致。Go2-W 部署链（HimLoco）的 kp（20–40）远低于 AS2-W GO（200），说明 AS2-W 的 GO 高增益在本参考系里是偏高的（但 GO 只用于静态，静态下高 kp 对平衡分支的影响已由 Phase E 消融覆盖）。

## 4. RL 训练与域随机化

- HimLoco：dt 0.005、16 DOF、57 维观测、torque_limits 33.5——`SOURCE_CONFIRMED`。
- **DR 参数范围**：本地副本（含 unitree_rl_lab deploy 配置）中不含训练期 DR 范围定义 → 双方的 DR 范围均标 `UNKNOWN`，不做推断。
- unitree_rl_lab UNITREE_GO2W_CFG：IdealPD（effort 23.5、stiffness hip/thigh/calf 25、wheel 0、damping 0.5、friction 0.01）——`SOURCE_CONFIRMED`。注意 stiffness 25/wheel 0：Go2-W Isaac Lab 官方训练把**轮 stiffness 置 0**（纯速度/力矩通道），再次与 AS2-W 的速度伺服设计同向。

## 5. 可迁移 vs 不可迁移结论

**可以借鉴（结构性先例，非参数拷贝）**：
1. 轮地接触按"轮接触"建模：condim=6（含扭转/滚转摩擦）+ 椭圆锥 + 高 impratio——AS2-W 交付 MJCF 目前用 condim=4、friction="1.0 0.05 0.001"（第 2/3 位在 condim 4 下无效），这一差异应在 G3 中作为变体测试（MuJoCo 侧 condim 6 已测过不敏感，但 elliptic/impratio 未测）。
2. mesh 轮与 cylinder 轮**必须区分口径**：Go2-W 官方两套表示宽差 7 mm；任何跨引擎结论都要注明用的是哪套碰撞。
3. 轮力矩低上限 + 速度型驱动是同类构型共识。

**不可迁移**：一切数值参数（PD、DR、摩擦）均未复制——任务书禁止，且两机质量/几何不同（19.3 vs 26 kg，0.386 vs 0.44 m 轮距）。

## 6. G1 对照表产物

`tables/GO2W_AS2W_COMPARISON.csv`：29 行、7 列（item/category/go2w_value/go2w_source/as2w_value/as2w_source/evidence/note）。证据分布：SOURCE_CONFIRMED 为主；外部 5 链接中 3 个 GitHub 训练仓库与 arXiv 论文共 4 行 UNKNOWN；DR 范围 UNKNOWN；AS2-W 真实轮胎 μ UNKNOWN。
