# AS2-W Sim2Real 根因诊断完整包（Phase A–J，2026-11-21）

本包把「AS2-W 静态站立时实机与仿真的 hip 力矩/角度缺口（6–7 Nm / 0.1 rad）到底是
执行器、接触模型还是求解器问题」这一轮诊断的全部材料打成一个自包含快照：

- **文档**：00–13 主报告 + Phase H0–H6 原始诊断 + Phase A–F 复核 + 外部咨询总结
- **脚本**：a1–a5、b/c/d/e/f/g/h/i 全链路 28 个 Python 脚本
- **结果表**：`runs/.../tables/` 下全部 CSV/JSON（含九假设三列 verdict）
- **原始数据**：实机 `recordings/`（low_capture_fast.log、debug_lowcmd.csv 等）
  与解析后的 `qc/a1_timelines/*.npz`（P55G2 等）
- **仿真资产与中间产物**：MuJoCo 变体 XML + STL、PhysX URDF + STL、每 run JSON/日志、USD 缓存
- **环境**：版本、pip freeze、环境变量、路径重定位脚本

解包即自洽：所有 `meshes` 符号链接已改为包内相对路径，不依赖原仓库。

---

## 1. 先读什么

| 顺序 | 文件 | 作用 |
|---|---|---|
| 1 | `docs/as2w_v2_simreal_rootcause_20261119/13_PHASE_J_FINAL_DECISION.md` | **一页式结论** + 九假设终判 + J1 基准五问 + J2 实机补测清单 + J3 不重训 |
| 2 | `docs/as2w_v2_simreal_rootcause_20261119/12_PHASE_I_PHYSX_BRANCH_DIAGNOSIS.md` | Phase I：公平对比 / 材质绑定 / 分支扰动 / 接触流形 / Go2-W 配置 / 七指标 |
| 3 | `docs/as2w_v2_simreal_rootcause_20261119/11_PHASE_H_ACTUATOR_TORQUE_AUDIT.md` | Phase H：执行器排除（源码合同、0.12 Nm RMS、悬空回放） |
| 4 | `docs/as2w_v2_simreal_rootcause_20261119/README.md` | 00–13 全文档索引 + 产物索引 |
| 5 | `docs/as2w_v2_simreal_rootcause_20261119/06_ROOT_CAUSE_DECISION.md` | Phase A–F 阶段结论（进入 H 之前的基线） |

## 2. 终局结论（供快速核对）

1. **执行器基本排除**：七会话命令合同干净；`tau_PD vs tau_est` 0.12 Nm RMS；
   悬空闭环 0.0096 rad；两引擎上显式 PD 与隐式 Joint Drive 的平衡点差 ≤0.3 Nm。
   执行器误差预算比缺口小 20–60×。
2. **接触模型为主因**：MuJoCo 圆柱-平面是**严格单点、零穿透、零接触斑**；
   PhysX 法向总量与水平力同样精确闭合（255.06 N / 0.0），但前腿切向摩擦
   4.8–5.6 N vs MuJoCo 0.09–0.10 N（约 50×）。摩擦约束阻抗 `impratio`
   能一阶移动前腿 hip 力矩（4.6→11.5 Nm，实测 11.8），而物理 μ 数值（0.8/1.0）只有二阶效应。
3. **求解器是分支选择器**：PhysX 同条件重复逐位相同（Δ=0），但初始 pitch ±2° /
   hip ±0.02 rad 使 settled pitch 跨条件极差 **1.76°**、hip q 极差 **0.052 rad**
   （`INIT_SENSITIVE_MULTIBRANCH`，浅势阱）；dt 0.005→0.002 使 pitch 4.0→1.3°。
   MuJoCo 在 dt 0.002 下是单吸引子 −6.37°。**分支是数值解而非物理唯一解。**
4. **无单一配置同时复现全部实测指标**：MuJoCo W_A 匹配 pitch/Fn/后腿 q，
   MuJoCo W_D 匹配前腿 hip，PhysX 落第三分支。**后腿缺口（3.6–5.0 Nm）
   无任何配置消除，是更硬的约束。**
5. **最可信仿真基准 = MuJoCo W_A**（`M_cylwheel` + G 场景），声明
   「前腿 hip 力矩不可信」后可用于相对比较，不可作绝对验收门。
6. **不重训**：J3 五条件全部不满足；瓶颈是接触保真度与实机测量缺口。

## 3. 关键实测锚点（P55G2 窗 8–19 s）

| 量 | 实测值 |
|---|---|
| hip q (rad) | 0.2142 / −0.2155 / 0.2779 / −0.2755 |
| hip τ (Nm) | −11.81 / +11.70 / −15.15 / +15.19 |
| pitch | −6.13° |
| 四轮 Fn (N) | 48.2 / 48.4 / 79.5 / 79.5 |
| μ_req | ≤ 0.37 |
| 整机自重 | 255.1 N（26.0 kg，称重） |

原始时间序列：`runs/as2w_v2_simreal_rootcause_20261119/qc/a1_timelines/P55G2.npz`
（字段 `t/q/dq/tau/e/roll/pitch/quat/...`）；原始日志在
`docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/`。

## 4. 证据标签

全部结论使用六类标签：`RAW_DATA_CONFIRMED` / `SOURCE_CONFIRMED` /
`SIM_RUN_CONFIRMED` / `MODEL_CALCULATION` / `INFERENCE` / `UNKNOWN`。
`UNKNOWN` 清单集中在 `runs/as2w_v2_simreal_rootcause_20261119/physx_i_logs/I_PHASE_SUMMARY.md`
与各文档末尾，分析时请勿把 `UNKNOWN` 当默认值。

## 5. 已知口径修正（阅读旧材料时注意）

1. **质量**：本 candidate 副本为 **26.000 kg**（MJCF / URDF 18 连杆 / 实机称重
   255.1 N 三源一致）。旧汇总中的 27.512 kg 属于另一资产版本，不适用于本包。
2. **PhysX 材质绑定**：请求的逐轮 `wheel_fric` 未生效；reset 后 4 轮实际绑定到
   整机材质 `/World/envs/env_0/Robot/material`（static=dynamic=0.8），
   即有效 μ=0.8 整机统一。PhysX 内部实际摩擦标量不可直接读（`UNKNOWN`）。
   修复后平衡点不变（pitch 4.036 vs 4.01）。
3. **I4 PhysX 逐接触点**：`UNKNOWN`。ContactSensor 只暴露合力 + 每 body 平均接触位置；
   5.87 Nm 的绕 CoM 力矩残差是「平均化近似误差 vs 真实 CoM/CoP 前后偏 2 cm」
   的混合，无法在 PhysX 侧分离。
4. **MuJoCo 圆柱接触**：严格单点、零穿透、零接触斑（spread 0.12–0.21 mm），
   不是有限接触斑模型。

## 6. 复现

见 `env/ENVIRONMENT.md`（版本、环境变量、逐阶段命令、6 个已知坑）与
`env/relocate_paths.sh`（换目录时重写脚本内绝对路径）。

## 7. 数据来源与边界

- 实机数据来自 2026-11-19 的 V2 会话 recordings（P55/P85/OG/OS 等），
  本包保留原始 `low_capture_fast.log` / `debug_lowcmd.csv` 与派生 npz。
- 所有仿真在 `runs/` 下的 candidate 副本上完成；未修改 `deliveries/` 正式资产，
  未做 RL 训练，未连接实机控制。
- 历史 `clip.npz`（`runs/as2w-r4000-realcal-wheelrl-20261114/.../clips/`）仅用于
  slot 映射与单位校验，不是本轮主数据。
