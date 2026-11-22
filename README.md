# AS2-W 双策略诊断资料包（2026-11-22）

这份资料包用于让外部 AI（ChatGPT）独立诊断 AS2-W 轮腿机器人**两条已上机策略**的问题，并评审同事的**下一轮训练计划**。

- 机器人：AS2-W（16 轴：12 腿 + 4 轮），单位：rad / rad·s⁻¹ / N / N·m / m / s
- 两条策略：**v2-step**（58 维观测 / 12 维动作，平地踏步，tanh 动作）与 **HIMLoco**（360 维观测 / 16 维动作，腿轮联合，tanh + 腿 0.6 增益）
- 时间：训练与实机集中在 2026-11-09 ～ 2026-11-22

## 目录

| 目录 | 内容 | 关键文件 |
|---|---|---|
| `01_sim2real_bundle/` | Sim2Real 根因诊断包（Phase A–J）：实机/仿真对拍、接触与执行器审计、MuJoCo/PhysX 消融 | `docs/AS2W_Sim2Real根因综述_20261121.md`、`docs/as2w_v2_simreal_rootcause_20261119/06_ROOT_CAUSE_DECISION.md`、`.../13_PHASE_J_FINAL_DECISION.md`、`runs/.../tables/*.csv`、`runs/.../qc/a1_timelines/*.npz` |
| `02_unitreestyle/` | 同事训练仓库（除大文件）：两条线的训练配方、会话归档、门禁、实机日志、`index.html` 交互任务书、下一轮任务书 | `sessions/2026-11-20-as2w-v2-step/`、`sessions/2026-11-20-himloco-dr-10000/`、`docs/next-training/任务书.md`、`index.html`、`docs/field-data-index-20261121/权重-训练方案-上机状态对照.csv` |
| `03_field_logs/` | 本项目现场记录：`INDEX.md` 全量、关键会话原始日志、现场操作文档（md）、训练协作文档（md） | `INDEX.md`、`logs/20261121_as2w_unitreestyle_pull/`、`logs/20261119-212534_r2000a/`、`02_现场操作/AS2-W训练到部署交付清单_v3_2_*.md` |
| `04_our_analysis/` | 本项目此前写的 17 份报告（RPT-001…017）——**可作线索，但请独立核验，不要当结论** | `RPT-016_RootCause_TwoPolicies_20261122.md`、`RPT-017_NextTrainingPlan_Advice_20261122.md`、`RPT-013…`、`RPT-008…` |
| `05_our_experiments/` | 本项目在自有服务器（MuJoCo 4.0.1 隔离环境、CPU 单作业）跑的同合同仿真结果与脚本 | `t3_3/results/*.json`（zero-B 消融：kp85、资产回填、原厂站姿）、`t2_1/results/*.json`（偏航 full/base-only、wz 扫描、residual-only）、`environment/` |
| `06_deliverables/` | 期望产出的两份文档模板/初稿 | `ROOT_CAUSE_ANALYSIS.md`、`NEXT_TRAINING_PLAN.md` |
| `07_gpt_qa/` | **问答回路**：对上一轮 Phase H/I/J 任务的执行状态（已做完 6/7）、数据目录、提问模板；含审计正文与复算脚本/结果 | `07_gpt_qa/README.md`、`AVAILABLE_DATA.md`、`ANSWERS_ROUND_0.md`、`phase_hij_audit/` |
| `MANIFEST.csv` | 全部文件清单（路径、字节、SHA256、原始路径） | — |

## 未包含（体积原因）

- 视频（`.mp4`）、网格（`.stl`）、USD 二进制缓存（`.usd`）、权重（`.pt/.onnx`）、多数压缩包。
- 需要时按 `MANIFEST.csv` 的 `original_path` 回到源目录取；原始路径前缀为 `部署参考/`、`部署测试/`、`GPUFREE服务器/`。

## 关键身份（先读，避免版本错位）

| 策略 | 上机权重 | SHA256（前缀） | 上机时间 |
|---|---|---|---|
| v2-step | `jit_src/step_policy.pt`（抖动随机化） | `99995126…c41c33` | 11-20 18:49 / 18:56 / 19:01 |
| v2-step | `mu025/policy.onnx`（μ=0.25 重训 600 轮） | `a3215caa…` | 11-20 20:04 / 20:06 / 20:10 |
| HIMLoco | `deploy-18200/policy.onnx`（`model_18200.pt`） | `02352011…678bf` | 11-20 18:03–18:42 |
| HIMLoco（楼梯线，对照） | `.himloco/policy.onnx`（model_7836） | `a74abbf7…` | 11-19 |

权威对照表：`02_unitreestyle/docs/field-data-index-20261121/权重-训练方案-上机状态对照.csv`。

## 阅读顺序建议

1. `TASK_BOOK_CHATGPT.md`（任务与规则）
2. `06_deliverables/ROOT_CAUSE_ANALYSIS.md`、`NEXT_TRAINING_PLAN.md`（初稿，待你核验/重写）
3. `02_unitreestyle/sessions/.../README.md` 与 `analysis/*.json`（两条线的症状与数字）
4. `01_sim2real_bundle/docs/.../13_PHASE_J_FINAL_DECISION.md`（接触/执行器结论）
5. 原始数据：`02_unitreestyle/sessions/.../data/*.csv`、`03_field_logs/INDEX.md`、`05_our_experiments/*/results/*.json`

> 注意：`04_our_analysis/` 与 `06_deliverables/` 都是**人的分析**，可能与原始数据冲突；请以 `01/02/03/05` 的原始数据为准，并在你的报告里明确指出冲突。

## 用网页版 GPT 打开时

1. 直接给它仓库地址 + TASK_BOOK_CHATGPT.md 的 raw 链接：https://raw.githubusercontent.com/bgzou/as2w-gpt-review-20261122/main/TASK_BOOK_CHATGPT.md
2. 任务书要求它**只在对话里输出两份 md**（ROOT_CAUSE_ANALYSIS.md、NEXT_TRAINING_PLAN.md），不要写仓库。
3. 若它读不到某些大文件，让它列出缺口，由人补贴内容；*.npz 是二进制，网页版无法解析，跳过即可。

## 答复上一轮 GPT 任务书

`07_gpt_qa/ANSWERS_ROUND_0.md` 是对 2026-11-21 那份《Phase H/I/J 最终证据审计任务》的**逐项执行状态**：

- 7 项要求中 **6 项已完成**（Fn 来源、tau_est 术语、q 通道、H1 源码范围、5.872 Nm 复算、W_A 重评、J3 范围、产物）；
- 第 3 项（I3 多分支）在 11-22 补做了同进程 240–480 s + 终态 ±0.1° 扰动，**4/4 通过**，升级为"两个观测到的局部稳定终态"；
- 第 4 项（刚性圆柱 vs 有限接触区域对照）**未做**，因为缺实机接触斑/柔顺数据，做了也只能证明结构敏感性。
- 审计正文与复算脚本/结果已上传到 `07_gpt_qa/phase_hij_audit/`。

`07_gpt_qa/ANSWERS_ROUND_0.md` §4 列出了希望 GPT 回答的 Q1–Q6（实验排序、接触斑替代观测、Fy 反演、有限接触斑最小设计、I3 跨 dt、RL gate）。
## Phase K/L/M（2026-11-22 夜）执行回传

- `07_gpt_qa/ANSWERS_ROUND_1.md`：对 GPT《下一阶段 Agent 任务书》的逐条回答（P0-A/P0-B/P1-C/P1-D/P1-E）。
- `04_our_analysis/RPT-018_PhaseKLM_Sim2Real_20261122.md`：汇总报告（含"任务书数字核对"表）。
- `08_sim_handoff/SIM_TASKS.md`：v2 任务清单（T3 关闭、T2 降级、新增 T10–T12）。
- `09_phase_klm/`：P0-A（MuJoCo 7 场景）与 P0-B（PhysX 10 次运行）的代码与原始 JSON。

三条关键更正：① 有限接触斑在本引擎**不可构造**（轮外倾 4.8°/12.1°，切 3/5 条仍单点），且 `Fn·b/2` 上界 1.25/2.05 Nm 不足以解释 7.2 Nm 缺口；② 缺口只能走侧向力（力臂 0.3425 m），前轮需要 μ≥0.44，μ=0.25 下差 3.1 Nm；③ 跨 dt 终态平移 0.22–0.68°，"两个局部稳定终态"保留但**不能**写成"两个静态平衡点"。
## Phase N（2026-11-22 深夜）执行回传

- `07_gpt_qa/ANSWERS_ROUND_2.md`：对 Phase N 任务书的 Q1–Q8 回答。
- `04_our_analysis/RPT-019_PhaseN_Ledger_Identifiability_20261122.md`：汇总报告。
- `01_sim2real_bundle/docs/as2w_v2_simreal_rootcause_20261119/14_/15_/16_*.md`：账本与可辨识性、实机 SOP、dt 与 Gate 修订。
- `09_phase_n/`：`n0_2_force_ledger.py`、`n0_4_couple_smoke.py` 与原始 JSON。

四条关键更正：
1. **7.2 Nm 髋缺口是跨位形比较的产物**：把模型放到实机 q + IMU 姿态后，四轮点接触可解释实测 `tau_est` 到 **max 1.985 Nm（26 kg）/ 1.123 Nm（27.512 kg）**，髋残差 0.22–2.0 Nm。**3.1 Nm 传动归因不成立**（UNKNOWN，T0 以下）。
2. **髋力矩主项是 `r_y F_z`（接触点横向偏移 × Fn），不是 Fy**；逐项分解见 doc 14 §2.3。
3. **Fy 方向**：前轮在两种资产下都外撑、不跨零；**后轮在 27.512 kg 资产下跨零** → 后轮方向 UNKNOWN。
4. **MuJoCo 有单点接触力偶通道**（`condim=4/6`，`friction` 三参数=滑动/扭转长度/滚动长度）；Phase K 的"没有通道"撤回，但准静态力矩受求解器阻抗限制，不能当自由上界。

另：`dt=0.00125` 单条对照给出 `hip_p002` 终态 **2.198°**（vs 0.0025 的 4.014°，差 1.82°）→ **未收敛**，停止更细步长盲试；"两个局部稳定终态"只能限定在 dt 0.0025–0.005 窗口。Gate 新增 **G1-K**（已知静态缺口，OPEN），正式 RL 仍禁止。
## Phase O（2026-11-22 深夜）执行回传

- `07_gpt_qa/ANSWERS_ROUND_3.md`：Q1–Q8 回答。
- `04_our_analysis/RPT-020_PhaseO_Ledger_Audit_20261122.md`：汇总报告。
- `01_sim2real_bundle/docs/as2w_v2_simreal_rootcause_20261119/17_/18_/19_*.md`：账本纠错、实机决策图与髋台架、数值/Gate。
- `09_phase_o/`：`o0_ledger.py`、`o3_cop_scan.py`、`o4_physx_audit.py` 与原始 JSON。

三处纠错均已复现并修正：
1. **接触点**：倾斜有限宽圆柱的最低点必须含端面项 `−w·sign(a_z)·a`；修正后接触点内移 **25 mm**、下移 **5.4–7.1 mm**，实机轮倾为 **12.08/12.39/15.71/15.81°**（旧 4.8/12.1° 是 W_A 落点，撤回）。
2. **摩擦锥**：旧 8 边形是外切（允许 1.0824 μFn），不是"保守内接"；改用内切 32 边形 + 外切对照后，最优点违反量为 0，`t*` 仅在 μ=0.2 变 0.009 Nm。
3. **Hip 分解**：只取该腿自身轮 + 关节轴 + 锚点，与 `A[hip_dof]·f` 逐位一致（diff=0）；`r_yF_z` 仍主项。

数值：`t*` 由旧 1.985/1.123 Nm 变为 **2.157 Nm（26 kg）/ 1.384 Nm（27.5 kg）**，且对 CoP 敏感（±0.7 Nm）——**不是物理结构下限**。**后轮 Fy 在两种资产、全 μ、CoP ±26 mm 下都跨零 → UNKNOWN**（旧 26 kg 不跨零的结论撤回）。PhysX 长时轨迹显示滚动与失载：`dt=0.010` 基座前移 1.35–1.41 m、`dt=0.00125` 60 s 时 RR `Fn=0`。Gate 不变：G1-K OPEN，G4 FORBIDDEN。
## Phase P（2026-11-22 深夜）执行回传

- `07_gpt_qa/ANSWERS_ROUND_4.md`：P0–P4 回答。
- `04_our_analysis/RPT-021_PhaseP_Contact_Feasibility_20261122.md`：汇总报告。
- `01_sim2real_bundle/docs/as2w_v2_simreal_rootcause_20261119/20_/21_/22_/23_*.md`：S1 几何与支撑、20 Hz 滚动时间线、跨会话留出、Gate。
- `09_phase_p/`：脚本与原始 JSON（7.4 MB 的 20 Hz 时序留在服务器，SHA 见报告）。

四条要点：
1. **旧 S1 越界**：每轮 5 点中 2 点超 `|t|≤26 mm`（8/20）；旧 ±0.7 Nm 敏感度降级为 `SENSITIVITY_ONLY_INVALID_AS_PHYSICAL_COP`。合法向内扫描只降 `t*` 0.14–0.21 Nm，后轮 Fy 仍跨零。
2. **平面/支撑**：水平面峰峰 4.266 mm；最佳斜面 0.253°、残差 1.94 mm；3 轮只有 2/4 子集几何可能且 `t*` 跳到 15.7 Nm；柔顺 k=2e4 N/m 才四轮载（103/18/73/61 N）。
3. **20 Hz 实测：两个"终态"都不静止**。末 60 s 轮速 P99 0.046（hip）/0.123（pitch）rad/s；`pitch_p2` 在 180–240 s 基座 +62.4 mm、轮转 0.6–1.0 rad、pitch −1.564°，并在 211.95/212.3 s 出现 RL 失载 → **迁移是滚动+失载**，不是两个静态平衡。
4. **跨会话**：P55G1/G2/G3 的 `t*` 2.00/2.16/2.02（26 kg）、前轮都不跨零；**OG 0.54 Nm、z 跨度 10.9 mm、四轮全跨零** → 姿态依赖。算子单测全过（Jacobian FD 0.0，随机力 Hip 叉乘 vs Jacobian 0.0）。
## Phase Q（2026-11-22 收口）

- `07_gpt_qa/ANSWERS_ROUND_5.md`：五个收口问题的回答。
- `04_our_analysis/RPT-022_PhaseQ_Stop_20261122.md`：汇总。
- `01_sim2real_bundle/docs/as2w_v2_simreal_rootcause_20261119/24_/25_*.md`：约束消融与 STOP、测量/策略轨道。
- `09_phase_q/`：锁基座/锁轮消融脚本与 JSON。

结论：末 60 s 是 **往复微动**（`Σ|Δq|/|net|` = 8.45/19.5，滑移代理 P95 ≤9.4 mm/s），不是持续滚动；**锁基座 xy 与锁轮 q 都抑制 180–240 s 的迁移**（基线 pitch 3.02→1.45°、+62 mm、47 个失载采样；B 稳定 −2.94°、C 稳定 −0.35°）→ 滚动/约束耦合，不能判接触根因。OG 记为 `OG_CONTACT_MODE_UNIDENTIFIED`（不共面 10.88 mm）。**正式状态：`STOP_OFFLINE_DIAGNOSIS_PENDING_REAL_MEASUREMENT`**；下一次有信息量的测量是 M2 四轮同步 Fn。Phase P 的 `compliance()` 过度解释已在 ANSWERS_ROUND_5 勘误。