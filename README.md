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
