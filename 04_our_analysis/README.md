# 统一阶段性报告中心（AS2-W / CPRO）

本目录集中放**跨阶段、可独立阅读、可被外部审阅**的阶段报告。目的：把此前散落在 `GPUFREE服务器/`、`部署测试/`、交接文档和 DeepSeek 候选稿里的结论，整理成**统一的编号报告**，每条结论都能追到原始 JSON / 日志 / 源码行。

**本目录不替代原始记录，只做可读的综合。** 一手证据仍在：

| 原始记录位置 | 内容 |
|---|---|
| `部署测试/04_调试记录/INDEX.md`（297 节，619 KB） | 全部实机/离线阶段记录的唯一权威流水 |
| `部署测试/04_调试记录/logs/` | INDEX 引用的原始日志 |
| `部署测试/02_现场操作/*.md` | 操作手册、门槛、复盘（含 v3.2 交付清单） |
| `部署测试/06_训练协作/*.md`、`*.zip` | 训练侧往返证据包与观测汇总 |
| `GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/` | Phase H/I/J 静态审计、I3 长时与扰动实验（params/code/logs/results/environment/SHA） |
| `GPUFREE服务器/04_诊断实验/systematic_project_audit_20261121/` | R2000A、unitreestyle 动态审计（脚本、结果 JSON、候选稿） |
| `GPUFREE服务器/00_管理与安全/handoff_deepseek_harness_20261121/` | 交接文档 v2 与事实底稿 |

## 报告清单与阅读顺序

| 编号 | 报告 | 覆盖对象 | 状态 | 建议读者 |
|---|---|---|---|---|
| **RPT-000** | 本 README | 索引、编号与证据规则 | 维护中 | 所有人先读 |
| **RPT-001** | `RPT-001_PhaseHIJ_Static_Audit_20261121.md` | P55/P85/OG/OS 静态接触/力矩证据（I4、H2/H3、W_A/W_D） | 已完成（审计阶段），一处原始错误结论已撤回 | 想弄清“48/48/80/80 N、0.12 Nm、5.872 Nm”这些数字到底是什么的人 |
| **RPT-002** | `RPT-002_I3_LongRun_Stability_20261121-22.md` | PhysX 静态平衡的长时行为与终态小扰动（阶段 2/3/4 合并） | 已完成：四条扰动 4/4 通过 | 想知道“两个 pitch 终态是否稳定”的人 |
| **RPT-003** | `RPT-003_R2000A_EndToEnd_20261121-22.md` | R2000A 从交付包、合同、仿真到实机与本次 P1 审计 | 部分：偏航因果对照（T2-1）未跑 | 关心零速/饱和/偏航失败的人 |
| **RPT-004** | `RPT-004_Unitreestyle_Families_20261121-22.md` | HIMLoco(model_18200) / v2-step / unified_dr 三族身份与证据 | 部分：身份链已核，zero-B 同合同仿真未跑 | 关心后腿下沉、右向/偏航缺口的人 |
| **RPT-005** | `RPT-005_Field_Deployment_Event_Ledger_20261107-20261121.md` | 全项目实机/离线事件台账（按策略族与日期），跨项目主表第一版 | v1 已出；待补 SHA/log_path/ruled_out 字段 | 想知道“哪台狗、哪版策略、哪天、做了什么、结果如何”的人 |
| **RPT-007** | `RPT-007_DLine_ZeroB_and_Contract_20261122.md` | HIMLoco zero-B：输入包 + 合同漂移（Python 参考 vs C++/yaml）+ 同合同 MuJoCo 仿真 | 已完成：同合同 sim 也下沉（0.13–0.38 rad）→ 不先归 Sim2Plant | 关心后腿下沉根因、以及 sim2sim 是否真的同合同的人 |
| **RPT-008** | `RPT-008_DLine_ZeroB_Ablation_20261122.md` | D 线深推：zero-B 增益/资产/站姿单变量消融 | 已完成：kp85 使下沉 −60~70%，资产回填无改善，原厂站姿不可作初态 | 关心后腿下沉可执行修法的人 |
| **RPT-009** | `RPT-009_CLine_Yaw_Causal_20261122.md` | C 线：full vs base-only 偏航因果（含 MuJoCo 3.17.0 交叉） | 已完成：base-only 也摔；版本不是差异来源；MIX 不复现 | 想知道 residual 是否必要的人 |
| **RPT-010** | `RPT-010_CLine_Yaw_Threshold_20261122.md` | C 线深推：wz 阈值、residual-only、资产/kp 重排 | 已完成：wz≤0.30 存活、0.45 摔；边界脆弱 | 关心偏航可用域与判据的人 |
| **RPT-011** | `RPT-011_ELine_Field_Events_Master_20261122.md` | E 线：跨项目事件主表 v2（39 行，含 schema/填充率） | 已完成 v2；SHA/log_path 待回填 | 要跨族对照与追溯的人 |
| **RPT-012** | `RPT-012_FLine_Training_Gate_20261122.md` | F 线：训练触发条件与前置信审计（决策备忘） | 已完成：暂不训练 + 七个前置门槛现状 | 想知道“要不要重训”的人 |
| **RPT-013** | `RPT-013_Unitreestyle_TwoPolicies_RootCause_20261122.md` | 两个上机策略（v2-step 两个权重 / HIMLoco 18200）的根因审查：训练 vs sim2real | 已完成：v2-step = 训练/门禁/摩擦域；HIMLoco = 部署合同 kp/站姿；含 index.html 新增证据（门禁/奖励错配、RR 几何） | 想直接看“到底该怪训练还是 sim2real”的人 |
| **RPT-014** | `RPT-014_Unitreestyle_ThreeQuestions_20261122.md` | 三问答复：step 有无 clamp 翻转（无）、资产与 sim2real 包对比（URDF 同族、修正只在 USD）、同事服务器访问失败与任务书方向 | 已完成 | 关心动作参数化/资产一致性/下一轮训练方向的人 |
| **RPT-015** | `RPT-015_Assessment_NextTrainingPlan_20261122.md` | 评估同事“下一轮训练任务书”：方向对、顺序错、接触因子选错层级 | 已完成：三处缺口 + 修订版 P0–P4 排程 | 要决定“照不照这份任务书开训”的人 |
| **RPT-006** | `RPT-006_Harness_Session_20261122.md` | 2026-11-22 接管会话（I3 收口 + R2000A P1 + unitreestyle 身份链 + 目录治理） | 已完成 | 需要复现本次动作的人 |

**报告之间不重复一手证据**：RPT 只给结构化结论与指针；数字一律下钻到 `results/*.json`、`runs/<id>/`、`environment_*` 或源码行。目录结构与进度另见 `../00_管理与安全/`。

均使用新编号与文件名，不覆盖任何既有文档；既有 `docs/AS2W_*.md` 仍是各阶段的原始结论文件，RPT 只做汇编与交叉引用。

## 证据等级（全目录强制，沿用交接文档 §4.2）

- `CONFIRMED` / `OBSERVED`：原始文件、原始日志或仿真输出**直接**支撑。二者区别：`CONFIRMED` 指我们自己重算/重跑并核对过；`OBSERVED` 指交付方或现场留下的记录，我们只核对了其存在与自洽，未独立复跑。
- `INFERRED`：由原始数据或多源一致性推断，未做独立测量。
- `UNKNOWN`：现有证据不足。
- 二级标签：`MODEL_CALCULATION`（模型反算）、`SIM_RUN_CONFIRMED`（仿真运行）、`RAW_DATA_CONFIRMED`（原始数据）、`SOURCE_CONFIRMED`（源码/文档行）。

**三条不可越界的表述规则**

1. **反演/估计量不得当实测**：四轮 Fn（48.2/48.4/79.5/79.5 N）、`tau_est`、`tau_est - tau_PD`、解析接触力，都不是独立传感器测量。
2. **仿真内部失败 ≠ Sim2Real**：MuJoCo/Isaac 里就摔的，不能先归到实机差异；反之仿真不摔而实机摔，也不能直接归到“接触模型”。
3. **不同策略族不得互借证据**：R1199 / R284 / R2000A / RISE63 / HIMLoco / v2-step / unified_dr 的异常必须各自成行（见 RPT-005 的族列）。

## 编号与命名规则

- 编号 `RPT-NNN`，按**主题**编号而不是按时间；同一主题的阶段进展追加到同一份报告的“更新”节，不新开文件。
- 报告内引用一律写 `相对项目根` 的路径 + 小节号/行号；引用 JSON 数字时同时给 SHA256 前 16 位（见各报告的“证据索引”）。
- 任何与原始 JSON 冲突的文字，以原始 JSON 为准，并在报告内注明“已按原始文件修订”。
- 本目录不写密码、令牌、私钥、`.env`；密钥只从环境变量读取。聊天中曾出现过的 DeepSeek key 应轮换。

## 维护状态

| 日期 | 动作 |
|---|---|
| 2026-11-22 | 建立本目录；写入 RPT-001…006；把 2026-11-21/22 的 I3 收口与 R2000A P1 汇入 RPT-002/003，把 unitreestyle 身份链核验（41 文件清单）汇入 RPT-004 |
