# RPT-006 Harness 会话记录：2026-11-22（接管 → I3 收口 → R2000A P1 → 目录治理）

编号：`RPT-006`
会话日期：2026-11-21 21:19 → 2026-11-22（Asia/Shanghai）
执行者：DeepSeek Harness（主执行）
详细逐项报告（含全部数值与 SHA）：`../04_诊断实验/systematic_project_audit_20261121/docs/AS2W_Stage_Report_20261122_Handover_I3_R2000A.md`
本文件只做**会话级索引**：做了什么、产出在哪、失败在哪、下一步。

---

## 1. 会话目标（来自交接与用户指令）

1. 接管工作区，读完 `AGENTS.md`、交接 v2、`SOURCE_FACTS.md` 与三份阶段文档；核对当前状态。
2. 推进 `pitch_p2 ±0.1°` 两条终态扰动并回拉判定（GPU 单作业、顺序）。
3. 并行做 R2000A 本地只读：哈希链、分布内统计、零速 vx 扫描。
4. 按用户要求：把测试/实验整理成完整阶段性报告，统一存放、分类清晰。

## 2. 动作与产出

| # | 动作 | 产出 | 状态 |
|---|---|---|---|
| 1 | 只读核对远端 I3 状态、GPU 占用、既有结果 | 远端 `results/` 无 `pitch_p2_perturb*`；GPU 85 MiB 空闲 | 完成 |
| 2 | 回拉前核对本地/远端 SHA（params/results/run.sh/code/i_common） | 全部一致；`i_common.py` `b2bf8557…` | 完成 |
| 3 | 启动并完成 `pitch_p2_perturb_p01_480s` | `results/pitch_p2_perturb_p01_480s.json` `fc05c6e6…` | 完成（21:20→21:28） |
| 4 | 启动并完成 `pitch_p2_perturb_m01_480s` | `results/pitch_p2_perturb_m01_480s.json` `38883c2c…` | 完成（21:28→21:37） |
| 5 | 写 I3 阶段四小结 | `phase_hij_.../docs/AS2W_Phase_I3_Stage4_Summary_20261122.md` | 完成 |
| 6 | R2000A T1-1 哈希链 | `results/r2000a_hash_chain_20261122_01.json` | 完成（发现 1 处清单缺陷） |
| 7 | R2000A T1-2 统计增强 | `results/r2000a_distribution_parity_20261122_03.json` | 完成 |
| 8 | R2000A T1-2b ONNX 内部读出 | `results/r2000a_onnx_internals_20261122_0{1,2,3}.json` | 完成（两次失败保留） |
| 9 | R2000A T1-3 零速扫描 | `results/r2000a_zero_cross_probe_20261122_01.json` | 完成 |
| 10 | R2000A T2-2 资产账本 | `results/r2000a_urdf_mjcf_asset_ledger_20261122_01.json` | 完成 |
| 11 | 写 R2000A P1 文档 | `systematic_.../docs/AS2W_R2000A_P1_HashStatsZeroCross_20261122.md` | 完成 |
| 12 | unitreestyle 身份链（T3-1 部分） | `results/unitreestyle_himloco_identity_20261122_0{1,2}.json` | 完成（41 文件：39+2 行尾） |
| 13 | 写阶段总报告（用户要求） | `systematic_.../docs/AS2W_Stage_Report_20261122_Handover_I3_R2000A.md` | 完成 |
| 14 | **目录治理**（用户要求） | `00_管理与安全/`（结构规范/进度看板/分类导航/文件清单/移动登记）、8 份目录 README、根 README 重写、`06_交付与报告/报告/RPT-001…006` | 完成 |
| 15 | 清理可再生文件 | 删除 3 个 `__pycache__`（12 个 `.pyc`），登记在移动与清理登记 | 完成 |

## 3. 关键结论（一句话版）

- **I3 四条扰动 4/4 通过**：两分支双侧回到本分支邻域（\|Δpitch\|≤0.0703°），`settle_ok=true`，无分支互跳；升级为“两个观测到的局部稳定终态”（`INFERRED`）。扰动前 300/360 s 的 pose/joint_q/Fn 与控制运行 **max\|Δ\|=0.0**。
- **R2000A 饱和是真实的、且来自投递 ONNX 本身**：base clip 前元素 98.8889%、帧级 100%、base_raw 绝对值最大 319.61；但训练与部署同用 `clamp(-1,1)` → **不是部署合同错误**。
- **R2000A 零速翻转在固定状态开环探针下不出现**；闭环仍 `UNKNOWN`。
- **训练 URDF 与 MuJoCo 资产不同**：27.512 vs 26.000 kg，base 11.592 vs 10.08，COM +15 mm → MuJoCo 偏航结论不能直接外推。
- **HIMLoco 身份链**：41 文件清单 39 精确 + 2 行尾差（可逐字节重建）= 0 不一致；ONNX `02352011…` 与合同一致；PT 仅声明（`85072560…`，本地缺失）。

## 4. 失败与修复（全部保留记录）

| 项 | 现象 | 处置 |
|---|---|---|
| ONNX internals v01 | 绝对 5e-5 判 base_raw，实测 9.92e-5 → `INCONSISTENT` | 保留 `_01.json`；v02 改相对判据 |
| ONNX internals v02 | 相对 1e-5 仍过严（实测 4.37e-5） | 保留 `_02.json`；v03 用 5e-5 且以 `clip(base_raw)` 恒等 + 后置动作 4.36e-5 为判据 → `COMPLETE` |
| HIMLoco 清单 v01 | 2 处 1 字节差判冲突 | 保留 `_01.json`；v02 增加行尾重建（末行 LF 前插 CR 后 SHA 完全一致）→ 通过 |
| 我的一次误比对 | 曾把 360 s 控制运行的 checkpoint 索引当成 300/360（实际从 60 s 起） | 按 `step` 字段重比，结论为“逐位相同（0.0）” |
| 远端 shell 引号 | PowerShell 吞掉嵌套引号，多次 `syntax error` | 改脚本经 stdin（`bash -s`）传入；注意 LF 行尾（CRLF 报 `$'\r'`） |
| 项目树 `.git` | 发现 2 个**既有** `.git`（第三方交付快照内） | 未删除、未新增；已在阶段报告与治理文档记录 |

## 5. 未做（明确留给下一步）

| 项 | 原因 |
|---|---|
| R2000A T2-1 MuJoCo full vs base-only 偏航对照 | 需上传约 33 MB 资产到自有服务器新实验目录；脚本已 `py_compile`，未运行 |
| unitreestyle zero-B 输入缺口表 + 同合同仿真 | 依赖缺失字段盘点（原始 LowState/LowCmd 多数已丢） |
| 跨项目事件主表交叉字段（SHA/log_path/ruled_out） | 需要逐条回填 INDEX 对应日志 |
| 任何 RL / 正式资产修改 / CPRO 同步 / 实机动作 | 硬禁区 |

## 6. 本会话新建/修改的文件清单（可审计）

**新建（治理与报告）**：`00_管理与安全/{结构规范与命名规则,进度看板,分类导航,移动与清理登记_20261122}.md`、`00_管理与安全/生成文件清单.py`、`00_管理与安全/文件清单_20261122.json`、`reports/{README,RPT-001…RPT-006}.md`、`phase_hij_.../{docs,runs,scripts,results,server_payload}/README.md`、`systematic_.../{deepseek,results}/README.md`、`handoff_.../README.md`。

**新建（证据）**：I3 两条扰动 JSON/log/env；R2000A 5 类结果 JSON + 7 个候选脚本；HIMLoco 2 个身份 JSON + 2 个脚本；I3 `SHA256_MANIFEST_20261122.json`；`code/run_one_physx.py`（回拉镜像）。

**修改（均为“以原始 JSON 为准”的修订或索引更新）**：`handoff_.../DeepSeek_Harness_Handoff_20261121_v2.md`（§5.2 表）、`handoff_.../SOURCE_FACTS.md`、`phase_hij_.../README.md`、`systematic_.../README.md`、`GPUFREE服务器/README.md`。

**移动**：18 个 DeepSeek 工作稿 → `phase_hij_.../docs/_worklog/`（SHA 不变）。
**删除**：3 个 `__pycache__`（可再生）。
**未覆盖**：任何既有 `results/*.json`（`_02.json`、旧 I3 结果均原样）。
