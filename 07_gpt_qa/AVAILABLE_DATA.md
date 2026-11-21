# AVAILABLE_DATA — 仓库里到底有什么（给 GPT 的目录）

> 先看这张表再提问，避免要"仓库里没有的文件"。所有路径相对仓库根。
> 仓库：`https://github.com/bgzou/as2w-gpt-review-20261122`

## 1. 一手证据（可引用）

| 目录 | 内容 | 文件数 / 体积 | 权威性 |
|---|---|---|---|
| `01_sim2real_bundle/docs/` | Sim2Real 根因诊断正文（Phase A–J、髋重力/接触分项、悬空负载 V2、外部咨询总结） | 39 文件 / 0.5 MB | **一手分析**（结论需与 runs 数据对账） |
| `01_sim2real_bundle/runs/as2w_v2_simreal_rootcause_20261119/` | `tables/`（假设更新、关节力矩分解、Fn/Fy 反演）、`scripts/`、`qc/a1_timelines/*.npz`（逐帧时间线）、`mujoco_abl/`（20 个 MuJoCo 变体场景）、`physx_abl/`（各次 PhysX 运行的 JSON/日志/USDA） | 861 文件 / 45.7 MB | **一手数据** |
| `02_unitreestyle/sessions/2026-11-20-as2w-v2-step/` | 会话总结、门禁矩阵、实机指标 JSON、摩擦量化、抖动对比、训练曲线 CSV、分析脚本 | 80 文件 / 1.1 MB | **一手数据 + 一手分析** |
| `02_unitreestyle/sessions/2026-11-20-himloco-dr-10000/` | 会话总结、能力矩阵、逐轴下沉、MuJoCo sim2sim 记录、合同审计、现场 checklist、deploy*.yaml、控制器源码、`field/`（LowState/保护流/发布端日志） | 42 文件 / 10.1 MB | **一手数据 + 一手分析** |
| `02_unitreestyle/v2-step/` | 训练配方（environment/task/domain/rewards/gate）、`configs/`、`scripts/`、`model/` 说明 | 86 文件 / 0.2 MB | **训练侧权威** |
| `02_unitreestyle/docs/field-data-index-20261121/` | 上机权重对照 CSV（哪份权重、SHA、父权重、是否上机、时间）、实机数据索引 | 4 文件 | **身份权威** |
| `02_unitreestyle/docs/next-training/任务书.md` | 被评审的下一轮训练计划 | 1 文件 | 评审对象 |
| `02_unitreestyle/index.html` | 交互任务书（门禁/奖励错配、RR 几何、摩擦 DR 区间） | 1 文件 | 一手总结（**与 md 有重复，注意版本**） |
| `03_field_logs/INDEX.md` | 本项目现场调试总账（297 节，按时间） | 0.6 MB | **一手记录** |
| `03_field_logs/logs/` | 关键实机会话原始日志（R2000A、RISE63、unitreestyle 回拉、R1199 低通/前进/零速复盘） | 162 文件 / 17.7 MB | **一手日志** |
| `03_field_logs/02_现场操作/`、`06_训练协作/` | 现场手册、交付清单 v3.2、低层硬件合同、各策略反馈包 md | 几十个 md | 一手文档 |

## 2. 本项目的二次分析（可参考，需核验）

| 目录 | 内容 |
|---|---|
| `04_our_analysis/RPT-*.md` | 17 份报告（A–F 线、合同漂移、zero-B 消融、偏航因果、双策略根因、训练计划评估） |
| `06_deliverables/` | 两份初稿：`ROOT_CAUSE_ANALYSIS.md`、`NEXT_TRAINING_PLAN.md` |
| `05_our_experiments/` | 同合同 MuJoCo 复跑结果：`t3_3`（zero-B 基线/kp85/资产回填/原厂站姿，6 JSON）、`t2_1`（偏航 full/base-only、wz 扫描、residual-only，10 JSON + 日志）、`environment/`（版本）、`服务器环境清单.json` |

## 3. 明确**没有**包含（不要向用户要）

| 缺失项 | 原因 | 需要时怎么办 |
|---|---|---|
| `.mp4` 视频 | 体积 | 按 `MANIFEST.csv` 的 `original_path` 回源目录取（本地项目树） |
| `.stl` 网格、`.usd` 二进制、`.pt/.onnx` 权重 | 体积（sim2real 包 USD 缓存 612 MB） | 同上；权重 SHA 已在文中 |
| `.zip` 交付包 | 体积（`06_训练协作` 约 1.6 GB） | 同上 |
| `docs/session-20261121/artifacts/` 的训练产物 | 约 400 MB | 只保留了其 markdown 摘要 |
| 实机四轮法向力 Fn 真值 | **从未测过** | 这是待测项（见 `ANSWERS_ROUND_0.md` Q4） |
| 实机接触斑/轮胎压缩 | **从未测过** | 同上 |
| 单关节方向/零位核对 | **从未做过** | 同上 |

## 4. 口径权威性（冲突时按此排序）

1. `MANIFEST.csv` 的 SHA256（文件级）
2. 原始 `.json / .csv / .log / .npz`（`01/**/runs`、`02/**/analysis`、`02/**/data`、`03/**/logs`）
3. 一手分析 `.md`（`01_sim2real_bundle/docs`、`02/**/docs`）
4. 二次分析（`04_our_analysis`、`06_deliverables`）——**可被推翻**

## 5. 已知的口径陷阱

- **资产质量/质心二义**：`26.0 kg / base 10.08 / CoM x 0.035`（官方 URDF）vs `27.512 kg / base 11.592 / CoM x 0.020`（load-v0）。两者都在仓库里，别混用。
- **摩擦值多套**：USD 标称 0.5、USD mu025 0.25、HIMLoco MJCF 0.62、R2000A MJCF 1.0、sim2real 包 MuJoCo 变体 0.05/0.3/0.8。
- **力矩口径**：`PD 需求`、`tau_est`、`接触反演 τ_from_contact`、`重力项` 是不同量。
- **`ckpt["iter"]`**：部分 run 停在 16800，不可用于判断版本；以文件名/SHA 为准。
- **`raw.githubusercontent.com` 缓存**：更新过的文件可能读到旧版；用 `github.com/.../blob/...` 页面。
- **`.npz` 是二进制**：网页版 GPT 无法解析，只能引用"存在该文件"，不能引用其内部数值。
