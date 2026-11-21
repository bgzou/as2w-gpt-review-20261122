# MANIFEST

打包时间：2026-11-21（CST）
源仓库：`/home/robot/z00945176/Rob2Nav`
git commit：`97488b10ffbb5898ca6607ffd4c33941ed610607`（main，工作树含 76 个既有改动）
文件总数：1103（含 34 个包内相对符号链接）
解包后大小：约 1.2 GB
全量逐文件清单：`FILE_INDEX.csv`（path, bytes, note）
全量校验和：`SHA256SUMS`

## 分区

| 目录 | 文件数 | 大小 | 内容 |
|---|---:|---:|---|
| `docs/` | 159 | 509 MB | 主报告 00–13、Phase H0–H6、Phase A–F 复核、Codex prompt、实机原始 recordings/builds/docs |
| `runs/` | 869 | 661 MB | 28 个脚本、全部结果表、qc npz、MuJoCo 变体、PhysX URDF/run/USD、历史 clips |
| `assets/` | 36 | 59 MB | MuJoCo STL（30 MB）+ URDF STL（30 MB），供包内相对链接使用 |
| `env/` | 4 | 28 KB | 版本、pip freeze、环境说明、路径重定位脚本 |

## 关键子目录

### docs/
- `as2w_v2_simreal_rootcause_20261119/` — **主文档**：00_SOURCE_AND_CONTRACT … 13_PHASE_J_FINAL_DECISION，README、REAL_V2_ALL_JOINT_WINDOWS.csv、RECOMMENDED_NEXT_ACTIONS.md
- `as2w_hip_gravity_contact_diagnosis_20261119/` — Phase H0–H6 原始诊断（00–06 + 表）
- `AS2W_悬空负载诊断V2_全部数据与分析_20261119/`
  - `recordings/` — 实机原始日志（low_capture_fast.log、debug_lowcmd.csv 等，487 MB）
  - `docs/` — V2 数据说明与脚本
  - `builds/` — 2026-11-19 两次构建产物
- `AS2W_悬空负载诊断V2_PhaseA-F复核与下一步计划.md`
- `AS2W_V2实机数据驱动的Sim2Real力学根因分析_Codex_Prompt_20261119.md`
- `AS2W_髋关节异常重力与轮地接触分项诊断_Codex_Prompt_20261119.md`
- `AS2W现状与问题总结_外部咨询用_20261121.md`

### runs/as2w_v2_simreal_rootcause_20261119/
- `scripts/` — a1/a2、b1、c1、d1、e0–e3、f0–f3b、g2/g2b/g3/g3_digest、h2/h3、i_common/i1–i5，共 28 个 .py
- `tables/` — 全部 CSV/JSON 结果表（A_* … I5_*、HYPOTHESIS_UPDATE_V2.csv 等）
- `qc/a1_timelines/` — P55G2/P55G1/P55S/P85DS/OG/OS 等解析后 npz
- `mujoco_abl/` — 20 个 MuJoCo 变体（A_C1_base、M_cylwheel、W_A–W_I、C_*、D_*、F_wide_patch 等）
- `mujoco_c1* / mujoco_s* / mujoco_sc1m*` — C/D 阶段 MuJoCo 运行目录
- `physx_abl/` — `urdf_{cyl,mesh,official}wheel`、`g3_*`、`i1–i4_*` 的 run JSON/日志、`usd_*` 与 `i_usd`/`g3_usd` 缓存
- `physx_usd_{g,go,s}/` — PhysX 场景 USD 缓存
- `physx_i_logs/I_PHASE_SUMMARY.md` — PhysX 侧 I1–I4 代理汇总 + UNKNOWN 清单

### runs/as2w-r4000-realcal-wheelrl-20261114/
- 仅保留 `data/extracted/SimDynamicsValidation_AS2W_20261114/clips/`（21 个 clip.npz，15 MB），
  用于 `a1_parse_sessions.py` 的 slot 映射与单位校验。

### assets/
- `mujoco_meshes/` — `as2w_scene.xml` 引用的 18 个 STL
- `urdf_meshes/` — URDF 引用的 STL
- 包内所有 `*/meshes` 符号链接均指向这两个目录（相对路径，已校验无断链）。

## 未包含 / 有意排除

- `deliveries/` 正式资产本体（只把 MuJoCo/URDF 所需 STL 复制进 `assets/`）
- 原仓库其余无关模块、训练权重、ONNX、视频
- `scripts/__pycache__`
- Isaac Sim / MuJoCo 的安装目录（版本见 `env/versions.txt`，pip 见 `env/pip_freeze_venv312.txt`）
