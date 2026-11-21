# 环境与复现说明

本包是 AS2-W Sim2Real 根因诊断（Phase A–J，2026-11-19 ~ 2026-11-21）的完整离线快照。
所有数字均来自本包内 `runs/as2w_v2_simreal_rootcause_20261119/tables/` 与原始日志；
文档结论见 `docs/as2w_v2_simreal_rootcause_20261119/13_PHASE_J_FINAL_DECISION.md`。

## 1. 软件版本（实测，见 `versions.txt`）

| 项 | 值 |
|---|---|
| Python | 3.12.15（`/home/robot/z00945176/isaac_env/venv312/bin/python`） |
| MuJoCo | 3.12.0 |
| NumPy | 2.3.1 |
| SciPy | 1.17.0 |
| PyTorch | 2.11.0 |
| Isaac Sim | 6.2.0.0（pip `isaacsim==6.2.0.0`，IsaacLab 同环境） |
| GPU | 2× RTX 5090，驱动 580.178.04 |
| OS | Linux 6.8.0-40-generic |

完整 pip 列表：`pip_freeze_venv312.txt`（230 个包）。**MuJoCo 脚本只需要 mujoco+numpy；
PhysX 脚本需要 Isaac Sim 6.2 + IsaacLab 的完整环境。**

## 2. 运行前必须设置的环境变量（PhysX 侧）

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export CUDA_VISIBLE_DEVICES=1
export LD_LIBRARY_PATH=/home/robot/miniconda3/lib     # 本机 Isaac 构建需要
```

MuJoCo 侧不需要 Isaac，也不需要上述变量（`MUJOCO_GL` 仅在渲染时需要，本包全部为无渲染物理计算）。

## 3. 路径重定位（重要）

所有脚本顶部硬编码了原仓库根：

```python
REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
```

在本机原路径可直接运行。换机器/换目录时，先执行：

```bash
bash env/relocate_paths.sh /新/的/包/根/绝对路径
```

该脚本会把 `scripts/*.py` 中的 `/home/robot/z00945176/Rob2Nav` 统一替换为给定路径。
`a1_parse_sessions.py` 另有两条绝对路径（V2 原始数据目录、历史 clip），同样会被替换；
替换后其相对位置与本包目录结构一致（`docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119`、
`runs/as2w-r4000-realcal-wheelrl-20261114/.../220051_fixedstand_hold/clip.npz`）。

## 4. 目录结构

```
as2w-sim2real-rootcause-bundle-20261121/
├── README.md                     # 本包导读
├── MANIFEST.md                   # 文件清单与大小
├── SHA256SUMS                    # 全包校验和
├── env/                          # 环境与复现脚本
├── docs/
│   ├── as2w_v2_simreal_rootcause_20261119/   # 主文档 00–13 + README + 汇总 CSV
│   ├── as2w_hip_gravity_contact_diagnosis_20261119/  # Phase H0–H6 原始诊断
│   ├── AS2W_悬空负载诊断V2_全部数据与分析_20261119/   # 实机原始 recordings + docs + builds
│   └── *.md                      # Phase A–F 复核、Codex prompt、外部咨询总结
├── runs/
│   ├── as2w_v2_simreal_rootcause_20261119/
│   │   ├── scripts/              # a1–a2, b1, c1, d1, e0–e3, f0–f3b, g2–g3, h2–h3, i1–i5
│   │   ├── tables/               # 所有 CSV/JSON 结果表（含 HYPOTHESIS_UPDATE_V2）
│   │   ├── qc/a1_timelines/      # 从实机日志解析出的 npz（P55G2 等）
│   │   ├── mujoco_* / mujoco_abl/ # MuJoCo 变体 XML + 运行输出
│   │   ├── physx_abl/            # PhysX URDF、run JSON、日志、USD 缓存
│   │   └── physx_usd_*/          # PhysX 场景 USD 缓存
│   └── as2w-r4000-realcal-wheelrl-20261114/.../clips/  # 历史 clip.npz（slot 映射校验用）
└── assets/
    ├── mujoco_meshes/            # as2w_scene.xml 引用的 STL（30 MB）
    └── urdf_meshes/              # URDF 引用的 STL（30 MB）
```

所有 `meshes` 符号链接已改为指向包内 `assets/` 的相对链接，解包即自洽，无需原仓库。

## 5. 复现入口（按阶段）

```bash
cd runs/as2w_v2_simreal_rootcause_20261119/scripts
PY=/path/to/python-with-mujoco          # 3.12 + mujoco 3.12
PYI=/path/to/isaac-sim-python           # 原机为 venv312

# A: 实机日志解析 -> qc npz + 窗口统计
$PY a1_parse_sessions.py && $PY a2_qc_windows.py

# B/C/D: 悬空重力、地面力账本、MuJoCo 闭环
$PY b1_suspended_gravity.py
$PY c1_ground_ledger.py
$PY d1_closed_loop_matrix.py

# E/F: PhysX 探针与消融
$PYI e1_physx_static.py
$PY  e2_mujoco_loadv0.py
$PYI f3_physx_ablation.py --variant A_loadv0_base
$PYI f3b_material_probe.py

# G: 跨引擎轮基元矩阵
$PY  g3_mujoco_matrix.py
$PYI g3_physx_matrix.py

# H: 执行器审计
$PY h2_torque_pd_analysis.py
$PY h3_torque_replay.py

# I: 公平对比 / 材质 / 分支 / 接触流形 / Go2-W 配置
$PYI i1_physx_explicit_pd.py
$PYI i2_material_runtime_check.py --check
$PYI i3_physx_branch_perturbation.py
$PY  i4_mujoco_contact_balance.py
$PYI i4_physx_contact_balance.py
$PY  i5_gow_contact_variants.py
```

PhysX 脚本均为「父进程 driver + 每 run 独立 app 进程」模式，直接运行脚本即可；
中途日志写入 `physx_abl/i*_driver.log` 与 `i*_logs/`。

## 6. 已知环境坑（原会话踩过）

1. `InitialStateCfg.joint_pos` 在 Isaac 6.2 的 reset 中**不生效**，必须 reset 后
   `write_joint_position_to_sim_index`（G3/I 系列已处理）。
2. 交付 MJCF 自带 position/velocity 执行器；用 `qfrc_applied` 做显式 PD 时必须把
   `actuator_gainprm/biasprm` 清零（G3/I5 已处理）。
3. `sim_utils.bind_physics_material` 在本构建返回 False；I2 改用
   `UsdShade.MaterialBindingAPI` 直接绑定。
4. 本机 venv312 的 ElementTree 缺 expat，XML 解析请用 grep/awk 或另装 lxml。
5. PhysX ContactSensor 不暴露逐接触点数据（只有合力 + 平均接触位置），
   I4 的逐点流形为 `UNKNOWN`。
6. 每个 PhysX run 必须独立进程；同进程连续 reset 会污染结果。
