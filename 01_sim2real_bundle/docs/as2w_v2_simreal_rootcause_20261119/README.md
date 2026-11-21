# AS2-W V2 实机数据驱动的 Sim2Real 力学根因分析（2026-11-19/20）

本目录回答一个问题：**为什么 AS2-W 实机地面 FixedStand 的 hip 力矩是 ~11–15 Nm，而悬空只有 2.7–3.0 Nm 且反号；MuJoCo（含轮心修正）与 Isaac/PhysX 能否在相同姿态/控制/支撑下复现？**

输入：V2 原始数据包 `docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119`、H0/H6 历史报告、当前训练资产链（residual1220 / load-v0 URDF）。全部结论可追溯到本节末的脚本、SHA 与运行命令。

---

## 1. 文档索引

| 文件 | 内容 | 关键证据 |
|---|---|---|
| `00_SOURCE_AND_CONTRACT.md` | 数据来源、控制合同、坐标/符号约定、精炼窗 | RAW_DATA_CONFIRMED |
| `01_SIGN_TIME_QC.md` | 全关节符号/时基 QC；V2 hip sign=+1 定案 | RAW_DATA_CONFIRMED |
| `02_SUSPENDED_GRAVITY_VALIDATION.md` | 悬空固定基座重力核验（g_hold=+qfrc_bias） | MODEL_CALCULATION |
| `03_GROUND_6D_FORCE_LEDGER.md` | 地面 6D 基座 + 12 关节受力账本（M0/M1/M2 反演） | MODEL_CALCULATION / INFERENCE |
| `04_MUJOCO_CLOSED_LOOP_MATRIX.md` | MuJoCo 闭环：C0 vs C1（H5 轮心修正）× S/G/GO | SIM_RUN_CONFIRMED |
| `05_ISAAC_PHYSX_PARITY_AND_SIMREAL.md` | PhysX 训练资产读回 + 双引擎交叉 + 同资产对照 | SIM_RUN_CONFIRMED |
| `06_ROOT_CAUSE_DECISION.md` | 十问直答 + 假设重排 + A–E 最终选择 | 综合 |
| `07_PHASE_G_REAL_MEASUREMENT_SOP.md` | 实机测量 SOP（四轮称重/hip 扫角/接触斑） | — |
| `08_PHASE_G_MEASUREMENT_TEMPLATE.csv` | 测量记录模板 | — |
| `09_PHASE_G_GO2W_REFERENCE_AUDIT.md` | Go2-W 开源项目审计与对照表 | SOURCE_CONFIRMED |
| `10_PHASE_G_DUAL_ENGINE_CONTACT_IDENTIFICATION.md` | G2 勘误 + G3 跨引擎矩阵 + 九问终答 | SIM_RUN_CONFIRMED |
| `11_PHASE_H_ACTUATOR_TORQUE_AUDIT.md` | H1 源码审计 + H2 日志重析 + H3 力矩回放 + H4 结论 | RAW_DATA/SOURCE/SIM_RUN_CONFIRMED |
| `12_PHASE_I_PHYSX_BRANCH_DIAGNOSIS.md` | I1 公平对比 + I2 材质绑定 + I3 分支稳定性 + I4 接触流形 + I5 Go2-W 配置 + I6 七指标 | SIM_RUN_CONFIRMED |
| `13_PHASE_J_FINAL_DECISION.md` | 一页式结论 + 九假设对照 + 仿真基准五问 + 实机补测清单 + RL 结论 | 综合 |
| `RECOMMENDED_NEXT_ACTIONS.md` | 实机补测（四轮称重、单腿 hip 扫角等） | — |

> 任务书模板中的 `03_GROUND_CONTACT_INVERSE_DYNAMICS.md` / `04_MUJOCO_ASSET_AND_CLOSEDLOOP_ABLATION.md` 在本目录对应 `03_GROUND_6D_FORCE_LEDGER.md` / `04_MUJOCO_CLOSED_LOOP_MATRIX.md`（内容等价，文件名沿用实际产物）。

## 2. 运行产物索引

```text
runs/as2w_v2_simreal_rootcause_20261119/
  scripts/
    a1_parse_sessions.py       # 原始日志 -> 时间线 npz
    a2_qc_windows.py           # 精炼窗统计
    b1_suspended_gravity.py    # 悬空重力核验
    c1_ground_ledger.py        # 地面 6D/12 关节反演
    d1_closed_loop_matrix.py   # MuJoCo C0/C1 闭环
    e1_physx_static.py         # PhysX S/G/GO 静态同合同
    e2_mujoco_loadv0.py        # MuJoCo C1m/C1m_mass 同资产对照
    e3_build_tables.py         # 交叉表生成
    f1..f3 / g2..g3_*          # Phase F/G 消融与跨引擎矩阵
    h2_torque_pd_analysis.py   # Phase H2 tau_PD vs tau_est
    h3_torque_replay.py        # Phase H3 力矩回放（A/B/C/D）
    i1_physx_explicit_pd.py    # Phase I1 PhysX 显式 vs 隐式
    i3_physx_branch_perturbation.py  # Phase I3 分支扰动
    i4_physx_contact_balance.py      # Phase I4 PhysX 接触合力
    i4_mujoco_contact_balance.py     # Phase I4 MuJoCo 逐点流形+平衡残差
    i5_gow_contact_variants.py       # Phase I5 Go2-W 配置 + I1 MuJoCo implicit
  tables/
    A_* B_* C_* D_* E_* G2_* G3_* H2_* H3_* I1_* I2_* I3_* I4_* I5_*
    MUJOCO_REAL_SIM_MATRIX.csv
    E_PHYSX_S/G/GO.json
    E_MUJOCO_LOADV0.json
    ALL_12JOINT_ENGINE_COMPARISON.csv
    PHYSX_MUJOCO_ASSET_PARITY.csv
    HYPOTHESIS_UPDATE_V2.csv   # 九假设 + h_verdict/i_verdict/final_verdict
  qc/a1_timelines/*.npz        # 原始时间线（只读派生物）
  mujoco_c1/ mujoco_c1m/ mujoco_c1m_mass/ mujoco_s*/  # 修正 MJCF 副本
  physx_usd_s/g/go/            # URDF->USD 转换缓存
```

## 3. 关键结果（5 条）

1. **悬空 hip 2–3 Nm = 重力项**：PhysX 与 MuJoCo 同符号同幅值（hip 残差 ≤0.26 Nm、位形 ≤0.004 rad；12 关节 RMSE 0.52 Nm）。悬空域定案。
2. **地面 hip 反号 = 接触项**：P55G2 接触项 −15.8（前）/−19.4 Nm（后），是重力项 +4.0 的约 4 倍且反号；姿态变化不能单独解释。
3. **H5 轮心修正必要但不充分**：MuJoCo C0→C1 后腿 hip −4.1→−11.7 Nm、pitch −2.4→−6.6°；但 PhysX 训练资产已含正确轮心（STL 网格、有效轮距 0.44 m）仍只给出前 −1.5/后 +1.5 Nm。
4. **地面域差距在接触模型**：同一资产（C1m，27.512 kg）MuJoCo 与 PhysX 定性分叉（G 后 hip −17.6 vs +1.5；GO 四腿反号）。PhysX 的侧向摩擦向内、抵消法向力矩；实测需要外撑。
5. **load-v0 的 CoM −15 mm 不被地面数据支持**：官方 CoM 的 C1 复现实测 pitch/Fn 最好（−6.58 vs −6.13；49/78 vs 47/80），C1m 过冲到 −18.7°/103 N。

## 4. 复现命令

```bash
# 环境（GPU0 被占用时用 GPU1）
export CUDA_VISIBLE_DEVICES=1
export LD_LIBRARY_PATH=/home/robot/miniconda3/lib
export OMNI_KIT_ACCEPT_EULA=YES
PY=/home/robot/z00945176/isaac_env/venv312/bin/python

cd runs/as2w_v2_simreal_rootcause_20261119/scripts
$PY e1_physx_static.py --scen S      # 悬空（无地面，固定基座）
$PY e1_physx_static.py --scen G      # 地面 P55 T2
$PY e1_physx_static.py --scen GO     # 地面官方
$PY e2_mujoco_loadv0.py              # MuJoCo 同资产 C1m/C1m_mass
/home/robot/miniconda3/bin/python3 e3_build_tables.py
```

**重要 harness 事实**：Isaac Lab 裸 `InteractiveScene` 循环必须 `scene.write_data_to_sim()` 后再 `sim.step()`，否则执行器目标不进 PhysX（关节塌限位、`applied_torque` 全零）；S 场景必须完全移除地面（固定基座 z=0.05 时轮子仍在 z<0）。

## 5. 资产 SHA 与来源

| 资产 | 路径 | SHA / 说明 |
|---|---|---|
| 训练 URDF | `/tmp/as2w-unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf` | `7c9740a95042bad7b73cadfda90bdc7c1873ac766afbb7c0e7e492367d45afb0` |
| MuJoCo 交付 MJCF | `deliveries/as2w-t15-layered-delivery-20261117/simulation/mujoco/as2w_scene.xml` | C0 原样；C1 加 `pos="0 ±0.0546 0"` |
| 实机原始数据 | `docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119` | 只读，未修改 |

## 6. 证据等级

`RAW_DATA_CONFIRMED`（实机日志直读）· `SIM_RUN_CONFIRMED`（仿真实际运行）· `MODEL_CALCULATION`（模型内计算/反事实）· `INFERENCE`（归因推断）· `SOURCE_CONFIRMED`（源文件读回）· `UNKNOWN`（无独立实测）。

## 7. 已知限制

- 实机四轮 Fn/Fy 是 **模型反演，非测量**（Phase C 标注 INFERENCE）；本报告未把它当作实测。
- PhysX 的 `ContactSensorCfg` 只给刚体合力矢量，接触点由「轮体位姿 + STL 已知偏置」重建（MODEL_CALCULATION），不是直接接触点读回。
- 未做 hip 扫角，因此 **M3（侧向接触）与 M5（hip 零位/摩擦）仍不可区分**；这是 `RECOMMENDED_NEXT_ACTIONS.md` 两项测量要解决的核心盲点。
- 未使用任何行走策略结果替代固定站立验证；R4000/12D 合同差异标注为 NOT_APPLICABLE。
