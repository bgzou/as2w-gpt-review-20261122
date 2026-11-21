# RPT-010 C 线深推：R2000A 偏航阈值、组合消融与"脆弱边界"

编号：`RPT-010`
日期：2026-11-22（Asia/Shanghai）
对象：R2000A `as2w_residual_run2000a`
任务：在 RPT-009（full vs base-only 基线）之后，补 **residual-only、资产回填、kp85、wz 扫描** 四个单变量，判断偏航失败是"能力缺失"还是"边界附近的脆弱行为"
一手文件：`04_诊断实验/闭环反事实/t2_1_yaw_contrast_20261122/results/*_20261122_03.json`、`code/run_mujoco_yaw_contrast_20261122_02.py`、`logs/*.log`
运行：自有服务器，隔离 venv，MuJoCo 4.0.1，CPU，单作业

---

## 1. 结论（四条，直接可用）

1. **策略并非"不会偏航"，但可用域不能用单一 wz 阈值描述**：纯偏航在 `wz = 0.10 / 0.20 / 0.30` 下 8 s 存活（tilt 3.0 / 3.1 / 18.9°，偏航累积 +15.9 / +62.8 / +112.6°）；**细扫 0.325 / 0.350 存活，0.375 摔（7.16 s），0.400 / 0.425 又存活，0.450 摔（5.80 s）**。⇒ 摔/不摔在 wz 上是**非单调且碎片化**的（见 §2.1），不能写成"wz>0.35 必摔"。`CONFIRMED`（同资产同 harness 单变量扫描）
2. **residual 单独不能产生运动**：residual-only（无 base）在 YAWL/YAWR 8 s 存活但 `wz_mean ≈ 0.005/−0.007`、偏航 ±2.5–3.1°，即**没有转向**。⇒ 转向能力由 base actor 提供，residual 只是叠加修正。`CONFIRMED`（诊断性单变量）
3. **失败与"是否摔"不是平滑函数，而是脆弱边界**：
   - 资产 base +1.012 kg / COM +15 mm：full 下 YAWL 摔倒从 5.58→**6.56 s**、YAWR 从摔→**存活**、MIX 从存活→**7.18 s 摔**；base-only 下 YAWL 从摔→**存活**、YAWR 仍摔。
   - 腿增益 55→85：full 下 YAWL 从摔→**存活**、YAWR 仍摔（5.50 s）；base-only 下 YAWR 摔得**更早**（4.50 s）。
   ⇒ 用"某个单因素导致偏航失败"的叙述**不成立**；正确表述是"系统在稳定性边界附近，±1 kg 或 ±30 kp 的改动就能改变哪个场景摔"。`CONFIRMED`（观测）；外推 `INFERRED`
4. **RPT-009 的因果结论维持**：base-only 也摔 ⇒ 偏航失败不需要 residual；residual 让 YAWL/YAWR 提前约 1 s 摔倒，是**加速项**而非根因。

## 2. 消融矩阵（MuJoCo 4.0.1，400 步 = 8 s）

| 变体 | 资产 | 腿增益 | STAND | YAWL | YAWR | MIX |
|---|---|---|---|---|---|---|
| full（基线，RPT-009） | 26.000 kg | 55/2 | 存活 2.2° | **摔 5.58 s**（roll −62.2°） | **摔 5.80 s**（roll +60.8°） | 存活 32.7° |
| base_only（基线） | 26.000 kg | 55/2 | 存活 2.2° | **摔 6.48 s** | **摔 7.02 s** | 存活 34.0° |
| residual_only | 26.000 kg | 55/2 | — | 存活 2.2°，`wz_mean +0.005` | 存活 2.0°，`wz_mean −0.007` | — |
| full + 资产回填 | 27.012 kg | 55/2 | 存活 2.6° | **摔 6.56 s** | **存活 20.0°** | **摔 7.18 s** |
| base_only + 资产回填 | 27.012 kg | 55/2 | 存活 2.6° | **存活 18.3°** | **摔 5.58 s** | 存活 32.1° |
| full + kp85 | 26.000 kg | 85/3.2 | 存活 3.8° | **存活 32.0°** | **摔 5.50 s** | 存活 26.1° |
| base_only + kp85 | 26.000 kg | 85/3.2 | 存活 3.8° | **存活 36.8°** | **摔 4.50 s** | 存活 36.4° |

**wz 扫描（full，vx=0，STAND 对照）**

| wz (rad/s) | 0 | 0.10 | 0.20 | 0.30 | 0.45 |
|---|---|---|---|---|---|
| 结果 | 存活 2.2° | 存活 3.0° | 存活 3.1° | 存活 18.9° | **摔 5.80 s**（tilt 60.2°） |
| 8 s 偏航累积 | −2.0° | +15.9° | +62.8° | +112.6° | +116.9°（摔中断） |

### 2.1 细扫（同一 harness，`t2_1_wz_fine_20261122_04.json`，SHA `ffd35323…`）

| wz (rad/s) | 0.325 | 0.350 | **0.375** | 0.400 | 0.425 |
|---|---|---|---|---|---|
| 结果 | 存活 | 存活 | **摔 7.16 s** | 存活 | 存活 |
| tilt_max | 27.6° | 22.9° | 62.2° | 32.9° | 25.7° |
| 8 s 偏航累积 | +114.8° | +127.6° | +119.5°（中断） | +146.3° | +137.6° |

⇒ **非单调**：0.375 摔而 0.400/0.425 存活、0.450 又摔。这不是"阈值"，而是**多个相邻命令点落在不同的吸引域/接触分支**；任何"把 wz 调到 X 以下就安全"的结论都不成立。

结果文件（SHA256 前 16）：

| 文件 | SHA |
|---|---|
| `t2_1_residual_only_20261122_03.json` | `076e9e0c527b9792` |
| `t2_1_asset_full_20261122_03.json` | `47c3b8cd5428ab87` |
| `t2_1_asset_base_20261122_03.json` | `e58520229d67fdf1` |
| `t2_1_kp85_full_20261122_03.json` | `14b36d144cec838f` |
| `t2_1_kp85_base_20261122_03.json` | `43bf4266f8fd9b41` |
| `t2_1_wz_sweep_20261122_03.json` | `04f7b9135e640a9c` |
| `t2_1_wz_fine_20261122_04.json` | `ffd353236ddcec5a` |
| `code/run_mujoco_yaw_contrast_20261122_02.py` | `0282bd04fc2dee16` |

本地与服务器 6 份 JSON `sha256sum` 一致（2026-11-22 回拉）。

## 3. 机理解读

- **边界**：`wz ≤ 0.30` 时侧倾角 <19°，还有余量；`0.45` 时侧倾单调增长到 60°。轮式底盘的横滚力矩≈轮速差×接地力臂，`wz` 越大轮速差越大，而 base actor 在偏航域输出以 ±1 为主（RPT-003：偏航场景 base_raw 帧级 100% 饱和），无法提供幅值调节 → 一旦横滚超过回复能力就不可逆。`INFERRED`
- **为何加质量会"救"某个场景**：质量增加降低重心相对轮距的比例（或不改变），同时改变接触力分配，使某些方向的横滚回复更强；但同一改动让 MIX 反而摔，说明这是**边界重排**而不是"变好"。`INFERRED`
- **kp85 的作用**：腿更硬 → 姿态修正更快，YAWL 从摔变存活；但 YAWR 反而更早摔（4.50 s），说明横滚方向存在**左右不对称**（与 D 线 zero-B 的左右不对称同源，候选：轮/腿质量分布、初始 y 偏移、接触求解）。`INFERRED`

## 4. 对交付证据与既有结论的修订

| 既有表述 | 本次结论 |
|---|---|
| "MuJoCo 偏航 5.2–7.3 s 摔倒" | 在 26.0 kg + kp55 下复现 YAWL/YAWR（5.58/5.80 s），MIX 存活（交付记 6.10 s 摔）→ 部分复现；MuJoCo 3.17.0 与 4.0.1 逐位相同，**版本不是解释** |
| "偏航失败不能归 Sim2Real" | 维持（仿真内也摔） |
| "residual/gate 是因果候选" | **下调为加速项**：base-only 也摔；residual-only 无转向 |
| "R2000A 不能偏航" | **不成立**：`wz ≤ 0.30` 存活并有真实偏航累积；`0.45+` 才摔 |
| 实机 yaw 锁（AGENTS 2026-11-19） | **维持**：仿真边界脆弱 + 实机未做完整偏航包络，不能解锁 |

## 5. 限制与下一步

- 限制：单一 MJCF（与训练 URDF 差 1.512 kg 总量）、单次运行、无种子、未做接触参数（friction 1.0 / solref / solimp）扫描、未做 vx–wz 组合网格、未做初始 y 偏移。
- 下一步（若要继续 C）：
  1. **wz 细扫** 0.30–0.45（步长 0.025）定位阈值与首次侧倾发散时刻；
  2. **左右对称性**：初始 y 偏移 ±5 mm、左右腿质量 ±2%，检验是否解释 YAWR 早摔；
  3. **接触参数**：friction 0.6/0.8/1.0、solref 缩放，检验阈值是否移动；
  4. 与 D 线共用"kp 敏感性"结论，写进交付合同检查项（zero-B 记录 `cmd−actual`，偏航记录 tilt 余量）。

## 6. 复现

```bash
EXP=/root/gpufree-data/as2w-cpro/systematic_project_audit_20261121/experiments/t2_1_yaw_contrast_20261122
PY=/root/gpufree-data/as2w-cpro/systematic_project_audit_20261121/experiments/t3_3_zerob_sim_20261122/venv/bin/python
cd $EXP
# 资产回填
$PY code/run_mujoco_yaw_contrast_20261122_02.py --package-root package --scenarios STAND,YAWL,YAWR,MIX \
  --steps 400 --work-dir w_a1 --base-mass 11.592 --base-com-x 0.020002 --variants full \
  --output results/t2_1_asset_full_20261122_03.json
# wz 扫描
$PY code/run_mujoco_yaw_contrast_20261122_02.py --package-root package --steps 400 --variants full \
  --work-dir w_d --scenarios STAND --extra-scenarios "WZ010:0:0:0.10;WZ020:0:0:0.20;WZ030:0:0:0.30;WZ045:0:0:0.45" \
  --output results/t2_1_wz_sweep_20261122_03.json
```
