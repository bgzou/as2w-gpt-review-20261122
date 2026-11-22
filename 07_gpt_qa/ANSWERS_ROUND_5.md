# ANSWERS_ROUND_5 — Phase Q 最终回传（收口）

日期：2026-11-22
状态：**`STOP_OFFLINE_DIAGNOSIS_PENDING_REAL_MEASUREMENT`**
新增运行：2 条（B 锁基座 xy、C 锁轮 q，各 260 s、20 Hz、`dt=0.005`、pitch_p2 同初态）。
未启动 RL、未改正式资产、未碰实机。

---

## Q1｜P1-B 允许确认哪些运动机制是必要通道？哪些仍无法辨识？

**确认（`SIM_RUN_CONFIRMED`，同初态同合同）**：

| 变体 | 180–260 s pitch | 基座 x | 轮 q | 失载采样 |
|---|---|---|---|---|
| A 基线 | **−1.56°**（3.02→1.45） | +62 mm | 0.6–1.0 rad | **47** |
| B 锁基座 xy | −0.00° | 0（锁定） | FL **+3.90 rad**（原地转） | 0 |
| C 锁轮 q | +0.02° | +0.6 mm | 0（锁定） | 0 |

- **B 与 C 都抑制了迁移** → 该事件需要**基座平移 + 轮转的耦合**；它不是单一自由度的产物。
- 三种状态的静态 pitch 分别是 **+1.44 / −2.94 / −0.35°** → 终态 pitch 对约束高度敏感，**不是接触模型的固有属性**。
- B 下轮子原地转 3.9 rad → 锁基座把能量推进了轮子；**这是虚构约束的产物**。

**仍无法辨识**：接触参数（μ/CoP/柔顺/力偶）对事件的贡献；哪一项真实物理机制（轮地切向漂移 / 胎面剪切 / 求解器正则化）主导；`tau_out`。

**不能**说"接触参数错"或"轮电机故障"。

---

## Q2｜末段是净滚动、往复抖动还是约束下微运动？

**往复抖动（`OSCILLATORY_WHEEL_MICROMOTION`），不是净滚动。**

末 60 s（20 Hz，`t∈[300,360]`）：

| 运行 | Σ\|Δq\| / \|net\| | 最大 TV | 符号切换 | 滑移代理 P95 |
|---|---|---|---|---|
| hip_p002 | **8.45** | FL 1.63 rad | 335–612 | ≤4 mm/s |
| pitch_p2 | **19.5** | RL 3.12 / RR 2.94 rad | 335–565 | ≤9.4 mm/s |

- `pitch_p2` 后两轮 60 s 内来回摆 3 rad，但净变化只有 0.12/0.02 rad；
- 纵向滑移代理 `vx_base + r·dq_wheel` 的 P95 ≤9.4 mm/s（**模型计算**，非真实接触点速度）；
- **原 0.02 rad/s 静止门保持 FAIL，不改门槛**。标签：
  - `hip_p002`: `NEAR_STATIONARY_POSE_WITH_OSCILLATORY_WHEEL_MICROMOTION`
  - `pitch_p2`: `ROLLING_AND_CONTACT_SWITCHING_HISTORY; LAST60_OSCILLATORY_MICROMOTION`

---

## Q3｜P55 / OG 的真实支撑与力矩误差能否仅靠现有数据判定？

**不能。**

- P55：四点水平不共面 4.266 mm、最佳斜面残差 1.94 mm；`t*` 2.157/1.384 Nm 只是**条件代数界**。
- OG：不共面 **10.880 mm**，四个 Fy 区间全跨零，`t*` 0.54 Nm —— **`OG_CONTACT_MODE_UNIDENTIFIED`**；不能判"OG 只有三轮受力"，也不能任选柔顺系数。OG 与 P55 是两种接触合同，**不能**把 0.54 与 2.16 解释成"同一台车的传动状态变化"。
- **勘误**：Phase P 的 `p0b_plane_support.py` 里 `compliance()` **只用 `ΣFn=W`** 定地面高度，没有同时强制整机力矩平衡与关节广义力矩。因此 `k=2e4 N/m` 的 103/18/73/61 N 只是**给定姿态与水平地面的弹簧算例**，不是闭环静态平衡；**不能**推出"真实胎必须这么软"或"k≥5e4 实机一定卸载"。表中 `t*=200` 是 LP 搜索上限/不可行标记，不是物理量。

---

## Q4｜下一次真正改变判断需要哪项独立实机测量？

**M2：P55 T2 四轮同步垂向载荷**（每轮 U95 ≤1 N），同时记录 12 关节 q、IMU、地面倾角、轮倾。

- 它是唯一能把"四轮是否同时承载、前后/左右分配"变成独立约束的测量；
- 只有 M2 后仍跨零，才轮到 M4（侧向 μ）或单轮 Fy 力板；
- 髋台架只在 M2+M4 后账本仍不闭合、且 U95≤0.3 Nm 时才做；
- M2 单独**不自动**重开 S1/CoP 建模；若 M2 否定四轮同时接触，先修接触集合。

---

## Q5｜是否进入 `STOP_OFFLINE_DIAGNOSIS_PENDING_REAL_MEASUREMENT`？

**是。**

- 三分法：`MODEL_INVALID`（四点不共面）、`NUMERICAL_OR_CONTROL_MODE_DEPENDENT`（锁消融改变事件与终态）、`INSUFFICIENT_INDEPENDENT_DATA`（Fn/CoP/μ/`tau_out` 未测）三条同时成立。
- 不再新增接触参数实验；S1/S2 与同轴多圆柱 `CLOSED_UNTIL_INDEPENDENT_CONTACT_DATA`。
- 重开条件：M2 + 至少一种独立接触区域/压缩/CoP 约束；或发现可验证的源代码错误；或用户明确改变目标。

---

## 附：G1-K 定稿

> **KNOWN_STATIC_CONTACT_AND_NUMERICAL_GAP: OPEN / MUST_DISCLOSE.**
> 条件 LP 残差 `t*`（μ=0.25）= 2.1573 Nm（26 kg）/ 1.3841 Nm（27.512 kg），不是轴端误差、物理下限或传动损耗。四点不共面（水平 4.266 mm，最佳斜面 1.94 mm），Fn/CoP/μ/`tau_out` 未测。PhysX `dt=0.005` 两条轨迹族在 180–240 s 有滚动 + RL 失载；末 60 s 未过轮速 P99 门，进一步分析为往复微动（TV/net 8.45/19.5）。锁基座/锁轮都抑制事件并改变终态 pitch → 滚动/约束耦合，不是接触参数的单一产物。`dt=0.0025` 末段静止性未验证。W_A 只作相对对照。
> 勘误：Phase N 旧值 1.9846/1.1231 Nm 为 `DEPRECATED_IMPL`。

---

## Artifacts

| 相对路径 | 类型 | 真正运行 |
|---|---|---|
| `09_phase_q/results/Q1_wheel_motion.json` | MODEL_CALCULATION | 是（读 Phase P 时序） |
| `09_phase_q/results/Q2_lock_ablation.json` | SIM_RUN_CONFIRMED | 是（2 条 260 s） |
| `09_phase_q/scripts/q1_wheel_motion.py`、`q2_lock_compare.py`、`i_common_lock.py` | SOURCE | — |
| `phase_q_20261122/results/q_p1b_lock_base_xy_260s.json` | SIM_RUN_CONFIRMED | 是（5.5 MB，20 Hz） |
| `.../q_p1b_lock_wheel_q_260s.json` | SIM_RUN_CONFIRMED | 是（5.5 MB，20 Hz） |
