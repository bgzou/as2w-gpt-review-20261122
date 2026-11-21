# 00 数据源与控制合同核验（Phase A）

日期：2026-11-20。所有数值由本仓库脚本从原始日志逐帧复算，标 RAW_DATA_CONFIRMED（直接来自 LOW_SAMPLE/LOWCMD_SAMPLE/debug_lowcmd）或 SOURCE_CONFIRMED（来自 V2 包文档并被复算印证）。

## 1. 数据源

- 包：`docs/AS2W_悬空负载诊断V2_全部数据与分析_20261119/`（recordings/ 3 官方会话 + field_sessions/ 5 会话）
- 七会话清单与 low_capture_fast.log SHA（前 16 位）：

| tag | 目录 | 类型 | 支撑 | 行数 | 时长 s | ~Hz | sha16 |
|---|---|---|---|---|---|---|---|
| OG | recordings/20261119-153922_official-og-trans | official | ground | 59992 | 120.0 | 500 | 9684306fed3670d3 |
| OS | recordings/20261119-165542_official-os | official | suspended | 29992 | 60.0 | 500 | 37486ea41cb0fa7b |
| P55G1 | field_sessions/20261119-182802_p55-hold | hipdiag | ground | 21529 | 43.1 | 500 | 975506f6c93f3b03 |
| P55G2 | field_sessions/20261119-185930_p55-hold | hipdiag | ground | 16301 | 32.6 | 500 | 73c132dbe6dc3a39 |
| P55G3 | field_sessions/20261119-190012_p55-hold | hipdiag | ground | 16188 | 32.4 | 500 | 765025a5784caa0f |
| P55S | field_sessions/20261119-194344_p55-hold | hipdiag | suspended | 61342 | 122.7 | 500 | 93b1933dde3d4e89 |
| P85DS | field_sessions/20261119-194837_p85d-hold | hipdiag | suspended | 67631 | 135.3 | 500 | 561e31aa75eaff6d |

全部会话 500 Hz、packet_count 单调、无缺帧尖峰（a1 解析 assert 全过）。

## 2. 槽位/符号映射（V2 修正版，证据见 04）

- 槽位源：历史实机 clip `runs/as2w-r4000-realcal-wheelrl-20261114/.../220051_fixedstand_hold/clip.npz` meta：`dds_slots=[3,0,9,6,4,1,10,7,5,2,11,8(,13,12,15,14 轮)]`，joint_names=[FL,FR,RL,RR]×[hip,thigh,calf]+轮。a1 内嵌单测 assert 一致（PASS）。
- **V2 hip 符号修正（本套关键修正，Phase D 定案）**：V2 构建（2026-11-19 固件/发布端）的 hip 模型坐标 = 原生值**不反号**（sign=+1），与历史 clip 的 hip sign=−1 相反。thigh/calf/轮 sign 不变（+1）。证据链（04 文档详述）：
  1. **悬空下垂方向**（物理锚点）：MuJoCo 模型中 hip q<0 = 腿向髋枢轴正下方内摆（几何验证）；重力必使外偏的腿/轮质量向内下垂。P55S/P85DS 实测原生 hip 下垂为负（−0.052/−0.036 FL）→ sign=+1 时模型坐标 −0.052 与闭环仿真收敛 −0.0524、力矩 +2.77 vs +2.88 一致；
  2. **官方目标对称内收**：官方实测原生目标 m3(FL)/m9(RL)=−0.15、m0(FR)/m6(RR)=+0.15——sign=+1 时模型坐标 [−0.15,+0.15,−0.15,+0.15] 为四腿对称内收站姿（物理合理）；sign=−1 时会得到非对称位形；
  3. sign=+1 后 Phase B 悬空重力残差从 ±6 Nm 降到 0.02–0.46 Nm。
- **纪律**：V2 数据不得套用历史 clip 的 hip sign=−1；V2 与历史两套模型坐标的 hip 列不可直接相减（符号相反）；本文档套所有表均为**V2 修正模型坐标**。

## 3. 控制合同（实测，RAW_DATA_CONFIRMED）

| 会话 | kp/kd（腿） | tau_ff | 目标（模型坐标 12 腿） | 证据 |
|---|---|---|---|---|
| OG (ai_stand_up) | 200 / 4 | 0 | [-0.15, 0.15, -0.15, 0.15, 0.6×4, -1.2×4] | lowcmd_capture 逐帧；站立窗内恒定 |
| OS | 200 / 4 | 0 | 同上 | 同上 |
| P55G1-3 / P55S | 55 / 2 | 0 | [0×4, 0.8×4, -1.6×4] | debug_lowcmd.csv FixedStand 行（每契约轴 7 字段 slot,mode,q,dq,kp,kd,tau），FixedStand 行数 P55S=51134 |
| P85DS | 85 / 3.2 | 0 | 同上 | 同上，kp uniq {85}、kd uniq {3.2} |

- 官方原生目标 hip：FL=-0.15/FR=+0.15/RL=-0.15/RR=+0.15；V2 hip sign=+1 → 模型坐标即原生值 [-0.15, +0.15, -0.15, +0.15]（四腿对称内收，见 §2 证据 2）。thigh 0.6、calf -1.2 不变。
- 全部合同会话 16 契约轴 mode=1、目标 std=0、tau_ff=0。
- 发布端 tick 与状态流发布窗内末帧差 ≤10 ms（V2 validate_session 结论，SOURCE_CONFIRMED）。

## 4. PD 一致性（st_tau ≈ kp·e_q，稳态）

修正官方目标符号后（a2 TGT 与 a1 同步），窗口内逐关节 `pd_resid = mean(tau - kp·(tgt-q))`：
- OG hip：0.042 / 0.135 / -0.123 / -0.034 Nm（≪4.3–5.8 Nm 力矩本身）
- hipdiag 会话 hip pd_resid 0.03–0.27 Nm
- 结论：地面/悬空会话的 tau_est 通道与位置误差、声明增益完全自洽，tau_est 可作为关节力矩真值用于后续静态分析（残差量级 0.1–0.3 Nm 进入误差预算）。

## 5. 产物

- `REAL_V2_ALL_JOINT_WINDOWS.csv`（docs/…rootcause_20261119/，7 会话×12 关节×精炼窗，含 pd_resid）
- `runs/…/tables/`：A_SESSION_INVENTORY.csv、A_SUMMARY_MODEL_COORDS.json、A_REFINED_WINDOWS_MODEL_COORDS.json、A_SEG10_WINDOWS.csv、A_CHANGEPOINT_HIP.csv
- `runs/…/qc/a1_timelines/{tag}.npz`：500 Hz 时间序列（模型坐标 q/dq/tau/e + IMU quat/roll/pitch + temp），供 Phase B-F 使用
- 脚本：`runs/…/scripts/a1_parse_sessions.py`、`a2_qc_windows.py`
