# RPT-022 Phase Q：约束消融、往复微动与离线诊断收口

编号：`RPT-022`
日期：2026-11-22
上游：`AS2W_PhaseQ_最后一轮离线判别与Sim2Real收口_20261122.md`
明细：`docs/24_PHASE_Q_DISCRIMINATION_AND_STOP_REPORT.md`、`25_PHASE_Q_REAL_MEASUREMENT_TRIGGER_AND_POLICY_TRACK.md`、`ANSWERS_ROUND_5.md`
新增运行：2 条（锁基座 xy / 锁轮 q，各 260 s、20 Hz、dt 0.005、pitch_p2 同初态）

---

## 1. 结论

1. **末段是往复微动，不是持续滚动。** 末 60 s 的 `Σ|Δq|/|net|` = 8.45（hip）/ 19.5（pitch）；`pitch_p2` 后两轮 60 s 内来回 3 rad，净变化仅 0.12/0.02 rad，符号切换 565/551 次；滑移代理 P95 ≤9.4 mm/s。→ `OSCILLATORY_WHEEL_MICROMOTION`，原 0.02 rad/s 静止门保持 FAIL。
2. **锁基座 xy 与锁轮 q 都抑制了 180–240 s 的迁移**：基线 pitch 3.02→1.45°、基座 +62 mm、47 个失载采样；B 锁基座后 pitch 稳定 −2.94°（轮原地转 3.9 rad）；C 锁轮后 pitch 稳定 −0.35°。→ 事件是**滚动/约束耦合**，不是单一自由度或接触参数的产物。
3. **终态 pitch 对约束高度敏感**（+1.44 / −2.94 / −0.35），说明它不是接触模型的固有量。
4. **OG 记为 `OG_CONTACT_MODE_UNIDENTIFIED`**：四点不共面 10.88 mm、四轮 Fy 全跨零；不做新的三轮/软胎拟合。
5. **勘误**：Phase P 的 `compliance()` 只用 `ΣFn=W` 定高度，未强制力矩平衡；`k=2e4` 的 103/18/73/61 N 只是弹簧算例，不是闭环平衡；`t*=200` 是搜索上限。
6. **正式收口：`STOP_OFFLINE_DIAGNOSIS_PENDING_REAL_MEASUREMENT`。** 三分法三条同时成立；S1/S2/多圆柱 `CLOSED_UNTIL_INDEPENDENT_CONTACT_DATA`。

---

## 2. P1-B 结果

| 时刻 | A 基线 | B 锁基座 xy | C 锁轮 q |
|---|---|---|---|
| 60 s pitch | +2.97° | −2.936° | −0.367° |
| 180 s | +3.02° | −2.944° | −0.372° |
| 240 s | +1.46° | −2.942° | −0.351° |
| 260 s base x | +0.07 m | 0（锁定） | −0.007 m |
| 180–260 轮 q | 0.6–1.0 rad | FL +3.90 rad | 0 |
| 180–260 失载 | 47 | 0 | 0 |

按任务书规则：B/C 都抑制 → 滚动/约束耦合；**不能**判接触根因，也**不能**判电机故障。

---

## 3. Q1 轮角分解（末 60 s）

| 运行 | Σ\|Δq\|/|net| | 最大总变差 | 符号切换 | 滑移代理 P95 |
|---|---|---|---|---|
| hip_p002 | 8.45 | FL 1.63 rad | 335–612 | ≤4 mm/s |
| pitch_p2 | 19.5 | RL 3.12 / RR 2.94 rad | 335–565 | ≤9.4 mm/s |

→ 标签：`NEAR_STATIONARY_POSE_WITH_OSCILLATORY_WHEEL_MICROMOTION` / `..._LAST60_OSCILLATORY_MICROMOTION`；都不是静态平衡。

---

## 4. 下一步唯一有信息量的测量

**M2（P55 T2 四轮同步 Fn，每轮 U95 ≤1 N）**。M2 之后仍跨零才做 M4；髋台架只在 M2+M4 后账本仍不闭合且 U95 ≤0.3 Nm 时做。实机不可用时：冻结候选与证据表，不再设新仿真参数当"物理真值"。

---

## 5. 交付物

| 内容 | 路径 |
|---|---|
| 本报告 | `06_交付与报告/报告/RPT-022_PhaseQ_Stop_20261122.md` |
| 收口报告 24 | `04_诊断实验/phase_klm_20261122/docs/24_PHASE_Q_DISCRIMINATION_AND_STOP_REPORT.md` |
| 测量/策略轨道 25 | `.../25_PHASE_Q_REAL_MEASUREMENT_TRIGGER_AND_POLICY_TRACK.md` |
| GPT 回传 | `.../ANSWERS_ROUND_5.md` |
| 脚本/结果 | `runs/as2w_phase_q_20261122/`；服务器 `phase_q_20261122/results/*.json`（各 5.5 MB） |
