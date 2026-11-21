# AS2-W FixedStand 腿 kp=55 / kd=2：实测姿态、力矩核对与实机 check

**打包日期**：2026-11-14 ｜ **数据截止**：2026-11-13 ｜ **对象**：AS2-W 16 轴（CPRO 低层链）

**一句话结论**：A 键 FixedStand 的腿 `kp=55/kd=2` 是项目自定的低层位置 PD 增益，不是官方默认值、也不是跨策略常量。在 T2 目标下，机器人实际稳定在 **pitch −4.73°～−5.50°**、相对 T2 逐轴最大误差 **0.263～0.270 rad**；这是 `τ_gravity/kp` 决定的确定性 PD 重力平衡点，`kd` 在稳态几乎不起作用。B 门槛 15 次实测 `pose_error` 为 **0.257～0.307 rad**，门槛 0.25→0.30→0.35 的每次调整都有实测依据。该稳态会作为策略接管初态，并与 R214 行走塌陷共享“实机质心偏后 + kp 有限”的机制。

> 本包只做离线数据整理与复核：不连接 DDS、不 ReleaseMode、不发布 LowCmd、无机器人动作。

---

## 1. 这个包回答什么

1. `kp=55/kd=2` 写在源码哪里、作用于哪些状态、启动自检如何校验；
2. T2 目标下的 FixedStand 实际姿态（逐轴 q / τ_est）与原厂 T1、训练 T2 的三方对照；
3. 四次独立 A→FixedStand 是否可重复、稳态是否静止；
4. 为什么 55/2 会产生约 −5° 后仰，`kd` 为什么不是主因；
5. 实机 B 门槛 15 次 `pose_error` 的分布、门槛演化历史、以及“0.25 会拒绝全部 15 次”这一事实；
6. Debug LowCmd 逐帧证据：腿 55/2 与 T2 确实下发到了电机；
7. 该稳态与 R214 行走塌陷（后腿 `cmd−actual` 0.34–0.45 rad、`55×0.40≈22 Nm`）的关系；
8. 55/2 的边界与工程建议（不要盲目降 kp；改增益算新版本）。

详细正文：`report/01_FixStand腿kp55_kd2实测姿态对照_20261114.md`（459 行）。
原始登记摘录：`report/02_INDEX摘录_FixedStand相关.md`；2026-11-10 历史专项分析：`report/03_20261110_原厂站姿与FixedStand对照_历史分析.md`。

---

## 2. 关键数字（先看这张表）

| 项目 | 结果 | 来源 |
|---|---|---|
| FixedStand 实际 pitch | −4.732°～−5.498°（4 次） | 4 个 stand trial |
| 相对 T2 最大逐轴误差 | 0.263–0.270 rad（RMS 0.174–0.190） | 同上 |
| 相对 T1 最大逐轴误差 | 0.633–0.639 rad | 同上 |
| 原厂站姿 pitch | +0.326°，相对 T1 误差 0.0075 rad | 611 帧原厂窗 |
| T1 与 T2 目标差 | 最大 0.374 rad（约 21°） | 目标常量 |
| 稳态腿 dq | 最大 0.0276 rad/s；`kd×dq≈0.06 Nm` | 主样本 1 s 窗 |
| 后小腿保持力矩 | `τ_est` 14.5 Nm；`55×0.263≈14.5 Nm` | 主样本 |
| B 门槛 `pose_error` | 15 次 0.257–0.307，均值 0.282，σ 0.016 | 15 个 publisher.log |
| FixedStand LowCmd 帧 | 1820 帧腿 `kp=55/kd=2`、q=T2、`τ_ff=0`；轮 `kp=0/kd=2` | debug_lowcmd.csv |

---

## 3. 包内结构

```text
FixedStand_kp55_kd2_real_robot_20261114/
├── README.md                         ← 本文件
├── MANIFEST.sha256                   ← 全部文件 SHA-256
├── report/
│   ├── 01_FixStand腿kp55_kd2实测姿态对照_20261114.md
│   ├── 02_INDEX摘录_FixedStand相关.md
│   └── 03_20261110_原厂站姿与FixedStand对照_历史分析.md
├── data/
│   ├── README.md                     ← 每个原始文件的说明、采样、字段
│   ├── stand_trials/
│   │   ├── 20261109-185025_factory/          low_capture_fast.log
│   │   ├── 20261110-100032_trial2/           low_capture_fast.log
│   │   ├── 20261110-101251_trial3/           low_capture_fast.log + low_protection.log
│   │   ├── 20261110-155128_stand/            low_capture_fast.log + low_protection.log
│   │   └── 20261110-193947_r284-zero-b/      low_capture_fast.log + low_protection.log
│   ├── b_gate/                       ← 15 个会话的 publisher.log（每个一条 B accepted）
│   └── lowcmd/                       ← 220051 会话的 debug_lowcmd.csv + publisher.log + session.log
└── tools/
    ├── analyze_stand_pose_comparison.py   ← 姿态/力矩对照（项目原脚本）
    ├── extract_b_gate.py                  ← 重新提取 15 次 B 门槛并统计
    └── verify_lowcmd_gains.py             ← 重新核对 FixedStand/FixStand 的 LowCmd 增益
```

> `report/01` 第 12 节里的路径是**项目内路径**（`04_调试记录/...`、`部署参考/...`）。包内对应关系见上面的树；`data/README.md` 有逐文件映射。

---

## 4. 三分钟复现

### 4.1 姿态与力矩对照

```powershell
python tools/analyze_stand_pose_comparison.py `
  data/stand_trials/20261109-185025_factory/low_capture_fast.log `
  data/stand_trials/20261110-101251_trial3/low_capture_fast.log `
  data/stand_trials/20261110-100032_trial2/low_capture_fast.log `
  data/stand_trials/20261110-155128_stand/low_capture_fast.log `
  data/stand_trials/20261110-193947_r284-zero-b/low_capture_fast.log
```

预期关键输出：

```text
factory_selected rows=611 span=86.66..92.96s pitch=0.326deg
fixed_selected rows=101 span=42.50..43.50s pitch=-5.498deg
distance factory->T1: max=0.007500 rms=0.003341
distance fixed->T1:   max=0.639310 rms=0.399303
distance fixed->T2:   max=0.270496 rms=0.190198
history 20261110-101251...,-5.498,0.270496,...
history 20261110-100032...,-4.969,0.262920,...
history low_capture_fast.log,477,23.47..28.23,-5.147,...   ← 155128 会话
history low_capture_fast.log,1502,46.89..61.92,-4.732,...  ← 193947 会话
```

### 4.2 B 门槛 15 次统计

```powershell
python tools/extract_b_gate.py data/b_gate
```

预期：`n=15 mean=0.281972 pstdev=0.015570 min=0.257115 max=0.307206`，并打印每次会话的值。

### 4.3 LowCmd 增益核对

```powershell
python tools/verify_lowcmd_gains.py data/lowcmd/20261112-220051_training-contract_debug_lowcmd.csv
```

预期：FixedStand `frames=1820`、12 腿 `kp=55.0/kd=2.0`、q=T2、`tau_ff=0`、轮 `kp=0.0/kd=2.0`；FixStand 段 `20/3` 894 帧、`55/2` 586 帧、`0/3` 11 帧。

### 4.4 校验文件完整性

```powershell
# 在包根目录；Git Bash / WSL:
sha256sum -c MANIFEST.sha256
# 或 PowerShell（用 .NET 按 UTF-8 读清单，避免中文文件名被 ANSI 误读）:
[IO.File]::ReadAllLines('MANIFEST.sha256') | ForEach-Object {
  $h,$p = $_ -split '  ',2
  if ((Get-FileHash -Algorithm SHA256 -LiteralPath $p).Hash.ToLower() -ne $h) { "FAIL $p" }
}
```

---

## 5. 数据来源与会话

| 包内目录 | 原始位置 | 内容 | 本文用途 |
|---|---|---|---|
| `stand_trials/20261109-185025_factory` | `04_调试记录/logs/20261109-185025_windows_LowCaptureFast.log` | 原厂高层站立，86.66–92.96 s 稳定窗 611 帧 | T1/原厂姿态基准 |
| `stand_trials/20261110-100032_trial2` | `04_调试记录/logs/20261110-100032_windows_LowCaptureFast.log` | 第 2 次 A→Y，42.08–45.20 s 稳定 | 重复性 |
| `stand_trials/20261110-101251_trial3` | `04_调试记录/logs/20261110-101251_windows_LowCaptureFast.log` + `LowProtection` | 第 3 次 A→Y（修复版），42.50–43.50 s 主样本 | 主样本/力矩 |
| `stand_trials/20261110-155128_stand` | `04_调试记录/logs/field_sessions/20261110-155128_stand/` | 一键 stand 会话 | 重复性 |
| `stand_trials/20261110-193947_r284-zero-b` | `04_调试记录/logs/field_sessions/20261110-193947_r284-zero-b/` | R284 zero-B 会话，B 未按；本机 capture 为当时已回收部分 | 重复性（仅用 46.89–61.92 s） |
| `b_gate/*_publisher.log` | 各 `field_sessions/<会话>/publisher.log` | 发布端 stdout，含 `B accepted` 行 | 15 次门槛 |
| `lowcmd/20261112-220051...` | `field_sessions/20261112-220051_training-contract/` | 500 Hz LowCmd 逐帧 | 增益下发核对 |

会话时间窗、采样率、字段和槽位映射见 `data/README.md`。

---

## 6. 结论（正文见 report/01）

1. **55/2 的含义是明确的**：FixedStand 12 腿位置 PD 为 `kp=55,kd=2`，轮 4 轴 `kp=0,kd=2`、`dq=0`；FixStand 两段为 `20/3`（1.8 s）→ `55/2`（1.2 s）。`--offline-audit` 会检查 `kp==55`。
2. **实际姿态是可重复的 PD 平衡点**：pitch −4.7～−5.5°，相对 T2 最大 0.263–0.270 rad；四次独立起立一致，稳态 `dq<0.03 rad/s`。
3. **力矩核对成立**：`55×(T2−q)` 与 `τ_est` 逐轴差 <0.3 Nm；后小腿约 14.5 Nm。kd 在稳态贡献约 0.06 Nm。
4. **“不像原厂”首先是目标不同**：T1 与 T2 相距最大 0.374 rad；即使完美跟踪 T2 也不会等于 T1。
5. **B 门槛不是姿态质量指标**：0.25 会拒绝全部 15 次；0.30 时 0.3033 仅超 0.003 rad；0.35 对 0.307 留 0.043 rad 余量，其他门槛未放宽。
6. **与 R214 行走塌陷同源**：后腿 `cmd−actual` 锁在 0.34–0.45 rad，`55×0.40≈22 Nm` 与实测 21 Nm 吻合；回站误差增至 0.7 rad → 38+ Nm，反而能拉正。
7. **不要默认改 55/2**：降 kp 会加重下沉；提 kp 只是治标且未做真机增益扫描。优先重力前馈、目标补偿或独立 `PolicyReadyStand`；任何改动都算新版本，需重新编译/audit/仿真/有界真机验收。

---

## 7. 边界与未验证

- `tau_est` 是驱动器估计力矩，不是足端接触力；本包不据此推断地面反力。
- 没有机身高度真值；后仰角由 IMU 四元数计算。
- 只有一台 AS2-W；未做温度、电压、负载对稳态误差的影响统计。
- 原厂控制器内部 `q_cmd/kp/kd/tau_ff` 不在 LowState 中，不能反推原厂增益。
- T1 目标从未在 FixedStand 真机链上跑过；已归档会话全部 `--stand t2`。
- 没有“只改 kp”或“只改 kd”的真机对照；报告 5.5 节的 kp 敏感性表是 `err=τ/kp` 的算术估算。
- B 门槛 0.35 是“实测上界 + 余量”，不是理论安全边界。

---

## 8. 版本与校验

- 文档版本：2026-11-14 初版。
- 数据全部来自工作站权威目录 `部署测试/04_调试记录/logs/`；本包复制后未改内容。
- 包内所有文件（除 `MANIFEST.sha256` 本身）的 SHA-256 见 `MANIFEST.sha256`。
- 校验方式见 §4.4。
