# AS2-W 实机动力学数据包：Isaac Lab / MuJoCo 校验用

**打包日期**：2026-11-14 ｜ **对象**：AS2-W 16 轴（12 腿 + 4 轮） ｜ **数据**：4 个真机会话 + 20 个对齐 clip

**一句话**：这个包把真机的 **500 Hz LowCmd + 500 Hz LowState/IMU** 按状态切成可直接回放的 clip，用来判断 Isaac Lab / MuJoCo 里的**质量/CoM、关节 PD、轮-地摩擦、接触、行走闭环**是否与实机一致。

> 纯离线数据整理：不连接 DDS、不 ReleaseMode、不发布 LowCmd、无机器人动作。

---

## 1. 先看这 8 条

1. **最有用的四类测试**：静态 T2 保持、FixStand 阶跃、轮制动、FixSquat 落地。它们受策略闭环影响小。
2. **静态 T2 实测基线**（2026-11-12/13）：pitch 均值约 **−6.0°～−6.5°**，后小腿误差约 **0.30 rad**、`tau_est` 约 **16–17 Nm**；2026-11-10 早期为 **−4.7°～−5.5°**、约 14.5 Nm。**跨会话有约 1°/2 Nm 差异**。
3. **原厂站姿** `factory_stand`：pitch **+0.33°**，后小腿 `tau_est` 仅 5–7 Nm；**无 LowCmd**，只能做姿态/力矩参考。
4. **命令是 500 Hz**，腿位置模式（`cmd_q/kp/kd`），轮速度模式（`cmd_q=0, cmd_kp=0, cmd_dq` 是目标，`kd=2`），`cmd_tau=0`。
5. **`tau_est` 不是力矩真值**；仿真 `actuator_force` 只比趋势/量级。
6. **没有基座线速度、基座高度、足端接触力**；静态/蹲下可假设基座速度≈0，行走不行。
7. **行走数据是“策略+动力学+接触”的闭环**：先开环 replay 1–3 s，再同策略闭环；不要用 120 s 末端误差判断动力学。
8. **不要只拟合一个 clip**：用多个静态/阶跃 clip 取范围；`*_brake_return` 的回站目标是冻结策略姿态，不是 T2。

---

## 2. 包内结构

```text
SimDynamicsValidation_AS2W_20261114/
├── README.md                       ← 本文件
├── MANIFEST.sha256                 ← 全部文件 SHA-256
├── docs/
│   ├── 01_仿真动力学校验数据指南.md   ← 测什么、怎么测、怎么判读
│   └── 02_字段_关节序_对齐说明.md     ← 数组字段、关节序、符号、tick 对齐
├── data/raw/
│   ├── 20261112-220051_training-contract/   ← 静态/阶跃/制动/蹲下主数据
│   ├── 20261113-112630_r4000/               ← 30 s 行走
│   ├── 20261113-114335_r4000/               ← 120 s 行走/转向
│   └── 20261109-185025_factory/             ← 原厂站姿（无 LowCmd）
├── clips/
│   ├── index.csv                   ← 20 个 clip 的时长/状态/幅值/用途
│   └── <name>/clip.npz + meta.json ← 对齐后的命令+状态（500 Hz）
└── tools/
    ├── build_clips.py              ← 从 raw 重建 clips（可审计）
    ├── load_clip.py                ← 加载/打印
    └── check_clips.py              ← 校验 tick/对齐/NaN/范围
```

原始会话含 `debug_lowcmd.csv`（500 Hz 实际 LowCmd）、`low_capture_fast.log`（500 Hz LowState + IMU）、`low_protection.log`（约 1 kHz 保护/电流/电压）、`publisher.log`、`session.log`、`policy_diag.log`。

---

## 3. 20 个 clip 怎么选

| 测试 | 用哪些 clip |
|---|---|
| 静态重力 / CoM / 关节刚度 | `220051_fixedstand_hold`、`112630_fixedstand_hold`、`114335_fixedstand_hold`、`factory_stand`、`220051_prone_passive` |
| PD / 执行器阶跃 | `220051_fixstand_rise`、`112630_fixstand_rise`、`114335_fixstand_rise` |
| 轮-地摩擦 / 制动 | `220051_brake_return`、`112630_brake_return`、`114335_brake_return` |
| 接触 / 落地 | `220051_fixsquat`、`112630_fixsquat`、`114335_fixsquat` |
| 行走 / 转向闭环 | `112630_policy_walk_30s`、`114335_walk_00_30` … `90_120` |
| 接管 / 零速 | `220051_handoff_zero` |
| 原厂姿态参考 | `factory_stand`（无命令） |

每个 clip 的 tick 起止、时长、轮速/`gyro_z` 幅值见 `clips/index.csv`；详细用途见 `docs/01` 第 2–3 节。

---

## 4. 快速开始

```bash
# 1) 看一个 clip 的摘要
python tools/load_clip.py clips/220051_fixedstand_hold --summary

# 2) 校验全部 clip（tick、对齐、NaN、范围）
python tools/check_clips.py

# 3) 从 raw 重建（可选；确认 CLIP_SPEC 后用）
python tools/build_clips.py
```

Python 依赖：`numpy`（仅此一项）。

加载示例：

```python
import numpy as np
d = np.load("clips/220051_fixedstand_hold/clip.npz")
q_des, kp, kd = d["cmd_q"][:, :12], d["cmd_kp"][:, :12], d["cmd_kd"][:, :12]
wheel_dq_des = d["cmd_dq"][:, 12:16]
q_meas, tau_meas = d["st_q"][:, :12], d["st_tau"][:, :12]
```

---

## 5. 建议的验证顺序

1. `factory_stand`：只比姿态/T1，确认零位/符号/URDF 对齐。
2. `*_fixedstand_hold`：静态重力/CoM/关节刚度；先调质量/CoM，再调摩擦。
3. `*_prone_passive`：趴姿接触/阻尼。
4. `*_fixstand_rise`：PD/执行器阶跃（3 s，20/3→55/2）。
5. `*_fixsquat`：大幅运动 + 落地接触。
6. `112630_brake_return`：轮速归零/摩擦。
7. `112630_policy_walk_30s`：开环 1–3 s，再同策略闭环。
8. `114335_walk_*`：长时/转向；分 4 段看，不要只看总平均。

---

## 6. 数据边界（务必看）

- **无基座平动/高度**：`quat/gyro` 只有姿态和角速度；静态/蹲下设基座线速度 0，行走需自备真值。
- **无足端接触力**：只能用 `tau_est`/IMU/关节运动间接判断接触时刻。
- **`tau_est` 是估计值**：含摩擦、齿隙、电流环误差，不是关节力矩真值。
- **跨会话差异**：2026-11-10 与 2026-11-12/13 的 T2 静态差约 1°/2 Nm（载荷/电池/地面/温度）。用多个 clip 取范围。
- **`*_brake_return` 不是 T2**：回站目标是冻结的最后一帧策略腿目标，pitch 可能为正。
- **行走髋命令是准静态偏置**（约 ±0.26～0.38 rad），实测髋在 0 附近；同一时刻可能同号或反号，这是真实控制/接触结果，不是符号错误。
- **`factory_stand` 无 LowCmd**：只能做参考，不能 replay。
- **clip 首行可能是过渡帧**：纯段分析先丢首行或按 `cmd_kp` 过滤。

---

## 7. 来源与可追溯性

| 包内目录 | 原始位置（工作站权威） |
|---|---|
| `data/raw/20261112-220051_training-contract` | `部署测试/04_调试记录/logs/field_sessions/20261112-220051_training-contract/` |
| `data/raw/20261113-112630_r4000` | `…/20261113-112630_r4000/` |
| `data/raw/20261113-114335_r4000` | `…/20261113-114335_r4000/` |
| `data/raw/20261109-185025_factory` | `部署测试/04_调试记录/logs/20261109-185025_windows_LowCaptureFast.log` |

原始日志原样复制，未裁剪/重采样；clip 的裁剪窗口写在 `clips/<name>/meta.json` 和 `tools/build_clips.py` 的 `CLIP_SPEC` 中，可重建。所有文件哈希见 `MANIFEST.sha256`。

---

## 8. 版本

- 数据包 v1，2026-11-14。
- 详细测量方法/判读：`docs/01_仿真动力学校验数据指南.md`。
- 字段/关节序/符号：`docs/02_字段_关节序_对齐说明.md`。
- 如需补充单关节扫频、轮子离地阶跃、基座位姿真值、足端力，见 `docs/01` 第 9 节。
