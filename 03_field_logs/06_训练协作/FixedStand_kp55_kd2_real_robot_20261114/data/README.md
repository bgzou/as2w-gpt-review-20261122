# data/ 原始数据说明

本目录只存放**原样复制**的现场日志，未做裁剪、重采样或改字段。所有时间戳都是该次采集的相对时间（`elapsed_ms` 或 `mono_ns`），不是墙钟；墙钟见各文件头 `Started:` 行或 `session.log`。

---

## 1. `stand_trials/` —— A→FixedStand 姿态与力矩

| 目录 | 来源会话 | 文件 | 说明 |
|---|---|---|---|
| `20261109-185025_factory` | 原厂高层站立 | `low_capture_fast.log` | 86.66–92.96 s 自动筛出 611 个稳定样本；T1/原厂姿态基准 |
| `20261110-100032_trial2` | 第 2 次 A→Y | `low_capture_fast.log` | 42.08–45.20 s 稳定窗；重复性 |
| `20261110-101251_trial3` | 第 3 次 A→Y（修复版） | `low_capture_fast.log`、`low_protection.log` | 42.50–43.50 s 为本文主样本（101 帧）；保护流可核对无告警 |
| `20261110-155128_stand` | 一键 stand 会话 | `low_capture_fast.log`、`low_protection.log` | 23.47–28.23 s 稳定窗；重复性 |
| `20261110-193947_r284-zero-b` | R284 zero-B 会话，B 未按 | `low_capture_fast.log`、`low_protection.log` | 仅用 46.89–61.92 s 稳定窗；capture 为当时本机已回收的部分（约 8.4 MB），远端原件未取全，不影响该窗 |

### 1.1 `low_capture_fast.log` 格式

- 采样：10 ms/帧（约 100 Hz），由 `low-capture ... 10` 采集。
- 每帧一行，以 `LOW_SAMPLE,` 开头；首行是 `LOW_SAMPLE_SCHEMA,...`。
- 字段顺序：

```text
LOW_SAMPLE, elapsed_ms, packet_count, age_ms, tick, mode_pr, mode_machine,
            quat_w, quat_x, quat_y, quat_z,
            gyro_x, gyro_y, gyro_z,
            m0_q, m0_dq, m0_ddq, m0_tau, m0_mode, m0_temp,
            ...,
            m34_q, m34_dq, m34_ddq, m34_tau, m34_mode, m34_temp
```

- `m0..m34` 对应 HG LowState 的 35 个电机槽；AS2-W 只用 0–15，16–34 为 0。
- `m*_tau` 是 `tau_est`（驱动器估计力矩），不是电流直接换算，也不是足端接触力。
- `m*_mode`：1 = 位置/伺服模式；0 = 被动/阻尼；本文用 mode 全 1 过滤稳定站立。
- 四元数为 body→world，顺序 `w,x,y,z`；pitch 计算为 `asin(2(wy−zx))`。

### 1.2 关节顺序与符号

`tools/analyze_stand_pose_comparison.py` 用项目实测映射把 DDS 槽位转成 contract 顺序：

```text
SLOTS = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
SIGNS = [-1, -1, -1, -1, +1, +1, +1, +1, +1, +1, +1, +1]
```

contract 顺序为 `FL,FR,RL,RR`，每腿 `hip,thigh,calf`。该映射来自 `config/r152_measured.map` 的现场实测，见项目 `02_现场操作/AS2W_16轴DDS关节序速查.md`。

### 1.3 `low_protection.log` 格式

- 采样：约 1 kHz，每行以 `PROTECT,` 开头。
- 关键字段：`tick`、`machine`、`modes`（16 位）、`motorstate_or`、`warning_or`、`board_state0`、`battery_v`、`motor_v`、`motor_a`。
- 本文只用它核对“无硬件保护位/无电机错误”；不用于姿态计算。

---

## 2. `b_gate/` —— 15 次 B 门槛实测

- 每个文件是某个现场会话的发布端 stdout（`publisher.log`），命名 `<会话目录>_publisher.log`。
- 关键行格式：

```text
[FixedStand] → PolicyHandoff (B accepted: pose_error=0.270298 wheel_dq=0.0122718 upright=0.996304 quiet_dq=0.0245437)
```

- `pose_error`：按下 B 瞬间 12 腿逐轴 `|q_measured − fixed_pose|` 的最大值（rad）；首次从趴下起立时 `fixed_pose = T2`。
- `wheel_dq`：轮速最大绝对值（rad/s）。
- `upright`：`1 − 2(qx²+qy²)`，越大越直立。
- `quiet_dq`：最近 500 ms 腿 `|dq|` 最大值（rad/s）。
- 同目录也可能有 `B rejected:` 行（例如 `20261112-224151` 首次因 `quiet_max_dq` 被拒），脚本只统计 `B accepted`。

---

## 3. `lowcmd/` —— 增益下发核对

| 文件 | 说明 |
|---|---|
| `20261112-220051_training-contract_debug_lowcmd.csv` | 500 Hz LowCmd 逐帧记录；用于确认腿 55/2、轮 0/2、q=T2、tau_ff=0 |
| `20261112-220051_training-contract_publisher.log` | 同会话发布端输出（状态切换与 B 门槛） |
| `20261112-220051_training-contract_session.log` | 现场会话预检/监测/归档记录 |

`debug_lowcmd.csv` 格式：

```text
DEBUG_LOWCMD_SCHEMA, mono_ns, lowstate_tick, state, mode_machine,
                     (slot, mode, q, dq, kp, kd, tau) × 16
```

- `state`：0=Passive，1=FixStand，2=FixedStand，3=PolicyHandoff，4=PolicyWalk，5=Brake，6=ReturnStand，7=FixSquat，8=EmergencyDamping。
- 每轴 7 个字段，从第 6 列（0-based 索引 5）开始；`q/dq/kp/kd/tau` 为该帧实际下发的 LowCmd。
- `tau` 是前馈力矩（本项目为 0），不是 `tau_est`。

---

## 4. 复现脚本

| 脚本 | 用途 | 预期 |
|---|---|---|
| `tools/analyze_stand_pose_comparison.py` | 姿态/力矩三方对照 | 主样本 pitch −5.498°，fixed→T2 max 0.270496，fixed→T1 max 0.639310 |
| `tools/extract_b_gate.py` | 从 `data/b_gate` 重新提取 15 次门槛 | n=15，mean 0.281972，pstdev 0.015570 |
| `tools/verify_lowcmd_gains.py` | 核对 FixedStand/FixStand 的 LowCmd 增益 | FixedStand 1820 帧；FixStand 20/3 894 帧 + 55/2 586 帧 |

---

## 5. 注意

- 不要用这些日志反推原厂控制器的 `q_cmd/kp/kd/tau_ff`：LowState 不包含这些量。
- `tau_est` 的符号与 contract 顺序一致（脚本已乘 `SIGNS`）；直接看原始 `m*_tau` 时注意 DDS 槽位与符号。
- 193947 的 capture 不完整是已知事实；只使用 46.89–61.92 s 稳定窗，且该窗数值与其余三次一致。
