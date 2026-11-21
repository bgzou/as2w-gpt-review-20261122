# data/raw 原始会话说明

本目录是**原样复制**的现场日志，未裁剪/重采样。clip 的裁剪结果在 `../clips/`；本目录用于追溯和重建。

| 会话 | 日期 | 时长 | capture | 内容 |
|---|---|---:|---:|---|
| `20261112-220051_training-contract` | 2026-11-12 | 59 s | 500 Hz (2 ms) | 趴姿 Passive → A 起立 → T2 保持 → B 零速接管 → PolicyWalk 零速 → Brake/ReturnStand → FixSquat |
| `20261113-112630_r4000` | 2026-11-13 | 65 s | 500 Hz (2 ms) | 同链 + 30 s 行走（±0.2 m/s） |
| `20261113-114335_r4000` | 2026-11-13 | 156 s | 500 Hz (2 ms) | 同链 + 120 s 行走（±0.4 m/s、wz ±0.1） |
| `20261109-185025_factory` | 2026-11-09 | 93 s | 100 Hz (10 ms) | 原厂高层站立（T1 参考，无 LowCmd） |

## 文件

| 文件 | 说明 |
|---|---|
| `debug_lowcmd.csv` | 500 Hz 实际 LowCmd 逐帧记录；`DEBUG_LOWCMD` 行，16 轴 × `(slot,mode,q,dq,kp,kd,tau)`。**raw DDS 符号**。 |
| `low_capture_fast.log` | 500 Hz LowState + IMU；`LOW_SAMPLE` 行，35 槽 × `q/dq/ddq/tau/mode/temp` + quat/gyro。**raw DDS 符号**。 |
| `low_protection.log` | 约 1 kHz 保护/电流/电压；`PROTECT` 行，含 `warning_or`、`motorstate_or`、`board_state0`、`battery_v`、`motor_a`。 |
| `publisher.log` | 发布端 stdout：状态切换、B 门槛、`WALK_CMD`、安全告警。 |
| `session.log` | 现场会话预检/监测/归档记录。 |
| `policy_diag.log` | 50 Hz 策略诊断（仅 HANDOFF/BRAKE 段较多），含 `cmd_vx/vy/wz`、raw/wrapped 动作、target_q。 |

`20261109-185025_factory` 只有 `low_capture_fast.log`。

## 关节顺序与符号

- 顺序：`FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …, RR_calf, FL_wheel, FR_wheel, RL_wheel, RR_wheel`。
- DDS 槽位：`[3,0,9,6,4,1,10,7,5,2,11,8,13,12,15,14]`。
- 符号：髋 `-1`，其余 `+1`。
- `debug_lowcmd` / `low_capture_fast` 的原始值都是 DDS 符号；clip 里同时提供 `*_raw` 和模型坐标。

详见 `../docs/02_字段_关节序_对齐说明.md`。

## 采样与时间

- `low_capture_fast.log` 的 `elapsed_ms` 是会话相对时间；`tick` 是 1 kHz LowState 计数器。
- `debug_lowcmd.csv` 的 `mono_ns` 是单调时钟；`lowstate_tick` 与 capture 的 `tick` 同源。
- 命令 500 Hz 与状态 500 Hz 不是严格同 tick：clip 用最近 tick 对齐，`st_offset_ms = st_tick - cmd_tick`，最大 ±7 ms。

## 来源

工作站权威路径：

```text
部署测试/04_调试记录/logs/field_sessions/<session>/
部署测试/04_调试记录/logs/20261109-185025_windows_LowCaptureFast.log
```

原文件哈希见包根 `MANIFEST.sha256`。
