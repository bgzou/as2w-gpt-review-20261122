# 数据格式说明（data/README.md）

每个会话目录（`field_sessions/<时间>_<模式>/`）为 standalone 完整档案，6 个文件：

| 文件 | 内容 | 频率 |
|---|---|---|
| `session.log` | 会话脚本预检链全记录（audit/LowState 探针/控制权/趴姿门/启动）+ 双端 SHA256 | — |
| `publisher.log` | 发布端控制台（FSM 状态迁移、GETUP_GATE、安全事件、退出码） | 事件 |
| `debug_lowcmd.csv` | **最终 LowCmd 逐帧**（每条发布命令的完整 16 轴 slot/mode/q/dq/kp/kd/tau） | 500Hz |
| `debug_policy.csv` | 策略输入/输出三元组（sensor/obs/raw_action/wrapped/请求目标/应用目标） | 50Hz |
| `low_capture_fast.log` | 独立 LowState 监测（与发布端进程无关） | ~1kHz |
| `low_protection.log` | 独立保护流监测（motorstate/warning/board 状态、电池/电机电压电流） | ~190Hz |

## 关节序（所有 m0..m15 / slot 0..15 一致）

```
0-2   FR (hip, thigh, calf)     6-8   RR (hip, thigh, calf)     12 FR 轮
3-5   FL (hip, thigh, calf)     9-11  RL (hip, thigh, calf)     13 FL 轮 / 14 RR 轮 / 15 RL 轮
```
注意：`debug_policy.csv` 的 sensor/action 使用**策略序**（FL,FR,RL,RR 分组），`debug_lowcmd.csv`/
`low_capture` 使用**DDS 槽位序**（FR,FL,RR,RL 分组）；两序转换见 `tools/analyze_chatter.py` 注释。
数值均为模型正号约定（hip 四轴符号 −1，其余 +1）已在采集时折算。

## CSV 头

- `debug_lowcmd.csv`：`DEBUG_LOWCMD,mono_ns,lowstate_tick,state,mode_machine,(slot,mode,q,dq,kp,kd,tau)×16`
  - state：0=Passive 2=FixedStand 5=Brake 6=ReturnStand 7=FixSquat(含 FixStand 段) 8=Passive 尾段 9=FixCrouchLite 10=PolicyGetup
- `debug_policy.csv`：`DEBUG_POLICY,mono_ns,phase,tick,age_s,cmd_xyz,remote...,gyro_xyz,quat_wxyz,imu_accel_xyz,
  sensor_q_16,sensor_dq_16,obs_count,obs_values(42),state_count,state_values,raw_action_16,wrapped_action_16,
  requested(q,dq,kp,kd)×16,applied(q,dq,kp,kd)×16`
- `low_capture_fast.log`：`LOW_SAMPLE,elapsed_ms,packet_count,age_ms,tick,mode_pr,mode_machine,quat_wxyz(4),
  gyro_xyz(3),m0..m15(q,dq,ddq,tau,mode,temp)`
- `low_protection.log`：`PROTECT,elapsed_ms,low_count,low_age_ms,tick,machine,modes(16),motorstate_or,
  warning_or,board_count,board_age_ms,board_state0,battery_v,motor_v,motor_a,...`

## windows_transcripts/

工作站 PowerShell 端完整控制台记录（含 CPRO 归档回传信息），文件名 `YYYYMMDD-HHMMSS_windows_getup1199_*.log`。
`20261113-180544` 一行即 bug2 入口门拒绝的当场输出。

## mujoco/

| 文件 | 内容 |
|---|---|
| `run_lp40_rise.json` | rev5 40ms 低通 MuJoCo 起身对照（8 起点：passed/survived/h/step_max/dq） |
| `probe_r3.json` | 开膝探针 rev3 配置（标称/×1.5/×1.8 躯干配重 × 3 种子，包络/力矩/接触峰） |
| `run_bank/`、`run_prepose/` | 阻尼轮 vs 锚定轮 MuJoCo 对照（各 40 例 diagnostic.json，与 11-13 基线逐数一致） |
| `eval_recovery_lp40.py`、`eval_recovery_damped_wheels.py` | 生成上述数据的评估器变体（腿 85/3.2、轮 0/2、LP40） |

## contract/

- `deploy.yaml`：现行 rev5 部署合同（含轮配方偏离注释、入口/成功门、staged rise 常量——与训练
  `as2w_recovery_control.py` 1:1 映射，除 `wheel_hold_kp/kd` 0/2 偏离与部署侧门限）。
- `run_getup1199_trial.sh`：会话启动器（强制 `--policy-lowpass-ms 40`）。

## 环境基线（对照实验时注意）

- 硬件电池电压 ~47.4-47.5V；LowState ~1kHz；控制链 CPRO→狗 eth0（192.168.123.x 私网）。
- 每会话前 LowState 质量门：≥1500 包/2s、16 轴 mode=0（无其他写者）、趴姿小腿基线门。
- 成功门 h_est 用腿 FK 估计（轮触地假设），与训练评估器同源阈值。
