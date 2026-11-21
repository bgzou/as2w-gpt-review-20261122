# 实机执行记录汇总

## 已归档的原始日志（`field/`）

| 文件 | 内容 |
|---|---|
| `20261120-180312_low-state_32209.log` | 只读 LowState 5 s：~1060 Hz、`mode_pr=0`、`mode_machine=2`、IMU 四元数、16 轴 q/dq |
| `20261120-180339_low-protection_32449.log` | 保护流 5 s：`modes=1111111111111111`、`motorstate_or=0x0`、`warning_or=0x0`、电池 48.49 V、电机电流 1.7 A |
| `20261120-180338_motion-mode_32394.log` | `MotionSwitcher form=1 name=ai`（原厂 ai 控制中） |
| `20261120-180339_service-list_32422.log` | 服务与保护标志列表 |
| `20261120-1810_zerob-publish-run1.log` | 第一次 zero-B 发布端完整日志（A→FixedStand→timeout squat→X） |

## 未能归档（CPRO 次日离线）

以下日志位于 CPRO `/tmp/`，会话结束时未落盘到工作站，CPRO 断线后不可取；关键行已在下表记录。

| 试次 | 关键行 |
|---|---|
| zero-B 成功轮（18:14） | `[FixedStand] → PolicyHandoff (B accepted: pose_error=0.202156 wheel_dq=0.0122718 upright=0.996734 quiet_dq=0.0429515)`；`[PolicyHandoff] → PolicyWalk (zero-speed handoff complete)`；`[PolicyWalk] brake trigger=bounded zero hold cap`；`[Brake] → ReturnStand`；`[ReturnStand] → FixedStand`；`[FixedStand] → EmergencyDamping (X key)`；`Publisher stopped, exit=0` |
| 全包络 fault 轮（18:26） | `STARTUP FAULT: stale/invalid LowState; stop publisher after 53160 frames age_ms=21.4536 max_age_ms=20 low_count=112671 mode_machine= crc_ok=1 finite_ok=1 state=PolicyWalk`；`Publisher fault; stop. exit=1` |
| 全包络推杆轮（18:33） | 36 条 `WALK_CMD`，`vx` 范围 −0.30…+0.036、`wz` 0…0.108；`FIELD_SAFETY_WARN/HARD` 0 条；`[PolicyWalk] → EmergencyDamping (X key)` |
| 静态补偿轮（18:38） | B accepted `pose_error=0.161174 wheel_dq=0.0122718 upright=0.999606 quiet_dq=0.0306796`；2.2 s 后 `X`；末段 `POLICY_TRACK` 见 `analysis/field-sink-per-axis.csv` |

## 安全事件

| 项 | 次数 |
|---|---|
| `FIELD_SAFETY_WARN` | **0** |
| `FIELD_SAFETY_HARD` | **0** |
| 电机 `warning_or` / `motorstate_or` 非零 | **0** |
| 断电/重启 | 1 次（18:40 左右，补偿试验后，机器人重连后 eth0 carrier 恢复） |

## 操作时序（每次均经现场当次同意）

1. ReleaseMode → 只读确认已释放（`form=1 name=`）
2. A → FixedStand（PD 保持，站立）
3. B → 门控校验（pose_error / wheel_dq / upright / quiet window）→ PolicyHandoff（100 步零命令 + 目标斜坡）→ PolicyWalk
4. A 刹车回站 / Y 蹲下 / X 急停 / 超时自动刹车
