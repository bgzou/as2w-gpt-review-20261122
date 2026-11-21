# 会话时间线（2026-11-20，Asia/Shanghai）

| 时间 | 事件 | 结果 |
|---|---|---|
| 14:45 | Isaac 回放渲染（model_18200，8 工况） | 8 段无 fall，`himloco-diagnostic.mp4` 6.1 MB |
| 15:05 | 巡检：consolidate 18608/20400 | 正常 |
| 16:30 | 导出 ONNX（第一版） | 发现 estimator latent 缺 L2 归一化，真实观测 parity 差 6.47 |
| 16:40 | 补齐 `F.normalize(z, p=2)` 重新导出 | 随机输入 parity 7.63e-06；真实 env 观测 2.38e-06 |
| 16:55 | 服务器构建工具链（cmake/g++/yaml-cpp + `/root/onnxruntime-linux-x64-1.22.0`） | 构建 rc=0、CTest 7/7 |
| 17:05 | 训练合同逐项审计 | 发现控制器缺 **100 步零命令交接斜坡**（`dr_environment` 有，部署没有）→ 补 `handoff_steps`，`MAX_LEG_STEP` 0.1206→0.0582 |
| 17:20 | 修正动作映射 | 以 `arc_environment._pre_physics_step` 为准：`tanh → ×0.6 → ×0.25`、轮 `(vx−wz·side)/R+2·tanh`（此前文档写成 clamp+×12 是错的） |
| 17:52 | CPRO USB 枚举成功（`VID_12D1&PID_5000`） | hdc `0123456789ABCDEF` Connected |
| 17:57 | 推源码/夹具到 CPRO（注意 chroot 看不到 `/data/local/tmp`，需先 cp 进 rootfs） | 成功 |
| 17:58 | CPRO ARM64 构建 + CTest | 8/8（含 `himloco-startup-audit`） |
| 17:59 | ARM64 replay vs x86 | 120 帧最大差 6.1e-07 |
| 18:00 | `as2w_himloco_publish` 构建 + `--offline-audit`（DR yaml） | AUDIT PASS |
| 18:02 | 机器人 eth0 carrier=1、ping 通 | 链路恢复 |
| 18:03 | 只读基线（low-state/motion-mode/service-list/low-protection） | ~1060 Hz、`form=1 name=ai`、无告警、48.5 V；姿态与 `make_prone()` 一致（趴下） |
| 18:06 | ReleaseMode | rc=0，`name=`（已释放） |
| 18:07–18:09 | zero-B 试跑 1：A→FixedStand→20 s 未按 B→timeout squat→X | 无告警；B 未触发 |
| 18:14 | zero-B 试跑 2：`--stand-hold-seconds 30 --b-gate-pose 0.45` | **B accepted**（pose_error 0.202、wheel_dq 0.012、upright 0.9967、quiet_dq 0.043）→ PolicyWalk 零速 → 60 s 上限刹车 → ReturnStand → X，0 告警 |
| ~18:20 | 摇杆版（vx 0.15 / wz 0.20）反向 −0.12 m/s 约 10 s | 无告警；10 s cap 自动刹车回站 |
| 18:26 | 全包络（vx/wz 0.30）试跑 | `STARTUP FAULT: stale/invalid LowState age_ms=21.45 max=20` → 发布端 fault stop（非策略问题） |
| 18:28 | 修复：`MaxStateAge` 20→50 ms（仅摇杆构建） | 重建 rc=0、audit PASS |
| 18:33 | 全包络（0.30/0.30）推杆 | 0.72 s、vx −0.30、wz 0.108、0 告警；操作员按 X |
| 18:36 | 逐轴 `cmd−actual` 分析 | 后腿 calf +0.33、thigh +0.24 → PD 撑不住 |
| 18:38 | 静态重力补偿 `leg_offset_rad`（RL/RR calf +0.34） | 后腿误差降到 +0.10/+0.20，但机身前倾、前腿反向对抗、力矩嗡鸣 → 操作员按 X |
| 18:42 | 补偿归零、保留机制 | `deploy-drive-full.yaml` 中 `leg_offset_rad` 全 0 |
| 19:35–19:53 | MuJoCo 第二仿真器尝试 | 场景/渲染管线通，但轮速显式力矩环不稳定，机器人翻滚 → **未通过** |
| 20:00+ | 训练侧变化 | consolidate 在 19688 被终止，另起 `him-motor-jitter-qhbUAk`（19600→21600） |
