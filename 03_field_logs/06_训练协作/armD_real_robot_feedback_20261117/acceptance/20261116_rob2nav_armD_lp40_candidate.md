# 2026-11-16 Rob2Nav armD-LP40 候选（选项 2）离线验收原始记录

- 性质：**新策略修订版**（训练合同 `filter: none` + 40ms 腿目标低通 = 包装变更），按硬规则全链重验；R1199 LP40 现场已验配方（一阶低通、只滤腿 q、轮直通）。
- MuJoCo 同合同复验（armd_lp_eval.py，与 C++ PolicyTargetLowpass 逐位同实现：applied=prev+α(req−prev)，α=−expm1(−0.02/0.04)，首帧 reset，轮不滤；gyro-1 ON、states 2/3/5）：
  - **hold**：3 初态全 PASS，腿目标步进 **max 0.037–0.052 / p99 0.017–0.018 rad/帧**；
  - 已验包络 6 例（straight±0.25、arc_right、brake±0.25）：**6/6 × 3 初态全 PASS**，跟踪值与未滤版一致（straight 1.08/1.06、arc_right 1.06–1.08/0.95–0.96、brake ±0.01–0.02）；
  - 诚实边界：MuJoCo 干净状态复现不了现场 0.23–0.36 的交替幅度（未滤 sim hold 步进仅 0.084 max）——仿真的验收意义是"滤波不破坏已验包络"，真实抖动抑制量以现场为准（按现场幅度推算滤波后步进 ≈0.49×幅度 ≈ 0.12–0.18 < warn 0.2）。
- C++（隔离候选，不动已验 zero-B 二进制）：CMake 新增 `AS2W_ARMD_LOWPASS_CANDIDATE`（仅 publish 目标，`AS2W_POLICY_ARMD=1 AS2W_ENABLE_POLICY_LOWPASS=1 AS2W_ENABLE_POLICY_ZERO_B_TRIAL=1`）；共享发布端低通三调用点（Handoff 末 reset/Walk apply/Brake apply）均为通用路径，无策略 #if。启动器 `run_armd_lowpass_trial.sh` **钉死 --policy-lowpass-ms 40**；会话脚本加 lp-zero-b/lp-zero-b-dry-run 模式；PowerShell 注册（lp-zero-b 须 -ConfirmMotion，dry-run 免）。
- x86：CTest **10/10**（新增 armd-lowpass-startup-audit）；`as2w_armd_lowpass_publish --offline-audit` PASS（含 PolicyTargetLowpass 数学/旁路/OFF 审计）。
- 权威树落盘 + 整包 Sync 完成；CPRO ARM64（build-armd-lowpass）构建+CTest+audit+100帧推理 replay 结果见下节追加。

## ARM64 与 dry-run（追加）

- CPRO build-armd-lowpass（AS2W_ARMD_LOWPASS_CANDIDATE）：configure+build+CTest **9/9**+`--offline-audit` PASS+**100 帧推理 replay RC=0**（ARMD_REPLAY_FRAMES=100，新固化的 ARM64 必过项）。全部 `ARMD_LP_ARM64_ALL_OK`。
- lp-zero-b-dry-run（只读）：step1 binary/audit PASS；step2 LowState **0 包** → 按设计 FATAL（狗已断电/链路断，只读发现，无任何动作）。待现场恢复上电后重跑 dry-run 再进入实机。
- 判定：armD-LP40 候选 S3 离线+双端门槛**全过**；实机 lp-zero-B 未执行，待狗上电+现场当次同意。
