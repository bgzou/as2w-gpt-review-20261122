# 2026-11-16 Rob2Nav armD S1 移植对齐 parity 原始记录

- 环境：caolin 服务器 /tmp/opencode/rob2nav_repro/work/（本地镜像；torch 2.15 CPU / ORT 1.31.1 / MuJoCo 3.17.0）
- 方法（包无 replay/ 夹具与 verify_package.py）：对包内参考实现 runtime/r11_mj_matrix_v2.py 做最小"录带"补丁（逐策略步记录状态/obs/raw/包装目标/ff/零位锁存），再与按合同文档独立实现的 armd_port.py 逐帧对拍。录带补丁本身不改任何控制/物理语义（只在既有调用点旁路复制数组）。
- 录带用例（gyro-1 ON：--yaw-kp 60 --ff-cap 4 --ff-gain 0；states=3.0；r11_legacy 合同）：hold、turn_left_0.10、straight_fwd_0.25、arc_right_0.25、brake_fwd_0.25，共 3550 帧。
- 录带用例结果与包内证据模式一致：hold/straight/arc_right/brake PASS，turn_left_0.10 INVALID_STAGE1（sup=0.433、nw=15、term=1）——与 evidence/mujoco/armD_1k_matrix.json 同模式（该例本就双仿真都不达标）。

## 对拍门槛（≤1e-6）与结果

| 门 | 内容 | max_err | 判定 |
|---|---|---|---|
| 关节序提取 | 我的 qpos/qvel→16策略序 vs 录带 | 0.000e+0 | PASS |
| G1 obs | 我的 65维 obs vs 录带 obs | 0.000e+0 | PASS |
| G2 raw | ORT(我的obs) vs ORT(录带obs) | 0.000e+0 | PASS |
| G4 包装 | 腿/轮最终目标 vs 录带 | 3.576e-8 | PASS |
| G4 ff | gyro-1 前馈（kp60/cap4/S=(-1,1,-1,1)） | 0.000e+0 | PASS |
| G5 锁存 | 零命令轮位锁存（含 None/值） | 0 / 0 mismatch | PASS |
| （参考）torch vs ORT | 同输入双引擎 | 9.537e-7 | 与包内 ONNX_PARITY 1.9e-6 同量级，float32 噪声 |

附加验证：dispatch = dq + clip(kd*(tgt−dq)+ff, ±13.5)/kd 逐帧断言通过；torque clamp ±13.5 含 ff（与 manifest 公式一致）。

## 过程记录（失败也记）

- 首轮对拍 G4 ff 全错（3521/3550 帧，差 8Nm=两个饱和帽）：定位为**我的录带补丁把 wz_meas 记在 32 个物理子步之后**，而 ff 用子步前 wz 计算——纯记录侧 off-by-one，非语义分歧；改用录带中子步前 qvel[5] 后 ff 逐帧精确为 0。
- 训练语义注记：MuJoCo 自由关节 qvel[3:6] 为**体系**角速度；包内参考实现 obs[3:6]=Rᵀ@qvel[3:6] 相当于二次旋转（直立时无差、大倾角时偏离 Isaac/IMU 语义）。parity 按参考实现逐位复刻通过；**硬件 C++ 侧按训练语义（Isaac root_ang_vel_b=IMU 体系角速度直读）实现**，见 armd_port.py 的 ang_semantics 开关与 S3 设计。

## 合同理解确认（本 parity 证明的理解）

obs: cmd−(0.4,0,0) | Rᵀqvel[3:6] | Rᵀ[0,0,−1] | 0.4 | q−默认站姿（轮 4π 回绕）| dq | prev_raw(未裁剪) | sin/cos(0.62s) | 零命令轮漂移(相对锁存, /0.20, ±3) | zero_flag
action: clip(raw,±6) → 腿 ×0.35+[0,0.8,−1.6]（kp55/kd2）；轮 ×16（kp0/kd6）+ gyro-1 前馈（S=(-1,1,-1,1)，clip(60×(wz_cmd−wz_meas),±4)），总力矩 clamp ±13.5Nm，dispatch=dq+tau/kd。

## 判定

**S1 PASS**（全门 ≤1e-6）。进入 S2 sim2sim（11 例冻结矩阵复现，gyro-1 ON）。
