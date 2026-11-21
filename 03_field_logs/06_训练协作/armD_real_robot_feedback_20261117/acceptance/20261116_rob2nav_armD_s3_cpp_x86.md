# 2026-11-16 Rob2Nav armD S3 C++ 适配（x86 部分）原始记录

- 环境：caolin 服务器；构建镜像 /tmp/opencode/rob2nav_repro/cpp_build/（rsync 自 SSHFS 唯一源码树，排除构建产物）；ONNX Runtime 1.22.0 x86（/tmp/opencode/onnxruntime-linux-x64-1.22.0）、unitree_sdk2（/tmp/opencode/sdk2_install）、yaml-cpp、CMake 3.28、g++（C++17）。
- 权威源码树已落盘（工作站 SSHFS `部署参考/cpp_deploy-as2w/`）：新增 `deploy/robots/as2w/armd/`（controller hpp/cpp + replay + CMakeLists）、`models/armd/`（deploy.yaml + policy.onnx，ONNX SHA 087c91bc…9821 与包一致）；共享文件改动：`common/as2w_publish.cpp`（AS2W_POLICY_ARMD 分支：策略别名、walk_cap 10s、命令来源分支、**armD 无命令滤波——绕过 slew 直通**、debug 排除列表）、`deploy/include/isaaclab/algorithms/algorithms.h`（OrtRunner 符号维修复）、顶层 `CMakeLists.txt`（AS2W_ARMD 选项）。

## armD 控制器设计要点（合同映射）

- obs 65 维自实现（不依赖 r284 观测类，因 R284 类内置摇杆命令低通滤波，armD 合同无滤波）；prev 动作回填 RAW 未裁剪（armD 合同）；轮 obs 4π 回绕；零命令锁存漂移项 obs[60:64]+标志 obs[64]；obs[3:6] 用 IMU 体系角速度直读（训练语义；包内 MuJoCo 参考实现的二次旋转偏差已在 S1 记录）。
- 包装：腿 q=默认站姿+0.35×clip(raw,±6)，kp55/kd2；轮目标 16×clip；**gyro-1 前馈 + 包内硬件公式 dispatch**：ff=S×clip(60×(wz_cmd−wz_meas),±4)，tau=clip(6×(tgt−dq)+ff,±13.5)，dispatch=dq+tau/6 作为轮速度命令（manifest gyro1.formula 原文）；轮 LowCmd 编码沿用 R1199 已真机验证的速度模式（kp=0）。
- 命令门：基础库仅接受精确零命令（zero-B）；AS2W_ARMD_FORWARD_CANDIDATE 独立编译目标接受脚本化 vx∈[0,vx_limit]（vy/wz 锁零，纯转禁令编译期内置）；AS2W_ARMD_REPLAY_UNGATED 仅存在于离线 parity 回放目标，任何 publish 目标不定义。
- deploy.yaml：vx_limit 0.25 / wz_limit 0.05（双仿真已验包络；纯转/arc_left 禁）；合同常数（clip 6/scale 0.35/16、dispatch kd 6、tau 13.5、yaw_kp 60、cap 4、clock 0.62）全部由 load_config 硬校验，不符即 throw。

## 构建与测试

```
cmake -S . -B b-armd -DAS2W_ARMD=ON -DAS2W_ARMD_PUBLISH=ON
       -DAS2W_ARMD_ZERO_B_TRIAL=ON -DAS2W_ARMD_FORWARD_CANDIDATE=ON
       -DONNXRUNTIME_ROOT=/tmp/opencode/onnxruntime-linux-x64-1.22.0
       -DCMAKE_PREFIX_PATH=/tmp/opencode/sdk2_install
ctest --test-dir b-armd   → 9/9 全过（含 armd-startup-frame-audit、armd-forward-startup-audit）
as2w_armd_publish --offline-audit      → AUDIT PASS（独立运行）
as2w_armd_forward_publish --offline-audit → AUDIT PASS
```

## C++ parity（S3 核心门槛）：3550 帧，5 门全 0.000e+00

- 夹具：S1 参考录带 5 段（hold/turn_left_0.10/straight_fwd/arc_right/brake_fwd，含 settle 段，3550 策略帧）；输入含四元数/体系陀螺（=MuJoCo qvel[3:6] 原值，training 语义）/16 关节位姿速度/命令。
- 期望侧：float32 逐运算镜像（与 C++ 同序同常数）+ 同版本 ONNX Runtime 1.22（PYTHONPATH=/tmp/opencode/ort122lib）。
- 结果：obs(65)、wrapped action、leg q、wheel dispatch、ff magnitude 全部 **0.000e+00**（bitwise 一致）。PASS（门槛 ≤1e-6）。

## 过程记录（失败也记）

1. 首编失败：`LegCount` 歧义（发布端全局常量 vs 命名空间导出）——armd 命名空间不再导出该名。
2. 二编失败：通用 debug 路径要求 `last_state_input()`（有状态策略专用）——ARMD 加入排除宏列表。
3. **OrtRunner 共享头缺陷**：armD ONNX 为动态 batch（'batch' 符号维），OrtRunner 预计算输入元素数得负值 → CreateTensor 抛异常。修复：符号维在 forward() 按运行时数据长度解析（静态 shape 路径不变）。R1199 回归重编 CTest 8/8 确认无破坏。
4. **C++ 时钟真偏差（修复）**：初版相位用 float 常量（Pi float、0.62f→double）参与 double 运算，与训练（numpy 真双精度）有 ~1e-8 相对漂移，tick 744 处 obs[58] 差 1.16e-6、经 ONNX 放大 dispatch 差 3.3e-6。修复为真双精度常数（2·π_double·tick·0.02/0.62）后全帧 bitwise 一致。此为合同级修复（长期有效：**所有新策略控制器时钟一律用双精度真常数**）。
5. 期间误诊两次（numpy SIMD libm vs glibc、ORT 版本差异）均被逐项排除后定位到真因。

## 尚未完成（S3 剩余）

- ARM64：CPRO 同步 + 远端构建 + CTest + audit + 两端口 SHA 核对（需经工作站两跳，待执行）。
- 现场入口三件套（run_armd_field_session.sh / 试验启动器 / PowerShell 注册）。
- 重新打包 `准备现场包.ps1`。
- 部署语义（training gyro）MuJoCo 6/11 包络复验（后台运行中，结果待记）。

## ARM64 与现场入口（追加，同日完成）

- 整包同步：工作站 `现场入口.ps1 -Stage Sync`（自动跑 `准备现场包.ps1` → tar → hdc file send → 远端 sha256sum -c 两包 OK → 解包 /root/as2w_deploy）；中文路径 -File 坑已按既有规则用 `& '...'` 形式绕过。
- CPRO ARM64（build-armd-zero-b：AS2W_ARMD=ON + PUBLISH + ZERO_B_TRIAL，ORT /root/ort122，SDK2 /root/unitree_sdk2_install）：configure+build+CTest **8/8**+`--offline-audit` **PASS**（nohup 后台 + base64 脚本法，日志 /root/armd_build_logs/build.log，ARMD_ARM64_ALL_OK）。
- 两端口 SHA 一致（caolin 镜像 vs CPRO /root/as2w_deploy）：armd_controller.hpp/cpp、armd_replay.cpp、run_armd_field_session.sh、run_armd_zero_b_trial.sh 全部逐字节相同；models/armd/policy.onnx 两端口 SHA=087c91bc…9821 与包 manifest 一致。
- 现场入口三件套：`tools/run_armd_field_session.sh`（zero-b/dry-run 白名单，全套预检链：binary/audit/SHA→LowState≥100包/s+mode_machine=2→趴下确认（factory 时 stand-down→ReleaseMode）→2s 1kHz+16轴mode0→趴姿小腿门→双监测 nice -n 10→发布端→5s 保护尾→归档）、`tools/run_armd_zero_b_trial.sh`、Windows `运行现场策略会话.ps1` 注册 `-Policy armd`（仅 zero-b|dry-run，其余模式 throw；dry-run 免 -ConfirmMotion，zero-b 必须）。
- 判定：**S3 完成**。S4 实机阶梯按 04 文档执行，dry-run 可只读先行，zero-B 及以后每步须现场当次明确同意。
