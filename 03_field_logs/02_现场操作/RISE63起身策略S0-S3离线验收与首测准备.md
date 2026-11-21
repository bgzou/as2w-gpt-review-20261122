# RISE63（AS2W_V6.3_from0_model_2000）S0–S3 离线全链验收与有界首测准备

日期 2026-11-20。交付包 `部署参考/AS2W-V6.3-from0-model2000-handoff-v3.2-physx-arm64_20261120`。
本页覆盖：收包 → parity → sim2sim → C++ 适配 → x86/CPRO ARM64 构建 → 会话入口 → 只读预检。
**本轮零真机动作**：未 ReleaseMode、未发任何 LowCmd、未改变狗的控制权。

## 结论一句话

RISE63（平地四足预备姿态**抬前轮、双后轮支撑保持**，固定零命令）已走完全部离线门槛：S0 收包 SHA 345/345、S1 独立端口 parity（obs **2.7e-7**、包装 **0**、ONNX 5.7e-6 与交付方自报 PT-vs-ONNX 5.72e-6 一致）、S2 MuJoCo FSM 仿真（名义 3 例复现 + 进/出/抗扰矩阵）、S3 x86 与 CPRO ARM64 各 CTest 1/1 + RISE63 AUDIT PASS + C++ 磁带 parity（obs 2.98e-7 / 目标 2.29e-5）、三方 SHA 8 文件一致、`rise63-dry-run` 只读预检跑通链路（因狗未上电 LowState=0 包按设计止步）。**可以进入有界首测，但必须先解决两件现场事项：见 §6。**

## 1. 策略合同（models/rise63/deploy.yaml，与交付 actual_flags 逐项一致）

| 项 | 值 |
|---|---|
| 网络/接口 | ONNX 单文件，`observation[1,53]` → `action[1,16]`，float32，opset 17 |
| 频率 | 50 Hz（20 ms），参考电机 PD 500 Hz |
| obs53 | 陀螺×0.25(3) ｜ 投影重力 R^T[0,0,-1](3) ｜ 零命令×[2,2,0.25](3，恒 0) ｜ q12−[0,0.65,−1.3](12，**轮角不入观测**) ｜ dq16×0.05(16) ｜ 上帧**裁剪后**raw 动作(16) ｜ 末裁剪 ±100 |
| 动作 | 腿目标 = 0.25×clip(a) + SUPPORT_BIAS(rad)；轮目标 = clip(5×a, ±98.47) 关节侧 rad/s；**无滤波/无插值/无轮锁/无航向补偿** |
| 增益 | 腿 40/1；轮 kp0/kd1（速度） |
| 力矩/限速 | 腿 60/60/90 N·m，25/25/16.67 rad/s；轮 15 N·m，98.47 rad/s |
| 入口 | 四足预备姿态 (0,0.65,−1.3)×4、四轮着地、根高 0.423439545 m、动作历史清零；**首帧立即出目标**（交付实测首帧腿目标相对实测最大跳 **1.281 rad** ≈ 51 N·m calf） |
| 范围 | 仅"起身 + 双后轮支撑保持"。行驶、松杆停车、楼梯、倒地恢复、多策略交接 **均不适用**；**无退出动作、无跌倒检测、无成功信号** |

**关节序**：交付合同按腿分组（FR,FL,RR,RL × hip,thigh,calf + 四轮）。本仓库 deploy.yaml 用**按类型序**（与 r1199/r214/r152 一致），目的是让共享发布端的硬编码姿态（T1/T2-cal/prone）与 FixSquat 折叠下标（4..8 / 8..12）保持正确；控制器内部用 `Pol2Cfg` 常量表在两序之间映射，DDS 槽位仍走 `config/r152_measured.map` 按**名字**匹配（hip 符号 −1，其余 +1）。

## 2. S0 收包（只读）

- `SHA256SUMS.txt` **345/345 全对**（Windows 与 caolin 各一遍）。
- 交付方自检：`replay/verify_replay.py` 3×1000 帧 PASS；`simulation/gate.py` MuJoCo 严格 3/3；`physx_gate.py` 名义 3/3、随机化 32/32 起身但**仅 4/32 严格静止**。
- 交付自述：`research_only`；ARM64 仅 QEMU 用户态对拍；**原生板卡与真机 not_run**。

## 3. S1 独立 parity（caolin，ORT 1.31.1 / numpy 1.26.4）

我们自己的 Python 端口（不 import 交付方 `runtime/reference.py`）逐帧复算 obs→ONNX→包装→目标：

| 磁带 | seeded obs | seeded ONNX raw | 包装 | 历史链 |
|---|---|---|---|---|
| nominal | 2.710e-7 | 5.722e-6 | 0 | 0 |
| vx+ | 2.850e-7 | 5.722e-6 | 0 | 0 |
| vx− | 2.719e-7 | 6.676e-6 | 0 | 0 |

seeded 模式（每帧注入磁带记录的 previous_action）隔离了 ONNX 版本差；closed 模式（自递归历史）漂移 ≤1.0e-5，与 r2000a 同类，属 ORT 版本属性而非移植缺陷。**判定：合同理解正确。**

## 4. S2 sim2sim（caolin，交付 MuJoCo 模型 + 我方 FSM，2026-11-20）

物理口径与交付 `simulation/mujoco_eval.py` 完全一致（2 ms 步、每 20 ms 十个子步、显式 PD + 力矩限幅 + 外推限速守卫）。

### 4.1 复现交付名义三例（我们的 ONNX + 合同实现）

三例均：起身 0.28 s、**机身零触地**、max effort 51.25 N·m（与磁带逐位相同）、末态重力 x≈−1、尾段后轮支撑 100%、尾段速度 0.002 m/s → 与交付 gate 3/3 一致。

### 4.2 我方 FSM 的进/出矩阵（关键结论）

| 场景 | 交接 | 起身 | 保持 10–15 s | 退出 |
|---|---|---|---|---|
| 交付标准 2 s 交接 | 2.0 s | ❌ 2 s 末仅升到 60°，随后**后坐到地**（机身 2.6 kN） | — | — |
| 我们的入口 | **0.5 s** | ✅ 0.5–0.52 s | ✅ 后轮 234–255 N、前轮 0 N、|gy|<0.03 | 见下 |
| 精确合同入口 | 0.0 s | ✅ 0.32 s | ✅ | 但把 1.281 rad 首帧阶跃直接打入（51 N·m、tau 峰 61 N·m） |

**结论：这个策略不能用共享的 2 s 交接**（2 s 内策略已过相位，物理跟不上 → 坐地）。0.5 s 是既避开坐地、又不把首帧阶跃硬砸的窗口。

退出方案对照（从抬高姿态）：

| 退出 | 机身触地峰值 | 收尾 | 判定 |
|---|---|---|---|
| `descend`（前腿先落地 1.5 s + 轮 −2.5 rad/s + 后腿 2.5 s 回站姿） | **0 N** | 四轮站立 gx≈0.05、位移 ≤0.32 m、前轮落地峰 ≈2.2 kN | ✅ 我方部署层下法 |
| `squat`（FixSquat 折叠） | 2.1–2.7 kN | 趴下/带侧滚 | ⚠️ 仅备用 |
| `x`（EmergencyDamping） | 4.5 kN（0.17 s） | 自由落体到趴 | ⚠️ 仅吊装/护垫场景 |
| ReturnStand→T2（仅腿目标混合） | 8–13 kN | — | ❌ 禁用 |

下法稳健性：μ 0.7/0.85/1.15/1.3、初始姿态 reset/t2sag、腿增益 ±10% 全部 **机身零触地、四轮收尾、位移 ≤0.32 m**；但相邻参数（前腿 2.5 s、轮 −6 rad/s）会失败，说明窗口不宽——**首次实机下法须 ≥1.5 m 空场 + 护垫**。

抗扰：reared 保持中施加 20/40 N 横向或前后推力（0.2 s），三例均无机身触地、尾段后轮支撑 100%、侧倾 <5°。

保持上限：策略自身证据窗口是 20 s（末 5 s 判静止）；本次 50 s 长跑中后段会漂移/倾倒 → **场上限设 10 s**。

### 4.3 用户提出的 Kp=85 复核

现场反馈 55/2 站立有**髋关节外挎**。MuJoCo 是对称模型，无法复现实机不对称外挎，但可量化"站姿误差"：

| 站姿增益 | 髋 | 大腿（目标 0.65） | 小腿（目标 −1.3） | 站姿静力矩峰值 | 起身 | 10 s 保持 | 下法 |
|---|---|---|---|---|---|---|---|
| 55/2 | ±0.008 rad | 0.60 | −1.45（垂 0.15） | 11.2 N·m | ✅ 0.48 s | ✅ | ✅ 0 触地 |
| **85/3.2** | ±0.006 rad | 0.63 | −1.38（垂 0.07） | **9.2 N·m** | ✅ 0.50 s | ✅ 后轮 ≥0.9 W | ✅ 0 触地 |

即：85/3.2 把站姿误差减半（=髋部需要代偿的偏置更少，这正是不对称外挎的驱动量），而静力矩反而**更低**，起身/保持/下法全部保持通过，μ0.8 与腿增益 −10% 亦然。**采纳 85/3.2**（本仓库既有先例：getup1199 rev2 起身链、T15-STIFF、hipdiag-P85D），仅作用于 A/FixStand→FixedStand 保持、交接起点增益与下法混合；**策略本身仍是 40/1，合同未改**。

## 5. S3 C++ 适配与双端构建

新增/修改（唯一维护源码 `部署参考/cpp_deploy-as2w/`）：

```
deploy/robots/as2w/rise63/{rise63_controller.hpp,rise63_controller.cpp,rise63_replay.cpp,CMakeLists.txt}
models/rise63/{deploy.yaml,policy.onnx}
tools/{rise63_cpp_parity.py,run_rise63_trial.sh,run_rise63_field_session.sh}
CMakeLists.txt（新增 AS2W_RISE63）
deploy/robots/as2w/common/as2w_publish.cpp（RISE63 分支）
deploy/include/isaaclab/algorithms/algorithms.h（共享修复，见下）
部署测试/运行现场策略会话.ps1（-Policy rise63 / -Mode rise63|rise63-dry-run / -RiseExit）
```

发布端 RISE63 分支要点：
- **交接 0.5 s**（编译期常量，launcher 不可改）；**PolicyWalk 上限 10 s**；
- **无命令通道**：摇杆永不映射，任何非零指令在控制器内直接 disarm；`--vx/--vy/--wz-limit` 一律拒绝；
- **守卫廓线**（rise63 专用，实测依据见 §4）：腿 dq 16/20、腿 tau 55/65、腿 PD 55/65、轮 dq 40/48、腿目标步进 0.75/1.0、轮目标步进 15/30；
- **warn 只记录、不自动刹车**（这条策略没有可"刹回去"的行走态，中途刹车本身就是摔倒），只有 hard（失控）才 EmergencyDamping；
- **reared 姿态守卫**（部署层新增，交付无跌倒检测）：arm 后 1.2 s 内豁免起身瞬态，之后要求重力 x < −0.50 且 |重力 y| < 0.60，否则 disarm → EmergencyDamping；
- **`--rise-exit descend|x|squat`**：本轮新增下法状态 `RiseDescend`（前腿 1.5 s 先落地 → 四轮 −2.5 rad/s 斜坡 1 s → 后腿按下俯仰进度 2.5 s 回站姿 → FixedStand → Y/超时趴下），2 s 卡死回退 + Y 随时中止；
- 首帧历史清零、无滤波无渐入、`0.25*a+bias` / `clip(5*a)` 与合同一致，全部由 `--offline-audit` 断言。

**共享代码两处修复（对既有策略向后兼容）**：
1. `deploy/include/isaaclab/algorithms/algorithms.h`：输出元素数改用 ORT `GetElementCount()`。原实现把 shape 各维相乘，遇到本交付的**动态 batch 轴（−1）**会溢出成巨大 size_t（`cannot create std::vector larger than max_size`）。固定 shape 模型行为不变（r2000a 回归 CTest 通过）。
2. `as2w_publish.cpp` 离线审计夹具：目标步进注入 1.0→1.5 rad、腿 dq 硬注入 9→25 rad/s（仅 RISE63 分支），warn 级注入在 RISE63 下断言"必须保持 log-only"。其余构建语义不变。

### 构建与门槛（全绿）

| 环境 | 结果 |
|---|---|
| caolin x86（ORT 1.22，gcc 13.3） | build OK；CTest `rise63-startup-audit` **1/1 PASS**；`RISE63 AUDIT PASS` + `AUDIT PASS` |
| caolin x86 回归 r2000a | 构建 OK；CTest `r2000a-joystick-startup-audit` **1/1 PASS**（共享改动无回归） |
| caolin C++ 磁带 parity | **PASS**：obs 2.978e-7 / 2.850e-7 / 2.719e-7，目标 2.29e-5 / 2.05e-5 / 2.29e-5（closed obs ≤8.9e-6，信息性） |
| CPRO ARM64（`build-rise63`） | 构建 OK；CTest **1/1 PASS**；`RISE63 AUDIT PASS`；二进制 SHA `ac8d6490…41ce60` |
| 三方 SHA 核对（8 文件） | 工作站 == CPRO，全部一致（见 §7） |

### 会话入口

- CPRO：`tools/run_rise63_trial.sh`（单次有界 B 环：`--zero-b-trial --stand t2 --debug --rise-exit <x>`，20 s 站姿保持 + 10 s 起身上限内置）、`tools/run_rise63_field_session.sh`（六步不可跳过链：audit→LowState 探测→控制权→质量+趴下位门槛→双监测→发布端，debug 三件套归档与断言）。
- 工作站：`运行现场策略会话.ps1 -Policy rise63 -Mode rise63|rise63-dry-run [-RiseExit descend|x|squat] [-Target <CPRO>] [-ConfirmMotion]`。
- **只读预检已通过**（2026-11-20 15:52，USB 目标 `0123456789ABCDEF`）：step1 二进制+SHA+audit PASS，step2 因**狗未上电 LowState=0 包**按设计 FATAL 止步，未做任何控制动作；会话目录已回拉 `logs/field_sessions/20261120-155224_rise63-dry-run/`。

## 6. 首测门槛（每一步须现场当次明确同意）

**当前必须先解决**：
1. **狗上电 / CPRO→狗 eth0 链路**（dry-run 停在 LowState 0 包）。
2. **吊装（用户方案）**：四轮必须着地；吊带松量 **≥0.35 m**（起身后机身升高约 0.19 m，松量不足会在最高点被拉死）；挂点在**机身重心正上方**，不要挂腿/轮；退出用 X 让吊带接住。

建议阶梯（每级单独同意）：

1. **P1 只读**：`-Mode rise63-dry-run -MonitorSeconds 30`（现仅差狗上电）。
2. **P2 吊装起身（只看能不能起）**：`-Mode rise63 -RiseExit x -MonitorSeconds 120 -ConfirmMotion`
   观察：B 后 0.5 s 内是否抬前轮 → 1 s 内是否到后轮支撑 → 保持到 10 s 上限 → X 被动退出由吊带接住。
   现场握原厂遥控器与物理急停；**X 是停止手段，Ctrl+C 不是**。
3. **P3 无吊装起身 + 下法**（P2 通过后）：`-RiseExit descend`，需 ≥1.5 m 空场 + 护垫；观察前轮落地 ~2.2 kN 冲击、是否四轮收尾、位移 ≤0.35 m，然后 Y 趴下。
4. **任何包络/增益/滤波/交接时长差异 = 新版本**，须重跑全链。

## 7. SHA（本轮构建输入与产物）

```
74b9f1f4676f760c6faba8e355909c57dac6180847331ccaeb5b191a00d1eeac  deploy/robots/as2w/common/as2w_publish.cpp
83ca1302d3456b41a47bc70ebde20fb99ea747b92ce83b398b56011331a0dfe2  deploy/robots/as2w/rise63/rise63_controller.cpp
7c32fba4e672e947a024c95193bdae7b26d1aa22cc531ce573d87ec9a6ba2917  deploy/robots/as2w/rise63/rise63_controller.hpp
2cf1225d68b66364f1b033f510b1dd699e5b994440b065e902e1c6c6068ad4d8  deploy/include/isaaclab/algorithms/algorithms.h
aaa96f821d4fcc82dcdb7f288916ec453ff28dd6ada87f6c4732f2d405474764  models/rise63/deploy.yaml
1f09372919d82d35b32aed032517571f03b48b67c99bfcc30760ea20db4c0031  models/rise63/policy.onnx   (= 交付 actor.onnx)
8d83ee1b4bfc4bc8740033a12c42c1c7d35f234af7dc86ba48fe1cba33d7630a  tools/run_rise63_trial.sh
81341ac0a49ec29f8dd4009b2f0df2ea9a2210708d675d12b1e728ab55dbf1ff  tools/run_rise63_field_session.sh
ac8d64907103442d1659ce8f4f7a6ef39a067ba546505bde5117e14f7441ce60  build-rise63/.../as2w_rise63_publish (CPRO ARM64)
```

## 8. 未验证 / 已知风险

- 原生 ARM64 **周期与 I/O** 未测（交付仅 QEMU 对拍）；本机 CTest 只证明逻辑与合同。
- 实机动力学（轮速环真实响应、摩擦、地面、载荷）未验证；MuJoCo 通过**不能**替代实机接管稳定性（R284 教训）。
- 40/1 增益下首帧 1.281 rad 阶跃在实机的冲击未实测（0.5 s 交接把它摊到 ~0.05 rad/帧，但策略自身在交接期内仍可能快速推目标）。
- 我方下法（RiseDescend）是**部署层新动作**，只有本模型证据；首次使用须空场+护垫。
- 交付方自述 PhysX 随机化严格静止仅 4/32：本策略在扰动下的"严格静止"能力有限，10 s 上限即为此。
- 轮是速度环（kp0/kd1），前轮在空中会以 32–34 rad/s 空转（交付行为，非故障）；reared 保持时前轮不触地。
