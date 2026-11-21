# 分析脚本清单（2026-11-20 会话）

本目录是这次会话里**实际用到的分析代码**。所有结论都能由这些脚本从 `data/`、`analysis/` 或原始日志重算，
但它们是在作者本机上跑的，输入路径写的是会话工作目录（`X:/Items/as2w/.stage58/...`）；
要复跑请把脚本里的输入路径指向对应文件即可，逻辑本身与机器无关。

## field/ — 实机日志分析

| 脚本 | 输入 | 输出 | 本会话得到的关键数字 |
|---|---|---|---|
| `analyze_zero_b3.py` | CPRO `debug_policy.csv`（WALK 相位） | p2p / 翻转率 / 每步增量 / 跟踪误差 / pitch·roll | 抖动权重零命令：p2p 0.360、翻转 8.2%、跟踪 0.101、pitch −0.51°、roll −2.49° |
| `analyze_zerob_mu025.py` | 同上（mu025 权重） | 同上 + 髋 PD 力矩 | mu025：翻转 12.85%、pitch +0.95°、roll −0.58°、髋 5.32/4.90/4.38/3.92 Nm |
| `analyze_lateral.py` | 摇杆第一轮（wz≤0.10） | 命令使用率 / yaw 跟踪比 | yaw 0.079，vy 有效帧仅 7.6% |
| `analyze_trial3.py` | 摇杆抬包络（wz≤0.30） | 同上 + 侧向加速度 | yaw 0.164（指令 0.26 → 实测 0.049 rad/s） |
| `analyze_lat_mu025.py` | mu025 摇杆轮 | 同上 | yaw 0.188（该轮指令仅 0.10，另有同幅对照见 `slopes.py`） |
| `tau_estimate.py` | 零命令日志 | 髋 PD 力矩 `τ=55·e−2·dq` | 髋 6.19/3.98/4.18/5.91 Nm（实机运行口径） |
| `gait_sign.py` | 摇杆日志 | 按 `cmd_vy` 符号分组的各腿 p2p | `cmd_vy>0` 时四腿全动（左向步态特征），右向特征在该轮无样本 |
| `imu_lateral_sign.py` | 摇杆日志 | 世界系侧向加速度（扣重力、扣静态偏置） | 偏置校正后 **−0.0014 m/s²** → **几乎没有侧向位移**（此前"符号反了"是静态倾角假象） |
| `compare_yaw_lat.py` | 两版权重的摇杆日志 | 分档 yaw/侧移响应 | 初步分档对照 |
| `slopes.py` | 两版权重的摇杆日志 | 有符号回归斜率（对混号鲁棒） | **yaw 0.199（旧）vs 0.146（mu025）**；侧移 +0.709 vs −1.141 |

## sim/ — 仿真侧分析

| 脚本 | 输入 | 输出 | 关键数字 |
|---|---|---|---|
| `torque_from_tape.py` | 传感器回放带 npz | 12 腿轴 PD 力矩（静态口径） | 原资产前髋 11.64 Nm、mu025 11.70 Nm（策略运行口径） |
| `compare_static.py` | `static_t2_probe` 输出 npz | 不同摩擦/关节摩擦/质量/质心下的髋负载表 | μ=0.5 → 0.39 Nm；μ=0.25 → 5.37 Nm（**13.8 倍**） |
| `compare_com.py` | 同上 | 质心扫描表（未加刚持约束，姿态会漂移） | 判定该口径不可比，改成 `compare_com_stiff.py` |
| `compare_com_stiff.py` | kp=200 刚性保持输出 | 真实负载口径（前髋） | μ=0.5 → **1.66 Nm**；μ=0.25 → **11.91 Nm**；实机 11–12 Nm |
| `jitter_metrics.py` | 回放带 | 部署口径抖动指标（p2p/增量/翻转/主频） | 用于 DR0/DR2 四组对照 |
| `compare_report.py` | 四次 eval 快照 | 生成对照表（md/json） | `docs/jitter-compare-20261120.md` 的数据来源 |

## parity/ — ONNX 与 C++ 逐帧对拍

| 脚本 | 用途 | 关键结果 |
|---|---|---|
| `make_fixture.py` | 从传感器回放带生成 C++ replay 用的 fixture | 300 帧、58 维观测 |
| `parity_cpp2.py` | C++ 目标 vs Python(numpy+ONNX) 递推 | 观测 7.39e-6、腿目标 1.85e-6、轮目标 0.0 → PASS |
| `parity_cpp.py` | 早期版本（错在没对齐 C++ 的 fresh-armed 初值） | 保留作为"坑"的记录：obs 差 1.01 全来自 prev 块 |
| `diag_parity.py` | 分块定位 obs 差异 | 证明差异只在 `prev` 块（递推起点） |
| `check_tanh_onnx.py` | tanh 烘入 ONNX vs tanh(记录动作) | 2.56e-6（抖动权重）/ 2.65e-6（mu025） |

## assets/ — 资产实验

| 脚本 | 用途 | 结果 |
|---|---|---|
| `make_com_variant.py` | 改 base_link 质心（参数顺序：`<src> <dst> <dy> <dx>`） | 用于质心敏感性实验 |
| `make_mass_variant.py` | 改 base_link 质量 | +1/+2/+3 kg 扫描 |
| `inspect_usd_com.py` | 打印 base_link 质量/质心 | 12.092 kg、(0.0492, −0.0007, 0.0201) |
| `check_usd_friction.py` | 尝试读轮胎材质 | 查不到显式绑定 → 轮胎摩擦取 Isaac 默认值（这是摩擦缺项的来源之一） |

## training/ — 训练与评测驱动

| 脚本 | 用途 |
|---|---|
| `eval_out.sh` | 固定回放 + 门禁 + 停车回放（DR0） |
| `eval_dr.sh <ckpt> <dir> <dr>` | 指定 DR 等级的评测；DR2 需 `AS2W_CAPTURE_TAPE=1` 才落回放带 |
| `eval_asset.sh <ckpt> <dir> <asset.usd>` | 换资产评测（mu025 摩擦对齐实验用） |
| `chain_mu025.sh` / `chain_jitter.sh` / `chain_compare.sh` | 200/400/600 自动复评链、抖动对照链 |

## plotting/ — 出图

| 脚本 | 产出 |
|---|---|
| `make_plots.py` | `plots/p1–p5`、`p7`、`data/field-*.csv`、`data/gate-matrix.csv` |
| `rebuild_training_plots.py` | `plots/p6`、`plots/p8`、`data/training-*.csv` |

## 结果溯源表（结论 → 脚本）

| 结论 | 由哪个脚本产出 |
|---|---|
| 实机抖动 30% → 8.2% | `field/analyze_zero_b3.py` + 2026-11-19 的现场报告 |
| 左倾消除（roll −2.49°→−0.58°） | `field/analyze_zerob_mu025.py` |
| 摩擦是仿真缺失项（0.39→5.37→11.91 Nm） | `sim/compare_static.py` + `sim/compare_com_stiff.py` |
| 关节摩擦/质量/质心不是解释项 | `sim/compare_static.py`、`sim/compare_com_stiff.py` |
| 摩擦对齐重训后实机转向更差（0.199→0.146） | `field/slopes.py` |
| 侧移几乎没有净位移 | `field/imu_lateral_sign.py` |
| C++/ARM64 与 Python/ONNX 一致 | `parity/parity_cpp2.py` |
| 右后轮全臂不抬 | `plotting/rebuild_training_plots.py`（`event_right_RR` 曲线） |
