# CPRO 实机部署记录（抖动权重）· 2026-11-20

对象：`as2w-v2-step-jit-20261120`（权重 `9999512641a16f20…c41c33`，部署 ONNX `7a0367e5…435b715`）
范围：**只做到部署 + 只读验证**。未发布任何 LowCmd、未改 ReleaseMode/SelectMode、未做任何动作测试。

## 0. 连接

| 项 | 值 |
|---|---|
| 工作站 WLAN | 192.168.10.38 |
| CPRO 当前地址 | **192.168.10.75:8710**（旧 192.168.10.180 已失效，DHCP 重分） |
| 连接 | `hdc tconn 192.168.10.75:8710` → `Connect OK`，`list targets` 显示 `TCP Connected` |
| 通道 | WiFi 8710 与 USB `0123456789ABCDEF` 双通；所有命令带 `-t` |
| 进入 Ubuntu | `/data/local/tmp/as2w_field/cpro_ubuntu.sh /bin/bash -lc '<b64>'`（chroot 到 `/data/ubuntu_in_hm/ubuntu_rootfs`） |

## 1. 模型同步

| 文件 | 目标 | SHA256 |
|---|---|---|
| `policy.onnx`（tanh 烘入） | `/root/as2w_deploy/models/unitree58-v2step/` | `7a0367e507ae2fdf144fe82acedf2094f023312f69eee122ef24390a0435b715` |
| `deploy.yaml` | 同上 | `88b288f01e52c11bc431ebfe2804d2cb72187df11f6419b59f5b3bac53a82200` |
| `README.md` | 同上 | — |
| `models/manifest.json`（含本策略条目） | `/root/as2w_deploy/models/` | 条目 SHA 与训练侧一致 |

传输路径：`hdc file send` → `/data/local/tmp/as2w_field/incoming/` → root 复制进 ubuntu rootfs（rootfs 在 `/data/ubuntu_in_hm/ubuntu_rootfs`）。

## 2. ARM64 构建

```bash
cd /root/as2w_deploy
cmake -S . -B build-unitree58-v2step -DCMAKE_BUILD_TYPE=Release \
  -DAS2W_UNITREE58=ON -DAS2W_UNITREE58_LP5=ON -DAS2W_UNITREE58_S15=OFF \
  -DAS2W_UNITREE58_PUBLISH=ON \
  -DONNXRUNTIME_ROOT=/root/ort122 \
  -DCMAKE_PREFIX_PATH=/root/unitree_sdk2_install
cmake --build build-unitree58-v2step --parallel 8
```

结果：`cmake_rc=0`、`build_rc=0`；产物
`build-unitree58-v2step/deploy/robots/as2w/unitree58/{as2w_unitree58_lp5_replay, as2w_unitree58_lp5_publish}`。
工具链：cmake 3.22.1 / g++ 11.4.0（Ubuntu 22.04）/ onnxruntime 1.22。

## 3. CTest

```
9/9 tests passed, 0 failed
（含 unitree58-startup-audit、unitree58-lp5-startup-audit）
```

`unitree58-lp5-startup-audit` 通过 = 加载期合同硬检查（model_version/58/12/5 Hz/0.25/55-2/0-2/HEIGHT .37/idle .02）全部满足。

## 4. ARM64 只读回放（无 DDS、无 LowCmd）

```
UNITREE58_REPLAY_FRAMES=300 LEG_ALPHA=0.466512 WHEEL_ALPHA=0.466512 \
  MAX_LEG_STEP=0.104907 MAX_WHEEL_TARGET=0
UNITREE58_MODEL input=obs[N,58] output=act[N,12] float32
```

- `LEG_ALPHA=0.466512` 与训练 `control.toml` 的 5 Hz 合同完全一致（旧 10 Hz 二进制会硬拒这份 yaml）。
- `MAX_LEG_STEP=0.1049 rad`，远低于理论最坏 0.358 与现场软件 warn 门槛 0.2。
- fixture：抖动权重 DR0 回放带的 300 帧真实传感器序列（trial 0，0–6 s，零命令段）。

## 5. C++ ↔ Python/ONNX 端到端 parity

同一 300 帧、同样从 fresh-armed（目标全零）开始递推：

| 量 | 最大绝对误差 |
|---|---|
| 观测（C++ vs numpy 运行时） | **7.39e-06** |
| 腿位置目标（C++ vs Python 包装） | **1.85e-06** |
| 轮速目标 | **0.0** |

判定 PASS（阈值 2e-5）。这一项等价于把「传感器 → 观测 → ONNX → tanh → 低通 → 16 轴目标」整条链在 ARM64 上重放了一遍。

## 6. 未做与原因

| 项 | 状态 | 原因 |
|---|---|---|
| 发布端启动 | 未运行 | 会进入 DDS 并由外层决定是否发布；需现场当次同意 |
| ReleaseMode / SelectMode | 未触碰 | 同上 |
| 任何动作测试 | 未做 | 硬规则：可能让狗动作的命令须先说明并获现场当次明确同意 |
| 机器人当前状态 | 未改变 | 全程只读 |

## 7. 下一步（等现场）

1. 现场就位、原厂遥控器与物理急停在手；
2. 当次许可后：`A` 固定 PD 站 → `B` 接管（零命令）→ 观察 7–10 s；
3. 首测命令上限：`vx=0`、`vy ±0.10`、`wz ±0.10`，单次 ≤ 30 s；
4. 记录 `debug_lowcmd.csv` / `debug_policy.csv` 与保护流，回传后做抖动与跟踪量化。
