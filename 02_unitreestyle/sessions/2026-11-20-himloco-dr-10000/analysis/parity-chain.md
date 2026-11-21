# 三级 parity 链（部署正确性证据）

## L1：ONNX ↔ PyTorch（真实环境观测）

`him_arc/deploy_parity_check.py`：在同一个 `ArcEnv` 里让 torch 策略与 ONNX 策略同步运行 60 步，逐帧比较动作。

```
max_abs_action_diff = 2.384185791015625e-06
mean_abs_action_diff = 1.1920928955078125e-06
parity_ok = true
obs_dim = 360
```

**踩过的坑**：HIM estimator 输出 latent 前有 `F.normalize(z, dim=-1, p=2)`。第一版导出漏了这层，真实观测 parity 差 **6.47**（等于完全错的策略），补上后降到 2e-06。

## L2：C++ 控制器 ↔ Python 参考（120 帧夹具）

用训练环境录 120 帧传感器夹具（vx=0.10、model_18200 闭环），C++ replay 与 Python 参考逐帧比较。

```
max_leg_target_diff_rad     = 1.1920928955078125e-07
max_wheel_target_diff_rad_s = 9.5367431640625e-07
parity_ok = true
```

C++ replay 输出：

```
HIMLOCO_MODEL input=obs[N,360] output=out[N,16] float32
HIMLOCO_GAIN variant=flat[0.6,0.6,0.6] leg_range_rad hip=0.15 thigh=0.15 calf=0.15
HIMLOCO_REPLAY_FRAMES=120 LEG_ALPHA=0.71539 WHEEL_ALPHA=0.466512
MAX_LEG_STEP=0.0582273 MAX_WHEEL_TARGET=2.57255 MAX_|action|=0.996055
```

**踩过的坑（两处，均为我方实现错误，C++ 原本正确）**：
1. 动作映射应为 `tanh(out) → ×0.6（腿增益）→ ×0.25`，轮为 `(vx−wz·side)/R + 2·tanh`；早期按 `QualityEnv` 单独推导成 "clamp + ×12" 是错的。
2. 历史栈首帧应**整栈填满当前帧**（训练 reset 行为）；Python 参考最初留零，导致首帧分歧。

## L3：ARM64 ↔ x86（同一权重、同一夹具）

CPRO ARM64 构建的 replay 与服务器 x86 输出逐帧对比：

```
x86 targets 120, arm64 targets 120
max_abs_target_diff = 6.11e-07  (纯浮点舍入)
```

## 附带：CPRO CTest / 离线审计

```
100% tests passed, 0 tests failed out of 8
AUDIT PASS: synthetic HG frames and wake timing; command slew and field safety guards;
            offline fault injection (warn/hard/motorstate/remote-timeout/slew);
            firmware acceptance unknown
```

元数据：[`../data/parity.json`](../data/parity.json)、[`../data/export.json`](../data/export.json)。
