# RPT-009 C 线深推：R2000A 偏航因果对照（full composite vs base-only）

编号：`RPT-009`
日期：2026-11-22（Asia/Shanghai）
对象：R2000A `as2w_residual_run2000a`（交付包 `as2w-rolling-full-bundle-20261120`）
任务：交接 T2-1 —— 同初态、同命令、同资产，**只改动作组合**（full composite / base-only），判断偏航摔倒的因果是在 residual/gate 还是 base/资产
一手文件：`04_诊断实验/闭环反事实/t2_1_yaw_contrast_20261122/`（3 份结果 JSON + logs + code）
运行位置：用户自有服务器，隔离 venv，CPU 单作业；MuJoCo **4.0.1** 与 **3.17.0** 各跑一遍

---

## 1. 结论

1. **偏航摔倒不需要 residual 也能发生。** base-only（去掉 residual 与 gate）在 YAWL/YAWR 同样摔倒，只是晚 0.9–1.2 s。⇒ 失败的主因在 **base actor + 资产**，不在 residual/gate。`CONFIRMED`（同初态同命令单变量）
2. **摔倒形态是"侧倾"而不是俯仰**：YAWL 终态 roll −62.2°、pitch −8.1°；YAWR roll +60.8°、pitch +2.2°。与交付包 `known_deviation` 里"pure-yaw 出现 roll divergence"一致。
3. **MIX 在本地复现里不摔**（tilt 32.7°，400 步完成），而交付证据记录 MIX 6.10 s 摔倒。**MuJoCo 版本不是原因**：3.17.0 与 4.0.1 结果**逐位相同**。⇒ 差异来自交付方那次运行的另一处条件（harness/合同修订版/资产副本），`UNKNOWN`，已排除"版本差异"这一候选。
4. 因此：**"MuJoCo 偏航必摔"不能当成策略固有属性引用**；能引用的是"在本地这套 MJCF（26.0 kg）+ 本 harness 下，YAWL/YAWR 5.6/5.8 s 侧倾摔倒，MIX 8 s 存活"。跨 harness 的复现性另需交付方 harness 版本。

## 2. 实验设置

| 项 | 值 |
|---|---|
| 包 | `package/{runtime/contract.py, model/weights.npz, simulation/mujoco/as2w_scene.xml, simulation/urdf/as2w-load-v0.urdf, simulation/mujoco/meshes}` |
| SHA | contract `b567568c…`、weights `4ce9f513…`、scene `89ea64da…`、URDF `7c9740a9…`（与 `SHA256SUMS.txt` 一致） |
| 变体 | `full` = `clip(clip(base)+gate·0.24·tanh(res))`；`base_only` = `clip(base)` |
| 场景 | STAND(0,0,0)、YAWL(0,0,0.6)、YAWR(0,0,−0.6)、MIX(0.45,0.2,0.45)；每场景 400 步 × 0.02 s = 8 s |
| 复位 | z=0.390、quat=identity、关节默认角、qvel=0；先 50 步（1 s）零命令 settling |
| 接触/执行器 | 同交付 MJCF：腿 position kp55 kv2、轮 velocity kv2、地面 friction 1.0；`timestep` 被设为 0.005 s（4×0.005=0.02） |
| 摔倒门 | tilt > 60° 或 base_z < 0.15 m |
| 环境 | MuJoCo 4.0.1 / 3.17.0，numpy 2.5.5，Python 3.12.3，隔离 venv，GPU 空闲 |

## 3. 结果

| 变体 / 版本 | STAND | YAWL | YAWR | MIX |
|---|---|---|---|---|
| **full / 4.0.1** | 存活，tilt 2.2°，roll +0.1° | **摔 5.58 s**，tilt 62.5°，roll −62.2°，z_min 0.283 | **摔 5.80 s**，tilt 60.8°，roll +60.8°，z_min 0.329 | 存活，tilt 32.7°，roll +21.8° |
| **base_only / 4.0.1** | 存活（与 full 相同） | **摔 6.48 s**，tilt 64.1°，roll −64.0° | **摔 7.02 s**，tilt 61.4°，roll −60.9° | 存活，tilt 34.0°，roll +24.5° |
| **full / 3.17.0** | 存活 | 摔 5.58 s（逐位同 4.0.1） | 摔 5.80 s（逐位同） | 存活（逐位同） |
| 交付证据（3.17.0，交付方 harness） | 存活 | 摔 5.22 s | 摔 7.34 s | **摔 6.10 s** |

- 时间差：YAWL full 5.58 vs base-only 6.48（Δ0.90 s）；YAWR 5.80 vs 7.02（Δ1.22 s）。**两者都摔**，超过"仅 full 显著恶化"的分支，但残差确实让摔倒提前约 1 s。
- STAND 三个变体/版本完全一致（tilt 2.23°、roll 0.08°、pitch 0.04°、z_min 0.381）→ 复位与执行器链正确。
- 动作统计（full）：`base_raw_frame_any_clip_frac=1.0`、`composite_pre_frame_any_clip_frac=1.0`、gate 均值 1.0、base_raw 绝对值最大 174.1 —— 与 RPT-003 的饱和结论一致（偏航命令下每帧都有 clip）。

### 3.1 判据映射（交接 §9 P2 / T2-1）

| 判据分支 | 本次结果 |
|---|---|
| base-only 与 full **都摔**且方向/时间接近（\|Δt\|≤0.5 s）→ 优先训练/base+资产 | **命中"都摔"**；Δt 0.9–1.2 s（略超 0.5 s 阈值）→ 主因归 base/资产，残差是加速项 |
| 仅 full 显著恶化 → residual/gate 因果 | 不成立（base-only 也摔） |
| 仅 MuJoCo 摔而 Isaac 同 trace 不摔 → 跨引擎审计 | 部分成立：交付 Isaac 8/8 存活，本地 MuJoCo 偏航摔；但 MIX 在本地存活，说明跨引擎/跨 harness 结论不稳 |

## 4. 机理线索（`INFERRED`）

- 摔倒由 **roll** 主导：YAWL 时 roll 从 0 掉到 −62°，而 pitch 只有 −8°。轮式底盘在纯偏航时，左右轮速差产生横滚力矩；base actor 在偏航命令下几乎全维饱和（base-only 也摔说明符号图样本身不足以稳定横滚）。
- residual 的作用是"延后约 1 s"，不是"阻止"：说明 gate×0.24×tanh(res) 只在有限幅值内提供修正，无法覆盖横滚累积。
- MIX 存活而纯偏航摔，与"gate=1 时 residual 也参与、混合命令的轮速差更小"一致；但这只是假设，未做单变量（例如固定 vx 改 wz 扫描）。

## 5. 与既有证据的关系

| 既有表述 | 本次修订 |
|---|---|
| "MuJoCo 偏航 5.2–7.3 s 摔倒（交付证据）" | **部分复现**：YAWL 5.58、YAWR 5.80 摔；MIX 8 s 存活。版本不是原因（3.17.0 与 4.0.1 相同） |
| "偏航失败不能主要归 Sim2Real" | 维持 `SIM_RUN_CONFIRMED`（本地也在仿真内摔） |
| "residual/gate 是因果候选" | **下调**：base-only 也摔 → residual 不是必要条件；只能说是加速项 |
| "MIX 版本翻转不构成解锁"（AGENTS） | 维持；本地 MIX 存活不改变实机 yaw 锁 |

## 6. 限制

1. 资产：MJCF 26.0 kg / base 10.08 kg，训练 URDF 27.512 kg / base 11.592（见 RPT-003 §3.5）。**未做资产回填对照**（下一步）。
2. harness 与交付方不同（本脚本按 `contract.py` 复述并旋转 ang-vel；交付方 harness 版本未知）→ MIX 差异归 `UNKNOWN`。
3. 单次运行、无种子；偏航是确定性 rollout，但接触求解在跨版本可能分叉（本次两版本恰好一致）。
4. 未做 vx/wz 混合扫描、未做 residual-only、未做 kp85 对照（D 线已证 kp 影响静态差，C 线尚未做）。

## 7. 下一步（C 线剩余）

1. **资产回填对照**：把 MJCF base 质量/COM 改到 URDF 值（26.0→27.012 或 27.512），重跑 YAWL/YAWR/MIX full+base_only，看是否仍摔、时间如何变。
2. **residual-only**：只用 residual（无 base）作为诊断，确认 base 符号图样是否必要。
3. **kp85 对照**：D 线已证 kp 主导静态下沉；C 线检验偏航摔倒是否也受 kp 影响（合同层，不改正式资产）。
4. **wz 扫描**：固定 vx=0，wz ∈ {0.1,0.2,0.3,0.45,0.6}，找摔倒阈值——比"单点 0.6"更能说明是否域外。

## 8. 证据索引

| 文件 | SHA256 |
|---|---|
| `results/t2_1_full_mj401_20261122_02.json` | `898ceec8ce06674b63cedd5744bf690557145d5220aa2cee53d1bec716767560` |
| `results/t2_1_base_only_mj401_20261122_02.json` | `9019f6f56b603da7e7dbee3dbb01189e3c7b82f55e218a5797207675b4dcb345` |
| `results/t2_1_full_mj3170_20261122_01.json` | `e5d5e0e68c44dd134a146a680c9f4895c759fe0e820a1fe1caff695becaee58a` |
| `logs/{full_mj401,base_mj401,full_mj3170}.log` | `44d7e10f…` / `1b03ddbd…`（3170 与 401 日志逐字节相同） |
| `code/run_mujoco_yaw_contrast_20261122_01.py` | `5838be2cae59465d7d5bd5d71e06c82f6612ef6f2d800b8fa67500bb9c43eea0` |

复现（服务器，单作业）：

```bash
EXP=/root/gpufree-data/as2w-cpro/systematic_project_audit_20261121/experiments/t2_1_yaw_contrast_20261122
cd $EXP
../t3_3_zerob_sim_20261122/venv/bin/python code/run_mujoco_yaw_contrast_20261122_01.py \
  --package-root package --work-dir work_full --output results/t2_1_full_mj401_20261122_02.json \
  --variants full --scenarios STAND,YAWL,YAWR,MIX --steps 400
./venv317/bin/python code/run_mujoco_yaw_contrast_20261122_01.py \
  --package-root package --work-dir work_full_317 --output results/t2_1_full_mj3170_20261122_01.json \
  --variants full --scenarios STAND,YAWL,YAWR,MIX --steps 400
```
