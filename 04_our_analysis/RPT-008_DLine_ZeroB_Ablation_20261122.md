# RPT-008 D 线深推：zero-B 后腿下沉的增益/资产/站姿单变量消融

编号：`RPT-008`
日期：2026-11-22（Asia/Shanghai）
对象：HIMLoco `model_18200`（unitreestyle 实机族），零命令（zero-B）站姿
一手文件：`04_诊断实验/闭环反事实/t3_3_zerob_sim_20261122/`（6 份结果 JSON + code + environment）
运行位置：用户自有服务器，隔离 venv，MuJoCo 4.0.1，CPU 单作业
边界：无 RL、无正式资产修改（所有改动在 experiment 内的模型副本/运行时覆盖）、无实机、无 CPRO

---

## 1. 结论（先看这四条）

1. **后腿下沉的主因是腿 PD 刚度不足（kp55），不是资产质量/质心，也不是接触模型。** 同一模型、同一协议，只把腿增益从 kp55/kd2 提到 **kp85/kd3.2**，末段"指令−实际"从 RR_thigh 0.378 / RR_calf 0.381 rad 降到 **0.110 / 0.162 rad**（约 −70%/−57%），pitch 从 −9.17° 回到 **−5.26°**（实机约 −6.13°）。`CONFIRMED`（同初态同协议单变量）
2. **把 MJCF 的 base 质量/质心回填到训练 URDF 数值，下沉反而略增**（RR_thigh 0.378→0.424，pitch −9.17→−10.15°）。⇒ 质量不是原因；`INFERRED`（回填只覆盖 base 1.012 kg，总量 27.012 vs URDF 27.512，未含其余 0.5 kg 差）
3. **原厂 `ai` 站姿不能作为 zero-B 初态**：以该姿态起跑，机身最低到 **0.148 m**（低于 0.15 m 跌倒门）——那是原厂运控的姿态，不是本策略的默认站姿。`CONFIRMED`（同协议）
4. **因此实机 0.33/0.24 rad 的下沉应归到"部署合同（kp55 静态刚度）/站姿"，不应归到 Sim2Real 植物或接触差异**。这与 T15 现场"kp55 撑不住、85/3.2 修复"的独立观察一致（RPT-005 §9）。`INFERRED`

## 2. 协议（六个变体完全一致，只改一个量）

- 资产：会话 `mujoco-scene_v2.xml` 副本（base 10.58 kg、整机 26.0 kg、地面摩擦 0.62、18 个 STL）
- 策略：`policy-model-18200.onnx`（SHA `02352011…678bf`），**按 C++/deploy.yaml 语义包装**（tanh → ×0.6 → 0.25 → α；轮 `ff+2tanh`；100 步交接 ramp）
- 时序：0.5 s 初态 PD 预稳定 → 2.0 s 交接（command=0）→ 保持到 9.7 s
- 初态：默认站姿 `[0,0.8,−1.6]×4`（主）或原厂 ai 实测站姿（对照）；`z0=0.33 m`；quat 取实机快照
- 指标：末 3 s 的 `leg_cmd − leg_actual` 均值（与实机 `POLICY_TRACK` 同口径）、pitch、base z、四轮法向力

## 3. 结果表

| 变体 | base kg | 总 kg | 腿增益 | 初态 | RL_thigh | RR_thigh | RL_calf | RR_calf | pitch 终 | z_min | 判定 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 基线 | 10.58 | 26.000 | 55 / 2 | 默认 | 0.133 | **0.378** | 0.196 | **0.381** | −9.17° | 0.340 | 下沉复现 |
| 资产回填 | 11.592 | 27.012 | 55 / 2 | 默认 | 0.125 | **0.424** | 0.204 | **0.417** | −10.15° | 0.330 | 更差 → 质量不是原因 |
| 原厂站姿 | 10.58 | 26.000 | 55 / 2 | 原厂 ai | 0.130 | 0.397 | 0.190 | 0.389 | −9.44° | **0.148** | 无效初态（近跌） |
| **kp85** | 10.58 | 26.000 | **85 / 3.2** | 默认 | **0.072** | **0.110** | **0.137** | **0.162** | **−5.26°** | 0.362 | **下沉 −60~70%** |
| 资产+kp85 | 11.592 | 27.012 | 85 / 3.2 | 默认 | 0.079 | 0.123 | 0.150 | 0.183 | −5.70° | 0.359 | 质量二阶 |
| 资产+原厂站姿 | 11.592 | 27.012 | 55 / 2 | 原厂 ai | 0.106 | 0.421 | 0.195 | 0.414 | −10.71° | **0.148** | 无效初态 |
| **实机（对照）** | — | — | 55 / 2 | B 键位姿 | **0.237** | **0.239** | **0.336** | **0.331** | ≈−6.13° | — | `OBSERVED` |

结果文件与 SHA256：

| 文件 | SHA256（前 16） |
|---|---|
| `t3_3_zerob_20261122_01.json`（基线） | `16ed6bea6436a566` |
| `t3_3_zerob_ablate_asset_backfill.json` | `b4149e37b3b8b2b5` |
| `t3_3_zerob_ablate_kp85.json` | `2cbafe95ad699a5e` |
| `t3_3_zerob_ablate_asset_kp85.json` | `a7cb3b51934fff7e` |
| `t3_3_zerob_ablate_factory_pose.json` | `b32102a698dea943` |
| `t3_3_zerob_ablate_asset_factory.json` | `5e6bf6b1bcda07b5` |
| 消融 harness `code/zerob_sim_ablate.py` | `80c1c3c8424f8374` |

本地与服务器 6 份 JSON `sha256sum` 逐一相同（2026-11-22 回拉）。

## 4. 机理解读（为什么是刚度）

零命令下，腿目标基本固定在 `default + applied`；实际关节角由静力矩平衡决定：

`q_actual ≈ q_cmd − τ_gravity / kp`（忽略摩擦与动态项）

- kp 55 → 85：同一重力矩下静差按 55/85 = 0.65 缩小；实测 RR 小腿 0.381→0.162（0.43×）、RR 大腿 0.378→0.110（0.29×）。小腿接近理论比例，大腿更低（大腿力矩小、还有几何耦合）——方向一致。
- base 质量 +1.012 kg（+3.9%）→ 髋/膝重力矩增加约同等比例，静差 0.378→0.424（+12%），比纯比例大，说明还叠加了姿态/接触重分布（见下）。
- 资产回填后 total 27.012 vs URDF 27.512（差 0.5 kg，未定位到具体 link，可能是 JT128/重复计），因此"资产回填"是**部分**回填，结论按 `INFERRED`。

## 5. 与实机的一致性 / 不一致

**一致**：方向（实际角低于指令角，即被重力压下）、量级（0.13–0.42 vs 0.24–0.34）、以及"kp55 不够"的机理。

**不一致**：左右不对称（仿真 RR 明显大于 RL，实机左右接近）、pitch 更负（−9° vs −6°）、仿真尾段 **FL 法向力可到 0**（离地）导致载荷压到 RR。这些差异指向**接触/载荷分配**这一层，但不是"下沉有无"的主因。⇒ 若要继续追左右对称性，下一步应是接触参数（摩擦/接触刚度/初始 y 偏移）单变量，而不是再改质量。

## 6. 为什么原厂站姿不能当初态

原厂 `ai` 站姿实测（`field/20261120-180312_low-state_32209.log`，策略序）：thigh ≈ 1.05、calf ≈ −2.77，比默认 `(0.8, −1.6)` 明显更伸展；在 `z0=0.33` 起跑会先把腿压弯/机身掉到 0.148 m（< 0.15 m 跌倒门）。**这反过来说明：现场 B 键时的 `pose_error=0.202156 rad` 意味着机器人当时已接近默认站姿，而不是原厂站姿**——这条对 zero-B 复现的初态选择很关键。

## 7. 下一步（D 线剩余）

1. **接触/对称性单变量**（可选）：初始 y 偏移、地面摩擦 0.62→0.9、接触刚度（solref/solimp），看 RL/RR 是否对称化；不改正式资产，只在 experiment 副本。
2. **把"kp55 静态差"写进交付合同检查项**：任何 zero-B 验收必须记录 `leg_cmd−leg_actual` 稳态值，而不是只看 B 门 `pose_error`。
3. **交叉到 C 线**：R2000A 也用 kp55；T2-1 之后应做同样的 kp55 vs kp85 对照（只在 candidate 副本），检验零速/偏航是否同源。
4. PT 权重仍仅声明（`85072560…`），若将来要复训，先用本报告的 kp/站姿结论约束站姿验收。

## 8. 复现

```bash
# 服务器（单作业；隔离 venv）
EXP=/root/gpufree-data/as2w-cpro/systematic_project_audit_20261121/experiments/t3_3_zerob_sim_20261122
cd $EXP/code
COMMON="--scene scene_v2.xml --policy policy-model-18200.onnx --input himloco_zeroB_input_20261122_01.json"
$EXP/venv/bin/python zerob_sim_ablate.py $COMMON --tag ablate_kp85 --leg-kp 85 --leg-kd 3.2 --out ../results/t3_3_zerob_ablate_kp85.json
$EXP/venv/bin/python zerob_sim_ablate.py $COMMON --tag ablate_asset_backfill --base-mass 11.592 --base-com-x 0.020002 --out ../results/t3_3_zerob_ablate_asset_backfill.json
```

环境记录：`t3_3_zerob_sim_20261122/environment/environment.txt`（MuJoCo 4.0.1、onnxruntime 1.32.0、numpy 2.5.5、Python 3.12.3、GPU 空闲）。
