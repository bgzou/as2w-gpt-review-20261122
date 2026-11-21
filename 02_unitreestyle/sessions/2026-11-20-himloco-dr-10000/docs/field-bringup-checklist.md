# HIMLoco 平地 DR（model_18200）实机首测准备单

**策略**：`/root/gpufree-data/as2w-runs/him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt`
**部署包**：`部署参考/cpp_deploy-as2w/models/himloco-dr-18200/`
**状态**：离线导出与 parity 已完成；**C++ 编译/CTest/audit、ARM64 同步、CPRO 部署、真机动作均未执行**。

---

## 1. 已完成的离线准备

| 项 | 结果 |
|---|---|
| ONNX 导出（estimator+actor 融合） | `policy.onnx`，输入名 **`obs`**，`[N,360] → [N,16]` float32 |
| ONNX SHA256 | `02352011169cbf1dc3acfe3b698de27b98a32b6ad56c6063118c3d6e999678bf` |
| 图内 parity（随机输入） | 7.63e-06 |
| **真实观测 parity**（60 步，env 观测逐帧对比） | **2.38e-06**，`parity_ok: true` |
| 参考控制器 selftest | 输入 `[batch,360]`、输出 `[batch,16]`、α_leg 0.7154、α_wheel 0.4665 |

**关键修正记录**：HIM estimator 的 latent 在返回前做了 **L2 归一化**（`F.normalize(z, p=2)`）。第一版导出漏了这一步，真实观测 parity 差 6.47；补上后降到 2e-06。任何重新导出都必须保留这一层。

---

## 2. 部署契约（与 `models/himloco-dr-18200/deploy.yaml` 一致）

| 项 | 值 | 与 stairs 线（`models/himloco`）的差异 |
|---|---|---|
| model_version | `himloco-360` | 相同 |
| 腿动作增益 | **[0.6, 0.6, 0.6]** → 各腿关节 ±0.15 rad | stairs 是 [0.6, 1.0, 1.0] |
| 相位时钟 | **`phase_clock: true`**，obs[58:60] = sin/cos(φ)·strength | stairs 恒为 0 |
| 轮映射 | `clamp((vx − wz·side)/R + 2·tanh(out), ±12)` | 相同 |
| PD | 腿 55/2，轮 0/2 | 相同 |
| 首飞包络 | vx ≤ **0.10**，vy = 0，wz = 0 | stairs 首飞 vx 0.15 |

`phase_clock` 是本次新增的控制器选项（`himloco_controller.{hpp,cpp}`），默认关闭，stairs 线行为不变。

---

## 3. 实机前必须补齐的门槛（按顺序）

| # | 阶段 | 判据 | 状态 |
|---|---|---|---|
| G0 | 收包验收 | policy.onnx SHA 与上表一致；deploy.yaml 常量齐 | ✅ 离线可查 |
| G1 | Python parity | `parity_ok: true`、max diff < 1e-3 | ✅ 2.38e-06 |
| G2 | x86 编译 + CTest + audit | `AS2W_HIMLOCO=ON`、`AS2W_POLICY_HIMLOCO=ON`，CTest 全过，audit PASS | ⬜ |
| G3 | 只读 shadow | 同一段 LowState 回放，C++ 与 Python ONNX 动作逐帧一致 | ⬜ |
| G4 | 上电只读 | ReleaseMode、16 轴 mode0、LowState ~1 kHz、无保护位 | ⬜ |
| G5 | A 段固定 PD 站 | 现场确认站稳，无报警 | ⬜ |
| G6 | B 段零速交接 | 100 步零命令，观测里 `cmd=0`、`applied` 从 0 缓入，无终止 | ⬜ |
| G7 | 有界前进 | `vx=0.10` 单次，≤2 s，自动刹轮回站 | ⬜ |
| G8 | 停车/回站 | 首秒位移记录，回站后 Y 趴下 | ⬜ |

**每次可能让狗动作的命令仍需现场当次明确同意**；G2/G3 是纯离线，不需要许可。

---

## 4. 构建与打包（在 CPRO / caolin 上执行，本机无工具链）

```bash
# CPRO ARM64（构建目录沿用既有 build-*，不要删 CMakeCache）
cmake -S /root/as2w_deploy -B /root/as2w_deploy/build-himloco-dr \
  -DAS2W_HIMLOCO=ON -DAS2W_POLICY_HIMLOCO=ON \
  -DONNXRUNTIME_ROOT=<ort-root> -DCMAKE_PREFIX_PATH=<sdk2>
cmake --build /root/as2w_deploy/build-himloco-dr -j4
ctest --test-dir /root/as2w_deploy/build-himloco-dr
```

打包前在工作站运行 `部署测试/准备现场包.ps1`，并核对 `01_现场部署包/SHA256SUMS.txt`。

---

## 5. 风险（写进现场交底）

1. **sim2real 静差未解决**：实机 T2 固定姿态下仿真比实机多沉约 7.5°，前髋力矩约为实机一半（3.7–5.1 vs 11–12 Nm）。本策略在该静差下训练，**首飞必须小步速、低增益、现场看护**。
2. **接触合同是 0.25 + min**（USD 轮胎与地面）。若实机轮胎/地面不是这一档，闭环与训练不一致。
3. **`ckpt["iter"]` 停在 16800**，版本只能看文件名 `model_18200`。
4. **观测 41:57 是“施加目标”**，不是策略原始输出；用错会让观测分布偏移。
5. 训练分布含 ±0.5 kg / ±10 mm 质心随机化；实机若超出（加电池/支架）需重训。
6. 姿态惩罚（weight 200，6° 持续 / 15° 瞬时）让策略更平（站立 tilt95 2.65°→1.06°），但停车尾部漂移在 0.15 m 量级，首飞需留出 0.5 m 前方空间。

---

## 6. 与训练侧的对应关系

| 训练侧 | 部署侧 |
|---|---|
| `him_arc/run_dr.py` / `QualityEnv` | `himloco_controller.cpp`（同一 60 维帧 + 6 帧历史） |
| `--asset ...mu025.usd`（0.25/min） | 实机轮胎/地面材质需一致 |
| `hip_sign=1` | 关节顺序 LEG-MAJOR，符号 +1 |
| `delay_steps 0/1`（timing 阶段） | 首飞按 0 延迟部署，若要复现 timing 需额外加队列 |

复核脚本（服务器上可直接跑）：

```bash
cd /root/as2w
./preflight/isaac-python him_arc/deploy_parity_check.py \
  --ckpt /root/gpufree-data/as2w-runs/him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt \
  --onnx /root/as2w/deploy/model_18200/policy.onnx \
  --asset /root/as2w/unitree-flat-v2-dr/assets/as2w-t2-v2-mu025.usd \
  --steps 60 --out /root/as2w/deploy/model_18200/parity.json
```

---

## 7. 训练合同逐项审计（2026-11-20 17:05）

逐项对比 him_arc（训练）与部署源码/配置，结论：

| 合同项 | 训练 | 部署 | 结论 |
|---|---|---|---|
| 观测 60 维布局 | quality_environment _frame | himloco_controller.cpp 同序 | ✅ 一致 |
| 历史 6 帧、新帧在前、首帧填满 | _get_observations | 控制器 history_valid_ 分支 | ✅ 一致 |
| 相位时钟 | phase += dt*gait_frequency，sin/cos*strength | phase_clock: true 实现同公式 | ✅ 一致 |
| 重力投影 | projected_gravity_b | 四元数同公式 | ✅ 一致 |
| 轮位置置零 | q[:,12:]=0 | 控制器 obs[21..24]=0 | ✅ 一致 |
| 动作 = tanh + 腿 0.6 增益 + 轮 ff | rc_environment._pre_physics_step | 同 | ✅ 一致 |
| 腿目标 lerp（α=0.7154） | quality_environment | 同 | ✅ 一致 |
| 轮目标 lerp（α=0.4665） | 同 | 同 | ✅ 一致 |
| **100 步零命令交接 + applied 线性斜坡** | dr_environment._pre_physics_step（w=(k+1)/100，command 强制 0） | **本次新增 handoff_steps: 100**，obs 的 applied 块用斜坡值 | ✅ 已补齐（此前缺失） |
| PD 55/2、0/2 | common/environment actuators | deploy.yaml | ✅ 一致 |
| 默认关节角 [0,0.8,-1.6]×4 | init_state | deploy.yaml | ✅ 一致 |
| HEIGHT 0.37 | 常量 | obs_height | ✅ 一致 |
| 陀螺/关节偏置与观测噪声 | 
andomize=False 时恒为 0 | 无（等价） | ✅ 一致 |
| 质量/质心/摩擦随机化 | ±0.5 kg、±10 mm、0.25×(0.8–1.2) | 名义固定 | ✅ 属分布内中心 |
| 0–1 步执行延迟 | consolidate 阶段 delay [0,1] | 部署 0 延迟 | ⚠️ 只覆盖 0 延迟这一档 |
| 相位初值 | 每回合随机 
and() | 固定 0 | ✅ 属分布内 |
| ONNX 输入名/形状 | — | obs[N,360]→out[N,16] float32 | ✅ 控制器启动即校验 |
| ckpt["iter"] | 停在 16800 | 只用文件名 | ✅ 已记录 |

**本次审计抓到并修复的纰漏**：控制器原先没有 100 步零命令交接斜坡，第一步会直接下发约 0.1 rad 的腿目标（训练时只下发 1%）。补齐后 replay 的 MAX_LEG_STEP 从 0.1206 降到 **0.0582**，C++↔Python parity 仍为 1.19e-07。

**仍存在的合同差异（非缺陷，需知情）**：
1. 1 步延迟档未在部署复现（训练见过 0/1 两档，部署只用 0）。
2. 部署固定名义质量/质心/摩擦，训练为随机化；首飞处于分布中心，但极端组合未覆盖。
3. 相位初值固定 0，训练为随机初值。

---

## 8. 最终选型与交付物（2026-11-20 17:55）

### 8.1 选定的实机部署权重：model_18200

候选对比（固定留出集，48 试次，全部零终止、零非轮接触）：

| 候选 | 俯仰最大 | 四轮中断 | 停车漂移 | 速度误差 | 六 gate |
|---|---|---|---|---|---|
| timing 15400 | 4.35° | 0.00 s | 0.1114 m | 0.1359 | ✅ |
| **consolidate 18200（选定）** | **3.89°** | 0.04 s | **0.1134 m** | 0.1504 | ✅ |
| consolidate 19600（备选） | 4.10° | 0.02 s | 0.1163 m | **0.1335** | ✅ |

**选 18200 的理由**：六条门槛里最紧的是停车漂移（占阈值 0.13 的 87% vs 19600 的 89%），18200 的余量最大；同时它是全部检查点里俯仰最平的（3.89°），符合本轮“抑制持续俯仰”的目标。19600 跟踪更好（0.1335），但停车余量更紧且需要重新走一遍 ONNX 导出与 C++ 对拍，留作备选。

### 8.2 交付文件

| 项 | 路径 / 值 |
|---|---|
| 训练权重 | /root/gpufree-data/as2w-runs/him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt（sha 850725606f4eff8021168e44890e3d828de57801fbbd4c94bff58b3c61e87a3e） |
| ONNX | models/himloco-dr-18200/policy.onnx（sha  2352011169cbf1dc3acfe3b698de27b98a32b6ad56c6063118c3d6e999678bf） |
| 配置 | models/himloco-dr-18200/deploy.yaml |
| **现场包** |  1_现场部署包/cpp_deploy-as2w.tar.gz（sha 57626996efdd57d35fea598e696a6acd3543ca2a2cf81450df03ef481b38979，9,092,050 B） |
| SDK2 包 |  1_现场部署包/unitree_sdk2-src.tar.gz（sha 85be5034013111af2448a068fd5f9d5e8c1156ee5249e27a52bb95e8eefc694e） |
| 校验清单 |  1_现场部署包/SHA256SUMS.txt |

### 8.3 已完成的离线门槛

- G1 Python parity：ONNX vs torch 真实观测 **2.38e-06**
- G2 构建与 CTest：x86 构建 rc=0、CTest **7/7**
- G3 C++ shadow：C++ vs Python 120 帧 **1.19e-07 rad / 5.96e-07 rad/s**
- 交接斜坡：handoff_steps: 100 已实现并验证（MAX_LEG_STEP 0.1206 → 0.0582）

### 8.4 仍需现场执行

G4 CPRO ARM64 编译/同步/只读 → G5 A 固定站 → G6 零命令交接 → G7 vx=0.10 有界前进 → G8 停车回站。每次动作需现场当次明确许可。
