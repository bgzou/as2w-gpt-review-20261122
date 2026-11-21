# RPT-001 Phase H/I/J 静态证据审计（P55 / P85 / OG / OS）

编号：`RPT-001`
日期：2026-11-21（原始审计），2026-11-22 汇编
覆盖对象：`部署参考/as2w-sim2real-rootcause-bundle-20261121/` 内的静态诊断数据（P55G1/G2/G3、P55S、P85DS、OG、OS 等会话）
性质：**纯离线复算**。没有运行 PhysX/MuJoCo/Isaac，没有实机动作，没有改任何正式资产。
一手文件：`GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/docs/AS2W_Phase_HIJ_Final_Evidence_Audit.md`、`results/{i4_balance_reaudit,h2_signal_reaudit,wa_candidate_reaudit}.json`、`scripts/reaudit_*.py`

---

## 1. 这份报告解决什么

此前多份材料把几个**模型反算量**写成了“实测”，并据此推测根因。本阶段逐项复算后：

| 原表述 | 复算后 | 等级 |
|---|---|---|
| 四轮 Fn 48.2 / 48.4 / 79.5 / 79.5 N“实测” | **G2 接触几何+力矩账本反演值**，无独立称重 | `MODEL_CALCULATION` |
| “静态 tau 通道真实误差 0.12 Nm” | `tau_est − tau_PD` 七会话静态一致性残差 0.119–0.134 Nm RMS；绝对真力矩未知 | `RAW_DATA_CONFIRMED` + `UNKNOWN` |
| “q 通道误差 <0.01 rad” | 悬空固定基座仿真平衡角与实机记录角 RMS 差 0.0096 rad；不是编码器精度 | `SIM_RUN_CONFIRMED`；编码器精度 `UNKNOWN` |
| “低相关 → tau_est 带宽不足” | 稳态信号接近 0.05 Nm 量化级差；未辨识动态带宽 | 低相关 `RAW_DATA_CONFIRMED`；带宽解释 `UNKNOWN` |
| “PhysX 5.872 Nm 残差来自平均接触位置” | **撤回**：原脚本把 root 刚体质心当整机质心；修正后聚合残差 **0.124 Nm** | 错误 `SOURCE_CONFIRMED`；复算 `MODEL_CALCULATION` |
| “多局部稳定平衡分支已证实” | 只证实初态/dt 敏感；多稳态需长时+终态扰动 | 敏感 `SIM_RUN_CONFIRMED`；多稳态见 RPT-002 |
| “点接触 vs 有限接触斑是根因” | 接触约束/求解器确实改变仿真静态解；实机接触斑与柔顺性 `UNKNOWN` | 前者 `SIM_RUN_CONFIRMED`；后者 `UNKNOWN` |
| “W_A 是最可信基准” | W_A 只是**当前配置下的候选相对对照** | 候选 `INFERENCE` |
| “J3 不重训” | 仅针对 P55 静态接触差异；不覆盖 R2000A / unitreestyle 动态问题 | 范围限定 |

## 2. I4：5.872 Nm → 0.124 Nm 的复算（本阶段唯一“撤回结论”）

**问题**：原脚本用 `robot.data.root_com_pose_w[0,:3]` 作为叉乘原点。Isaac Lab v3.0 文档说明该属性是**根刚体质心**，不是整机质心。

**做法**：用归档圆柱轮 URDF、已记录的髋/大腿/小腿稳态平均角，按完整关节树 FK 对连杆质心做质量加权；固定在 base 的 JT128 计入根刚体。交叉校验：

| 校验项 | 结果 |
|---|---|
| URDF 根刚体质量 | 10.58 kg |
| 由根姿态重建的根刚体质心 vs PhysX 归档 `root_com` | 均值差 0.047 mm、最大 0.084 mm |
| URDF 整机质量 | 26.0 kg（与 PhysX 运行记录相同） |

**结果**：

| 量 | 值 |
|---|---|
| 根刚体质心（base 系） | (0.045707, −0.000700, 0.020441) m |
| 整机质心（base 系） | (0.017358, −0.000317, −0.053520) m |
| 整机相对根刚体质心的世界 x 差 | −0.023076 m |
| 原报告“先平均后叉乘” | 5.872134 Nm |
| 仍用根刚体质心，改逐帧叉乘 | 5.872096 Nm |
| **用整机质心** | **0.124494 Nm** |
| 修正后力矩向量 (x,y,z) | (0.016145, −0.015744, −0.122434) Nm |

23.1 mm × 255.06 N（模型重力）≈ 5.89 Nm，正对应被消除的力矩项。**“先平均后叉乘”在这个稳窗只值 0.00004 Nm，不是主因。**

**剩余限制（必须随数字一起引用）**：归档只有稳态平均关节角，没有逐帧关节角；`contact_pos_w` 是聚合位置，没有逐点力—位置配对；0.124 Nm 是该近似账本的残差，不是“精确真实外力矩”。`weight_model=255.06 N` 也是模型 26 kg 算出来的，不是独立称重。

## 3. H2/H3/H1：为什么“相关性低”不能推出“带宽不足”

- `tau_PD = kp(q_des−q)+kd(dq_des−dq)+tau_ff` 是**算出来的**理想 PD 力矩；`tau_est` 是 HG LowState 的**固件估计**；`residual = tau_est − tau_PD`。
- 七窗口复算的 12 腿轴 RMS：OG 0.1220、OS 0.1195、P55G1 0.1264、P55G2 0.1337、P55G3 0.1263、P55S 0.1193、P85DS 0.1199 Nm —— 与 H2 表一致，但不能据此排除承载传动损耗或固件削顶。
- P55G2 四髋 `tau_PD` 稳窗标准差仅 **0.016–0.022 Nm**，`tau_est` 每髋只有 **4–5 个量化值**（最小级差 0.05 Nm）；±20 ms 互相关峰在 **0–2 ms**，无不一致延迟。P55S 悬空信号标准差 0.092–0.276 Nm，相关系数自然高（0.974–0.997）。
- 结论：地面稳窗相关低是事实，**原因待辨识**；不能用这些静态窗估真实电机带宽。
- H3 的 0.0096 rad 是“悬空固定基座仿真平衡角 vs 实机记录角”的 RMS 差；H1 的历史发布端源码 SHA 与当前维护源码未对齐，命令记录才是该会话合同的直接证据。

## 4. W_A / W_D：候选对照，不是验收基准

P55G2 窗口 8–19 s，实机 q/`tau_est`/pitch 从归档 NPZ 直接重算；四轮 Fn 取 G2 analytic 反演值。

| 指标 | 实机反馈/反演 | W_A | W_A−参考 | W_D | W_D−参考 |
|---|---|---|---|---|---|
| pitch | −6.1349°（IMU） | −6.37° | −0.2351° | −0.51° | +5.6249° |
| hip q FL/FR/RL/RR (rad) | +0.2142/−0.2155/+0.2779/−0.2755 | +0.084/−0.084/+0.210/−0.211 | −0.1302/+0.1315/−0.0679/+0.0645 | +0.208/−0.208/+0.184/−0.185 | −0.0062/+0.0075/−0.0939/+0.0905 |
| hip τ (Nm) | −11.805/+11.695/−15.153/+15.188（`tau_est`） | −4.61/+4.64/−11.57/+11.60 | +7.195/−7.055/+3.583/−3.588 | −11.47/+11.47/−10.10/+10.15 | +0.335/−0.225/+5.053/−5.038 |
| Fn (N) | 48.22/48.38/79.47/79.48（**反演**） | 49.4/49.6/77.9/78.1 | +1.18/+1.22/−1.57/−1.38 | 58.1/58.3/69.3/69.4 | +9.88/+9.92/−10.17/−10.08 |

W_A 的姿态匹配有效（pitch 差 0.24°），但前髋角差 ~0.13 rad、力矩差 ~7.1 Nm；后髋角差 0.065–0.068 rad、力矩差 ~3.6 Nm。W_A 的 Fn 接近反演值属**非独立一致性**，不能证明四轮真实负载匹配。W_D 前髋接近，但 pitch/后髋/反演 Fn 失配。

## 5. 仍未知（不要在这些点上写结论）

1. 四轮真实施加法向力、接触斑尺寸、真实摩擦/柔顺性（无独立秤测）。
2. `tau_est` 的固件计算/滤波/绝对标定/高载行为。
3. PhysX `contact_pos_w` 只暴露平均位置，0.124 Nm 残余无法精确拆分。
4. 多个**局部稳定**平衡是否成立（由 RPT-002 接手）。
5. 历史发布端源码 SHA 与当前维护源码的对齐性。

## 6. 复现

```powershell
python 'GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/scripts/reaudit_i4_balance.py'
python 'GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/scripts/reaudit_h2_signal.py'
python 'GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/scripts/reaudit_wa.py'
```

环境：Windows Python 3.13.6、NumPy 2.2.6；不依赖 Isaac Sim。三个脚本只读原包，输出只写 `results/`，结果 JSON 内记录输入 SHA-256。

## 7. 证据索引

| 文件 | 说明 |
|---|---|
| `04_诊断实验/phase_hij_evidence_audit_20261121/docs/AS2W_Phase_HIJ_Final_Evidence_Audit.md` | 本报告的原始结论文件 |
| `.../scripts/reaudit_i4_balance.py` → `results/i4_balance_reaudit.json` | I4 复算（两源 SHA 写入 JSON） |
| `.../scripts/reaudit_h2_signal.py` → `results/h2_signal_reaudit.json` | 七会话 NPZ 逐源 SHA |
| `.../scripts/reaudit_wa.py` → `results/wa_candidate_reaudit.json` | P55G2 NPZ、G2 反演表、I5 仿真表 |
