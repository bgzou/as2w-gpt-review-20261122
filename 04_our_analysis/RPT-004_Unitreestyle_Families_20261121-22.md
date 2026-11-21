# RPT-004 unitreestyle：HIMLoco(model_18200) / v2-step / unified_dr

编号：`RPT-004`
日期：2026-11-21（候选稿）→ 2026-11-22（身份链独立复核 + 汇编）
一手文件：`部署参考/as2w-unitreestyle/sessions/2026-11-20-himloco-dr-10000/`（清单、合同、字段分析、原始日志）、`部署参考/as2w-unitreestyle/{v2-step,unified_dr}/`、`GPUFREE服务器/04_诊断实验/systematic_project_audit_20261121/results/unitreestyle_himloco_identity_20261122_0{1,2}.json`
状态：**进行中**——三族身份已分离、HIMLoco 文件清单已核；**zero-B 输入缺口表与同合同仿真未做**

---

## 1. 三族必须严格分开（`CONFIRMED`，来自各包 manifest/contract）

| 族 | 来源 | 权重 / ONNX | obs / act | 频率 | 状态 |
|---|---|---|---|---|---|
| **HIMLoco** | 训练 run `him-dr10000-consolidate5000-n4Y47n/train/model_18200.pt` | `policy.onnx`（360→16，融合 estimator+actor）SHA `02352011…678bf`；另有 `estimator.onnx` 360→19、`actor.onnx` 79→16 | **360（60 维 × 6 帧）/ 16** | 50 Hz（20 ms） | **唯一实机族** |
| **v2-step** | snapshot `v2-stephard-eval-600` | checkpoint `45b36ee7…`、asset `ebb08719…`、resume `aa41bbe8…` | 附件未给（`UNKNOWN`） | `UNKNOWN` | `gate_qualified=false` |
| **unified_dr** | `unitree-unified-reward-20261119-010208/train/model_1000.pt` 链 | `policy.onnx` `c909f680…`、`policy.pt` `8e9f2e0f…` | **58 / 12** | 50 Hz（policy_dt 0.02，physics 0.005） | `research_only`、`isaac_gate_pass=false`、`second_sim/cpro_arm64/field = not_run` |

**红线**：v2-step / unified_dr 的方向、偏航、MuJoCo 数值**不得**用来解释 HIMLoco 实机；HIMLoco 的 360/16 合同也不得套到另外两族。

## 2. HIMLoco 合同（`CONFIRMED`，来自 `docs/deployment-contract-18200.md` + 本次重算）

| 项 | 值 |
|---|---|
| obs 布局（60 维/帧，新帧在前，`history.reshape(-1)`） | 0:3 cmd、3:6 gyro、6:9 gravity、9:25 q−default（轮位置强制 0）、25:41 dq×0.05、41:53 applied_leg/0.25、53:57 applied_wheel/12、57 常数 0.37、58/59 相位 sin/cos |
| act 包装 | `tanh → 0.6 腿增益 → 0.25`；轮 `(vx − wz·side)/R + 2·tanh`，clamp ±12；`side = [+0.1654, −0.1654, +0.1654, −0.1654]`，`R = 0.086 m` |
| PD | 腿 55/2；轮 kp 0 / kd 2 |
| 低通 | α_leg = 1−exp(−2π·10·0.02) = **0.71539**（声明 0.7154 ✓）；α_wheel = 1−exp(−2π·5·0.02) = **0.46651**（声明 0.4665 ✓） |
| 默认角 | `[0, 0.8, −1.6] × 4`；HEIGHT=0.37 |
| 交接 | `handoff_steps=100`，`w=(k+1)/100`，command 强制 0；obs 的 applied 用斜坡值 → 补齐后 `MAX_LEG_STEP` 0.1206→0.0582 rad |
| 导出 parity | `export.json` 声明 `parity_max_abs_err = 7.629e-06`；`parity.json`：60 步 `max_abs_action_diff = 2.384e-06`、`mean = 1.192e-06`、`parity_ok=true` |
| 训练随机化 | 质量 ±0.5 kg、质心 ±10 mm、摩擦 0.25×(0.8–1.2)、delay [0,1]、相位初值随机；部署取名义中心、相位初值固定 0 |

## 3. 本次身份链复核（T3-1，`CONFIRMED`）

脚本：`04_诊断实验/systematic_project_audit_20261121/deepseek/candidates/verify_himloco_session_manifest_20261122_02.py`
结果：`results/unitreestyle_himloco_identity_20261122_02.json`

| 项 | 结果 |
|---|---|
| 会话清单声明文件数 | **41** |
| 精确匹配 | **39** |
| 仅行尾差、可逐字节重建 | **2**（`analysis/field-sink-per-axis.csv` 391 vs 390 B、`make_plots.py` 2310 vs 2309 B；在末行 LF 前插入单个 CR 后 SHA 与声明完全一致） |
| 无法解释的不一致 | **0** |
| 未列入清单的现存文件 | **0** |
| `policy-model-18200.onnx` | 实算 `02352011169cbf1dc3acfe3b698de27b98a32b6ad56c6063118c3d6e999678bf`，与 `manifest.md` 及 `docs/deployment-contract-18200.md` 声明**一致** |
| 训练 checkpoint `model_18200.pt` | **本地不存在**；`docs/field-bringup-checklist.md` 声明 `850725606f4eff8021168e44890e3d828de57801fbbd4c94bff58b3c61e87a3e` → `DECLARED_ONLY_NOT_VERIFIABLE_LOCALLY` |

⇒ **ONNX↔合同↔会话清单** 三段闭合（`CONFIRMED`）；**ONNX↔PT** 只能到“声明一致”，PT 本体未落地（`UNKNOWN`）。

严格版 `_01.json`（把 2 处 1 字节差判为冲突）保留为失败记录，不删除。

## 4. zero-B 与后腿下沉（全部 `OBSERVED`，现场记录；原始日志部分已丢）

### 4.1 逐轴稳态偏差（`analysis/field-sink-per-axis.csv`，36 采样均值 / 最大）

| 轴 | 无补偿均值 (rad) | 无补偿最大 | 加静态补偿后末值 |
|---|---|---|---|
| RL_calf | **+0.336** | +0.350 | +0.105 |
| RR_calf | **+0.331** | +0.376 | +0.199 |
| RL_thigh | +0.237 | +0.278 | +0.020 |
| RR_thigh | +0.239 | +0.279 | +0.117 |
| FL_hip / FR_hip | +0.209 / −0.193 | +0.300 / −0.253 | +0.176 / −0.128 |
| FL_thigh | −0.159 | −0.037 | −0.112 |
| FL_calf | +0.006 | +0.078 | **+0.142（变差）** |
| FR_thigh | −0.004 | +0.023 | **−0.305（严重过冲）** |
| FR_calf | +0.053 | +0.106 | −0.053 |
| RL_hip / RR_hip | +0.144 / −0.145 | +0.156 / −0.110 | +0.021 / −0.076 |

（该 CSV 是“指令−实际”的差；正值 = 指令角高于实际角 = 腿被重力压下去。）

- 后腿 thigh+calf 合计约 **0.57 rad**，对应机身下沉约 **3–4 cm**；7 s 内稳定不增长。
- **静态重力补偿试验把后腿顶起来了，但 FR_thigh 过冲到 −0.305、FL_calf 变差，重心前移、前腿反向对抗、力矩嗡鸣；操作员 2.2 s 后按 X 终止，`leg_offset_rad` 全退回 0。** ⇒ 不能继续简单加 offset。

### 4.2 zero-B 会话（4 轮，均经现场当次同意）

| 轮次 | 关键行 | 结果 |
|---|---|---|
| zero-B 成功轮（18:14） | `B accepted: pose_error=0.202156 wheel_dq=0.0122718 upright=0.996734 quiet_dq=0.0429515`；`zero-speed handoff complete`；`brake trigger=bounded zero hold cap`；`Brake→ReturnStand→FixedStand→EmergencyDamping(X)`；`exit=0` | 通过 |
| 全包络 fault 轮（18:26） | `STARTUP FAULT: stale/invalid LowState … age_ms=21.4536 max_age_ms=20 … state=PolicyWalk`；`exit=1` | 启动门拦截 |
| 全包络推杆轮（18:33） | 36 条 `WALK_CMD`，`vx` −0.30…+0.036、`wz` 0…0.108；`FIELD_SAFETY_WARN/HARD` **0** 条 | 有界运动完成 |
| 静态补偿轮（18:38） | `B accepted: pose_error=0.161174 … upright=0.999606`；2.2 s 后 X | 补偿方案否决 |

安全事件：`FIELD_SAFETY_WARN` 0、`FIELD_SAFETY_HARD` 0、电机 `warning_or/motorstate_or` 非零 0、断电/重启 1 次。

**原始日志缺口**：zero-B 四轮的发布端原始日志在 CPRO `/tmp/`，CPRO 次日离线后不可取，只留下上表关键行与 `field/run-log-summary.md`。因此这些结论最高只能到 `OBSERVED`。

### 4.3 训练域能力（`OBSERVED`，交付方矩阵，非本次复跑）

`analysis/capability-matrix.md`：72 试次（4 姿态 × 3 载重 × 6 命令，每试 350 步、前 100 步零命令交接窗，12 并行环境，`friction 0.25`）→ **终止 0/72**、非轮触地 0 帧、最大 \|pitch\| **6.70°**、三轮以上支撑比例 1.00。名义工况达成率：forward vx 0.498（83%）；arc_left wz 0.256（85%）；arc_right wz −0.294（98%）。

两个**已知工具缺陷（必须一起引用）**：① `stop_same` 组在 `matrix_eval.py` 里 250 步全程 `cmd=0.6`，**不能**用来判停车；② npz 无角速度通道，`vel[:,2]` 是机体线速度 z，不是偏航率——用它会误判“弧线不转”。

## 5. 右向 / 负偏航：这是**实验设计缺口**，不是“策略行为未知”

| 层次 | 事实 |
|---|---|
| HIMLoco 实机 | 只下发 `vx −0.30…+0.036`、`wz 0…0.108`；**从未下发右向 vy 或负 wz** → 对这些方向**既非通过也非失败**（`UNKNOWN`） |
| v2-step gate | `right: survival or tracking` 失败、`right: fewer than two events on a wheel` 失败；`modes.right` four_wheel_support_min 0.124、tracking_ratio_min 0.233 |
| unified_dr | `research_only`，未做任何方向评估 |

⇒ 结论只能写“**HIMLoco 右向/负偏航缺实机证据**”，不能写“HIMLoco 转向不足”（那是 v2-step 的证据）。

## 6. 阈值冲突（部署合同层面，未调和）

- 发布端启动门：`max_age_ms = 20`（单位未写，`UNKNOWN`）。
- 部署合同：LowState 超时 **>50 ms** 切 Passive。
- 两者并存且未对齐；引用时必须区分“发布端 fault 记录”与“合同 Passive 超时”，不得混成同一阈值。

## 7. 仍然 `UNKNOWN` 的（写作红线）

1. 0.33 rad 是否纯 PD 静差 —— 未做独立力矩核算（`tau_est` 是固件估计）。
2. 训练域是否覆盖 0.33 rad 级静差 —— 训练含 ±0.5 kg 随机化，但“未见静差建模”只是“未见”。
3. 实机轮胎/地面真实摩擦、接触斑；`as2w-t2-v2-mu025.usd` 与 v2-step asset `ebb08719…` 的对应关系。
4. PT→ONNX 的真实同源性（PT 未落地）。
5. HIMLoco 实机 pitch 数值（传感器原始数据未归档）。

## 8. 下一步（未执行，按交接 P3）

1. **zero-B 时间窗输入包**：从 `field/` 现存日志 + `analysis/field-sink-per-axis.csv` + 发布端关键行，构造初 q/dq/quat、command、handoff、kp/kd、滤波、action history；**显式列出缺失字段**（预计缺：逐帧 LowState、逐帧 LowCmd、apply 目标序列）。
2. **同合同仿真**：同 ONNX、同合同（360/16、50 Hz、α、pre-feedforward）、同初态、2 s handoff 跑 zero-B；输出 pitch、后腿 thigh/calf q、`tau_PD`（仿真侧为 applied torque 则显式说明）、四轮 Fn、接触状态。
3. 判据：**同合同 sim 不下沉、real 下沉** → 才进入植物/接触一变量对照；**sim 也下沉** → 先排训练/站姿/合同，不归 Sim2Real。
4. v2-step / unified_dr 只做各自身份与 gate 审计，禁止借其解释 HIMLoco 实机。

## 9. 复现与 SHA

```powershell
python 'GPUFREE服务器\04_诊断实验/systematic_project_audit_20261121\deepseek\candidates\verify_himloco_session_manifest_20261122_02.py' `
  --session-dir '部署参考\as2w-unitreestyle\sessions\2026-11-20-himloco-dr-10000' `
  --output 'GPUFREE服务器\04_诊断实验/systematic_project_audit_20261121\results\unitreestyle_himloco_identity_20261122_02.json'
```

| 文件 | SHA256 前 16 位 |
|---|---|
| `results/unitreestyle_himloco_identity_20261122_02.json` | `4204fc27b1122a83` |
| `sessions/.../data/policy-model-18200.onnx` | `02352011169cbf1d` |
| `sessions/.../manifest.md` | `35abf4eca22b40d0` |
| `sessions/.../analysis/field-sink-per-axis.csv` | `ad1e725143ad9ad7`（声明值；本地为行尾差版本，重建后一致） |
