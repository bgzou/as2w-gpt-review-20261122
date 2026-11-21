# RPT-007 D 线进展：HIMLoco zero-B 输入包、合同漂移、同合同仿真

编号：`RPT-007`
日期：2026-11-22（Asia/Shanghai）
覆盖：unitreestyle / HIMLoco `model_18200`（D 线：T3-2 + T3-3 + 一项合同发现）
一手文件：`04_诊断实验/实机日志反演/results/himloco_zeroB_input_20261122_01.json`、`04_诊断实验/开环回放/results/himloco_wrapper_drift_20261122_01.json`、`04_诊断实验/闭环反事实/t3_3_zerob_sim_20261122/`（params/code/logs/results/environment）
边界：全部离线/仿真；无 RL、无正式资产修改、无实机、无 CPRO。MuJoCo 运行在**用户自有服务器**，单作业。

---

## 1. 一句话结论

**同合同（tanh + leg_gain 0.6 + 轮前馈 + 100 步交接）仿真在零命令下也出现后腿"指令−实际"偏差 0.13–0.38 rad，量级与实机记录的 0.24–0.34 rad 相当** → 按交接 §7.6 的判据，后腿下沉**不应先归因于 Sim2Real 植物/接触差异**，应优先排查训练/站姿/部署合同。等级 `INFERRED`（单一资产、单一初态、MuJoCo 4.0.1）。

另外发现一处会污染任何 sim2sim 的合同漂移（`CONFIRMED`）：会话自带的 `deploy_ref_controller.py` 与 C++/`deploy.yaml` 的动作包装**不一致**。

## 2. 合同漂移（`CONFIRMED`）

同一 ONNX、同一份 360 维历史（零命令、q/dq 取自归档 LowState），两种包装跑 100 步：

| 量 | C++/deploy.yaml | `deploy_ref_controller.py` |
|---|---|---|
| 动作整形 | `tanh(raw)` → ×`leg_gain 0.6` | `clip(raw, ±1)`（无 tanh、无 0.6） |
| 腿目标范围 | ±0.15 rad（=0.6×0.25） | ±0.25 rad |
| 轮目标 | `(vx−wz·side)/R + 2·tanh(raw)`，零命令下 ≤2 rad/s | `clip(raw×12)`，零命令下可到 ±12 rad/s |
| 100 步后腿目标 | 例：`[0.083, 0.149, 0.133, …]` | `[0.210, 0.250, 0.250, …]` |
| 最大腿目标差 | **0.1616 rad** | — |
| 最大轮目标差 | **10.95 rad/s**（6.5×） | — |

⇒ 任何用 `deploy_ref_controller.py` 跑的 sim2sim（包括会话里的 `mujoco-mujoco_eval.py` 与 `videos/mujoco-validated-20261121.mp4` 这一路）**不是同合同仿真**；引用其结论必须标注这一点。本报告用的是按 C++ 语义重写的包装。

## 3. T3-2 zero-B 输入包（`usable_for_sim=true`）

来源：`field/20261120-180312_low-state_32209.log`（5 s、~1060 Hz、原厂 `ai` 控制）、`analysis/field-sink-per-axis.csv`、`field/run-log-summary.md`、`data/deploy.yaml`。

- 已具备：初值 q16/dq16（LowState 序 → 策略序的映射 `[3,4,5,0,1,2,9,10,11,6,7,8]`）、quat、gyro、command=0、腿/轮 kp-kd、α_leg=0.71539、α_wheel=0.46651、handoff=100、height=0.37、逐轴实机偏差。
- **未归档**（6 项，已逐条写进 JSON）：zero-B 期间的逐帧 LowCmd/applied 目标（发布端日志留在 CPRO `/tmp` 后丢失）、机身高度、轮法向力、地面摩擦/轮胎柔顺、IMU 与控制环的时间对齐、逐次试验的逐行判决。
- 注意：归档的 LowState 是**原厂 `ai` 站姿**（thigh 1.05、calf −2.77），不是 B 键按下瞬间的位姿（现场门 `pose_error=0.202156 rad` 说明当时接近默认站姿）。因此仿真按**默认站姿**起跑，另把"原厂站姿"留作敏感性对照（未跑）。

## 4. T3-3 同合同 zero-B 仿真（服务器，`t3_3_zerob_sim_20261122`）

| 项 | 值 |
|---|---|
| 环境 | 自有服务器，MuJoCo **4.0.1**、onnxruntime 1.32.0、numpy 2.5.5、Python 3.12.3（隔离 venv，不污染 isaaclab 环境） |
| 资产 | 会话 `mujoco-scene_v2.xml` 副本（仅把 meshdir 改为本地 `meshes/`）+ 18 个 STL；**base 10.58 kg、整机 26.0 kg、地面摩擦 0.62** |
| 策略 | `policy-model-18200.onnx`（SHA `02352011…678bf`） |
| 协议 | 0.5 s 默认站姿 PD 预稳定 → 2.0 s 交接（command=0，ramp (k+1)/100）→ 保持到 9.7 s；`z0=0.33 m`，quat 取实机快照 |
| 结果 | `results/t3_3_zerob_20261122_01.json`（SHA `16ed6bea…`），日志与本地 SHA 一致 |

### 4.1 后腿"指令−实际"（末 3 s 均值，rad）

| 轴 | 仿真 | 实机（field CSV） | 差 |
|---|---|---|---|
| RL_thigh | **0.133** | 0.237 | −0.104 |
| RR_thigh | **0.378** | 0.239 | +0.139 |
| RL_calf | **0.196** | 0.336 | −0.140 |
| RR_calf | **0.381** | 0.331 | +0.050 |

⇒ **量级同阶、方向一致（实际角低于指令角 = 腿被压下去），但左右不对称**。

### 4.2 姿态与接触（同一 JSON）

- pitch：起 −10.41°、终 −9.17°（实机约 −6.13°，方向一致、幅度更大）。
- base z：最小 0.33998 m，终 0.3501 m。
- 尾段最小法向力：FL **0.0 N**（尾段某步离地）、FR 53.9、RL 63.9、RR 99.8 N。
- **FL 离地**说明仿真平衡点与实机不同（载荷集中到 RR），这也是左右不对称的直接来源；属仿真侧的接触/资产效应，不能当实机结论。

### 4.3 判读（按交接 §7.6）

- `INFERRED`：**同合同仿真也下沉** → 先排训练/站姿/部署合同，**不先归 Sim2Real**。
- 不成立：把实机 0.33 rad 解释成“纯植物差异”；也不成立：用本仿真反推实机载荷分配。
- 限制：① 资产 26.0 kg vs 训练 URDF 27.512 kg（差 1.512 kg，见 RPT-003 §3.5）；② MuJoCo 4.0.1；③ 初态=默认站姿而非 B 键瞬间位姿；④ 单次运行、无种子重复；⑤ 无实机轮法向力可比。

## 5. 下一步（D 线，未执行）

1. **资产单变量**：把 MJCF 的 base 质量/COM 改回训练 URDF 数值（27.512 kg、base 11.592、COM x 0.020002），重跑同协议，看后腿偏差是否收敛 —— 这一步才能把"资产 vs 站姿/合同"分开。
2. **站姿单变量**：若资产回填后仍下沉，改初始站姿（默认 vs 原厂 ai vs 实机 B 键位姿）重复；三者差异可量化。
3. **对照 R2000A 的同类现象**：R2000A 走同一条轮式路线，其 `kp55` 静态刚度不足已在 T15/R2000A 现场出现（RPT-005），可在 C 线 T2-1 后一起做"kp55 vs kp85"的合同对照（不得改正式资产，只在 candidate 副本）。
4. 若要继续，需用户确认是否允许在自有服务器上扩展 MuJoCo 版本/资产对照（默认允许，仍是 CPU、单作业）。

## 6. 证据索引

| 文件 | SHA256 前 16 位 |
|---|---|
| `04_诊断实验/开环回放/results/himloco_wrapper_drift_20261122_01.json` | 见文件内 `generator_sha256`/inputs |
| `04_诊断实验/实机日志反演/results/himloco_zeroB_input_20261122_01.json` | 同上 |
| `04_诊断实验/闭环反事实/t3_3_zerob_sim_20261122/results/t3_3_zerob_20261122_01.json` | `16ed6bea6436a566` |
| `.../t3_3_zerob_sim_20261122/code/zerob_sim.py` | `513b5212f41246cf` |
| `.../t3_3_zerob_sim_20261122/code/scene_v2.xml` | `1bcf70ab4322a5bd` |
| `.../t3_3_zerob_sim_20261122/code/policy-model-18200.onnx` | `02352011169cbf1d` |
| `.../t3_3_zerob_sim_20261122/environment/environment.txt` | 版本与协议记录 |
| `部署参考/as2w-unitreestyle/sessions/2026-11-20-himloco-dr-10000/data/deploy_ref_controller.py` | 漂移对照的 Python 参考实现 |

复现：

```bash
# 本地（只读策略/日志，不做仿真）
python 04_诊断实验/开环回放/check_himloco_contract_drift_20261122_01.py \
  --onnx <session>/data/policy-model-18200.onnx \
  --field-log <session>/field/20261120-180312_low-state_32209.log \
  --output 04_诊断实验/开环回放/results/himloco_wrapper_drift_20261122_01.json
python 04_诊断实验/实机日志反演/build_zerob_input_package_20261122_01.py \
  --session <session> --out 04_诊断实验/实机日志反演/results

# 服务器（MuJoCo，CPU，单作业）
cd /root/gpufree-data/as2w-cpro/systematic_project_audit_20261121/experiments/t3_3_zerob_sim_20261122/code
../venv/bin/python zerob_sim.py --scene scene_v2.xml --policy policy-model-18200.onnx \
  --input himloco_zeroB_input_20261122_01.json --out ../results/t3_3_zerob_20261122_01.json
```
