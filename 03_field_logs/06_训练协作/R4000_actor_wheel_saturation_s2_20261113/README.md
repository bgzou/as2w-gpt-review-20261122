# R4000 轮速残差饱和：服务器进阶版（actor-wheel-saturation-loss）与我方独立复验

**打包日期**：2026-11-14 ｜ **对象**：`Rob2Nav` 快照 commit `2de96cba…` 内 `runs/rolling/actor-wheel-saturation-loss.Uliv0s`
**一句话结论**：训练方针对 R4000 实机"四路轮速残差 100% 撞限幅"加了轮速越界惩罚重训一版，**离线同输入对照确认饱和率 100% → 0%（这是实质修复）**，但 MuJoCo 低速驻停/转向的完整通过率**没有**同步提升；该检查点 `model_1200` 是 4000 轮训练的中间产物、终版未归档，**不建议上实机**。

---

## 1. 背景：问题是什么

R4000（`rolling-4000-stage020-research`，filtered-4000 终版 `model_3999`，2026-11-13 已实机）在实机与离线上都有一个已知缺陷：

- **实机**：`20261113-114335` 会话 4157 个运动帧回放，ONNX 轮输出 **100% 饱和于 ±6**（±6 × 动作缩放 16 = ±96 rad/s，即 AS2-W 轮速上限），且 steering 分量恒为 0——差动全靠包装里的开环 `0.66·wz` 注入。
- **训练方自证**：同一现象在 487 帧实机低速日志回放中复现（report/02 第 19、27、37、39 行），四个检查点的四路轮速残差"全程触及限幅"。

训练方为此新建一条训练线：`runs/rolling/actor-wheel-saturation-loss.Uliv0s`，在原 R1199 初始权重、4096 环境、seed 311、原控制合同下，**加入权重 0.025 的轮速越界损失（|a| > 0.9 生效）**，预算仍是 4000 轮（`data/run-contract/run-contract.json`）。

## 2. 结论（按证据强度排序）

### 2.1 去饱和确实有效（我方独立复算，最强证据）

同一批 **4536 帧**闭环观测、同一评估器，分别喂两个 actor：

| 权重 | 轮输出饱和率 | 峰值 |
|---|---|---|
| R4000 终版 `model_3999` | **100% 帧饱和于 ±6**（四轮全饱和） | 16×6 = 96 rad/s 上限 |
| 进阶版 `model_1200` | **0%** | `max|a| = 1.23` ≈ 19.6 rad/s |

训练方同期自证（report/02「轮速越界惩罚第 250 轮对照」）：487 帧实机日志上，四路轮速残差饱和率 **100% → 0%**，限幅后腿部残差变化 P99 两组均为 0.00504 rad。

### 2.2 但性能门槛没有跟着上来

- **训练方 24 组同种子对照**：稳定/跟踪仍 24/24，10 s 驻停仍 9/24，末段 fixed stand 仍 18/24；刹停 22/24 → 24/24，但**完整通过 4/24 → 3/24**，原地左右转 1/4 → 0/4。原文结论："离线动作饱和改善已证实，**低速驻停或转向完整通过的提升尚无证据**"。
- **我方 S2（72 例，0.20 m/s 档，含推扰/丢包/质量随机）**：
  - `model_1200`：gate **12/72**、`safety_gate=false`（1 例 back 触发 knee_surface）、非轮触地 0/72；跟踪 forward 0.192–0.195/0.20（优秀）、turn 实际 0.080–0.094 / 命令 0.12（约 75%）、失败集中在**驻停漂移 2.0–3.7 cm**（门限 2 cm）。
  - R4000 终版 `model_3999` 同机同协议复跑：gate **18/72**、`safety_gate=true`、held 38（vs 31）、膝面 0（vs 1）。
  - 结论：**进阶版综合仍不如已实机的 R4000 终版**。

### 2.3 判定

1. `model_1200` 是 4000 轮训练的第 1200 轮中间检查点，**该 run 的终版 `model_3999` 未归档**，无法验收。
2. **不建议**用 `model_1200` 上实机；rolling 线继续沿用已实机的 R4000。
3. 建议向训练方索要 actor-wheel-saturation run 的**终版 `model_3999` + 服务器验收证据**，按同条件做对照；若仍保持去饱和、且驻停/膝面达标，即为 R4000 的**继任候选**——接口与 R4000 完全一致（obs 65 / act 16 / 20 ms），现有 S3 部署链可直接复用。

## 3. 证据链一览

| 阶段 | 做了什么 | 结果 | 位置 |
|---|---|---|---|
| S0 收包 | 三份权重/ONNX SHA 复核 | 与合同一致；v3 manifest 自述 `scope=research diagnostic`，C7 双仿真 `not_run` | report/01 §S0 |
| S1 parity | ORT vs 录制 actor-preclip | 1512 帧 `5.96e-07`（与训练方一致） | report/01 §S1 |
  s2_outputs/rolling_1200/                   ← 进阶版 72 例 MuJoCo 原始输出：acceptance/diagnostic JSON + 观测 npy
| S2 对照 | R4000 终版同协议复跑 | gate 18/72、held 38、膝面 0 | `data/s2_outputs/rolling_3999_rerun/` |
| 饱和对照 | 4536 帧同批观测过两个 actor | 100% → 0% | report/01 §追加 |
| 训练方 A/B | 250 轮轮速越界惩罚对照 | 饱和 100%→0%，门槛无提升 | report/02 |
| 训练方快照 | 该快照能提供什么/不能提供什么 | 只有到 model_1200 的中间检查点 | report/03 |

## 4. 包内结构（建议读序）

```
README.md                                   ← 本文件（先读这 1 页）
report/
  01_我方S2验收_三条最新权重.md              ← 我方完整 S2 报告（含 4536 帧饱和对照、判据门槛）
  02_训练方检查点对照_轮速越界惩罚.md        ← 训练方 R4000 检查点对照 + 250 轮饱和 A/B（原始中文文档）
  03_训练方服务器快照说明.md                 ← 训练方对该快照范围的自我声明（英文原文）
  04_INDEX摘录_20261113.md                   ← 我方 INDEX 中 R4000/三条最新权重相关小节
data/
  s2_outputs/rolling_1200/                   ← 进阶版 72 例 MuJoCo 原始输出：acceptance/diagnostic JSON + 观测 npy
  s2_outputs/rolling_3999_rerun/             ← R4000 终版同协议复跑（对照）
  s2_outputs/{stepping_1199,rec_*}/          ← 同批另两条线（stepping/recovery），本包保留以免断链
  run-contract/                              ← 该训练 run 的契约与启动脚本（4000 迭代/4096 环境/seed 311/loss 0.025·0.9）
  model/model_1200.pt                        ← 本次讨论的检查点（1.0 MB，研究诊断用）
  model/policy.onnx                          ← 对应 ONNX（175 KB，opset17）
  model/{actor-preclip.npy,policy-observations.npy,SHA256SUMS,manifest.json,onnx-parity.json}
tools/
  run_s2.sh                                  ← 7 个并行 S2 运行的入口
  parity_check.py / parity_part2.py          ← S1 parity 脚本
MANIFEST.sha256                              ← 本包除自身外全部文件的 SHA256
```

## 5. 复现

环境：caolin 服务器 `MuJoCo 3.17.0 / ORT 1.31.1 / torch 2.15.0+cpu / numpy 1.26.4`
（训练方为 MuJoCo 3.2.3 / ORT 1.23.2 / torch 2.11.0+cu130，跨版本差异已在下文标注）

```bash
# S1：ONNX ↔ 检查点 parity（期望 rolling 1512 帧 max err 5.96e-07）
python tools/parity_check.py

# S2：7 个并行运行（期望 rolling_1200 gate 12/72、rolling_3999_rerun gate 18/72）
bash tools/run_s2.sh
```

## 6. 边界与未验证项（务必随包转述）

- 训练方 v3 manifest 自评：`scope=research diagnostic`，**C7 双仿真正式验收 `not_run`、ARM64 shadow `not_run`、实机闭环 `not_run`**。
- 训练方 MuJoCo 3.2.3 归档 R4000 为 27/72，我方 MuJoCo 3.17 复跑 18/72；结论方向一致，**绝对值不可跨版本直接比较**。
- 4536 帧饱和对照是一次性脚本产出，未单独归档中间文件；本包保留结论与其来源报告（report/01 §追加）。
- `model_1200.pt` / `policy.onnx` 为**研究诊断权重，未获实机授权**；任何实机动作仍须现场人员当次明确同意并走既有阶梯（zero-B → 有界低速 → 包络内）。
- 本文所有数字均来自包内文档与原始输出，未做二次推断。

## 7. 关键件 SHA256

| 文件 | SHA256 | 大小 |
|---|---|---|
| `data/model/model_1200.pt` | `a5e79209320161faff430ac1fbb67c80e3de938b9b70b4be222ac0a9bdcde0c9` | 1,042,741 B |
| `data/model/policy.onnx` | `d047a83e20e864f8d8c59fe4ccb34f0f5836256c2473d4a387d751a692b08d4b` | 175,299 B |
| `data/model/actor-preclip.npy` | `0228b419d6410011e2a13b495b353b420a8f673fd46fa17877df0f7a19fbf542` | 96,896 B |
| `data/model/policy-observations.npy` | `2478271e76441425a70bce2f74e1e5433462af0b7923ad7393fa1596f0ae484c` | 393,248 B |
| `data/run-contract/run-contract.json` | `c2d3e19a094c0baea3473d55a34ed00ea16dae6f672c8a4d2d4a4c3ecb8988c1` | 352 B |

对照用的 R4000 终版（不在本包内，见 `06_训练协作/R4000_field_turn_feedback_20261113/`）：
`model_3999.pt e46954b6…7769`、`policy.onnx dd42ada2…83775`。

## 8. 与既有交付包的关系

| 包 | 内容 | 与本事的关系 |
|---|---|---|
| `R4000_field_turn_feedback_20261113/` | 实机直行/转向反馈、转向机制、6101 帧回放夹具 | 提供"实机轮残差 100% 饱和"的现场证据与根因 |
| `R4000_turn_gap_rootcause_20261113.zip` | 转向轮差 ~10× 缺口根因（轮速环力矩上限 × 轮地阻力） | 同源问题的部署侧解释；本包是其训练侧修复尝试的验收 |
| **本包** | 服务器进阶版（轮速越界惩罚）训练契约、权重、我方 S2 复验、饱和对照 | 回答"那个解决了轮速满载超限问题的版本，到底行不行" |
