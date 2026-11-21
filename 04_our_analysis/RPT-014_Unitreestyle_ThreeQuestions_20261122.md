# RPT-014 三问答复：step 有没有 clamp 翻转、资产是否与 sim2real 包不同、同事服务器训练方向

编号：`RPT-014`
日期：2026-11-22（Asia/Shanghai）
答复对象：用户三问（step 的 clamp 满幅翻转 / unitreestyle 资产 vs `as2w-sim2real-rootcause-bundle-20261121` / 同事服务器最新训练任务）
一手来源：`部署参考/as2w-unitreestyle/**`（只读）、`部署参考/as2w-sim2real-rootcause-bundle-20261121/**`（只读）、本项目 RPT-003/007/008/013

---

## Q1. step（v2-step）有没有"clamp 满幅翻转"问题？

**没有。** 这条线正是为修掉该问题才改成 tanh 的（`CONFIRMED`，两侧证据）：

| 环节 | 证据 |
|---|---|
| 训练 | `v2-step/code/environment.py:161` `actions = torch.tanh(actions)`；注释原文："clamping put ~99% of action dims on the boundary"；`v2-step/README.md` 记录**旧 clamp 基线 94.5% 的动作维贴在边界上**，所以"clamp had zero gradient at the rail" |
| 部署 | `docs/cpro-deploy-20261120.md`：部署件是 **`policy.onnx（tanh 烘入）`**，SHA `7a0367e5…`；C++↔Python parity 覆盖"ONNX → tanh → 低通 → 16 轴目标"，腿目标差 **1.85e-06**、轮 0.0，判 PASS |
| 现场 | 零命令抖动符号翻转率 8.2%（抖动权重）/12.85%（mu025），而不是 R2000A 那种 ±1 交替 |

对照 R2000A：仍用 `clip(base,±1)`，clip 前 **98.89% 元素、100% 帧**超界（RPT-003），现场因此出现"±1 饱和翻转驱动 25 Hz 腿目标交替"（INDEX 007）。**两者不是同一个动作参数化，不能把 R2000A 的翻转现象套到 step 上。**

⚠ 但"没有 clamp 翻转"≠"没有抖动"：step 现场仍有 8–13% 的目标符号翻转与 0.36–0.45 rad 的 p2p，来源是踏步相位/策略本身，不是轨顶饱和。要分开表述。

## Q2. unitreestyle 的资产是不是和 sim2real 包里的不一样、是否已自己做修正？

**两个层次要分开：URDF/网格是同一族、且未带修正；修正做在 USD（Isaac）层，而且不在这个仓库里。**

实测（本次重算，`CONFIRMED`）：

| 资产 | 总质量 | base 质量 | base COM x | base ixx/izz | 轮碰撞体 |
|---|---|---|---|---|---|
| `unitreestyle/assets/as2w/as2w.urdf` | **26.000 kg** | **10.08** | **0.035002** | 0.050695 / 0.150582 | mesh |
| `bundle/.../urdf_official/as2w-official-mass.urdf` | **26.000 kg** | **10.08** | **0.035002** | 0.050695 / 0.150582 | mesh |
| `bundle/.../urdf_cylwheel/as2w-load-v0.urdf` | 26.000 kg | 10.08 | 0.035002 | 同左 | **cylinder** |
| `unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf` | **27.512 kg** | **11.592** | **0.020002** | 0.058299 / 0.173170 | mesh |
| `R2000A/simulation/urdf/as2w-load-v0.urdf` | 27.512 kg | 11.592 | 0.020002 | 同左 | mesh |
| `HIMLoco MJCF as2w_scene_v2.xml` | 26.000 kg | **10.58** | 0.020441（quat 旋转） | diag 0.183/0.165 | mesh |

结论：

1. **unitreestyle 的基础 URDF 与 sim2real 包的 `urdf_official` 是同字节量级的同一资产**（26.0 kg / base 10.08 / COM x 0.035 / 同惯量），只是包内提供了 cylinder 轮与 official-mass 变体。→ 不是"完全不同的资产"。
2. unitreestyle 另带一份 **load-v0（27.512 kg / base 11.592 / COM x 0.020）**，与 R2000A 训练资产完全一致 —— 这是"加载版"候选，不是默认训练资产。
3. unitreestyle 真正的训练资产是 **USD**（`as2w-t2-v2.usd`），其修正版是 **`as2w-t2-v2-mu025.usd`（μ=0.25 + min 合并）**；任务书写明它位于**原工作区** `AS2W-unitree-flat-v2-dr/assets/`，**不在这个仓库**。仓库里只有 `prepare_v2_usd.py`（URDF→USD + 碰撞排除，**不改摩擦**）与 `check_usd_friction.py`（只读检查）+ `make_mass_variant.py`/`make_com_variant.py`（实验变体）。
4. **摩擦值在整个生态里至少五套**：USD 标称 0.5、USD mu025 0.25、HIMLoco MJCF 0.62、R2000A MJCF 1.0、sim2real 包 MuJoCo 变体 0.05/0.3/0.8。→ 任何"摩擦已对齐"的结论都必须绑定到具体资产文件与 SHA，不能跨资产默认。

所以对"是否已自己修正"的回答：**是，但只修了摩擦（USD μ=0.25）并在 USD 层做质量/质心变体；URDF/MJCF 副本不带这个修正**。这也解释了为什么 v2-step 的实机摩擦对齐重训没改善转向——修正只进了训练资产，实机侧的接触/几何（后髋载荷分布）没有跟着对齐。

## Q3. 同事服务器（`n1ddtnhc-...:30022`）最新训练任务

### 3.1 访问结果：**失败（未读到任何远端内容）**

| 尝试 | 结果 |
|---|---|
| SSH 握手 | 成功，banner **`SSH-2.0-SSHPiper`**（转发型网关，非普通 sshd） |
| `password` 认证（3 个用户名：`n1ddtnhc-kw8vvfcc.root` / 无 `.root` / `root`） | `False` / `AuthenticationException` |
| OpenSSH `PreferredAuthentications=password` + askpass | `Permission denied (password,publickey)` |
| OpenSSH `keyboard-interactive` + askpass | 同上（服务端只广播 `password,publickey`） |
| session 通道 / SFTP | 打开即 `EOFError`（该网关只放行 `direct-tcpip` 转发） |

判断：**凭据被服务端拒绝，或该网关需要交互式/网页端登录后再转发**。我没有把密码写入任何文件或报告（只在进程环境变量中用过），但**该密码已在聊天中出现，建议轮换**。要继续读远端，需要：① 一个新的、未过期的密码或可用的只读 key；或 ② 你在网页控制台里跑我给的一小组只读命令并把输出贴回；或 ③ 平台侧把 30022 指向真实 sshd（而不是 SSHPiper 网关）。

### 3.2 但"最新训练任务"本地已有：11-21 交付的任务书

`部署参考/as2w-unitreestyle/docs/next-training/任务书.md`（SHA `1bff7841994c0ffc`，2026-11-21）+ `index.html` 就是他们这一轮写下的**下一轮训练设计稿**，内容比"最新 run 的 loss 曲线"更能回答"改进方向"：

**A. 原厂风格线（step 的父线）**
- 父权重：1220 / 1320 / 1000 各自按原配置先复验，再统一协议比较；1220 作步态行为参考，**步频按实机重新定**。
- 动作：12 轴腿 + **独立轮速映射**；复核轮半径、轮轴符号、vx→轮速单位与限幅；纯侧移/转向时**轮速指令保持零**；混合命令单独测。
- 资产/接触：**两档摩擦对照点（0.25+min、0.50+max）**，先同 USD 固定两档评估，再插值；每回合整块地面同档；实测轮径/受载半径/轮宽/轴距/轮心高度；**禁止用穿地换姿态得分**。
- 新增待辨识项：重力前馈、电机/减速器静摩擦与启动力矩、动摩擦与黏性、`armature`、执行延迟/滤波、扭矩-转速限幅、质量/质心/惯量（±0.5 kg / ±10 mm 压力点）。
- 排程 **S0–S5**（无策略→标称平地→平地+缓坡→单因素诊断→联合随机化→冻结部署）；出口条件逐阶段写死。
- 历史坑清单：续训最好的权重会"不踏步"、纯 vx 出现周期抖动；视频穿地/三轮滚动仍被旧汇总写成通过；**实机 T2 后腿下沉、髋承重远高于旧仿真（悬空残差 0–0.45 Nm，着地 15.06–19.36 Nm）**；0.25+min 改善静态却在动态出现后轮卸载；只加俯仰惩罚会让低摩擦停车变差。

**B. HIMLoco 线**
- 父权重：**9836 优先复验**，13900 作停车对照；10436 的 A/B 用于验证俯仰项（B 暂为候选）。
- 保留父权重的 HIM runner / 历史输入 / 估计器 / 观测归一化；任何算法替换单独开实验。
- 俯仰项 A/B 已有数字：B = 5 Hz 姿态低通、同向 >4° 计时、回 3° 清零、0.30–0.60 s 渐增 → 低摩擦下运动持续前倾 **6/48 → 0/48**，但停车位移 P95 **14.6 → 16.5 cm**；高摩擦下接管持续后仰 **48/48 → 29/48**。下一轮要 A / 原 B / 按阶段处理的候选**同场景对比**。
- 平地滚动**禁止持续三轮支撑**；障碍能力作为独立扩展；坡面用现场姿态并把世界俯仰与地形相对俯仰分开记。

**C. 与本次审查的一致性**
- 任务书把"摩擦/接触几何/质量分布"列为**训练前的资产与标定问题**，与我们 RPT-013 的"v2-step 主要是训练域+门禁、HIMLoco 主要是部署 kp"并不冲突：任务书讲的是下一轮训练前要先做的标定与判据；我们讲的是**已发生失败的归因**。
- 任务书要求"门禁每个判据都要映射到奖励/可观测项"（`index.html` 也说 unified_dr 的 eligible 掩码让 7/16 组合不可得分，导致 32/33/31 项 FAIL）——这条**必须写进交付合同**，否则下一轮还会出现"门禁不可满足"的假失败。

## 4. 三条线的改进方向（综合任务书 + 我们的实测 + 现有否定结果）

| 线 | 先做什么（便宜、能排除） | 再做什么 | 不要做什么 |
|---|---|---|---|
| **v2-step / 原厂风格** | ① 单关节方向/零位核对（P0，12 轴 `+0.02 rad`，与仿真对符号）② 把门禁逐条映射到奖励/可观测项 ③ 两档摩擦（0.25/0.5）同 USD 评估 | ④ 轮差速前馈作独立新版本（合同变更）⑤ 接触几何/COM 单变量 ⑥ 从 μ=0.25 起按 S1–S4 重训 | 不要只在低摩擦上继续续训；不要把"right 方向门禁 FAIL"当能力问题（几何死结）；不要再靠调权重 |
| **HIMLoco** | ① 站姿 kp85/3.2 候选（仅站姿段）+ zero-B 记录 `cmd−actual` ② 修 MuJoCo 对照里的 Python 参考包装（RPT-007 的漂移）再谈"仿真通过" | ③ 俯仰项 B/分段候选的同场景 A/B（沿用 5 Hz + 4°/3° 门控）④ 平地三轮支撑门禁 | 不要为"后腿下沉"重训（同合同 sim 复现、kp85 减半）；不要用偏移补偿（已证过冲+嗡鸣） |
| **unified_dr** | 先修 eligible 掩码/门禁映射，再谈训练 | 之后按统一奖励 + DR 重跑 | 不要把 32/33/31 FAIL 解读为训练不足 |

## 5. 证据索引

| 文件 | SHA256 前 16 |
|---|---|
| `v2-step/code/environment.py`（tanh squash） | 见仓库；行 159–171 |
| `v2-step/README.md`（94.5% 边界） | 见仓库 |
| `sessions/.../docs/cpro-deploy-20261120.md`（tanh 烘入 ONNX、parity 1.85e-06） | `a393180e80032211` |
| `sessions/.../docs/tanh-baseline.md`（clamp→tanh 的 gate 变化） | `050897cd0a1987d5` |
| `assets/as2w/as2w.urdf` | `05da624085b16c91` |
| `calibration/candidates/load-v0/as2w-load-v0.urdf` | `7c9740a95042bad7` |
| `bundle/.../urdf_official/as2w-official-mass.urdf` | `4253…`（见包内 SHA256SUMS） |
| `docs/next-training/任务书.md` | `1bff7841994c0ffc` |
| `index.html`（eligible 掩码 / RR 几何 / 摩擦 DR 区间） | `411af8f5523d807e` |
| 本项目 RPT-003/007/008/013 | — |
