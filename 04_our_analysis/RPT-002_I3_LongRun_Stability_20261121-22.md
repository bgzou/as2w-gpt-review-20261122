# RPT-002 I3：PhysX 静态平衡的长时行为与终态小扰动（阶段 2/3/4）

编号：`RPT-002`
日期：2026-11-21 → 2026-11-22（阶段 4 于 2026-11-21 21:20–21:37 在本 Harness 下完成）
一手文件：`GPUFREE服务器/04_诊断实验/phase_hij_evidence_audit_20261121/runs/i3_continuous_20261121/`（params / code / logs / results / environment / `SHA256_MANIFEST_20261122.json`）；阶段文档 `docs/AS2W_Phase_I3_Stage2_Decision.md`、`..._Stage3_Continuous.md`、`..._Stage4_Summary_20261122.md`
环境：自有服务器 `gpufree-container`，Isaac Lab 3.0.0、Isaac Sim 6.2.0-rc.14+release.52805.02189a84、RTX 4090 24 GiB、驱动 580.126.09、Python 3.12.14（原归档为另一构建 + 双 5090 → 只做**跨环境近似一致**判定）

---

## 1. 实验对象与固定条件

| 项 | 值 |
|---|---|
| 场景 | `scenario=G`（四轮着地静态平衡），`init_condition=real` |
| 轮碰撞体 | 圆柱 `cyl` |
| 求解 | `mode=implicit`，`dt=0.005 s` |
| 增益 | `kp=55 kd=2` |
| 摩擦 / 轮-地间隙 | `friction=1.0` / `gap_mm=2` |
| 初态 | `pitch_init=0°`（hip_p002，`hip_offset=0.02 rad`）或 `2°`（pitch_p2）；`hip_offset=0` |
| 终态扰动 | 同进程 step 72000（360 s）对**当前 root pose** 施加 ±0.1° pitch，**不写 root velocity**，续跑至 480 s（96000 步） |

**为什么必须同进程**：阶段 2 发现“把终态存盘再重启”会人为把未记录的根速度置零、并重建接触/求解器历史，`pitch_p2` 重启后从 3.0225° 漂到 1.433°，无法区分“原轨迹不稳定”与“重启引入了扰动”。因此阶段 3/4 全部改为**同一进程内连续跑**。

## 2. 阶段 2（60–180 s，含重启对照）

| 检验 | 结果 | 边界 |
|---|---|---|
| I4 基线 60 s 复跑 | pitch 4.020° vs 原 4.038°；四髋最大差 ~0.0005 rad；`settle_ok=true` | 环境具备近似复现条件 |
| I4 新运行力矩账本 | 根质心账本 5.852 Nm；整机质心修正后 0.147 Nm | 再次支持“5.872 Nm 主要是质心口径错误”（见 RPT-001 §2） |
| 两初态延时 | 60/120/180 s 的 pitch 差 1.740/1.878/1.682°；髋最大差 0.0456/0.0481/0.0453 rad | 差异持续到 180 s，但不足以证明两个稳定吸引子 |
| `hip_p002` 终态重启 60 s | pitch 4.7042→4.704°，髋角基本保持 | 该重启方式下保持 |
| `pitch_p2` 终态重启 60 s | pitch 3.0225→1.433°；后髋约 0.035 rad 变化；`settle_ok=true` | **对照失败**，不能据此判定局部稳定性 |

阶段 2 结论：`INIT_SENSITIVE_MULTIBRANCH` 不能恢复为已证实；先做同进程长时。

## 3. 阶段 3（同进程 240/360 s 连续观测）

| 用例 | 60 | 120 | 180 | 240 | 300 | 360 s | 汇总 |
|---|---|---|---|---|---|---|---|
| `hip_p002` | 4.7140 | 4.7060 | 4.7042 | 4.7022 | 4.6898 | 4.7114 | 4.6960 |
| `pitch_p2` | 2.9738 | 2.8279 | 3.0225 | **1.4579** | 1.4473 | 1.4480 | 1.4430 |

- `pitch_p2` 在 180→240 s 之间由约 3.0° 迁移到约 1.45°，并在 300–360 s 保持；**同一进程内发生**，说明重启不是必要条件（但不构成因果证明）。
- 由此确立“初始条件敏感 + 存在 >180 s 长瞬态”（`SIM_RUN_CONFIRMED`）；多稳态仍 `UNKNOWN`。
- 阶段 3 的下一个门槛写成“`hip_p002` 先延长到 360 s，再在同一进程做 0/±0.1° 扰动”。

## 4. 阶段 4：四条终态扰动（本轮收口）

### 4.1 指标语义（先纠正一处长期误读）

`i_common.py` 的口径：

- `dqmax`（逐 step）= 12 腿关节速度绝对值的最大值。
- **`settle_time_s` = 最后一次 `dqmax>0.2` 出现的时刻**，是“还在动”的标记，**不是收敛时间**。
- `settle_ok` = 四轮 Fn>5 N **且** 末步 `dqmax<0.2` **且** pitch 有限。

因此 `pitch_p2` 的 `settle_time_s`≈479 s 只说明该分支**间歇**超过 0.2 rad/s；其无扰动基线也是 357.49 s（360 s 运行）。交接文档与阶段 3 文中把它当“收敛时间”的写法已在 RPT 与本轮文档中更正。

### 4.2 结果

| 用例 | 300 s | 360 s（扰动前） | 372 s | 420 s | 480 s | 汇总 pitch | settle_ok / settle_time | dq_last | dq_p99 |
|---|---|---|---|---|---|---|---|---|---|
| `hip_p002` +0.1° | 4.6898 | 4.7114 | 4.6637 | 4.7003 | 4.6710 | 4.6890 | true / 32.64 | 0.0053 | 0.0321 |
| `hip_p002` −0.1° | 4.6898 | 4.7114 | 4.7038 | 4.6301 | 4.7817 | 4.6680 | true / 360.0 | 0.0025 | 0.0326 |
| `pitch_p2` +0.1° | 1.4473 | 1.4480 | 1.4404 | 1.4356 | 1.4347 | 1.4360 | true / 479.25 | 0.0106 | 0.4057 |
| `pitch_p2` −0.1° | 1.4473 | 1.4480 | 1.4529 | 1.4386 | 1.4344 | 1.4440 | true / 478.84 | 0.0164 | 0.4014 |

同分支无扰动基线：`hip_p002_continuous_360s`（4.6960 / 32.64 / 0.0104 / 0.0397）、`pitch_p2_continuous_360s`（1.4430 / 357.49 / 0.0218 / 0.7038）。

### 4.3 单变量性（CONFIRMED）

把两条扰动运行 300/360 s 的 checkpoint 与控制运行同一时刻逐项比对：`root_pose_w`、16 维 `joint_q`、四轮 `Fn` 的 **max|Δ| 全部为 0.0**（两个分支 × 两条运行 × 两个时刻，共 8 组）。⇒ 扰动前轨迹逐位可复现，`step=72000` 的 pose 写入是唯一变量。

### 4.4 判定

| 判据 | hip +0.1 | hip −0.1 | pitch +0.1 | pitch −0.1 |
|---|---|---|---|---|
| \|480−360\| pitch ≤0.15° | 0.0404 ✓ | 0.0703 ✓ | 0.0133 ✓ | 0.0136 ✓ |
| `settle_ok=true` | ✓ | ✓ | ✓ | ✓ |
| 无分支跳转 | ✓ | ✓ | ✓ | ✓ |
| `dqmax_last`<0.05 | 0.0053 ✓ | 0.0025 ✓ | 0.0106 ✓ | 0.0164 ✓ |

**4/4 通过** → 可把“疑似多平衡分支”升级为 **“两个观测到的局部稳定终态”**，等级 `INFERRED`（不是 `CONFIRMED`：只有 4 条轨迹、单一扰动幅度、单一 dt、每分支一个初态）。

## 5. 不能推出的结论（写作红线）

1. **不能**说“物理有限接触斑/轮胎柔顺性是根因”——本轮仍是单点接触模型，实机接触斑未测（见 RPT-001 §5）。
2. **不能**排除混沌或吸引域问题：没有 Lyapunov 指数、没有跨 `dt`（0.0025/0.01）同吸引域检验、没有 ±0.5°/±2° 幅度扫描、没有 `impratio`/`friction`/`gap_mm` 敏感性网格。
3. **不能**把“局部稳定”读成“关节速度平静”：`pitch_p2` 分支持续存在 >0.2 rad/s 的间歇腿速。
4. 迁移（180→240 s）只被观测到一次，没有重复统计。

## 6. 复现与镜像

```bash
# 自有服务器（GPU 单作业，必须顺序）
cd /root/gpufree-data/as2w-cpro/phase_hij_20261121/experiments/i3_continuous_20261121
setsid nohup bash run.sh pitch_p2_perturb_p01 > logs/launcher_pitch_p2_perturb_p01.out 2>&1 &
# 完成后
setsid nohup bash run.sh pitch_p2_perturb_m01 > logs/launcher_pitch_p2_perturb_m01.out 2>&1 &
```

`run.sh` 自带 `test ! -e results/<tag>.json`，不会覆盖；每次运行写 `environment_<tag>.txt`（含 params/code/`i_common.py` 三方 SHA）。

| 结果文件 | SHA256 |
|---|---|
| `results/hip_p002_perturb_p01_480s.json` | `1b27522c2f4ebb76f1b489902928d15ec14242a9c3f0479d35f64d8e0361ab2e` |
| `results/hip_p002_perturb_m01_480s.json` | `e9fe1229d206b5a950192fdb6be426bc947f5fef53befafe9b51a4c2b7b7f59c` |
| `results/pitch_p2_perturb_p01_480s.json` | `fc05c6e606106565595211ce58fa35cb50feb70818606a6d4e73247356c117f5` |
| `results/pitch_p2_perturb_m01_480s.json` | `38883c2c0bb9d7e1d5e08f306f9d679a0c688acabdd2915859302ab57b858e5f` |
| `results/hip_p002_continuous_360s.json` | `16ac02738f0b2150da01d85ea5c3ef2c2c03de2ba8d2840bed754235c681d92d` |
| `results/pitch_p2_continuous_360s.json` | `bfd74982e8d90101a383e04b974637f0d23a4c493fc708af8ff84eecc0153346` |
| `results/pitch_p2_continuous_240s.json` | `f7881232bd31fd5d0a5559e849f2e9e90017955d1d9a94cf9f533fa15006dda2` |

代码：`code/run_one_physx.py` `4df001873b8c1abbceae052d0e8061f6510e8b75979b5daef23852240cce6c18`；`candidate/i_common.py` `b2bf8557b0eb3e3ecdf52f884c6b1dc21998e214981c33bc6ffb8f956c0339f4`；`run.sh` `4a5282869dfd46fba2b9a6ef87a90b7a1035ee0d8f79bef7c19e13d0f4c95fd1`。

本地 30 个文件与远端逐一 SHA256 相同：`runs/i3_continuous_20261121/SHA256_MANIFEST_20261122.json`（`verified=30`、`mismatched=0`）。
