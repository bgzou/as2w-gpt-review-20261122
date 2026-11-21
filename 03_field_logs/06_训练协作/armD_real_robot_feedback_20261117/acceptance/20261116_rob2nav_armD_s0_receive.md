# 2026-11-16 Rob2Nav armD (deploy/as2w) S0 收包验收原始记录

- 执行环境：caolin 服务器 eai-MS-7E17，/tmp/opencode/rob2nav_repro/（本地镜像，源=SSHFS 部署参考/Rob2Nav-deploy-as2w）
- python3 = torch 2.15.0+cpu / onnxruntime 1.31.1 / mujoco 3.17.0（服务器系统环境）

## 1) SHA256SUMS 全量校验

```
cd /tmp/opencode/rob2nav_repro/deploy/as2w && sha256sum -c SHA256SUMS
# 48/48 全部 OK（locale 输出"成功"），exit 0
# 权重: armD_turn65_final.pt de61de58...d926（与 manifest 一致）
# ONNX: policy.onnx 087c91bc...9821（与 manifest 一致）
```

## 2) preflight.py（包自带拒绝门）

```
--mode sim    : PREFLIGHT_OK, exit 0; WARN unverified hardware fields (19)（包内 hardware_slots 全 not_measured，属预期）
--mode dry    : exit 1（"dry run: no approval is granted by this mode"）
--mode hardware: PREFLIGHT_REFUSED, exit 1（硬件模式正确拒绝未核实槽位）
```

## 3) contracts/as2w_diff_formula_selftest.py

```
DIFF_FORMULA_SELFTEST_PASS（差速/对角公式自检通过）
```

## 4) ONNX↔checkpoint parity（本项目自复核，N(0,1) 随机 4096 样本）

```
torch actor (build_actor from r11_mj_matrix_v2) vs ORT policy.onnx
max_abs_err = 3.815e-06  （包内记录：1.907e-06/4096 样本，pass）
接口: obs [batch,65] f32 -> action [batch,16] f32（与 manifest/deploy.yaml 一致）
```

- 注：两值均为 torch/ORT float32 噪声量级；部署工件是 ONNX 本身，C++ 侧将按 ONNX 输出做回放 parity（S1/S3 再测）。

## 5) MJCF 加载（mesh 从 Rob2Nav-as2w-backup 按 XML 相对路径重建）

```
mujoco.MjModel.from_xml_path('simulation/r11_mjcf_excl.xml')
MJCF_LOAD_OK nq=23 nv=22 ngeom=19 nu=16 timestep=0.00125
# 与包 README 声称一致（nq=23 nv=22 ngeom=19 nu=16）
```

## 6) 发现的差异点/待补件

1. deploy.yaml `policy.checkpoint` 指向 `weights/armE_support2p0_final.pt`（sha 4bdf4d26...=manifest 的 line_init_checkpoint），与选中权重 armD_turn65_final.pt 不一致；ONNX_PARITY.json 确认 ONNX 由 armD 导出。deploy.yaml 疑为早一版（gyro-1/eval-3）模板未更新字段。
2. 包引用但未随包的文档：evidence/LOSS_DETECT_DELAY.md、FRAME_DROP.md、BURST_LOSS.md、DUAL_SIM_QUADRANT.md、C3_OBSERVATION_CONTRACT.md、weights/SHA256。
3. 无 replay/ 夹具（native_samples.npz）与 checks/verify_package.py；S1 parity 改为对包内参考实现 r11_mj_matrix_v2.py 做逐帧对拍。
4. 包状态 research/deployable=false；四项纯转双仿真都不达标（已知）。
5. 观测差异点（vs R1199/mlp57）：命令偏置 obs[0:3]=cmd-(0.4,0,0)、obs[9]=0.4 常数、clock sin+cos（58/59，周期 0.62s）、轮角 ±2π（4π 周期）回绕、obs[60:64] 零命令轮漂移项（相对锁存轮位回绕后 /0.20 clamp ±3）、obs[64] 零命令标志；腿 scale 0.35+站姿 [0,0.8,-1.6]、轮 scale 16/kp0/kd6 与 R1199 同族；新增 gyro-1 轮前馈 S=(-1,1,-1,1)、yaw_kp60、cap4Nm、总 clamp ±13.5Nm、dispatch=dq+tau/kd。
6. 关节序（策略侧）：FL/FR/RL/RR × hip/thigh/calf/foot（part-major、FL-first）；硬件 DDS 槽位（G1 已验证）：腿 FR,FL,RR,RL leg-major，轮 FR=12,FL=13,RR=14,RL=15 → S3 需重排映射。

## 7) 判定

S0 通过（SHA/自带校验/接口/MJCF 加载全过，preflight 拒绝门工作正常）。差异点 1/2 记录在案，不阻塞 S1/S2 离线阶段；hardware slots not_measured 由本项目 G1 实测映射在 S3 填入。进入 S1（parity 移植对齐）。
