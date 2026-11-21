# `as2w-v2-step-jit-20261120` S0–S3 执行单（实机前准备）

配套交付包：`06_训练协作/as2w-v2-step-jit-20261120/`；部署注册：`../部署参考/cpp_deploy-as2w/models/unitree58-v2step/`。
总流程与脚本索引见 `策略交付到实机全流程/README.md`，本文件只列**这份策略**要做的动作和判据。

## 这一版与上一版（DR0 权重）的区别

合同完全相同（obs 58 / act 12 / 5 Hz / 55-2 / 0-2 / tanh 烘图），**只换了权重**：本版权重在 DR2 + 电机抖动下续训 586 轮。

| 条件 | 上一版 | 本版 |
|---|---|---|
| 干净（DR0）left/yaw_left/yaw_right | 4/4, 4/4, 4/4 | 4/4, 4/4, 4/4 |
| 扰动（DR2）left | **3/4**（t0.984） | **4/4**（t1.080） |
| 扰动（DR2）yaw_right | 4/4（t0.974） | 4/4（**t0.832**） |
| 抖动翻转率（干净） | 0.0968 | **0.0866** |

选它做首测的理由：**抗扰更强**；代价是扰动下转向跟踪余量变小。

## 能力边界

| 支持 | 不支持 / 已知失败 |
|---|---|
| 平地 vy 侧移踏步（left 四轮达标） | **right 方向右后轮不抬**（跟踪 0.274），首测 vy 限 ±0.10 |
| 平地 wz 转向踏步（两个方向四轮达标） | vx 前滚/后滚（用 `models/unitree58` 的 vx 权重） |
| 站立接管后 50 Hz 连续控制 | 起立、趴下、停车保持、楼梯、恢复 |

## S0 收包验收（工作站）

```powershell
cd "D:\Projects\Athena\运动控制\Collie\AS2-W_CPRO\部署测试\06_训练协作\as2w-v2-step-jit-20261120"
Get-Content SHA256SUMS.txt | ForEach-Object {
  $h,$p = $_ -split '  ',2
  if ((Get-FileHash $p -Algorithm SHA256).Hash -ne $h.ToUpper()) { "MISMATCH $p" }
}
cd replay; python verify_replay.py      # 期望 REPLAY_CONTRACT_OK
python verify_onnx.py                   # 期望 ONNX_CONTRACT_OK（需 onnxruntime）
```

关键 SHA：权重 `9999512641a16f20…c41c33`；部署 ONNX（tanh 烘入）`7a0367e5…5b715`；原始 ONNX `7b731baa…5611301`。
**门槛**：无 MISMATCH + 两个 `*_CONTRACT_OK` + `deployment-check.json` 里三项 `sha256_matches=true`。

## S1 移植对齐

- 参考实现 `runtime/as2w_runtime.py`，合同见 `runtime/obs_contract.md` / `runtime/action_contract.md`。
- 逐帧对拍目标：obs ≤ 2e-6、包装后目标 ≤ 1e-6（本权重实测 8.34e-7 / 3.57e-7 / 3.81e-6）。
- 三个必查差异点：① 动作压缩 `tanh`（部署 ONNX 已烘入，`action_clip=1.0` 即恒等）；② 轮目标来自 `command[0]/0.086`；③ 观测第 9–24 维是 `q − 默认角`、轮位强制 0。
- 物理/硬件未闭合项见 `runtime/deployment_contract_audit.md`（轮地摩擦未实测、硬件 map 适用性、LowCmd 方向未验证）。

## S2 sim2sim

- 训练方已做：MuJoCo 闭环 9 段命令（同合同，记录在 `simulation/mujoco_replay.json`）。
- **未做**：Isaac 与 MuJoCo 同一传感器帧的逐帧对比。用 `replay/sensor_tape.npz`（300 帧 × 48 试次）由部署方补，判据建议 ONNX 输出差 ≤ 1e-5。

## S3 CPRO 只读影子

```text
1) 同步 models/unitree58-v2step/ 到 CPRO（两跳：工作站 scp → hdc file send）
2) ARM64 载入：policy-list / policy-verify，核对 manifest.json 里 unitree58-v2step 的 7a0367e5…
3) 只读 shadow：喂真实 LowState 的 58 维观测，记录预测动作、推理周期、输入龄期；不得发 LowCmd
4) 对比影子输出与上一版（50d04e38…）的差异：本版应更平滑（翻转率低），转向命令下幅度略小
```

## S4 首测（须现场当次明确同意）

- 上限：`vx = 0`、`vy ±0.10`、`wz ±0.10`，单次 ≤ 30 s，接管前先固定 PD 站，接管时清零动作低通状态。
- 监测：倾角 > 8° 或非轮接触立即回站；向右（vy<0）重点看右后轮与倾角。
- **本版注意**：扰动下 `yaw_right` 跟踪仅 0.832，转向命令不要一上来给满幅；若现场转向响应偏弱，优先降 wz 而不是加幅。
- 记录并回传：命令、逐轴 q/dq、力矩估计、IMU、四轮接触力。

## 与上一版的切换

两个权重共用同一模型目录名与同一份 `deploy.yaml`（合同相同），切换只需替换 `policy.onnx` 并核对 SHA：

| 版本 | 部署 ONNX SHA256 | 适用 |
|---|---|---|
| 上一版 DR0 | `50d04e3883606a90e7b831212327d09c887b2fe17d8c0c0462bf76f772c87e46` | 转向跟踪精度优先 |
| **本版 jitter** | `7a0367e507ae2fdf144fe82acedf2094f023312f69eee122ef24390a0435b715` | 抗扰/抖动优先 |
