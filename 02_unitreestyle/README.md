# AS2-W 原厂风格训练复现

当前方案冻结第 1,220 轮策略，只在侧移、转向及混合命令下训练有界腿部残差。新服务器的启动命令、完整奖惩设计、四轮载荷和关节角验收见 [残差续训说明](residual1220/README.md)。

## 历史方案：第 1,220 至 3,220 轮全策略续训

从第 **1,220 次 PPO 更新**的策略和优化器状态继续训练。运行设置为 **8,192 个环境 × 48 步 rollout**，seed 42，每段 2,000 次更新，计划新增 10,000 次。每段结束后用原始步态做验收，再决定是否继续。

## 在新服务器运行

在具备 NVIDIA GPU 的 Linux 主机上，先按[运行环境说明](repro/runtime.md)配置 Isaac Sim 和 Isaac Lab。已验证机器为 NVIDIA L40（nvidia-smi 报告 46,068 MiB）。以下命令从仓库根目录执行。每次尝试使用一个新的输出目录。

```bash
git clone https://github.com/YuTaoV5/as2w-unitreestyle.git
cd as2w-unitreestyle
./repro/check_runtime.sh

RUN="$PWD/runs/resume-1220"
./continuation1220/prepare_run.sh "$RUN"
./evaluation/replay.sh "$RUN"
./continuation1220/smoke.sh "$RUN"
nohup ./continuation1220/run_blocks.sh "$RUN" >"$RUN/progress.log" 2>&1 </dev/null &
tail -f "$RUN/progress.log"
```

- `prepare_run.sh` 校验所有冻结输入的 SHA-256，再建立 `$RUN/source`。其中 `best.pt` 包含模型和优化器，来源记录见 `baseline/best.json` 和 `baseline/run.json`。种子 checkpoint SHA-256 为 `d8402f10e85956bc9dd5fcdcaa91e5e14d63cdb4fc55d46b96a2e0186f571a50`。
- `replay.sh` 在 48 个平地环境中复跑起点策略，输出 `flat.npz`、`flat-motion-audit.json` 和 `gate.json`。先确认起点的 `gait_preserved` 为 true，并核对噪声计数和冻结对照。原始轨迹包含实际关节角、限幅后未经过控制低通的腿部目标、滤波后目标和各轮法向接触力。
- `smoke.sh` 载入起点权重，实际跑一次完整 PPO 更新（393,216 条环境步），写到 `$RUN/smoke`。查看 `$RUN/smoke.log` 中的 `AS2W_RESUME`、`AS2W_PHASE_METRICS` 和 `AS2W_TRAIN_SMOKE_OK`。正式训练仍从第 1,220 轮启动。
- 正式训练先写入 `$RUN/blocks/u3220/train`。每段完成时比较段末 checkpoint 和按训练回报保存的 `best.pt`。选中的路径记在 `selection.json`。通过步态验收且毛刺计数严格低于起点的候选，才进入 `$RUN/validated-best.json`。`AS2W_HELD_FOR_GAIT_REVIEW` 表示该段所有候选均未通过步态验收，脚本已主动退出。训练日志与编号 checkpoint 保留在各段目录。
- 如果服务中途中断，保留已生成的 `blocks`。`run_blocks.sh` 从头启动整条流水线，重跑时使用新目录。脚本 `train.py` 的 `--resume`、`--offset`、`--iterations` 可用于从明确指定的 checkpoint 单独恢复一段。

验证记录见 [完整单轮日志](repro/smoke-evidence.txt)和[起点回放结果](repro/replay-evidence.json)。

## 哪些行为保持第 1,220 轮的定义

58 维策略观测和 12 维腿部动作，50 Hz 策略控制。轮目标来自 `vx / 0.086`，限于 ±12 rad/s。纯侧移或转向时轮速度目标回到 0，由腿负责踏步。腿部位置目标低通 10 Hz，轮速目标低通 5 Hz。仿真使用 `load-v0` URDF，基座质量和惯量乘 1.15，基座质心沿 x 后移 15 mm。沿用平地、坡、起伏、台阶地形和原始域随机化，包括摩擦及恢复系数、质量 ±5%、基座三轴质心 ±5 mm、PD 增益 ±5%，具体以 `common/environment.py` 为准。

训练继续使用原来的速度跟踪、机体稳定、腿偏置和动作差分奖励。唯一新增奖励项为 `-dt * 0.02 * short_reversal_cost`。只对平地纯侧移、纯转向，且足够稳定地锁定触地相位的样本生效。对比起点策略在对应相位的动作模板，只处罚幅度超过阈值的 1–2 帧反转残差。相位模板由第 1,220 轮平地轨迹生成，按最早的合格轮载触地事件划分周期、重采样、取中位数并做周期平滑。可重建模板：

```bash
./preflight/analysis-python continuation1220/build_reference.py \
    baseline/flat.npz /tmp/as2w-reference.npz
./preflight/analysis-python evaluation/plot_trial.py baseline/flat.npz left /tmp/as2w-1220-left.png
```

验收同时比较指令跟踪、跌倒、关节周期和活动幅度、抬轮次数、轮载分布以及相位去除后的关节抖动。用于自动筛选的是平地数据。粗糙地形参与训练，平地验收结果只说明这组平地命令下的表现。细看单次试验时，`flat.npz` 的 `q[..., :12]` 是腿部实际角度（rad），`wheel_data[..., 15]` 是四轮的竖直法向力（N），轮序为 FL、FR、RL、RR。

## 来源和目录

- `baseline/` 冻结的起点权重、记录、平地数据、统计和参考相位。
- `common/` 原始环境、PPO runner 和 checkpoint 规则。若要用同一原始环境重新从零训练，入口为 `./preflight/isaac-python common/run.py --mode unitree_style --asset calibration/candidates/load-v0/as2w-load-v0.urdf --num-envs 4096 --rollout-steps 48 --iterations 1220 --out <新目录>`。从零训练的数值轨迹受 PhysX 与硬件影响，复现续训以归档权重为起点。
- `continuation1220/` 相位参考生成、续训、分段选择和 gait gate。
- `evaluation/` 和 `quality/` 平地回放、轮地接触计算和统计。
- `assets/as2w/` 来自 [unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/157ff950351e5f535e03c5d85a16b7d594b3dbcc/robots/as2w_description) 的官方 URDF 和 mesh，许可证见 `LICENSES/`。训练使用的 load-v0 校准 URDF 位于 `calibration/candidates/load-v0/`。

## 已归档的续训结果

第 3,220 轮完成首段，两个候选均未通过固定平地验收。已归档权重、完整回放、关节角与轮载、周期性诊断和两种物理引擎视频。 [第 3,220 轮边界结果](review/u3220/README.md)

## 2026-11-19 统一奖励与分阶段域随机化研究交付

[交付入口与复跑命令](unified_dr/README.md)包含选中 checkpoint、单文件 ONNX、CPRO 控制参考、机器人资产、精确阶段源码、配置和逐帧对拍数据。

三个累计随机化阶段各完成 600 次更新。全部阶段未产生通过完整步态验收的新权重，保留基础 model_1000 输入，状态为 research_only。Python/C++ 接口对拍通过；第二仿真器、CPRO ARM64 和真机控制尚未运行。不能把接口对拍当作运动验收通过。

基础 10000 轮任务仍在运行。截至 2026-11-19 07:26，第 4000 索引权重仍未通过滚动支撑与踏步要求。[最新过程记录](review/unified_dr_20261119/PROCESS_REPORT.md)和[第 4000 索引验收](review/unified_dr_20261119/U4001_REPORT.md)单独保存；unified_dr 目录保持交付快照字节及 SHA 清单，避免把后续记录混入已归档版本。

> **2026-11-21 核对更新**：该基础任务**没有完成 10000 轮**——2026-11-19 10:29 在 update 6034 停止（`exit-code=1`，最后 checkpoint `model_6000`，`best.json completed_updates=6000`），u6001 意图审计未通过。详见 [训练与部署任务书](docs/taskbook-20261121/)。

## 2026-11-21 训练与部署任务书 + 服务器核对

- [交互式任务书（HTML，下载后用浏览器打开）](docs/taskbook-20261121/index.html)
- [训练与部署任务书（Markdown，含包络 / 奖励 / 域随机化 / 部署问题库 / 配置字典）](docs/taskbook-20261121/AS2W-训练与部署任务书-20261121.md)
- [服务器只读核对报告（版本 / 权重 SHA / 训练状态 / 磁盘 / 安全）](docs/taskbook-20261121/AS2W-服务器核对报告-20261121.md)

本版任务书修正了旧文档 6 处事实：Python 实际 3.12.3、训练速度 6–8 s/更新、基础 10000 轮实际在 6034 停止、服务器路径布局、hip 符号三层区分、MuJoCo 版本；并补充了“包络怎么定 / 奖励怎么按任务拆 / 域随机化串行还是并行 / 怎么保证泛化”的完整方法。

## 2026-11-21 实机数据索引（不含原始数据）

- [实机数据索引：数据 ↔ 权重 ↔ 训练方案](docs/field-data-index-20261121/AS2W-实机数据索引-20261121.md)
- [实机数据-权重-训练方案对照.csv](docs/field-data-index-20261121/实机数据-权重-训练方案对照.csv)（154 个实机会话 + 18 个关键数据目录）
- [权重-训练方案-上机状态对照.csv](docs/field-data-index-20261121/权重-训练方案-上机状态对照.csv)（24 个关键权重，含 SHA256）
- [目录说明](docs/field-data-index-20261121/README.md)

**原始实机数据（日志 / CSV / 视频，约 16 GB）不随仓库上传**，仅保留在本地工作区；本目录只放索引、SHA256 指纹与结论。核心事实：v2-step 正式交付 `45b36ee7…` **从未上机**，11-20 实机用的是 jit `99995126…` 与 mu025 `e90157b3…`。
