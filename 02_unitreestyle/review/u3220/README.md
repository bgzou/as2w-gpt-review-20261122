# 原厂风格第 3,220 轮边界归档

从第 1,220 轮完成了 2,000 次 PPO 更新。第 3,160 轮是训练回报最佳（55.1394），第 3,220 轮是段末权重。两者均未通过固定平地验收，续训流水线停在第 3,220 轮。已通过的续训最佳权重为空。

| 固定平地指标 | 第 1,220 轮 | 第 3,160 轮 | 第 3,220 轮 |
| --- | ---: | ---: | ---: |
| 短反转次数 | 8,824 | 7,648 | 7,783 |
| 前进四轮承重 | 99.07% | 96.47% | 96.53% |
| 倒车四轮承重 | 95.20% | 83.00% | 83.07% |
| 左移时左前轮抬起次数 | 40 | 19 | 2 |
| 右转时左后轮抬起次数 | 14 | 4 | 1 |
| 右转实际关节相位残差 RMS | 0.515° | 0.608° | 0.567° |

## 奖励检查

原续训仅新增 `-0.020 * 0.02 * short_reversal_cost`，并保留 `load-v0`、原有奖励和 PPO 优化器状态。该项只在平地纯侧移（`|vy|=0.15–0.45 m/s`）或纯转向（`|wz|=0.35–0.80 rad/s`）且相位有效时生效。纯 `vx` 的该项恒为零。纯前后滚动的四轮目标角速度仍为 `vx / 0.086` 并限幅。腿部目标仍由同一个 actor 输出，因此不同命令之间共享更新。

原奖励包含速度跟踪、机身稳定、腿角偏离默认站姿与全工况动作差分惩罚。直线滚动没有防止四轮失载的专门项，也没有平稳站姿随动的明确参考。平地固定回放第 3,160 轮的前进目标差分/二阶差分代价均值为 0.01794（起点 0.00294），同时默认站姿偏差代价由 0.10950 降到 0.04822。倒车目标差分代价 0.00467 → 0.01422，同时站姿偏差代价 0.10919 → 0.06789。这些分别是乘步长前的子项，不能合并当作完整训练回报。倒车实测平均速度 -0.5354 → -0.5403 m/s（指令 -0.60 m/s），可见速度跟踪可以维持而四轮承重下降。

高斯探索标准差在三个 checkpoint 分别为 14.80、91.38、94.01，远大于动作区间 ±1。第 3,160 轮的确定性腿目标限幅占比 99.93%。新增相位奖励的环境覆盖率在 201 个训练日志采样点上的中位数为 8.02%。没有同起点、同环境设置的奖励权重为零对照，现有结果无法分离相位项、探索参数和普通续训漂移各自的影响。

## 周期性与附件

纯 `vx` 是四轮滚动，腿只做承重姿态和小幅反馈修正。纯 `vy/wz` 的基本周期和抬落顺序需要保留。第 3,160 轮左移的触地周期为 0.68 秒。左前小腿目标角和实测角均有 5.8 Hz 峰，接近四次步态谐波。前进时该腿下发角近乎恒定，实测仍有 6.8 Hz 峰。其他关节和轮地受力可能共同激励该振荡，详见同时间轴图和逐关节统计。

- [左右侧移关节角](angles/AS2W-3160-joint-angles-lateral.png)、[转向关节角](angles/AS2W-3160-joint-angles-turn.png)、[前后滚动关节角](angles/AS2W-3160-joint-angles-rolling.png)、[站立和混合指令](angles/AS2W-3160-joint-angles-stand-mixed.png)。
- [关节角时序](angles/AS2W-3160-joint-angles-timeseries.csv)、[六试次汇总分位数](angles/AS2W-3160-joint-angles-quantiles.csv)、[目标与轮载对齐图](angles/phase-diagnostic-3160.png)、[周期分析](periodicity.json)。分位数剔除指令启动后的前 1 秒。相位统计留出整个试次交叉验证，加速度/误差谱为 50 Hz 采样下的探索性诊断。
- [MuJoCo 闭环视频](video/AS2W-1220-3160-3220-mujoco.mp4)和[PhysX 记录渲染](video/AS2W-1220-3160-3220-physx.mp4)。MuJoCo 每命令一次，同一名义模型和 load-v0 基座惯性，六条命令触发跌倒阈值数为 2 / 3 / 2。PhysX 固定验收作为本归档的通过依据。
- `training-best/`、`endpoint/` 含候选权重、全部 48 条名义平地轨迹、轮接触与验收报告。`run.json` 记录本段配置，`training-phase-samples.json` 是本段的间歇训练指标。相对路径和内容哈希见 `SHA256SUMS`。

## 复算

从仓库根目录执行。下列绘图和分析输出写到新目录。

```bash
mkdir -p /tmp/as2w-u3220
(cd review/u3220 && sha256sum -c SHA256SUMS)
./preflight/analysis-python review/u3220/tools/plot_angles.py "$PWD" /tmp/as2w-u3220
./preflight/analysis-python review/u3220/tools/audit_periodicity.py "$PWD" /tmp/as2w-u3220
./preflight/analysis-python review/u3220/tools/plot_periodicity.py "$PWD" /tmp/as2w-u3220

# 另外需要 MuJoCo 3.12.0、PyTorch、Pillow、ffmpeg、DejaVu Sans 和可用的 EGL。
MUJOCO_GL=egl ./preflight/analysis-python review/u3220/tools/render_comparison.py /tmp/as2w-u3220 "$PWD" physx
MUJOCO_GL=egl ./preflight/analysis-python review/u3220/tools/render_comparison.py /tmp/as2w-u3220 "$PWD" mujoco
```
