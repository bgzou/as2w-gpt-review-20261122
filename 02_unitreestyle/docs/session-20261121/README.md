# AS2-W 会话分析归档

包含原厂风格续训、HIMLoco、实机数据分析、仿真资产校准和部署预检产物。每个目录保留原工作区相对结构。部分复跑脚本需要现场原始记录或 Isaac 服务器环境。

- [下一轮任务书（Markdown、HTML、生成脚本）](../next-training/)
- [训练过程日志](artifacts/AS2W-training-journal.md)
- [HIMLoco 俯仰 A/B 报告](artifacts/AS2W-HIMLoco-pitch-ab/REPORT.md)
- [逐轴悬空与着地重力矩残差](artifacts/AS2W-actuator-calibration/gravity-residual.csv)

| 资料目录 | 文件数 |
|---|---:|
| [AS2W-1220-residual-fix](artifacts/AS2W-1220-residual-fix/) | 89 |
| [AS2W-1220-target-review](artifacts/AS2W-1220-target-review/) | 6 |
| [AS2W-1220-vs-2000](artifacts/AS2W-1220-vs-2000/) | 12 |
| [AS2W-3160-joint-angles](artifacts/AS2W-3160-joint-angles/) | 7 |
| [AS2W-3220-review](artifacts/AS2W-3220-review/) | 6 |
| [AS2W-DR-scene-videos](artifacts/AS2W-DR-scene-videos/) | 16 |
| [AS2W-HIMLoco-13900-validation](artifacts/AS2W-HIMLoco-13900-validation/) | 65 |
| [AS2W-HIMLoco-pitch-ab](artifacts/AS2W-HIMLoco-pitch-ab/) | 31 |
| [AS2W-actuator-calibration](artifacts/AS2W-actuator-calibration/) | 2 |
| [AS2W-coverage-1000-20261119](artifacts/AS2W-coverage-1000-20261119/) | 16 |
| [AS2W-coverage5hz-1000-20261119](artifacts/AS2W-coverage5hz-1000-20261119/) | 28 |
| [AS2W-himloco-calibration](artifacts/AS2W-himloco-calibration/) | 225 |
| [AS2W-model-5550-sim2sim](artifacts/AS2W-model-5550-sim2sim/) | 54 |
| [AS2W-model-5550-video](artifacts/AS2W-model-5550-video/) | 4 |
| [AS2W-official-front-rear](artifacts/AS2W-official-front-rear/) | 7 |
| [AS2W-official-stop-replay](artifacts/AS2W-official-stop-replay/) | 39 |
| [AS2W-unified-DR-20261119](artifacts/AS2W-unified-DR-20261119/) | 312 |
| [AS2W-unified-current-best-video](artifacts/AS2W-unified-current-best-video/) | 5 |
| [AS2W-unified-deployment-preflight](artifacts/AS2W-unified-deployment-preflight/) | 99 |
| [AS2W-unitree-flat-v2-2000](artifacts/AS2W-unitree-flat-v2-2000/) | 31 |
| [AS2W-unitree-flat-v2-dr](artifacts/AS2W-unitree-flat-v2-dr/) | 52 |
| [AS2W-v2-eval-1000](artifacts/AS2W-v2-eval-1000/) | 7 |
| [AS2W-v2-eval-900](artifacts/AS2W-v2-eval-900/) | 6 |
| [HIM-arc-training](artifacts/HIM-arc-training/) | 13 |
| [HIM-stairs-training](artifacts/HIM-stairs-training/) | 13 |
| [HIMLoco-5836-video](artifacts/HIMLoco-5836-video/) | 2 |
| [HIMLoco-7836-checkpoint](artifacts/HIMLoco-7836-checkpoint/) | 15 |
| [HIMLoco-current-audit](artifacts/HIMLoco-current-audit/) | 8 |
| [HIMLoco-stairs-7672](artifacts/HIMLoco-stairs-7672/) | 2 |
| [工作区报告及预览](artifacts/) | 14 |
| [cpro_candidate](artifacts/cpro_candidate/) | 10 |
| [field_analysis](artifacts/field_analysis/) | 9 |

压缩副本、原始运行日志、依赖与部分资产二进制留在原归档。列举的未复制文件见 [范围明细](omitted.txt)。

## 主要结论与复跑顺序

1. 原厂风格的目标是 vx 四轮滚动、vy/wz 周期踏步。1220 是步态行为参考。后续筛选比较完整关节角、毛刺和逐轮支撑，避免给停止踏步的权重更高名次。
2. T2 悬空重力回放四髋平均残差幅值为 0–0.45 Nm，着地为 15.06–19.36 Nm。着地差额包含接触负载，不能用作减速器摩擦参数。优先核对接触几何与轮载，再由双向扫角辨识摩擦。
3. 同 USD 的低、高摩擦对照展示了静态髋力矩和动态支撑的取舍。两档都是仿真对照设置。HIMLoco 9836 和 13900 的停车结果需要按相近入速比较。
4. HIMLoco 持续俯仰 A/B 的 B 在低摩擦下减少持续前倾，高摩擦时改善接管，但低摩擦停车偏移变大。B 保留为姿态候选。
5. 新任务从平地和标称资产起步，逐步加入缓坡和经验证的随机化。最终按同一权重的全部场景成绩筛选，再通过 sim2sim、ONNX 对齐与实机分阶段验收。

主要资料按实验目录组织。曲线和视频与对应数值、复跑脚本保存在同一实验目录。
