# AS2-W 四足与轮足开源项目调研

AS2-W 的目标是纯 vx 四轮滚动，纯 vy 和纯 wz 轮速目标归零、由腿踏步。对照八套开源实现后，奖励上最值得借鉴的是按运动模式区分支撑与摆动，域随机化上优先覆盖摩擦、负载、驱动、观测和延迟，选模上用 RoboGauge 的思路做独立的分工况压力测试。

旧续训说明了三个环节为什么要分开。第 3,070 轮左转更接近 0.60 rad/s 指令，但右后轮有效抬落从第 1,220 轮的 21 次降到 5 次。只看回报、跟踪误差甚至关节平滑度，都可能把这种步态退化排在前面。

## 八套公开实现怎么做

| 项目 | 奖励或评分 | 域随机化 | 对 AS2-W 的启发 |
|---|---|---|---|
| **RoboGauge** | 评测分项包括速度误差、关节限位、功率、姿态、力矩平滑、摩擦余量与 ZMP 余量。用几何加权汇总，线速度和偏航误差的权重各为 2，其余各为 1。 | 压力测试组合地形、指定摩擦系数、基座质量增量和种子。评测配置还包含观测噪声和动作延迟。仿真实现给碰撞几何首个摩擦分量赋值，基座质量用加量。 | 借鉴固定工况的压力矩阵和逐项原始报告。AS2-W 额外把滚动支撑和有效踏步设为通过门槛。 [评分配置](https://github.com/wty-yy/RoboGauge/blob/add2c7ee0185ea8fd93d242f86e1c8cac6e9913b/robogauge/tasks/gauge/base_gauge_config.py) · [压力测试入口](https://github.com/wty-yy/RoboGauge/blob/master/robogauge/tasks/pipeline/stress_pipeline.py) · [模型参数与噪声](https://github.com/wty-yy/RoboGauge/blob/master/robogauge/tasks/simulator/mujoco_config.py) · [摩擦和质量赋值](https://github.com/wty-yy/RoboGauge/blob/master/robogauge/tasks/simulator/mujoco_simulator.py) |
| **RoboGauge 论文配套 Go2 训练** | go2_rl_gym 中定义速度跟踪、姿态、力矩、功率、动作一阶和二阶变化、碰撞、关节限位、足端调节、髋部姿态等项。 | 开启摩擦 0–2、基座质量增量 ±1 kg、连杆质量 0.9–1.1 倍、基座质心 ±30 mm、PD 增益 0.9–1.1 倍、电机零偏与强度、动作延迟和推扰。 | 参考随机化维度与生效时机。训练仓库与 RoboGauge 评测仓库分开，这些 Go2 范围只作对照。 [论文](https://arxiv.org/abs/2602.00678) · [Go2 训练配置](https://github.com/robogauge/code/blob/f116b00a934262348f41cbfd0d07cc4cb6fbf8d8/go2_rl_gym/legged_gym/envs/go2/go2_config.py) |
| **RobotLab Go2W** | 腿位置与轮速分别作为策略动作。粗糙地形配置的线速度跟踪权重 3.0、偏航 1.5，保留腿力矩、软限位、动作速率、碰撞和接触力惩罚。离地时长、脚滑、脚高、步态项权重为 0。 | 继承材质、基座和其他连杆质量、质心、复位和推扰。父类静摩擦 0.3–1.0、动摩擦 0.3–0.8、基座质量增量 −1 到 +3 kg。 | 借鉴腿轮分别控制、观测和记分。AS2-W 保留纯 vx 映射轮速、纯 vy/wz 轮速归零的接口，另外用抬落目标定义踏步。 [Go2W 配置](https://github.com/fan-ziqi/robot_lab/blob/26acb225aad129e222da7a34ba11f39e29b73e39/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/rough_env_cfg.py) · [父类事件](https://github.com/fan-ziqi/robot_lab/blob/26acb225aad129e222da7a34ba11f39e29b73e39/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/velocity_env_cfg.py) |
| **Unitree RL Lab Go2** | 速度跟踪、离地时长、离地时间方差、足端滑移、非法触地、软限位和能耗。机身非法触地可以终止回合。 | 启动时随机化材质与基座质量，复位时随机化状态，运行中推扰。Go2 配置静、动摩擦各为 0.3–1.2，基座质量增量 −1 到 +3 kg，推扰间隔 5–10 秒。 | 非轮部触地适合作保护项。旧基线在不同指令下有轮位分工，离地时间方差不宜直接用来要求各轮等频。 [Go2 配置](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py) |
| **legged_gym** | 首次落地时结算上一段离地时间，并做接触滤波。原实现以平面线速度指令门控，纯原地转向不触发该项。另有动作速率、关节软限位、碰撞等惩罚。 | 基类开启摩擦 0.5–1.25 和间歇推扰，基座质量随机化的默认开关为关。 | 借首次落地结算方式，让纯 wz 也触发，并把离地时间、净空、事件间隔限制在有效区间内。 [奖励代码](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py#L799-L831) · [基类参数](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py#L112-L162) |
| **Walk These Ways** | 按目标接触相位分开处理支撑与摆动。摆动期高足端载荷扣分，支撑期足端速度过大也扣分。 | 基类开启摩擦、动作延迟和推扰，并提供负载、质心、驱动增益及电机强度的开关。实验覆盖值决定最终生效范围。 | 若某轮仍长期拖行，可在现有抬落项上试软接触相位。按四种纯踏步指令分别标定，允许相位切换误差。 [奖励实现](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/rewards/corl_rewards.py#L58-L105) · [域参数](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym/envs/base/legged_robot_config.py#L227-L254) |
| **MuJoCo Playground Go1 Joystick** | 将净空、摆动峰值、支撑滑移和离地时间拆成四项。摆动峰值在首次接触时结算，有助于分辨抬得低和抬得少。 | Go1 随机化器包含地面摩擦 0.4–1.0、关节摩擦损耗、关节附加转动惯量、基座质心、质量和默认关节位置。Joystick 另有观测噪声和可选外扰。 | 用轮胎最低点的局部净空替代脚端世界坐标高度。轮胎滑移按接地点速度统计，并计入自转。 [Go1 奖励和噪声](https://github.com/google-deepmind/mujoco_playground/blob/3ff05b53d303f92b30a457c2973036186f2ffe13/rss2025/mujoco_playground/_src/locomotion/go1/joystick.py) · [Go1 随机化器](https://github.com/google-deepmind/mujoco_playground/blob/3ff05b53d303f92b30a457c2973036186f2ffe13/rss2025/mujoco_playground/_src/locomotion/go1/randomize.py) |
| **B2-W 社区实现** | 策略联合控制腿与轮。轮子参与度项用平面指令速度范数乘平均绝对轮速，另外有速度跟踪、姿态、腿力矩和动作速率。 | 混合任务公开配置包含基座复位扰动和 10–15 秒间隔的水平推扰，场景摩擦参数固定。 | 这个轮速奖励也能给纯横移时的轮转、反向轮转或空转加分。AS2-W 可将轮子参与度改为按指令方向、轮载和滑移报告的指标。 [B2-W 训练环境](https://github.com/LauraMQuiros/b2w-rl/blob/ad82b971bf69a84170f027035c1e2e3bf97e4ef9/b2w_hybrid_env.py) |

## AS2-W 奖励取舍

当前任务已有速度跟踪、姿态和动作变化惩罚，并增加有效抬落、久未抬轮、带载横滑、落地速度、非轮部触地、关节软限位和纯 vx 缺载。域随机化续训时固定这版奖励、动作接口和控制周期，便于比较随机参数引起的变化。[现有训练配置](unified-reward-training/experiment.json) · [新增奖励实现](unified-reward-training/source/run.py)

- **纯 vx** 同时看速度跟踪、四轮承重、各轮连续缺载时间和轮胎接地点纵向滑移。坡面和指令切换可单独统计，避免短暂卸载主导整体分数。
- **纯 vy／wz** 同时看零轮速目标、逐轮有效抬落、净空、周期、关节摆幅和承重侧滑。需要进一步保住时序时，再对比软接触相位。峰值或事件次数已经够用时，不再靠增大奖励追求更高净空。
- **抖动和能耗先诊断。** 对照滤波前目标、滤波后目标、实际角度和触地时机，定位高频动作来自策略输出、接触冲击还是驱动响应。力矩平滑、机械功率和落地尖峰先作为筛选指标，避免额外惩罚压缩正常摆幅。

## 域随机化续训参数

以下数值是 AS2-W 的首轮试验建议。现有任务已随机化材质、质量 ±5%、质心 ±5 mm 和驱动增益 ±5%。保留平地 60% 与斜坡 40% 的训练配置，从当前参数附近扩大。核对 PhysX 里机器人与地面材质的组合规则，MuJoCo 与 PhysX 中同名的摩擦数值不保证给出相同的有效轮地摩擦。

| 阶段 | 建议范围 | 实施和验证 |
|---|---|---|
| **A · 接触和负载，600 轮** | 静摩擦 0.55–1.15、动摩擦 0.45–0.85 且不高于静摩擦，恢复系数 0–0.05。质量倍率 0.92–1.08、基座质心各轴 ±8 mm、腿与轮驱动增益分组 ±10%。 | 静、动摩擦关联采样，修改质量后重算惯量。对于已知额外载荷，用对应的基座附加质量和质心偏移联合采样，避免多个独立极值叠加。 |
| **B · 观测与执行，1,000 轮** | 每步编码器角度噪声 ±0.01 rad、速度 ±0.15 rad/s、陀螺仪 ±0.05 rad/s，并在独立试次测试回合内固定偏置。腿与轮分开采样 0–1 个 20 ms 周期的执行延迟。 | 在物理单位上加噪，再按观测尺度归一化。奖励和验收读真实状态和接触。延迟队列每回合清零，分别测稳定期和指令切换期，记录残余轮速目标多久归零。 |
| **C · 外扰，900 轮** | 每 6–10 秒采样一次 0.10–0.25 m/s 水平速度扰动，覆盖前后和左右。 | 核实事件是叠加速度还是设置速度。按实际速度变化标记扰动强度，逐指令记录跌倒、恢复时间、滚动支撑和重新抬落。 |

与训练区分开保留一组标称和一组独立随机种子。阶段内或加入新随机化时，用同一个起点权重做配对回放。若双重扰动后某个轮位的抬落明显下降，回看单因素结果再决定扩大哪一维。随机化范围最终用实机的有效摩擦、额外负载、步进响应和时戳记录校准。

## 把 RoboGauge 的做法用到权重验收

RoboGauge 把策略拿到独立仿真测试中，按地形、种子、质量和摩擦统计各项误差。AS2-W 第一版可先留在 Isaac Lab，用独立种子完成配对回放；之后再用 MuJoCo 做交叉验证，先核对轮胎几何、关节顺序、时序、接触和坡面。ZMP 余量适合做诊断，侧移、转向及轮地接触变化时需连同有效支撑区域一起解读。 [RoboGauge 项目说明](https://github.com/wty-yy/RoboGauge/blob/master/README.md) · [摩擦余量及稳定性指标](https://github.com/wty-yy/RoboGauge/blob/add2c7ee0185ea8fd93d242f86e1c8cac6e9913b/robogauge/tasks/gauge/metrics/stable_metric.py)

| 评测层 | 最小工况 | 用于选模 |
|---|---|---|
| **标称回放** | 平地、上坡、下坡、横坡分别测站立、前后、左右移、左右转向、混合和指令切换。 | 先通过纯 vx 滚动支撑和纯 vy／wz 的零轮速目标、逐轮抬落、周期和关节摆幅。坡面按方向建立自己的匹配基线。 |
| **单因素压力** | 低摩擦、载荷变化、20 ms 延迟、观测噪声、推扰逐项测试。 | 同一工况用同一批种子配对比较，定位最先失效的指令和轮位。 |
| **组合压力** | 低摩擦＋载荷、延迟＋横向推扰、斜坡＋载荷。 | 同时报告通过率、最差工况和逐轮指标，避免总体评分掩盖动作失效。 |
| **通过后的排序** | 速度误差、带载横滑、实际关节高频残差、短时大反转、落地速度、高分位电机力矩。 | 优先比较最差工况，其次才看总体趋势。训练回报最高的 best.pt 用于提名候选，交付从通过固定验收的权重里选。 |

现有自动验收只做固定平地回放。正式续训前还要补上斜坡分模式回放，固定验收种子和压力矩阵。按前一轮 600／1,000／900 的预算逐阶段筛权重，把训练回报、标称性能和随机化性能分别存档。 [当前验收入口](unified-reward-training/check_motion.sh)
