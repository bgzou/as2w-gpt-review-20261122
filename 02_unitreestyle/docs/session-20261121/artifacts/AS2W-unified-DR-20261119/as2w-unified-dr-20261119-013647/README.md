# AS2-W 统一奖励及域随机化策略交接

## 本次交付实值

| 项目 | 实值 |
| --- | --- |
| 用途状态 | research_only |
| 选中候选 | incoming |
| Isaac 步态验收 | 未通过；失败项 31 |
| 输出 checkpoint SHA256 | 8e9f2e0fae9701853ca6b21c03b8e325c2a24733642f08d27969e08144e7dbce |
| ONNX SHA256 | c909f68033f6efc84eef82187d3e837505889aeb75e7dec8d470ebf460536d69 |
| 与选中输出哈希相同的阶段初始化 | [1, 2, 3] |
| 控制周期与接口 | 20 ms；58 维输入，12 维网络输出，包装为 16 轴目标 |
| 域随机化阶段更新数 | 接触载荷 600；噪声延迟 600；外部推扰 600 |
| 第二仿真器 / CPRO ARM64 / 真机控制 | not_run / not_run / not_run |

失败项计数不是跌倒次数。若输出哈希与阶段初始化相同，表示保留输入，不能将其称为新续训权重。正式用途以本表和 manifest 为准。


版本、仿真验收状态和权重哈希见 manifest.json。阶段输入及候选来自 source 下的分支选择和三份阶段选择文件。原始训练路径仅供溯源。

策略输入为 58 维 float32，输出 12 个腿动作，20 ms 一帧。运行时将腿动作变成位置目标，并将机体 vx 映射为四个同号轮速，纯 vy/wz 的稳态轮速目标为零。舵杆、接管、驻停及紧急退出由外层 CPRO 状态机处理。松杆时轮速滤波趋向零，有制动阻尼。训练从站姿附近启动。自动起身及楼梯不适用，松杆闭环停车和运动中交接的能力以实测为准。

runtime/contract.json 列出每一维观测、16 轴的默认值、映射、控制输出及增益。腿 55/2，轮 0/2。训练中的接触、载荷、噪声和延迟参数见 source/stages。干净回放关闭随机化，压力回放按目录后缀 level 启用累计扰动。

## 离线验收

从解包目录运行下列命令。x86_64 验证过的 Python 和依赖版本见 source/export-environment.json。ARM64 运行结果见 manifest。

~~~bash
sha256sum -c SHA256SUMS.txt
python runtime/verify.py model/policy.onnx replay/flat-sensor-tape.npz
python runtime/verify.py model/policy.onnx replay/slope-sensor-tape.npz
~~~

回放数组以帧、试次、字段组织，使用仿真原始关节、角速度和姿态数据。脚本报告从传感器构造的观测、ONNX 原始动作、滤波后腿角和轮速，以及下一帧状态与训练侧参考的最大差值。重置帧的动作历史由环境初始化规则更新。

名义验收由 source/stages/score.py 判分。gate.json 记录逐例统计与失败项。压力回放各阶段以该阶段输入权重作同域对照。验收涉及反复抬轮、滚动四轮支撑、关节活动幅度、速度和存活。滑移与综合分数参与筛选。最终状态见 manifest.json。

## CPRO

runtime/runtime.py 可从 unitree_hg LowState 读取实机数据，复算模型目标并输出 JSONL。须为 --zero-map 提供现场标定 JSON，joints 使用合同顺序，zero_dds_rad 是相同顺序的 16 个原始编码器偏置。轮角没有进入网络，但轮速度保留。软件使用读取方向，现场输出端使用已验证的发送方向。

~~~bash
python runtime/runtime.py --model model/policy.onnx --interface eth0 \
  --zero-map cpro-calibration.json --seconds 10 --output cpro-shadow.jsonl
~~~

接入现有发布器时，用 Controller 的 enter()、step()、exit() 对接站立、策略和交还状态。输入为模型顺序、机体系、SI 单位，输出为模型顺序的 q,dq,kp,kd,tau。DDS 发送槽位、命令方向、控制权和保护由现有现场状态机管理。脚本的 40 ms 样本龄用于 CPRO 只读诊断。实机测试需现场核对站姿工作点、首帧目标变化、轮命令、退出和急停。

C++ 现场接入候选见 runtime/cpro_policy.hpp 和 runtime/cpro_ort_adapter.hpp。
后者使用现场 cpp_deploy-as2w 已有的 isaaclab/algorithms/algorithms.h。把 OrtPolicy 接到既有发布器的策略状态下，进入 PolicyWalk 时以已映射 q 调用 enter，策略周期调用 step，退出时调用 exit。外层使用现场映射将模型坐标转换成发送坐标。step 抛出异常时由外层切入已有安全状态。纯数学层可独立编译，ARM64 集成状态见 manifest。

~~~bash
g++ -std=c++17 -O2 runtime/cpro_math_replay.cpp -o /tmp/as2w-math-replay
python runtime/cpro_tape_text.py replay/flat-sensor-tape.npz | /tmp/as2w-math-replay
python runtime/cpro_tape_text.py replay/slope-sensor-tape.npz | /tmp/as2w-math-replay
~~~

如需用本次 Isa​ac Lab/Isaac Sim 环境复跑，设置 ISAAC_PYTHON 为支持 isaaclab AppLauncher 的 Python 启动器路径，然后运行 bash reproduce.sh replay flat /tmp/as2w-flat-replay。训练环境版本与训练配置见 source。

C++ 原生 ONNX 回放使用 runtime/cpro_onnx_replay.cpp，依赖的现场 OrtRunner 头文件已附在 runtime/include。该文件保留原版权头和现场已有的动态 batch 处理。外部 ONNX Runtime C++ SDK 依赖版本为 1.22.0，x86_64 的原生推理与完整目标对拍结果位于 replay/*-cpro-onnx.json，库文件 SHA 另附。ARM64 运行结果见 manifest。

设置 ORT_DIR 为本机对应架构的 ONNX Runtime 1.22.0 SDK 目录，从交付包根目录运行。

~~~bash
g++ -std=c++17 -O2 -I runtime/include -I "$ORT_DIR/include" \
  runtime/cpro_onnx_replay.cpp "$ORT_DIR/lib/libonnxruntime.so.1" \
  -Wl,-rpath,"$ORT_DIR/lib" -o /tmp/as2w-onnx-replay
python runtime/cpro_tape_text.py replay/flat-sensor-tape.npz | /tmp/as2w-onnx-replay model/policy.onnx
python runtime/cpro_tape_text.py replay/slope-sensor-tape.npz | /tmp/as2w-onnx-replay model/policy.onnx
~~~

## 训练起点与精确源码复跑

每阶段的实际初始化权重分别附在 model/stage-1-initialization.pt、stage-2-initialization.pt、stage-3-initialization.pt，SHA 与对应配置、源码路径列在 manifest.stage_initializations。所选最终输出另在 model/policy.pt。各阶段源码哈希索引位于 source/stages/stage-N/source-sha256.json，对应 source-snapshot 下的文件。源码快照保持训练时字节内容。

设置 ISAAC_PYTHON 为已配置 Isaac Lab 训练环境的 Python 启动器，输出目录须尚未存在。下面命令按本次实际配置训练，会产生完整运行。

~~~bash
bash reproduce.sh base /tmp/as2w-base
bash reproduce.sh recorded-stage 1 /tmp/as2w-dr1
bash reproduce.sh recorded-stage 2 /tmp/as2w-dr2
bash reproduce.sh recorded-stage 3 /tmp/as2w-dr3
~~~

recorded-stage 从本次记录的对应阶段输入权重独立复跑，使用该阶段源码、种子、随机化级别和优化配置。要从自选 checkpoint 接续，可使用 reproduce.sh stage LEVEL CHECKPOINT RUNNER_INDEX OUTPUT_DIR。训练中的噪声、延迟和推扰是随机状态，复跑后的采样与轨迹受仿真和 GPU 版本影响，实际环境版本见 source。


## 仿真环境与启动器

实际安装版本见 source/training-environment.json。本次是 Python 3.12.14、Isaac Sim 6.2.0-rc.14、Isaac Lab 28.0.0、rsl-rl-lib 5.5.1。Isaac Lab 工作目录没有可读取的 git commit，因此用已安装分发版本标识框架；本项目源码另有逐文件快照。交付包不含 Isaac Sim 二进制和完整 Python 环境，也未证明这些版本能在另一台服务器重新安装。解包回放与训练检查是在原服务器的独立目录完成，不能替代新服务器安装验收。

完成对应 Isaac Sim / Isaac Lab 环境安装后，可使用随包启动器。它保留本次清理外部 Python/共享库路径、指定虚拟环境 Python、关闭自动 ROS 环境注入的实际行为。以下路径须替换为新服务器真实安装位置：

~~~bash
export ISAAC_SIM_ROOT=/path/to/isaac-sim
export ISAACLAB_PYTHON=/path/to/isaaclab-venv/bin/python
chmod +x source/isaac-python-environment.sh
export ISAAC_PYTHON="$PWD/source/isaac-python-environment.sh"
"$ISAAC_PYTHON" -c 'import sys, importlib.metadata as m; print(sys.version, m.version("isaaclab"), m.version("rsl-rl-lib"))'
~~~

然后按上面的 reproduce.sh 命令运行。换版本后的物理轨迹和训练收敛需重新验收，不将环境检查或一次训练检查写成完整训练复现。
