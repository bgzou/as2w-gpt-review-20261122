# RISE63 B 后 50 秒保持候选

2026-11-20。按用户要求，只把 B 进入策略后的自动上限从 10 秒改为 50 秒；X 手动失能、A/Y 语义、固定站姿时长、训练模型、动作包装和增益未改。

- 候选 policy_id：`AS2W_V6.3_from0_model_2000-hold50-candidate-20261120`。
- 唯一维护源码 `部署参考/cpp_deploy-as2w/deploy/robots/as2w/common/as2w_publish.cpp` 在 `AS2W_RISE63_HOLD50_CANDIDATE` 编译定义下使用 `kRise63WalkCapSeconds = 50.f`；默认构建仍为 10 秒。
- 独立构建目标：`as2w_rise63_publish_hold50`；默认关闭。候选合同副本：`models/rise63/deploy-hold50-candidate.yaml`。
- 已部署 CPRO 二进制和 `D:\as2w_field\start_rise63.ps1` 仍为已验收的 10 秒版本。没有把 50 秒候选编译、同步或用于真机动作。

已用 `20261120_rise63_s1s2_evidence/harness/rise63_sim.py` 及交付模型/场景，在 Windows Python 3.13、MuJoCo/ONNX Runtime 环境按 0.5 s 交接、85/3.2 预备站姿、50 s 策略保持、`x` 退出运行两种初态（证据 `04_调试记录/logs/20261120_rise63_hold50_candidate/`）：

- `reset` 名义初态：运行至 50 s，末态重力 x≈−1、后轮约 255 N，未触发姿态门槛；这只证明该名义初态的仿真。
- `t2sag` 合成站姿下垂初态：策略阶段 t≈5.02 s 的姿态量越过 C++ 守卫阈值，t≈5.10 s 机身高度低于 0.30 m，t=10 s 高度约 0.10 m、重力 x≈+0.23。**仿真器没有实现 C++ 姿态守卫，因此这里只是轨迹越阈，并非守卫实际触发。**
- 同条件 `hold=10` 对照在同一时间出现相同下沉；到 12.5 s 自动退出前，10/50 秒轨迹 625 帧的关节位置和机身高度最大差均为 **0**。所以下沉不能归因于 50 秒上限。`t2sag` 也不是这台实机的 A 站立实测状态。

50 秒名义初态的仿真保持到上限；合成下垂初态暴露的是 **10 秒版和 50 秒版共有的入口/闭环问题**。这两例不足以验收 50 秒实机保持，也不能把共同失败误判为延长上限造成。**未做回放、x86/ARM64 构建、审计或真机验收；未同步 CPRO。**

## caolin 姿态守卫核对

caolin `/tmp/opencode/rise63` 的 `rise63_sim.py`、ONNX 和场景文件 SHA 与本项目副本一致；服务器以 MuJoCo 3.17.0、ONNX Runtime 1.31.1 重跑 `reset/50s`、`t2sag/10s`、`t2sag/50s`。原始逐帧轨迹与摘要已回收至 `04_调试记录/logs/20261120_rise63_hold50_candidate/caolin_guard50/`，SHA 与服务器逐文件一致。

按部署 C++ 的守卫逻辑逐帧回放：B 在 t=2.5 s `arm()`，前 1.2 s 豁免，t=3.7 s 起检查 `gravity_x<=-0.50` 且 `|gravity_y|<=0.60`。两条初态在 t=3.7 s 均已抬起，姿态符合阈值，**守卫不会拦截名义起身瞬态**。`reset/50s` 在 50 s 保持段内不越阈；`t2sag/10s` 与 `t2sag/50s` 都在 t≈5.02 s 因侧倾量 `gravity_y≈-0.650` 首次越阈（约 B 后 2.52 s），与保持上限无关。详情 `caolin_guard50/guard_results.jsonl`。

这个回放只判定**首次可能失能时刻**；MuJoCo 原仿真未执行 C++ 守卫，回放也没有模拟失能后的被动动力学、吊带或实机传感器。因此它不能替代真机结果，也不能据此断言 50 s 实机可保持。
