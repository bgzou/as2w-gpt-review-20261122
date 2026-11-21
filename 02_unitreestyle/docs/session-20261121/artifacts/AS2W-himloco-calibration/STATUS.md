# HIMLoco 部署校准：第一阶段，2026-11-19

状态：确认交接观测反馈缺陷，已生成隔离源码候选。未编译、未同步 CPRO、未发送 LowCmd；尚未证明后仰已修复。

## 原始数据复核

以数据包实际 DEBUG_POLICY 行的 obs_count/state_count 动态解析，严格校验行长。requested 与 applied 各为 64 列；旧 an_tau.py 的 r[-128:] 首 64 列为 requested，不是 applied。

两次有策略帧会话共 800 帧（713+87），逐帧回查观测 [41:57]：

| 会话/阶段 | 与上一帧实际施加目标差 max | 与上一帧请求目标差 max |
|---|---:|---:|
| vx015 HANDOFF | 0.904319454 | 3.53e-7 |
| vx010 HANDOFF | 0.904360164 | 3.50e-7 |
| vx015 WALK | 3.53e-7 | 3.53e-7 |

数值是归一化观测差，不是 rad。发布端 HANDOFF 混合目标后未回填控制器。WALK 两者相同，因此旧 WALK 力矩表没有因取列问题失效。校正后的逐阶段 PD 需求见 field-audit.json；尚未对齐 tau_est，不得称为实测力矩。

实测交接前髋角约 ±0.20～0.28 rad，原摘要只列大腿/小腿平均值不足以描述初态。必须使用本次日志的全部逐轴角度和四元数。高度及根部平移速度无真值，不能默认为已测得。

## 隔离修复候选

candidate/ 保留当前本地权威源码的副本及 unified diff、SHA256。新增 accept_applied：独立存储施加目标反馈，不覆盖低通滤波内部状态。发布端 HANDOFF/WALK/BRAKE 完成混合/过滤后回填。直接调用控制器的无混合离线程序默认反馈自身输出，保留原正常步进语义。

这仍是控制线程的最终目标，不等同于已验证的 500 Hz DDS 电机收包；需要 debug_lowcmd 进一步核对。未修改权威源码/原始数据包，未完成 C++ 编译、CTest 或闭环验收。

## 复演纠偏

p9_checked.py 禁用旧 r11 隐式默认，要求显式 XML 和 URDF；校验 timestep × decimation = 20 ms；拒绝带 actuator 的 XML，避免 qfrc_applied 与执行器重复施力；使用 URDF 逐轴 effort 并明确不是硬件限幅证明；打印输入文件哈希。生成两次会话实测初态 JSON。

此脚本仅为初态诊断，尚未实现完整交接闭环、资产几何等价校验或真实命令回放；不得用它宣称 sim2real 已通过。原 p9 的 -2° 结果仍为旧资产/旧启动流程的结果。

## 已运行与下一阶段

- 已运行 audit.py、build_candidate.py、prepare_replay.py；Python 语法检查通过。
- 本机 PATH 未发现 clang++/g++；C++ 编译 not_run。
- 下一阶段：取得对应训练资产及执行器实际配置，核对 XML 几何/惯量/接触；取得本次最终 LowCmd，完成无策略命令回放与 tau_est 对照；同初态同交接流程比较原反馈与修复反馈。未完成这些之前，不把反馈缺陷认定为全部后仰根因，不修改增益或奖励。

复现：在工作区根目录，将 AS2W-official-stop-replay/.deps 加入 PYTHONPATH 后运行 python AS2W-himloco-calibration/audit.py。候选生成运行 build_candidate.py；复演脚本生成运行 prepare_replay.py。
