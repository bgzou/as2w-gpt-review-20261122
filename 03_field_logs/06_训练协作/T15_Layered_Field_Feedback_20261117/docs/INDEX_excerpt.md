# INDEX T15 条目摘录（项目调试日志 04_调试记录/INDEX.md）

以下为部署方项目总账中与本候选相关的全部阶段记录原文（S0 收包→S1 parity→S2 sim2sim→
S3 C++/ARM64→zero-B→摇杆→塌陷定位→stiff 候选→三次验证→本反馈包）。每条含日期、执行环境、
命令、实际输出摘要、原始日志路径与判断。

## 2026-11-16 深夜：T15 Layered（as2w-layered-t15-20261117）S0 收包验收——SHA 72/72、离线校验 8/9（1 项为 numpy 版本伪失败）、判定 PASS 进 S1

- **对象**：训练同事新交付 `部署参考/as2w_layered_rolling_test/as2w-t15-layered-delivery-20261117`（44 MB/72 文件，T15 监督 BC 腿策略 65→14 + LayeredWheelController 独立轮控制器）；验证 run 包 `as2w-layered-mujoco-validation-20261117`（scripts/eval/reports/扁平部署包）一并接收。参照《策略交付到实机全流程》S0。
- **完整性**：SHA256SUMS 72/72 全过；onnx `ec7b9dcf…` + **外部数据 .onnx.data `d1909d07…`（173 KB，成对分发，部署拷贝时不得改名分离）**；opset 17，65→128×3 ELU→14 纯前馈。
- **包自带校验**：verify_offline **8/9**——唯一失败 `4_obs_builder: wrap` 为校验脚本 float32/float64 精确等号在 numpy 1.x（本机）与 2.x（训练机 NEP 50）下的语义差异，数值探针 wrap 误差 ≤1.1e-07（float32 舍入），判定伪失败；smoke 5/5 PASS（需在扁平布局 `deployment_package_completed/` 运行，嵌套包内 README 命令因布局不符会报 MISSING）；fixed replay 16 例 worst 5.364e-07 PASS。训练侧原报告 9/9 ALL PASS。
- **合同差异清单（对 R4000/R1199/armd）**：①obs[0:3]=(cmd−[0.4,0,0])**×8 命令增益**（本 run 特有）；②obs[42:58] 上一动作=**14 维有效动作的 16 通道语义展开**（轮=K/16·(tanh(u_c)+tanh(u_y)·side)，u 为控制器覆写后值）；③动作 14 维、腿=Defaults+0.35·action **无任何 clip/滤波/补偿**、轮通道弃用改 LayeredWheelController（u_yaw 与 wz_cmd **反号**、teacher 表重播种、5 s 看门狗 hold-last）；④命令无滤波直入 obs（区别于 R1199/R4000 的 0.12 低通）；⑤重力 Rᵀ[0,0,−1] 站立 z=−1（与 r4000 parity 级实现同款）；站姿参考训练用 Defaults[0,0.8,−1.6]（report 12）。
- **接收训练方如实缺口**：PhysX 前进误差 20–49% 未达 15% 门限（真机不以前进速度为先期判据）；目标 shadow/ARM64 加载/真机链路 not_run（即本流程 S3/S4 待办）；评估脚本依赖训练服务器模块未随包（S1/S2 按 armd 先例用交付模块+包内 MJCF 重建）。
- 详见 `logs/20261116_t15_layered_s0_receive.md`。交付包未改动。下一步 S1：独立 Python 移植 + 录带逐帧 parity（≤1e-6）。

## 2026-11-16 深夜二：T15 Layered S1 移植对齐 PASS——训练 run 快照核清全部合同疑点，独立移植 6150 帧逐帧 parity ≤8.5e-07

- **训练 run 目录补交**：`部署参考/as2w_layered_rolling_test/as2w-yaw-physics-aware-retrain-20261115/`（source/semantic 训练 env 快照、reproduce.sh、oracle/、eval/）。逐行核清 S0 遗留疑点：①`AS2W_COMMAND_GAIN=8` 为 run 实值（reproduce.sh:62），obs[0:3]=(cmd−[0.4,0,0])×8；②obs[10:22] 减参考=default_joint_pos **(hip 0, thigh 0.8, calf −1.6)**（init_state.joint_pos，与 R1199/R4000 族一致；交付 MuJoCo 自验用的 stand_reference() 只是近似）；③训练 reset=Defaults 姿态+基座 0.37+策略在环 settle；④零速轮记忆 episode 首帧即捕获→部署 was_zero=False 初始化与训练语义一致；⑤腿动作无 clip（Isaac JointPositionAction 无 clip 配置）；⑥轮组 target=K·tanh(u_c)+K·tanh(u_y)·side、执行器腿 55/2 轮 0/2；⑦robust "expanded" 的观测滞后/噪声/动作延迟/域随机化为训练增强非部署合同（交付矩阵干净链全过）。分层部署的 prev 反馈=控制器覆写后的有效 14 维（交付参考链语义，与训练分布一致因控制器即教师 law）。
- **独立移植**：`deploy_side_tools/t15_port.py`（numpy，obs/轮控制器/包装全链）。**录带**：交付参考模块原样 import + 交付 MJCF + 重建 set_gains（腿 55/2、轮 kd2）+ Defaults 姿态高度二分 + 策略在环 settle；11 工况 6150 帧（含 6s 命令失联后恢复），全工况 upright≥0.97、方向符号正确。**parity**：obs 1.06e-07 / u_common 1.74e-07 / u_yaw 2.23e-07 / omega 8.48e-07 / 腿 3.72e-08，状态失配 0 帧，**PASS（门限 1e-6）**。差异源=参考 torch float32 vs 移植 float64。
- **踩坑（已修，重要）**：部位序默认角错写进腿序名字循环导致初态完全错位，曾误判"交付 MJCF 轮-底盘自碰撞"——实际 Defaults 姿态无碰撞、策略 settle 完美（h=0.350, up=1.000）。策略序（part-major）vs 腿序（leg-major）双序并存，赋值一律按名字查表。
- 详见 `logs/20261116_t15_layered_s1_parity.md`。产物在 `部署参考/as2w_layered_rolling_test/deploy_side_tools/`。下一步 S2 MuJoCo 矩阵复现（同判据：yaw ratio [0.70,1.30]、摔倒/非轮触地/力矩饱和≤0.10、前进误差单列）。

## 2026-11-17 凌晨：T15 Layered S2 sim2sim PASS——部署链 MuJoCo 矩阵 154/154 全过（训练方精确判分），数值与交付几乎重合

- **链路**：t15_port（S1 parity 已过）= 部署链，交付 MJCF + 重建 set_gains（腿 55/2、轮 kd2）+ Defaults 姿态高度二分 + 策略在环 settle（训练 C5 入口语义，训练源码 init_state 0.37+Defaults 实证）。MuJoCo 3.17。
- **结果**：基础 26×5（s311×settle2/3/5 + 512/777）全过；扩展弧线 E 8×3 全过；seq120 12 段全过；release 松杆 vx→−0.0002 全停。A 直线 vx_err **3.3–4.0%**（交付 3.4–3.9%）、B/C ratio 与交付同小数位、h_min 0.325–0.347（交付 0.32–0.35）、sat/非轮触地全零、upright≥0.967。前进直线达 15% 门限；E(−0.10,±0.12) 弧线 vx_err ~29%（交付 ~17%，同性质超出）。
- **判分修正（重要）**：训练方 verdict **不 gate support**（仅记录）——我首版自设 0.90 门槛误判 3 例 E 弧线；按其 mujoco_matrix_26.py verdict 逐行核对后 154/154 全 PASS。E(±0.20,±0.12) support 0.82–0.87 低于交付 0.907–0.960，如实记录（无门限影响）。
- **与交付差异**：入口按训练语义（Defaults+策略 settle）而非其 stand_reference 近似；rates 直读（其脚本有 MuJoCo 双旋转偏差）；ONNX 前向。差异下数值仍重合，跨链一致性成立。
- 详见 `logs/20261116_t15_layered_s2_sim2sim.md`；产物 `deploy_side_tools/s2_*`（154 例 JSON+聚合 CSV+seq 逐帧）。
- **进 S3**：C++ 适配（t15 控制器+CMake+deploy.yaml+as2w_publish 分支+现场入口），x86 CTest/audit + S1 带 C++ parity；ARM64/CPRO 同步待 CPRO 重新上电（HDMI 事件后现场已下电）。

## 2026-11-17 凌晨二：交付清单 v3 → v3.1（T15 交付复盘增补；纯文档，无设备/源码动作）

- 用户指出：T15 包按 v3 清单交付，但部署方 S1/S2 仍多处理解只能去训练 run 原始源码找答案，要求更新清单把需求沟通清楚。已在 `02_现场操作/AS2-W训练到部署交付清单_v3_通用核心与按需模块.md` 就地升版 **v3.1**（原文结构保留）：
  - C3/C4/C6/C7/C8 核心表逐项强化（运行开关 env 变量实值表、默认角 16 个数、验收脚本 import 全随包+增益最终生效值表、判分口径可复算、**传感器级**回放带、非 NN 分层组件参考实现）。
  - 新增 **§2.5 打包自检十项**（每项对应 T15 真实卡点）：①开关实值表（AS2W_COMMAND_GAIN 默认 1/run 8 教训）；②默认角/站姿参考数字（两套参考并存二义教训）；③参考实现 import 随包（mujoco_diff_sweep 未交付教训）；④README 快速命令打包后实跑（smoke 布局教训）；⑤校验跨 numpy 版本容差（check4 伪失败教训）；⑥传感器级回放带（T15 只交 obs 级教训）；⑦非 NN 组件对拍材料（LayeredWheelController 教训）；⑧ONNX 外部数据成对约束（.onnx.data 教训）；⑨判分 gate vs 记录量（support 教训）；⑩MuJoCo 参考实现已知偏差注明（qvel[3:6] 双旋转教训）。
  - §6 handoff 模板新增 `runtime_env_overrides`/`default_joint_pos_reference`/`non_nn_components` 字段；§7 发训练同事话术同步。
- 优先级分级：1/2/6/7 不随包必然导致部署方翻训练源码；3/4/5 打包自跑可发现；8/9/10 省沟通。**下次同事按 v3.1 交包，S1 不应再需要训练 run 目录。**
- 本阶段仅文档修改，未动源码/CPRO/狗。T15 主线（S3 C++ 适配）继续进行。

## 2026-11-17 凌晨二：T15 Layered S3 C++ 适配 x86 全链完成——控制器/发布端/双二进制 CTest 9/9+回归 12/12、C++ parity ≤3.7e-07 PASS、重新打包；ARM64 待 CPRO 上电

- **新增**：`deploy/robots/as2w/t15/`（t15_controller.{hpp,cpp}、t15_replay.cpp、CMake 默认全 OFF；zero-B 发布端 + 30s cap 有界摇杆候选两个构建）+ `models/t15/`（deploy.yaml 冻结合同全量 pin + onnx/onnx.data 原名成对）。load_config 硬校验全部冻结常数/包络≤0.20/0.12/**gyro_filter_hz≠0 拒绝**（训练合同无滤波，滤波=新版本）/onnx+sidecar 双 SHA256（自带 SHA-256；工程内首个外部数据 ONNX）。
- **发布端**（common/as2w_publish.cpp 11 处 `AS2W_POLICY_T15` 分支）：joystick 映射共享、单次 zero-b-trial 门、T15 专属 POLICY_DIAG（raw14/effective14/u 通道/watchdog/zero 标志）、walk cap 30s、**命令不进发布端 slew**（训练合同阶跃命令；armD 同款处理）。
- **验证**：x86 CTest **9/9**（T15 JOYSTICK MAPPING/GUARD INJECTION/AUDIT PASS）；回归构建（r214/R4000/armd/getup1199）**12/12**；**C++ parity 5500 帧**（硬件 float32 输入）：obs 1.36e-07 / 腿 1.13e-07 / 轮 3.67e-07 / u 通道 ≤6e-08，全 ≤1e-6 **PASS**。过程修三处真实缺陷：相位 float32 常数→double；轮 wrap float32（大轮角 ~4e-6 舍入）→double（r4000 先例）；参考 prev_action off-by-one。
- **SHA 事件（load_config 校验拦截真问题）**：交付包 `.onnx.data` 真实哈希 `…c188ec5e`（SHA256SUMS/manifest 正确、文件完好），但 handoff.md/C1_C8_AUDIT.md 写 `…c188ec5c`（末位笔误）；按 handoff 写入的 deploy.yaml pin 被哈希校验正确拒绝。已改 pin 真实值；v3.1 清单新增**第 11 项**（跨文档 SHA 逐字一致自检）。此事件实证外部数据 ONNX 配对校验设计有效。
- **现场入口**：`tools/run_t15_trial.sh` + `run_t15_field_session.sh`（六步预检链；t15|t15-dry-run|zero-b|zero-b-dry-run 白名单）；Windows 会话注册 `-Policy t15`（运动必须 `-ConfirmMotion`、强制 `--debug`）；`build_on_cpro.sh` 加 `AS2W_T15_BUILD`。**重新打包**：cpp 包 `36efcfc9…`、SDK 包 `be292581…`（含 t15 全部文件，已验证 tar 内清单）。
- **x86 二进制 SHA**：zero-B `c6158de9…`、joystick `9c9f1008…`、replay `2935c160…`；as2w_publish.cpp `abc82fb7…`。
- **阻塞项**：CPRO 因 HDMI 黑屏被现场下电（11-16 记录"不再向 CPRO 发任何命令"），**ARM64 构建/CTest/audit、双端口 SHA、真机 dry-run 待设备重新上电并获现场同意**。S4 阶梯每步仍须当次明确同意。
- 详见 `logs/20261117_t15_layered_s3_cpp_adaptation.md`（含实机风险注意：腿动作极小运动主要在轮、前进速度不作先期判据、watchdog 5s hold-last 需人值守）。

## 2026-11-17：T15 ARM64 双端闭环 + 首次 dry-run 全过（含一次 SSHFS 写丢失事故与恢复；零狗动作）

- **SSHFS 写丢失事故（重要教训）**：ARM64 首链构建时链接失败（main 引用 `as2w_r214::` 符号）→排查发现权威树 `as2w_publish.cpp` 丢失全部 11 处 `AS2W_POLICY_T15` 编辑（期间曾现 EIO/posix_spawn 失败；服务器侧缓存曾把编辑版喂给 rsync 镜像，Windows 侧权威文件却是编辑前版本，且两次观察哈希不同=缓存不一致）。**恢复**：以 x86 镜像中的编译验证版（`eaf03404…`，含 8 处 T15 分支）为准，**绕过 SSHFS 从 Windows 侧 Copy-Item 写回**，双侧+延时复验一致。其余 SSHFS 编辑文件逐一核对：会话 ps1/INDEX/清单/日志/其余源码全部完好，仅此一文件受损。**此后规则：发布端等多轮编辑的关键文件，写后立即 sha256+内容 grep 双验；权威落位优先走工作站侧写。**
- **修正 S3 记录**：此前 INDEX 所记 as2w_publish.cpp `abc82fb7…` 实为回退版哈希；**正确 SHA `eaf03404…`**（x86/ARM64/权威树/包内四处一致）。重新打包 cpp 包 `24d73765…`（发 CPRO 前先在 caolin 侧解包验证 publish.cpp=eaf03404）。
- **ARM64 完成**（CPRO ubuntu rootfs，cmake 3.22/g++ 11.4）：`build-t15-zerob` CTest **8/8**、`build-t15-joystick` CTest **9/9**（含两个 startup-audit AUDIT PASS）；**ARM64 C++ parity 5500 帧全过**（obs 1.23e-07 / 腿 1.13e-07 / 轮 3.67e-07 / u 通道 ≤6e-08，与 x86 同精度；夹具经 file send 推送、输出拉回对拍）。双端 SHA 全表核对一致：publish `eaf03404…`、controller `8fe9d636…`、hpp `b950731e…`、deploy.yaml `93851e38…`、onnx `ec7b9dcf…`+data `…c188ec5e`、trial/session 脚本一致；**ARM64 二进制：zero-B `ce49966d…`、joystick `27624c27…`、replay `ea59b33b…`**。
- **首次 dry-run 通过**（`20261116-172005_t15-dry-run-dry-run`，只读、零狗动作）：step1 SHA+AUDIT PASS；step2 LowState 1058 包/s、mode_machine=2；step3 狗处于**已释放状态**（name 空，继续）；step4 2 秒 2099 包（~1050Hz）、16 轴 mode 全 0（无其他发布端）、趴下小腿门槛 yes（calf≈−2.69）。停在"would start monitoring + T15 launcher"。CPRO 新上电（HDMI 事件后首次），只读复核未碰显示驱动。
- **S4 前置全部就绪**。下一步 zero-B（首次狗动作）须现场当次明确同意后执行；命令将用 `-Policy t15 -Mode zero-b -MonitorSeconds 240 -ConfirmMotion`。

## 2026-11-17 00:50–17:22：T15 zero-B 首次真机动作通过——B 一次接受、2s 零速交接+7.7s 零命令保持无跳变、全程零保护位；现场观感"按B没反应"恰为 zero-B 通过标准

- **会话 `20261116-172141_zero-b`**（CPRO 时间 17:21，现场当次同意，操作员手持遥控器；ARM64 zero-B 二进制 `ce49966d…`）：预检全过→Passive ready→**A** FixStand→FixedStand（B gate 数据：pose_error=0.305<0.35、wheel_dq=0.012、upright=0.992、quiet_dq=0.028）→**B 一次接受**→2s PolicyHandoff→PolicyWalk 零命令保持 **7.7s**（操作员按 Y 结束）→Brake→FixSquat→Passive→RC=0 干净退出。**现场报告"按 B 好像一点反应都没有"——判定为设计预期**：zero-B 验证的就是策略零命令接管无跳变/无塌陷/无异动，狗保持站立即通过。
- **量化（全部优于或持平仿真预测）**：腿目标单帧步进 **max 0.00012 rad**（仿真预测 ≤0.0024；对比 R1199 训练合同版 0.003、旧故障版 0.27——本狗史上最平滑 B 交接）；四轮目标**恒为 0.0000 rad/s**（u_common/u_yaw/vx_served 全零，LayeredWheelController 零命令输出符合设计）；raw BC 腿动作 max 0.0047（仿真 ≤0.013）；策略腿目标距 Defaults ≤0.0017 rad；整机 up_z 全程 min **0.9825**（门槛 0.85，最差瞬时≈10°出现在起立插值段）；看门狗从未过期、零命令标志全程激活；**摇杆隔离验证**：保持期内操作员推 ly ±0.9 均被忽略（vx 恒 0，zero-B 构建不映射摇杆）。
- **保护流 9449 行**：motorstate/warning 全 0x0；modes 全 1 仅出现在发布窗口（本端 16 轴受控正常态），窗口外全 0；board_state0 bit0=0x1 与昨日 armd 干净会话运行期基线一致（正常态位）。debug 全链完整：debug_lowcmd 21769 行 dropped=0、debug_policy 515 行、policy_diag 488 行（1 行截断可忽略）、low_capture 24782 帧。
- **运维记录**：本会话由服务器 SSH 驱动工作站入口，本地流中断把包装进程杀掉导致自动归档未执行——已手动 `file recv` 补收 7 文件 33MB 并核完整性。**建议**（沿用 R4000 先例）：T15 运动会话改由操作员直接在工作站 PowerShell 跑入口（输出实时可见），服务器侧不作为运动 会话的驱动端。
- 原始档案：`logs/field_sessions/20261116-172141_zero-b/`（7 文件）。**下一步（有界摇杆：LY vx/RX wz 包络 0.2/0.12、30s cap、首次真实运动命令）须现场当次明确同意后执行。**

## 2026-11-17 00:56–00:58：T15 有界摇杆两次真机完成（速度链全通、零保护事件）；定位"往后塌"=后腿 PD 跟随下垂（策略目标正确，kp55 不够）

- **会话 `20261116-172821_t15`（joyA）与 `20261116-173005_t15`（joyB）**（现场当次同意，操作员工作站直跑，自动归档成功）：两次完整链 A→B（gate pose_error 0.314/0.316、upright 0.991/0.990）→PolicyWalk **29.9s/13.0s**（joyA 到 30s cap 自动刹、joyB 按 Y 刹）→Brake→ReturnStand/FixSquat→Passive→**RC=0×2**。**零 FIELD_SAFETY、零保护位、零报警**。速度域全开且丝滑：vx 命令达 ±0.200 满量程（685+454 有效帧）、wz 达 ±0.117/±0.115、轮目标 ≤4.16 rad/s、vx_served 满域跟随。现场观感"速度控制比较丝滑"与数据一致。
- **"往后塌"定位（两次同签名）**：策略腿目标全程钉死 Defaults（后大腿 0.800–0.803、小腿 −1.600~−1.601，**策略没有命令塌**）；塌的是**实际关节跟随**——后大腿实际最小垂到 **0.436/0.448 rad**（目标 0.8，**跟随误差 0.31–0.32 rad**），前后腿差 −0.29/−0.27 rad，up_z 瞬时最低 0.923（≈22.7°，仍远高于 0.85 门），出弯即回 0.94+。机制=运动中轮驱动+载荷下 **kp55/kd2 静态刚度不足**，与 getup1199 爬坡实测"kp55 失速→85/3.2 开膝通过"、armD 19–20Hz 摆振同族：目标不动、实际跟不上。
- **T1 vs T2 判断**：本策略 obs[10:22]=q−Defaults、动作偏置=Defaults，与站姿链解耦；塌发生在 PolicyWalk 而非 A 起立段（A 段 T2 站立良好）。**换 `--stand t1` 不解决 PolicyWalk 的 PD 垂塌**（t1/t2 只改 A 段固定站姿态），且 t1 未经 T15 链离线验收。**有意义的候选方向**（都=新版本，须全离线链+当次许可）：①腿增益 kp55→85/kd2→3.2（与训练合同增益是否可改须训练方确认——训练用 55/2，改增益即新动作包装）；②40ms 腿目标低通（getup1199 rev5/armD-LP40 已有先例，但本策略目标本就平滑，无命令抖动可滤，预期无效）；③接受现状（垂而不倒、可恢复、保护全零）作为当前档事实。
- 档案：`logs/field_sessions/20261116-172821_t15/`、`20261116-173005_t15/`（各 7 文件）。**T15 摇杆档当次验收通过；任何增益/滤波变体须另行逐次许可。**

## 2026-11-17：T15"往后塌/拖地"根因定位——非电机、非策略发疯，是 kp55 静态刚度撑不住训练默认姿态（sim2real 缺口）；当前档暂停行走测试

- **量化闭环（两次摇杆会话数据一致）**：塌**与速度无关**（|vx| 0–0.05 与 0.18–0.25 档后大腿实际同为 ~0.54/0.59，最差 sag 均出现在 vx=0 站立时）——静态姿态问题而非运动动力学。后大腿实际 0.560/0.595（目标 0.8，**塌 0.24–0.26 rad**）、后小腿实际 **−1.925/−1.932**（目标 −1.6，**折 0.33 rad**）→ 机身后仰 22.6°（up_z min 0.923）→ 后小腿/脚跟拖地。
- **电机无罪（实测力矩）**：塌陷窗口后大腿 tau_est 均值 **+13.3/+11.6 Nm、峰 +17.7 Nm**——远低于力矩限（MJCF 大腿 effort ±60，硬件更高），无任何保护位。前大腿仅 2.4–2.6 Nm。**静态自洽验证**：kp55×塌陷误差=13.2/11.3 Nm ≈ 实测 13.3/11.6——PD 完全按指令工作，塌到"55×误差=后腿载荷力矩"的平衡点为止。
- **策略无罪**：BC 输出残差 ~0.005，目标钉死训练默认角 (0.8,−1.6)，忠实执行训练合同。
- **根因**：训练合同"低刚度 kp55/kd2 + 低默认站姿 (0.8,−1.6)"在真机后重载（前后不对称为已知事实）下的静态平衡就是塌 0.24–0.33 rad。MuJoCo 26kg 轻模型塌得少（h≈0.35 稳定）；真机后重塌出可见拖地。**R4000 家族同姿态不拖地的原因：它们有训练侧站姿补偿包装（rolling_stand_bias 标定 ±0.38——恰好同量级的塌陷被补偿掉）；T15 纯 BC 合同无补偿机制。**这正是交付 known_gaps 未覆盖的 sim2real 发现。
- **t1-stiff 评估**：t1/t2 只改 A 段固定站姿态；塌发生在 PolicyWalk 段（策略目标与 A 站姿无关，2s 交接后收敛到同一塌陷平衡）——**换 t1 治不了拖地**。现场观察"零速切换无感"=交接插值按设计工作，与站姿选择无关。
- **处置**：①当前档**暂停行走测试**（拖地=持续非轮接触，按训练自身判据即 FAIL 状态，磨损小腿/脚跟）；②候选修复=腿增益 55→85/2→3.2 独立候选（getup1199 rev2 同夜同机制真机先例；预计塌 0.24→~0.16，**减轻未必根除**；改增益=新动作合同，须训练方确认+全离线链+逐次许可）；③根修=反馈训练方：真机需要更高默认站姿（参考实测自然站立 thigh≈0.60/calf≈−1.23）或部署侧站姿补偿包装——均为新版本。

## 2026-11-17 01:0x–01:2x：T15-STIFF（腿 PD 85/3.2）候选全离线链完成——压力工况塌陷 −80%/零非轮触地、双端 parity 逐位、回归二进制 SHA 不变；待当次许可实测

- **教师数据审计（回答"负载变了还知道平衡吗"）**：`bc/teacher_proportional.npz` 9000 样本中**教师腿动作逐样本恒为 0**（12 轴 std/max 全 0）——T15 腿策略是纯姿态保持器，无任何负载反馈行为可被破坏；"平衡"由轮控律+PD 被动稳定性承担。**这同时把 stiff 候选的语义风险降到最低**：改增益不改变策略要执行的任务，只让执行逼近 MuJoCo 验证假设（理想跟踪）。转向能力来自 LayeredWheelController 轮差速，与腿策略无关——均已记入给训练方反馈要点。
- **压力工况构建**：MuJoCo 标称 26kg 模型塌陷不显（无判别力），扫描质量/COM 找到**真机等效工况 `质量×1.3 + COM −0.03m`**——完美复现实测塌陷签名（后大腿 0.587 vs 实测 0.56–0.60、后小腿 −1.939 vs −1.93）。
- **85/3.2 对照矩阵（真机等效压力）**：hold/fwd/bwd/turn±0.12/arc/release 全工况——塌陷 0.24→**0.03 rad（−80%）**、后小腿 −1.94→−1.73、up_min 0.923→**0.99+**、h_min 0.28→0.358；**非轮触地全 0（不拖地）**；四轮载荷（直线 61.7N）全过 5N 门。标称 26kg 工况无副作用（塌陷本就小）。原地转向瞬时轮卸载为 55/2 与 85/3.2 共有固有现象（非新副作用）。
- **实现**：独立宏 `AS2W_T15_STIFF_CANDIDATE`（hpp 注释含完整根因+验证记录），只改腿目标携带的 kp/kd（55/2→85/3.2）；obs/动作/策略链零改动——x86 stiff replay vs 合同构建 **obs/目标逐位 0.000e+00** 证实。A 段固定站保持合同 55/2（`config.stiffness`），handoff 期间 kp 由 blend_targets 自然插值无跳变。新 `build-t15-stiff` + `run_t15_stiff_trial.sh` + 会话脚本 `t15-stiff|t15-stiff-dry-run` 模式 + Windows 入口注册（运动必须 `-ConfirmMotion`）。
- **双端验证**：x86 CTest **10/10**+audit PASS（含 T15 JOYSTICK MAPPING）；CPRO ARM64 CTest **9/9**+stiff replay 5500 帧全过+**双端数值 parity ≤4.8e-07**；**回归关键证据：zero-B `ce49966d…`/joystick `27624c27…` 重编后二进制 SHA 与已验收版逐位一致**（宏门控无泄漏）。重新打包 cpp `3d0c4df1…`；stiff ARM64 二进制 `4c4b883c…`；controller.cpp `db745dd7…`/hpp `7ab86c31…`。
- **预期与判据（下次实测）**：B 交接后后腿不再可见塌陷/拖地；如仍有残余下陷（MuJoCo 预测塌 0.03–0.06 rad）应远小于当前 0.24–0.33 且不触地。风险：kd2→3.2 增加阻尼力矩（动态段 |dq|≤2 rad/s 时 ≤6.4 Nm，叠加 kp85×误差≤~24 Nm 稳态，仍远低于 35 warn 门）；handoff 段 kp 渐变 55→85 预计不可感知。
- 产物：`/tmp/opencode/t15_work/stiff_candidate_sim.py`（扫描+对照矩阵+接触门，将拷入 deploy_side_tools）、`stiff_sweep_55.json`、`stiff_matrix_result.json`。
- **下一步：t15-stiff 首测须现场当次明确同意**（A 起立→B 交接→杆驱动 30s 包络内前后/转向→A/Y 收尾），判据=无拖地、姿态目视挺直、保护流零。

## 2026-11-17 01:2x：T15-STIFF 三次真机完成——塌陷修复实证 0.24→0.04 rad、零保护、零拖地、速度响应与 55/2 无差异；用户整体评价"不抗扰不鲁棒"成立（策略本质=纯姿态保持器），反馈包交训练侧

- **三次会话 `20261116-175600/175724/175829_t15-stiff`**（当次同意，操作员工作站直跑，三次 RC=0）：B gate 三次全过（pose_error 0.30–0.33、upright 0.987–0.994）；两次到 30s cap 自动刹、一次 Y 提前刹；**三次保护流全零（motorstate/warning 0x0，共 36205 行）**。
- **塌陷修复定量（PolicyWalk 段精确切分，tick 对齐）**：后大腿塌 **0.24（55/2 两次实测）→ 0.036–0.057 rad**（三次 stiff，与 MuJoCo 压力工况预测 0.03–0.06 一致）；后小腿 −1.93→**−1.728~−1.746**（离趴下域 −2.6 以上，**零拖地帧**）；up_z min 0.923→**0.989/0.996/0.994**；后大腿力矩 3–5 Nm、峰 8.2 Nm（warn 35 的 23%）。起立/趴下段（55/2 合同段）仍有正常姿态变化，非本候选范围。
- **速度响应无退化**：三次 vx 满量程 ±0.200（有效帧 1084–1294/1493）、wz ±0.115–0.117、腿目标步进 max 0.00372 rad（与 zero-B 同量级、远低于 0.2 warn）——85/3.2 无抖动放大、无动态副作用。交接段 kp 55→85 渐变无异常事件。
- **用户最终评价"这个策略不抗扰、不鲁棒"——数据完全支持，且已定位为设计本质**：教师数据腿动作逐样本恒零（BC 学到的是恒定姿态保持器），无负载反馈行为；平衡完全依赖轮控律+PD 被动刚度。抗扰/变负载站姿调节需要训练侧换教师（腿通道非零、含姿态反馈），不是部署侧能补的。
- **STIFF 候选判定**：作为部署侧缓解措施**通过**（塌陷 −80%、零拖地、零副作用、回归零泄漏）；但**它不是策略能力提升**，只是执行刚度修正——鲁棒性问题移交训练侧。
- 下一步：打包 standalone 反馈包交训练侧（`06_训练协作`），含全部六次运动会话原始数据、双版对比、教师数据审计、根因报告。
