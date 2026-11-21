# INDEX 摘录：getup1199 全部阶段记录（逐字，按时间序）

> 来源：`部署测试/04_调试记录/INDEX.md`（工作站权威总账）。仅摘录 getup1199 相关节。

## 2026-11-13 下午：起身 getup1199 实机部署前置验证 + 部署方案（离线，无狗动作）

- 对象：recovery 最新权重 deep-side model_1199（SHA 4fe19b8e…）+ ONNX 970e692e…。四组 MuJoCo 实验（证据 `logs/20261113_getup1199_deploy_precheck/`）：
  1. **实机真实趴姿提取**（114335 会话尾段）：Y 趴姿 calves −2.73~−2.77、thighs≈1.0、高 0.215m——与训练 prone 库（thighs 2.4–2.9、calves −1.5~−1.7、躯干平贴）完全不同，且 **calf −2.77 超策略 90% 包络（−2.55）与 URDF 限位（−2.65）**。
  2. **入口消融 8 配置×40 例**：真实 Y 趴姿直接起身 0/40（全部 joint_angle@t0 被安全层拒绝）；经 FixProne 预趴姿（库中位）或库 prone：起身 40/40、严格门 24/40（失败全为末段推扰时序）。**预测守卫与 effort 0.8 对成功率零影响**（合同安全要素不损失成功率）。
  3. **FixProne 过渡预演**（A 键 crouch→prone 6s 插值 kp55/kd2）：3 种子终点误差≤0.083rad、水平、接触峰 650–911N（躯干落地瞬态）——预趴姿动作可行。
  4. **部署安全阈值实测**（16 例逐帧）：腿步进 max 0.61rad、>0.2 最长 6 连（**现行行走档 warn 会误杀，getup 档调 0.35/0.75**）；腿 dq max 6.5（>5 最长 3 连，不触发）；**腿 tau max 仅 37.2Nm**（不触发，远低于 R284 事故值；此前 ratio=1.0 为轮瞬态触自身剪幅）。
- 部署与测试方案：`02_现场操作/getup1199实机部署与测试方案_20261113.md`（42 维 obs 合同含**重力符号与 R1199 相反**等差异点、动作链六步规格、A=FixProne/B=PolicyGetup/Y/X FSM、getup 专用安全档、S4 四级阶梯；v1 只做趴地起身不做侧倾）。S3 C++ 编码尚未开始；任何实机动作仍逐次当次同意。

## 2026-11-13 傍晚：getup1199 真实趴姿直接起身的补充诊断 + 开膝中间态（离线，无狗动作）

- 用户追问"真实数据直接输能不能起"后的修正实验（证据 `logs/20261113_getup1199_deploy_precheck/` 实验5/6）：
  1. 前述"真实趴姿 0/40"是安全审计在 t=0 判入口超包络即终止，**策略从未运行**——结论已修正。
  2. 用模型关节范围（calf −2.7677）表达真实趴姿、仅放开入口包络检查（命令仍剪 90%）：
     **33/40 起身、28/40 完整通过、90% 包络 0.07s 内自动回归**；另 7/40 为起始膝贴死硬限位的
     t=0.01s 边界伪影（真实 Y 趴姿恰处贴限位状态）。策略能力上不怕蹲伏趴姿；
     "中间态"的本质是合同问题（入口超包络+膝压限位）。
  3. **新增 FixCrouch-lite 方案**（2.5s 仅开膝 −2.77→−2.43、不后扫腿、躯干不落地、
     接触峰 69–73N、终点全回包络，预演 3 种子稳定；随后起身 40/40）——方案文档已更新为
     v1 首选（替代动作更大的 FixProne，后者留作备选）。
- 方案文档 `02_现场操作/getup1199实机部署与测试方案_20261113.md` 第 0/2/3 节已相应修订。

## 2026-11-13 傍晚：getup1199 测试流程改为单键 B 起身链（设计确认，离线）

- 用户确认测试操作只保留：趴姿下按 B 一次（起身）+ Y（趴下）。方案文档 FSM 已改为单键四段内部链：入口检查→FixCrouch-lite 开膝 2.5s（包络内自动跳过）→包络复检（未达自动 Passive 不硬起）→PolicyGetup 15s 上限→FixedStand 保持→Y/30s 自动趴下；A 在该构建禁用、X 随时急停。当次同意的告知口径已写入文档（"按 B 后开膝约 2.5s→起身 3–5s→站立保持"）。S3 编码未开始。

## 2026-11-13 下午：getup1199 S3 起步——C++ 控制器线 + 三层 parity 完成（离线，无狗动作）

- 新建唯一源码线 `deploy/robots/as2w/getup1199/`（控制器/回放/CMake 默认 OFF）+ `models/getup1199/`（ONNX `970e692e…`，deploy.yaml 全合同常量），根 CMakeLists 注册 `AS2W_GETUP1199`。x86 构建（ORT1.22 镜像）零告警；修复两个移植 bug（ORT Unowned 悬垂临时、staged 斜坡 [起点,时长] 语义）。
- **三层 parity（550 帧预趴姿起身全程，干净传感器+部署估计门夹具）**：逐帧 oracle **1.55e-6 PASS**（python 训练实现重放 C++ obs/blend 时间线，隔离包装数学）；闭环全带 max 4.4e-2，根因=ONNX fp32 基线（训练方自身 1.9e-6 同源）经 previous-action 反馈放大 + ≤1 帧门边界翻转；功能等价=估计门版 40/40 与真值门一致。判据与复现脚本见 `logs/20261113_getup1199_deploy_precheck/README.md` 实验7。
- 运维：SSHFS 资产在高负载下 `from_xml_path` 109s，评估改用本地拷贝。
- **下一步**：共享发布端 `AS2W_POLICY_GETUP1199` 分支（FixCrouchLite/PolicyGetup 状态机、getup 安全档、入口/成功门、A 禁用）→ CTest/offline-audit → 会话脚本/Windows 注册 → ARM64/CPRO 同步 → dry-run。

## 2026-11-13 傍晚补充：getup1199 发布端集成规格固化（离线）

- 共享发布端 as2w_publish.cpp 的 getup 集成规格（State 追加、getup 安全档、B 入口链、FixCrouchLite/PolicyGetup 细节、debug/CTest/会话件）已逐条写入方案文档第 5 节，作为下一段机械执行的输入；本段不改动共享安全文件（长会话尾部不赶工）。已完成物：控制器线+parity+估计器验证+方案文档，全部归档。

## 2026-11-13 晚：getup1199 发布端分支完成——S3 代码全部落地，x86 全绿（离线，无狗动作）

- `common/as2w_publish.cpp` 落地 `AS2W_POLICY_GETUP1199` 单键链（B=入口门→FixCrouchLite 开膝→PolicyGetup→FixedStand→Y；15s 上限；A 禁用；一次会话一链），getup 专用安全档（仅步进 0.35/0.75），`fill_targets` 轮位置保持扩展（kp!=0 分支，行走状态不受影响）；CMake `AS2W_GETUP1199_PUBLISH` + `AS2W_ENABLE_GETUP_TRIAL` + `--getup-trial` CLI。
- 验证：getup 构建 **CTest 8/8**（audit PASS）；**R214+R4000 回归构建 CTest 8/8**；闭环 parity 4.4e-2（根因已归档）+ **oracle 1.552e-6 PASS**（最终二进制）。
- **过程事故与教训**：staged 斜坡修复曾误写镜像未回写工作区，被后续同步覆盖回退（干净构建复现定位后在工作区修复）。方向规则（工作区→镜像单向）必须遵守。
- 剩余 S3：会话三件套 + Windows 注册 + 重打包 + CPRO/ARM64 同步构建 + dry-run。全部完成后按方案 §3 阶梯逐次申请实机动作。

## 2026-11-13 晚：getup1199 S3 收尾——会话三件套/Windows 注册/重打包/CPRO ARM64/双端 parity/CPRO dry-run 全部完成（狗上电趴姿；运动待现场当次同意）

- **会话三件套**：`tools/run_getup1199_trial.sh`（--getup-trial + 强制 --debug + 30s 站立保持）、`tools/run_getup1199_field_session.sh`（六步预检链+趴姿小腿基线门+监测+Debug 归档校验）、Windows `运行现场策略会话.ps1` 注册 `-Policy getup1199 -Mode getup|getup-dry-run`（运动需 -ConfirmMotion；getup 无低通候选）。
- **CPRO 同步与 ARM64**（经工作站 zoubu@192.168.10.34，USB 目标 `0123456789ABCDEF`——WiFi 8710 Offline，tconn 失败）：10 文件 tar 同步、源码/模型 SHA 双端一致；`build-getup1199` ARM64 全构建零错误、**CTest 8/8、--offline-audit AUDIT PASS**，二进制 SHA `9bdbb2a3…b52c12`；**ARM64 parity = 4.429e-2 与 x86 完全一致**（同 ORT 图跨架构一致）。
- **重打包**：`准备现场包.ps1`（经英文包装 as2w_package.ps1）cpp 包 SHA `c0edef37…5b69`、SDK 包 `9fe26cd2…84f4`。
- **CPRO dry-run（20261113-173702）通过**：step1 audit PASS；step2 **LowState 1040 包/s、mode_machine=2——狗已上电**，实时姿态 calf≈−2.748~−2.769/thigh≈0.97~1.02（趴姿，与部署前分析一致）、IMU 水平；step3 原厂 ai 模式（dry-run 按设计停在此处）。档案 `logs/field_sessions/20261113-173702_getup-dry-run/`。
- **运动会话就绪**：命令 `运行现场策略会话.ps1 -Policy getup1199 -Mode getup -MonitorSeconds 150 -ConfirmMotion`（当前用 `-Target 0123456789ABCDEF` USB 通道）。按键语义：B 一次=开膝→起身→站立保持→Y/30s 趴下；A 禁用；X 急停；一会话一链。**实机动作须现场人员当次明确同意并握原厂遥控器/物理急停。**
- 证据：ARM64 parity/构建输出、dry-run session.log 已归档；转输中转文件已清理。

## 2026-11-13 夜：getup1199 首晚三次实机会话——三个同源集成 bug 全部定位修复，链路已通到开膝段（狗零动作、无风险暴露；下次可直接测）

- **会话1 `175045_getup`**：B 无反应。根因=FSM 按键总门缺 `getup_trial`（FSM 线程提前 return）。已修。
- **会话2（流式）**：B 收到但入口门拒绝 `up=0.999985 (max 0.6)`——`up` 方向写反（趴地本就水平，应为 `up>0.6` 排除侧躺）。已修（`entry_min_up_z`）。
- **会话3 `181327_getup`**：B 通过、正确进入 FixCrouchLite、开膝 2.5s 命令全部发出（1240 帧 LowCmd）——但**写线程状态分派清单缺新状态，整段发的是 Passive 阻尼帧（kp=0）**，狗不动，2.5s 后"envelope NOT reached"安全回 Passive 停机。实测核对：state=9 帧恒 q=测量值/kp=0/kd=3，leg_step=0。已修（写线程 fill_targets 清单加 FixCrouchLite/PolicyGetup）。
- 教训：向共享发布端加新状态，必须 `grep State::FixSquat` 逐一核对全部状态清单站点（state_name/audit 矩阵/写线程分派/policy_safety_state/FSM 总门/squat_by_timeout/remote_timeout 矩阵）。
- **修复后状态**：x86+回归 8/8，CPRO ARM64 重建 8/8+AUDIT PASS，二进制 SHA `fa4d74c1…`，`as2w_publish.cpp` `ce185513…`。**整条软件链（按键→入口门→FixCrouchLite→写线程）已实机验证到状态切换，仅剩 kp=55 命令的物理执行未确认。**
- 遥控探针教训：1Hz 采样抓不住 200ms 按键边沿，"全零"不能证明遥控故障（操作员质疑正确）。
- **下次测试入口文档**：`02_现场操作/getup1199下次bringup操作单_20261113.md`（含预期日志顺序、失败分流、第一观察点=开膝段 kp=55、证据索引、遗留事项）。

## 2026-11-16 傍晚：getup1199 修复版实机首测——B→开膝链物理打通，FL 轮 kp25/kd4 保持振荡触发安全刹停与主板保护自复位（狗全程趴姿未离地；未获新许可前禁止再跑当前二进制）

- **背景**：接续 2026-11-13 夜三个集成 bug 修复（二进制 `fa4d74c1…`）。今日现场人员在场、当次同意后执行 `-Policy getup1199 -Mode getup -MonitorSeconds 150 -Target 0123456789ABCDEF -ConfirmMotion`（USB 通道；WiFi 因 wifi-x MLO 故障仍 Offline，见上节）。
- **链路恢复**（会话前）：CPRO 当日重启过，eth0 丢 IPv4，只读 GetState 报 `rc=3104`、SDK2 报 eth0 不存在；按既有流程核对 `cpro_host_net.sh` 双端 SHA `ee5ead45…` 后 `apply`（仅补 192.168.123.100/24+子网路由），`.161` ping 3/3；dry-run（`20261116-160849_getup-dry-run`，已回收）六步预检全过、趴姿基线门通过。
- **会话**：第一次启动因服务器侧工具超时杀掉 SSH 链（狗零动作，仅 kd≤3 阻尼帧；其 CPRO 残档 `20261116-161010_getup` 待回收），此时狗停留 released 态（GetState 3104 属预期）。第二次启动 `20261116-161825_getup`（已自动回收至本机 `logs/field_sessions/`）：already-released 分支正确跳过 SelectMode，step4 质量门过，Passive ready 727 帧，现场按 B 一次。
- **核心结果**：11-13 bug3 修复**实机确认**——FixCrouchLite 段腿 kp=55/kd=2 命令真实发出、目标自实测平滑起步（单帧步进 ~1e-4 rad），开膝物理开始（小腿 ~0.02 rad 后被安全刹停）。狗全程趴姿、未离地、无摔倒风险。
- **事件链**（debug_lowcmd.csv 22757 帧 + low_capture_fast.log + low_protection.log 三方对齐）：
  1. FixCrouchLite 入口有 **10 帧（20ms）全零 LowCmd**（FSM 线程 ~50Hz 与写线程 500Hz 竞态：状态原子切换后目标要等下一拍；阻尼闪断，未触发故障，属代码疣）。
  2. 轮 kp=25/kd=4 锁角保持接入 ~300ms 后，**FL 轮（槽13）单独发散成力矩限幅振荡**：dq 从 ±0.05 指数增长至 ±4.3 rad/s、tau ±13.6 Nm、持续 ~150ms；另三轮（12/14/15）全程安静。位置误差全程 <0.005 rad，力矩全部来自 kd×dq 项（4×4.15≈16.7 Nm，与守卫打印的 wheel_pd=16.6886 精确吻合）。
  3. 软件守卫按设计工作：wheel_pd 超硬门限 15 → FIELD_SAFETY_HARD ×2 → count=5 SOFT_TRIP → Brake。
  4. **主板保护级联（现场听到的报警）**：Brake 后 ~2s `board_state0` 0x1→0x1000→0x1001，末端 `motorstate_or=0x8`、`board_state0=0x4001001`，LowState 流冻结于 tick 3657140；发布端 FOC ack 超时故障退出（22756 帧，RC=1）。
  5. 狗主板**自复位**：随后只读探针 tick 计数清零、GetState 恢复应答、`CheckMode form=1 name=ai`、`fsm_name=ai_damping`，16 轴回趴位（calf −2.75~−2.77）、tau≈0。与 2026-11-09 两次 LowCmd 保护报警同类：低层命令流事件触发主板保护并复位。
- **根因判定**：① 直接原因=**轮位置保持 kp=25/kd=4**（`deploy.yaml:38-39` 合同值）：getup1199 新引入的增益组合，R1199 系实机轮从未用过 kp>0 位置保持（一直 kp=0/kd=2），仅 MuJoCo 理想执行器验证过；真机 FOC 轮环路裕度不足，FL 单轴临界发散（是否 FL 轴硬件边际/编码器问题待下次物理检查）。② 20ms 全零帧竞态是独立代码疣，与振荡起点无直接因果。③ 腿侧命令与守卫无任何异常。
- **下一步（均未执行，未获新许可前不做任何实机动作）**：修全零帧竞态（状态切换同步填首拍目标）；轮保持增益改用实机已验证配方（kp=0/kd=2 或小 kp 渐增）——按训练合同硬规则该差异视为新版本，需 bump policy 修订、回放、audit、ARM64 重建后再逐次申请；回收 `20261116-161010_getup` 残档；`02_现场操作/getup1199下次bringup操作单_20261113.md` 已加暂停警告；下次接触狗时物理检查 FL 轮（编码器/轮毂间隙/异物）。

## 2026-11-16 夜：getup1199 轮配方修订（B 预填阻尼帧 + 轮全链 0/2）——离线全套+MuJoCo 对齐+CPRO ARM64 全绿，待实机当次许可（无狗动作）

- **修订内容**（复刻实机已验证配方，全在 wrapper 层）：① `as2w_publish.cpp` B 处理器在状态切换前同步预填阻尼帧（A 链同款，堵 50Hz FSM/500Hz 写线程竞态的 20ms 全零帧；两条分支均覆盖）；② FixCrouchLite 段轮 kp=25/kd=4→**kp=0/kd=2**；③ `models/getup1199/deploy.yaml` `wheel_hold_kp/kd` 25/4→**0/2**（PolicyGetup 段轮随动）。腿路径零改动（kp=85 训练合同、FixCrouchLite 55/2 均不动）。25/4→0/2 为**部署偏离训练合同**，已在 deploy.yaml 内注释记录现场证据；策略腿命令路径经 550 帧回放逐位一致直接证明不受影响。
- **x86**（镜像 `/tmp/opencode/getup1199_build/`）：getup 构建零错误、CTest 8/8、`--offline-audit` AUDIT PASS；回放 550 帧 `GETUP1199_TARGET` 行与 11-13 归档**逐位一致**（`getup1199_replay_r2.txt` 与 `getup1199_replay_final.txt` 仅差一行帧计数尾行；parity 脚本 4.429e-2 为 11-13 已记录的 ONNX fp32 闭环基线，非回归）。回归构建 `b-regress` 增量+CTest 8/8（改动全在 `AS2W_POLICY_GETUP1199` ifdef 内）。
- **MuJoCo 对齐**（`eval_recovery_damped_wheels.py`：轮 kp=0/kd=2±10%，腿 85/3.2 不变；与 11-13 A1/C1 同配置同种子对照）：bank_prone **24/40 通过、0 起身失败**（基线 24/40、0）；prepose **24/40、0**（基线 24/40、0）。阻尼轮滚转峰 0.718/0.623 rad（锚定基线 0.123/0.114），腿 dq 与目标步进统计同量级——起身对自由滚轮不敏感，通过率与基线完全一致。注意：MuJoCo 是理想力矩执行器，不能复现真机 FOC 伺服失稳，本对齐只验证"轮自由滚动不破坏起身功能"。
- **CPRO ARM64**（两跳同步 `as2w_publish.cpp`+`deploy.yaml`，宿主路径 `/data/ubuntu_in_hm/ubuntu_rootfs/root/as2w_deploy/...`；hdc `file send` 直写 `/root/...` 会报 no such file——必须用宿主全路径）：双端源码 SHA 一致（`a0603c99…`/`5fb727ca…`），`build-getup1199` 增量构建零错误、CTest 8/8、AUDIT PASS，**新二进制 SHA `8b1437750f8932b24f83c7f73420f87991af252ab0f1d3760671ad17191c238e`**。
- **实机预测**：开膝段轮纯阻尼（与 R214 起身同配方，FL 发散机制被移除）；残余风险=FL 轴硬件边际（下次接触狗时物理检查）与固件保护位语义未文档化——首测仍按有界试验对待，开膝段听轮嗡鸣立即 X。
- 证据：`logs/20261116_getup1199_wheel_recipe_revision/`（两组 diagnostic.json+日志+回放+变体脚本）；操作单 `02_现场操作/getup1199下次bringup操作单_20261113.md` 顶部已换为修订现行版。**未执行任何实机动作；下次测试须现场当次明确同意。**

## 2026-11-16 晚：getup1199 轮修订实机复测——报警消失、开膝物理执行，但 kp=55 力矩不足未达包络（安全回 Passive）；当晚完成增益修订 rev2（离线+MuJoCo+ARM64 全绿，待当次许可）

- **会话 `20261116-171913_getup`**（轮修订二进制 `8b143775…`，现场当次同意）：预检链全过、B 正确接受、**预填生效（再无全零帧）、轮全程安静（修复的 FL 振荡完全消失）、零安全事件、零报警**，发布端干净退出 RC=0。现场观察"关节动一点就没反应"与日志吻合：开膝 2.5s 命令与目标完全正确（小腿平滑步进至 −2.43），但实测小腿仅从 −2.76 开到 −2.70，斜坡终点包络复检未过（需 ≥−2.546），按设计安全回 Passive、不起身。狗零风险。
- **根因（三方数据钉死）**：四个小腿在斜坡终点全部处于力矩平衡——目标 −2.43/实际 −2.70/误差 0.266 rad，kp=55×0.266=14.6 Nm 与实测 tau_est 14.4-14.7 精确吻合，dq≈0.02 rad/s（近失速爬行）；髋被反推开 0.02 rad（3.6-3.8 Nm 顶不住）、机身 pitch 仅 −0.36°（未抬起）、轮基本未滚（地面摩擦锁定）。**纯力矩饥饿：kp=55 的 PD 摆不平趴姿负载**。对照 11-13 MuJoCo 探针（kp=55 六秒内开到包络、机身抬 3cm）：仿真最大小腿力矩仅 10.8-12.4 Nm，真机负载约为仿真 1.5 倍+（CPRO 背包增重/真实地面），属 sim-real 力量差，非代码缺陷。
- **rev2 修订**（`as2w_publish.cpp` FixCrouchLite 段三处，策略路径零改动）：
  1. 腿增益 55/2 → **合同值 85/3.2**（`config.stiffness/damping`，与紧随的策略段一致；满误差力矩 28-40 Nm 仍低于守卫 warn 35 的瞬态仅出现在完全卡死情形→SoftBrake 安全中止，与本次行为同路径）。
  2. 小腿目标 −2.43 → **−2.30**（仍在 90% 包络 [−2.546,−0.963] 内；留出负载/kp 稳态误差余量：L≤21 Nm 时稳态仍过包络，实测 L≈14.6）。
  3. 斜坡终点后**保持目标 1.5s 持续复检包络**再决定（抗爬行裕量），仍失败才回 Passive。
- **MuJoCo 复验**（`fixcrouch_lite_probe_r2.py`，部署精确配置 85/3.2+阻尼轮+2.5s+小腿−2.30）：标称质量 3 种子全部在斜坡终点进包络（小腿 −2.38~−2.44，抬升 0.268m，接触峰 78N，小腿力矩峰 12.35Nm）；**躯干配重 +50%/+80%（39/47kg）仍全部进包络（小腿 −2.25，余量 0.3 rad）**。起身段证据沿用实验 2/5（B2 从 calf−2.426 起身 40/40；实验 5 证明 −2.77 也能 33/40，起身对起点不敏感，包络门才是约束）。
- **构建**：x86 CTest 8/8+AUDIT PASS+550 帧回放与 11-13 逐位一致（第三次确认策略路径不变）；CPRO ARM64 CTest 8/8+AUDIT PASS，`as2w_publish.cpp` 双端 SHA `a0963d9a…`，**新二进制 `280faf3f3eb96b5bcd98f0ac817865505e8e516c9d72ef1f21b1172efccaacbd`**。
- 证据：`logs/20261116_getup1199_crouch_gains_revision/`（探针脚本+run_lite85 三组质量工况+回放）；会话档案 `logs/field_sessions/20261116-171913_getup/`；首场被杀残档 `20261116-161010_getup/`（无 B、零动作、RC0）已回收。**下次测试同入口命令不变，须现场当次明确同意。**

## 2026-11-16 深夜：getup1199 rev2 实机复测（后腿差 0.013-0.030 rad 卡包络门）+ rev3（小腿目标 −2.22）全链绿，待当次许可

- **会话 `20261116-182438_getup`**（rev2 二进制 `280faf3f…`，当次同意）：B 接受、预填生效、轮安静、零安全事件零报警、RC=0 干净退出。**rev2 大幅推进**：85/3.2 把小腿从 −2.76 开到 FR−2.500/FL−2.488/RR−2.576/RL−2.559，稳态力矩前腿 16-17/后腿 22-23 Nm（vs rev1 的 14.6 Nm 失速）——**前腿已过包络门（−2.546），后腿差 0.013-0.030 rad**，保持 1.5s 复检仍未过（真静态平衡非爬行），按设计回 Passive。狗零风险。
- **负载不对称新事实**：后腿稳态负载（22-23 Nm）比前腿（16-17 Nm）重 ~40%，与 11-14 FixedStand 文档"后小腿力矩 14.5-14.9 vs 前 6.6-7.2 Nm"前后不对称一致——趴姿下重心偏后，后膝开角更难。
- **rev3**（用户选定方案）：小腿目标 −2.30→**−2.22**（仍在 90% 包络内；由实测稳态反推 q_pred=−2.49~−2.52，余量 0.03-0.06 rad）。x86 CTest 8/8+AUDIT PASS+550 帧回放逐位一致；MuJoCo 开膝探针（部署精确配置）标称/×1.5/×1.8 质量 9/9 进包络；**全链仿真**（阻尼轮评估器、rev2 实测失速姿态+扰动 8 起点）：**8/8 起身成功 0 失败、终高 0.383-0.385m**（5/8 完整过末段推扰门，与基线比例一致）。CPRO ARM64 CTest 8/8+AUDIT PASS，`as2w_publish.cpp` 双端 SHA `19790525…`，**二进制 `b0a7bfff9f4b1bdd7d92930d1322e08613c7b1194a61b4362f84c7850c36a03d`**。
- 证据：`logs/20261116_getup1199_rev3/`（探针 r3 脚本+三组质量 JSON+回放）；会话档案 `logs/field_sessions/20261116-182438_getup/`。**测试命令不变；须现场当次明确同意。**

## 2026-11-16 深夜二：getup1199 rev3 实机首破起身段——包络通过、策略真实起身至 h=0.29 后被软件 warn 门刹停；现场报告明显关节抖动=策略命令 25Hz 交替（训练固有）

- **会话 `20261116-182855_getup`**（rev3 二进制 `b0a7bfff…`，现场当次同意，操作员在场按 B）：
- **链路实机全部打通**：B→FixCrouchLite 开膝→**`envelope reached → PolicyGetup`（rev3 的 −2.22 目标生效）**→策略驱动真实起身 **1.28s 内 h_est 0.22→0.29（站立门 0.31，差 2cm）**、jerr 0.90→0.62 收敛→SOFT_TRIP→Brake→ReturnStand→FixedStand→自动 FixSquat 趴下→RC=0 干净退出。**零板级报警、零硬件保护位、零 X 急停**（与前两次板级保护复位完全不同，纯软件安全中止）。现场确认"大幅起身了"。
- **刹停原因**：`FIELD_SAFETY_SOFT_TRIP state=PolicyGetup leg_pd=36.38/leg_tau=36.16 Nm 连续 5 帧超 warn 35`。对照 11-13 实验 4：**仿真起身段腿 tau 峰本就 37.2 Nm**——35 warn 门沿自行走档未随 getup 档调整（当时只调了 leg_step），真机负载略重即触顶。hard 50 未接近，硬件 calf 极限 81（R284 事故 73.4）。
- **关节抖动（现场报告"明显关节抖动"，操作员质疑正确）量化定位**：实测 12 腿全抖——小腿 dq 符号翻转 37-55 Hz、髋 11-17 Hz，100ms 窗 dq 峰峰 2-4 rad/s（可见可闻量级）。**根因=策略命令自身交替跳**：`debug_policy.csv` 80 拍显示 calfFR 命令 −0.387/−0.545/−0.336/−0.576/−0.336/−0.576…**±0.24 rad 完美交替（25Hz=50Hz 策略率之半）**，交替自相关 −0.89~−0.99（hipFR/calfFR/hipFL/hipRL/thighRL/calfRL 六关节强交替，hipFL 19 步>0.2）。**非部署缺陷**：11-13 仿真同款命令抖动已存在（实验 4 单帧步进 0.61、>0.2 连续 6 帧、sim dq 峰 6.5）——训练策略经 previous-action 反馈的极限环，MuJoCo 理想执行器"跟得住"，真机把它暴露成可见抖动。实测 dq 峰 2.1/tau 36.2 均在仿真包络（6.5/37.2）内。
- 已核实共享发布端 `PolicyTargetLowpass` 只接在 PolicyWalk/Brake 状态，**未接 PolicyGetup**；若要滤波需新代码+按训练合同硬规则重验证（R1199 40ms 低通为隔离候选先例，需逐次许可）。
- **下一步（用户已选定）**：getup 档 leg_tau/leg_pd warn 35→45（hard 50 不动；依据=仿真 37.2/实测 36.2/硬件 81/R284 事故 73.4，余量充足），全离线链复验后待当次许可复测；关节抖动如实记录，本轮不加滤波（策略固有、持续仅 ~1.3s、在仿真包络内），滤波作为后续隔离候选议题。

### getup 档 warn 门修订（当晚完成，全链绿）

- `as2w_publish.cpp` FieldSafetyProfile：getup 构建 leg_tau_warn/leg_pd_warn 35→**45**（声明内 `#if AS2W_POLICY_GETUP1199` 与 leg_target_step 同模式），hard 50 不变；audit 预测 PD 注入幅值 0.7→**0.88 rad**（kp55×0.88=48.4 Nm，新旧 warn 门下均落 warn<hard 区间，5 帧触控行为不变）。
- 验证：x86 CTest 8/8（首跑 7/8 失败即注入幅值旧设计，修正后全绿）+ AUDIT PASS + 550 帧回放逐位一致；回归构建 8/8（注入改动对行走构建同样成立：35<48.4<50）；CPRO ARM64 CTest 8/8+AUDIT PASS，双端源码 SHA `9a8df53d…`，**二进制 `484d18946eccb10e0fcd01bc505b734eea20995bf635f166be6e2dbe9bdc3281`**。
- **预期**：上次链路在 h=0.29 被刹停时 leg_pd=36.4<45，同工况将不再触 SOFT_TRIP，起身应能走完 h≥0.31 成功门→FixedStand 保持→自动趴下。关节抖动会依旧存在（策略固有），现场应有心理预期。**复测须现场当次明确同意。**

## 2026-11-16 深夜三：getup1199 rev4 实机完整成功——B→开膝→起身→站立保持 30s→自动趴下全链走通（狗首次策略起身站立）；rev5=40ms 滤波候选就绪待测

- **会话 `20261116-184500_getup`**（rev4 二进制 `484d1894…`，现场当次同意，操作员按 B）：
- **完整链全程走通**：B→FixCrouchLite（包络通过）→PolicyGetup 起身 **h_est 0.22→0.376m、up=0.9996、jerr 0.90→0.04**→`[PolicyGetup] → FixedStand (success; hold=100 frames h_est=0.376007)`→站立保持 30s→自动 FixSquat 趴下→RC=0。**零 SOFT_TRIP、零板级报警、零硬件保护**（warn 45 生效：起身段 tau 峰 ~33 未触门）。**这是该狗第一次由策略驱动自主起身并站稳。**
- **抖动实测（滤波前基线）**：起身段（5.24s 含保持）12 腿仍全抖——dq 峰 1.3-2.5、100ms 峰峰 1.0-4.9、flip 27-88 Hz；站立保持段显著更稳（dq 峰 ~0.8-0.97、峰峰 0.5-1.1）。现场反馈"还是抖，加滤波"。
- **rev5=40ms 腿目标低通候选**（按 R1199 实机验证配方，新动作合同）：
  - MuJoCo（`eval_recovery_lp40.py`，rev4 实测入场姿态 8 起点）：**8/8 起身 0 失败、终高 0.383-0.385 与无滤波完全一致**；命令步进 max 0.61→**0.022-0.028**（交替跳被完全滤平）、dq 峰 1.96-3.06。
  - 实现：`PolicyGetup` 段 `controller.step()` 输出接 `PolicyTargetLowpass`（τ=40ms 只滤腿、进入段首拍 reset 无切换跳变）；CMake 加 `AS2W_ENABLE_POLICY_LOWPASS=1`；**运行守卫：getup-trial 不带 `--policy-lowpass-ms 40` 直接拒绝**（rev4 无滤波是另一份动作合同，不共存一个二进制路径）；`run_getup1199_trial.sh` 默认 `--policy-lowpass-ms 40`。
  - 验证：x86 CTest 8/8+AUDIT PASS+守卫实测（无 40 拒绝/带 40 过参数门）；CPRO ARM64 CTest 8/8+AUDIT PASS。三文件双端 SHA 一致（publish `daeebd4c…`/trial `8d355a74…`/CMake `fe0a3259…`），**rev5 二进制 `7f074dcf067fa8e22ad32d6ec086afdb7ca03369c002842db34311eec576c86a`**。
- 证据：`logs/field_sessions/20261116-184500_getup/`（完整链档案）；`/tmp/opencode/getup1199_rev_damped/run_lp40_rise.json` 已拷入 `logs/20261116_getup1199_rev3/`；`eval_recovery_lp40.py` 同目录。**rev5 复测须现场当次明确同意；预期=全链成功+起身段抖动显著减轻（命令级 20 倍抑制）。**

## 2026-11-16 深夜四：getup1199 rev5（40ms 滤波）实机复测成功——全链走通且抖动大幅下降；getup1199 起身链当夜收官

- **会话 `20261116-185131_getup`**（rev5 二进制 `7f074dcf…`，现场当次同意，操作员按 B）：B→开膝（包络过）→滤波起身 `FixedStand (success; hold=100, h_est=0.377069)`→站立保持 30s→自动 FixSquat→RC=0。零 SOFT_TRIP、零板级报警。**与 rev4 成绩持平（0.376 vs 0.377），抖动大幅改善。**
- **抖动对比（同一分析口径，PolicyGetup 全窗）**：dq 峰值约降一半（hipRL 2.48→1.27、hipFR 2.25→1.06、其余同量级）；100ms 峰峰降 60-75%（最差 hipRL 4.93→1.58、hipFR 3.72→0.95、典型小腿 2.1-2.4→1.2-1.4）；起身时长 5.24→5.38s（滤波代价可忽略）；flipHz 变化不大（残余为低幅高频纹波，幅值已在 0.1 rad/s 量级）。现场确认"明显改善"。
- **getup1199 当夜五版迭代收官**：rev1 轮纯阻尼+预填（消板级报警）→rev2 85/3.2（消力矩失速）→rev3 目标 −2.22（过包络门）→rev4 warn 45（过力矩门）→rev5 40ms 滤波（消抖动）——从"按 B 报警"到"策略起身站稳+干净退出"共 5 次实机会话全部逐次许可、零硬件损伤、零未遂风险。**现行合同=rev5：二进制 `7f074dcf067fa8e22ad32d6ec086afdb7ca03369c002842db34311eec576c86a`（publish `daeebd4c…`/trial `8d355a74…`/CMake `fe0a3259…`，getup-trial 强制 `--policy-lowpass-ms 40`）。**
- 待办（非阻塞）：站立保持段仍有轻微 dq 波动（峰 0.8-0.97，属 FixedStand 保持合同内）；起身段 25Hz 命令交替的源头在训练策略本身（previous-action 极限环），滤波是部署侧缓解，根治需训练方知晓——下次交付对齐时随证据包反馈；`AGENTS.md` 根规则下次会话更新 getup1199 现行状态。
- 证据：`logs/field_sessions/20261116-185131_getup/`（rev5 档案）。**后续任何 getup 变体（如滤波 τ 调整、缩短保持、Y 提前）仍须逐次当次许可。**
