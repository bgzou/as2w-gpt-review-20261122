# RISE63 原厂 FixStand A 单独候选

日期：2026-11-20。用户指定将 RISE63 的 A/FixStand 改为本机原厂静态站姿与 Kp/Kd；用户确认髋角按原厂左右镜像。17:28 的 B 首试已触发软件硬门槛并伴 FOC 故障，因此本版本**只构建 A-only stand-trial，B 被二进制门控拒绝**。

## 参数来源与新旧区别

2026-11-19 原厂 `ai_stand_up` 的 `rt/lowcmd` 实测：硬件侧 FR/RR hip `+0.150`、FL/RL `−0.150`，四条腿 thigh `+0.600`、calf `−1.200` rad；腿 `Kp=200,Kd=4`，轮 `Kp=0,Kd=0.5,dq=0`。本项目映射的四髋符号均为 `−1`，因此发布端模型坐标、配置轴序 FL/FR/RL/RR 的髋目标是 `[+0.15,−0.15,+0.15,−0.15]`。来源：`AS2W悬空负载对照与Hip扫角_20261119/05_总报告_悬空负载诊断V2_20261119.md` 与原厂会话数据。

独立宏 `AS2W_RISE63_OFFICIAL_STAND_CANDIDATE` 使 A 从实测当前 q 以 200/4 腿 PD、0/0.5 轮 PD 在 3 秒内线性插值到该姿态；FixStand 不再用项目旧 B1 thigh +0.25/20/3 和 B2 85/3.2。FixedStand 保持同一目标与增益。策略模型、训练 reset 参考 `[0,0.65,−1.30]`、53 维观测、动作包装与策略 40/1 **未改**。这不是原厂内部起立曲线的逐帧克隆，只对齐公开录到的目标与增益；趴下退出仍是项目原有 FixSquat，200→35 增益过渡尚未实机验收。

## 构建与证据

- 唯一源码 `部署参考/cpp_deploy-as2w/deploy/robots/as2w/common/as2w_publish.cpp`；独立 CMake 目标 `as2w_rise63_publish_official_stand`；合同文件 `models/rise63/deploy-official-stand-candidate.yaml`，policy_id `AS2W_V6.3_from0_model_2000-official-stand-a-only-20261120`。
- caolin x86 目标编译通过、`rise63-official-stand-startup-audit` CTest 1/1；旧 hold15 目标增量重编与 CTest 1/1 通过。新目标用 `--zero-b-trial` 启动时明确拒绝 `RISE63 official-stand candidate requires A-only stand trial`。
- CPRO WiFi `192.168.10.75:8710` 连接后，发现原 `/root/as2w_deploy` 共享源码 SHA 与本机不一致；**未覆盖它**。把本机现场包 SHA `fdddef89605508aa0663c757a3d0c5b0934bade1af0fd2359002935ea5e5298e` 解压到新的 `/root/as2w_official_stand_candidate`，独立 ARM64 编译与 CTest 1/1 通过；新二进制 SHA `ccb4306ae74c80cf397db3c6d6cd5d972768f2c15e4499a4fe6692e9f7044d5a`。CPRO 新目录三份源码/合同 SHA 与工作站逐项一致。构建记录在 `../04_调试记录/logs/20261120_rise63_official_stand_candidate/`。
- 合同 YAML 仅作注释澄清后已单独同步 CPRO，SHA `a670df2ed9138616b19b3fd05d1c6cad13e4c1302380793545e43cfc29527f9d`，再次 CTest 1/1。最终重打包的现场源码包 SHA 见 `../01_现场部署包/SHA256SUMS.txt`（当前 `edddf3829f63c1687f11ba57b333f84cbe991a79dbef4ccfa09fb45105ba8a9e`）；二进制未因注释变化重编。
- MuJoCo **what-if**：用交付资产/actor，从训练 reset 初态在原厂目标 200/4（轮 0/0.5）保持 2.5 秒，再做 0.5 秒 B 插值、15 秒策略。起身后侧倾，轨迹首次越部署姿态阈值 t=4.98s，即 B 后 2.48s；原始 `rise63_official_reset15.npz`、脚本、summary、守卫回放同目录。这不是完整 A 起立复演，也不模拟吊带或实机 FOC；只表明**不能把原厂 A 参数视为 B 故障的现成修复**。

## 当前状态

没有运行新候选的任何真实 LowCmd。CPRO 机器人侧 `eth0 DOWN`，本机 hold15 的 Windows 运动入口因 B 首试故障已暂停，只读预检保留。恢复实机前需先核设备状态、保护位含义与控制权，并单独验收 A→退出链；B 仍需针对轮速和 FOC 故障另立修复候选与离线验证。
