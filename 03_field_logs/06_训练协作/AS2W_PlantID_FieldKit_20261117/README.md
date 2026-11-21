# AS2-W 物理资产辨识：第一批同条件现场采集包

日期：2026-11-17。当前只开放 **T2 / 腿 55/2 / 轮 0/2 / 固定 PD Stand** 的重复采集。没有称重设备时，先测同一合同下的会话分布和进入历史敏感性。`run_t2_55.ps1` 默认只打印计划；必须现场人员对**该次具体动作**明确同意后，由操作员显式加 `-Execute -ConfirmMotion -ConsentRecord` 才调用现有已验收的 R214 stand-only 链。本包不会自动执行多次试验；每次只跑一条。其他案例在下方列为后续独立候选，不要拿其他策略的不同站姿/增益冒充同条件数据。

## 要回答的三个问题

1. 同一目标/增益/地面/装载和同一进入路径，真机 pitch、四髋与后小腿的**段间分布**到底多宽？原始记录的 T2 55/2 pitch −9.30…−5.19°，但来源混杂。
2. 若同条件仍出现两簇姿态，分簇是否与 `factory_ai_damping` 进入或 `released_after_prior_lowlevel_squat` 进入、起立前轮角/轮速、机身初姿、保护/限幅记录有关？
3. 用侧视频中的尺子取得机身绝对高度与前后差，避免仅凭画面主观判断“矮多少”。

没有四轮承重读数时，**质量、COM、轮地横向力与髋执行器受载柔顺不能被唯一分离**。这批数据是下一轮参数拟合和增益对照的门槛，不应直接用于宣布某个质量/COM 值为真。

## 现场条件

- 在同一块平整地面上给地面命名，如 `lab_floor_A`；记录所有附挂件、线缆/电池配置。每轮前都从稳定趴姿开始，记录入口为 `factory_ai_damping` 或 `released_after_prior_lowlevel_squat`。两类入口**分别分组**。
- 先做 3 次**同一入口**的 T2 55/2；建议间隔至少 30 s，确认每次已完全趴下和停止发布。不要在同一命令里自动串跑。电量能读到就填，读不到用默认 `-1`。首次从原厂 `ai_damping` 释放后，后续通常是 `released_after_prior_lowlevel_squat`；这时首轮单独成组，后续再采 3 轮同入口，不能把 1+2 算作 3 次重复。无需为了凑原厂入口而额外切换控制权。
- 操作员持原厂遥控器和物理急停，机器人四周清空；A 后不按 B、不推摇杆。每次 A→T2 约 3 s，固定 PD 保持最多 15 s，Y 或超时进入既有 FixSquat/Passive。若现场姿态异常、拖地、保护或抖动，按现场已验收的 X/物理急停处理，并保留失败日志。Ctrl+C 不能当运动急停。
- 如果初始为原厂控制，现有 launcher 可能先发原厂 stand-down，再 ReleaseMode；这个过程也包含在本次动作告知与许可中。脚本预检会检查 LowState、16 轴 mode 和趴姿门，失败即停止。

## 一次试验的命令

以下在项目根目录的 PowerShell 中运行；先只打印计划，确认本次动作与地面/装载记录：

```powershell
& '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/run_t2_55.ps1' -TrialId T2_55_01 -FloorId lab_floor_A -Attachments none -EntryId factory_ai_damping
```

若不确定当前控制权入口，可先用现有只读预检查 `step3`，再填写与实况一致的 `-EntryId`：

```powershell
& '部署测试/运行现场策略会话.ps1' -Policy r214 -Mode dry-run -Debug -Target 0123456789ABCDEF
```

现场人员针对这一次 A→T2 55/2→Y/15 秒超时链（含可能的原厂趴下和 ReleaseMode）明确同意后，操作员运行：

```powershell
& '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/run_t2_55.ps1' -TrialId T2_55_01 -FloorId lab_floor_A -Attachments none -EntryId factory_ai_damping -Target 0123456789ABCDEF -Execute -ConfirmMotion -ConsentRecord '现场当次同意：A到T2 55/2保持最多15秒，Y或超时趴下，可能先原厂趴下并ReleaseMode'
```

脚本会再次要求输入 `TrialId`，再调用现有 `运行现场策略会话.ps1 -Policy r214 -Mode stand -Debug`；后者负责二进制审计、设备预检、保护监测和拉回原始日志。记录 `TrialId` 的 metadata 在启动前就写入 `runs/`，即使现场链失败也留有本次条件。成功时还写入会话目录并运行离线分析。每次换新的 `TrialId`，例如 `T2_55_02`、`T2_55_03`。目前 USB 目标是上例；地址会变，不要沿用旧 WiFi IP。

输出在 `部署测试/04_调试记录/logs/field_sessions/<会话>/`：原始 `debug_lowcmd.csv`、`low_capture_fast.log`、`low_protection.log`、`publisher.log`，以及 `plantid_metadata.json`、`plantid_summary.json`。QA 会核验实际每轴目标/增益/轮命令、16 轴槽位和 mode、连续保持时间、命令状态对齐、保护流、Debug 写入及入口路径。带 QA 标志的段保留原始值，但不进入拟合。每次至少 6 s 精确固定站立才作为有效统计。

## 离线输出给两套仿真

每轮自动生成稳态分位数；再导出完整逐帧对齐的真实命令/反馈 clip，供 PhysX 和 MuJoCo 用**同一份输入**复演：

```powershell
python '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/export_clip.py' '部署测试/04_调试记录/logs/field_sessions/<会话名>' --meta '部署测试/04_调试记录/logs/field_sessions/<会话名>/plantid_metadata.json' --out '部署测试/04_调试记录/logs/field_sessions/<会话名>/plantid_clip.npz'
```

`plantid_clip.json` 同时给出源文件和 clip SHA、命令/状态 tick 配对率；低于 95% 的 clip 不用于时序拟合。`plantid_clip.npz` 沿用 `SimDynamicsValidation` 的主要数组名（`cmd_q/dq/kp/kd/tau`、`st_q/dq/tau`、`quat`、`gyro`、tick、轴序/符号），可直接接入已有离线回放工具。

3 次完成后按同一入口/地面/装载分组：

```powershell
$s = @(Get-ChildItem '部署测试/04_调试记录/logs/field_sessions' -Recurse -Filter plantid_summary.json | Select-Object -ExpandProperty FullName)
python '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/compare_runs.py' $s --out '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/runs/comparison.json'
```

分组器要求同一 case、进入路径、地面、附挂件和 launcher；至少 3 个通过 QA 的会话才标为可估计分布。电量跨重复试验超过 15 个百分点也不标为同一分布。

## 独立高度证据（无需秤）

将手机固定在侧面，画面放一段**与机身同一前后平面**的已知长度尺。车身前后贴两个在 CAD 机体坐标里等高、间距已知的标记；在稳态视频一帧上记录各标记及其正下方地面像素 Y。离线命令例子：

```powershell
python '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/measure_height.py' --trial-id T2_55_01 --ruler-mm 100 --ruler-pixels 400 --front-floor-y 800 --front-mark-y 400 --rear-floor-y 800 --rear-mark-y 420 --mark-spacing-mm 300 --frame-ref 'side.mp4@00:10.000' --out '部署测试/06_训练协作/AS2W_PlantID_FieldKit_20261117/runs/T2_55_01_height.json'
```

这里的数字只是语法例子，必须换成现场标记。工具输出两个高度、前后差和量测不确定度；IMU pitch 与前后高度差可互相核验。镜头、尺、标记的位置在三轮中保持不动；透视明显时这项标记为 `not_run`，不拿它拟合绝对高度。

## 后续增益与动态案例

下一批最关键的是**同一 T2 目标、同一 A 起立轨迹**下的 55/2 与 85/3.2。需要一个独立 stand-only 候选：A 仍以现有 55/2 完成，固定目标不动，用约 1 s 把腿增益从 55/2 平滑切到 85/3.2，保持 10 s，然后走既有 Y/超时趴下；B 编译锁、逐帧 Debug 和保护门不变。它必须先在 MuJoCo/PhysX、x86/ARM64、离线 audit 和逐帧命令检查通过，再单独交付现场。**不要用当前 T15-stiff 的 B 后策略段或 getup1199 的起身链代替**：它们同时改变进入历史/轮控制/目标，不能单独识别增益。

T1 55/2 曾真机后仰约 28°，只用既有原始记录，不为标定重演。T1 85/3.2 的旧实机记录可作跨姿态验证，但要先核其当前源码/二进制合同；本包不调用它。刹轮、短直行、转向属于静态门槛之后的另一轮实验，不在当前采集脚本里偷偷启用。

## 离线验证记录

`analyze_session.py` 对历史 `20261116-175600_t15-stiff` 的 33,754 条命令与 36,825 条状态识别出 17.257 s 精确 T2 55/2 段，最后 4 s pitch 中位 −9.304°，与原始分析一致。历史地面信息未知、入口不是本次 R214 stand-only 链，QA 正确标记为不入本次同条件拟合。`export_clip.py` 对同会话配对 33,754/33,754 帧（100%），并已在 caolin 的现有 MuJoCo 回放器读取。`run_t2_55.ps1` 的默认计划模式已在本机执行，确认没有 HDC/实机动作。现场运动分支未运行。
