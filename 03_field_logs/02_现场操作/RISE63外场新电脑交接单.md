# RISE63 外场交接单（换新电脑执行）

**2026-11-20 更新：RISE63 B 运动测试已暂停。** 17:28 首试触发软件硬门槛，随后 FOC 确认故障；本页历史 B 命令不要再运行。两次原始数据与复核见 `../04_调试记录/logs/20261120_rise63_field_sessions/analysis.md`，原厂姿态 200/4 的新版本仅为 A-only 离线候选。

版本：2026-11-20 15:5x。适用：**RISE63 = AS2W_V6.3_from0_model_2000**（平地四足预备姿态抬前轮、双后轮支撑保持，固定零命令）。
本单假设：**CPRO 上已经构建好并通过全部离线门槛**（本轮已在原工作站完成 x86 + ARM64 构建、CTest、offline-audit、C++ 磁带 parity、三方 SHA 核对）。新电脑**只需要通过 HDC 驱动现场会话，不需要编译**。

> **第一原则（不可绕过）**：任何可能让狗动作的命令，必须事先说明该次具体动作与风险，并取得现场人员**当次明确同意**。预告不等于同意。Ctrl+C **不是**运动急停，**X / 物理急停**才是。只读命令（dry-run）不需要动作许可。

---

## 1. 新电脑要准备的东西（5 分钟）

把随本单一起交付的最小工具包 `外场最小工具包_20261120.zip` 解压到**任一英文路径**（例：`D:\as2w_field\`）。包内全部文件名都是 ASCII，避免换机后中文名/编码问题：

```
D:\as2w_field\
  hdc\hdc_6.1.1.280\hdc.exe              ← HDC 客户端（必需）
  hdc\hdc_6.1.1.280\libusb_shared.dll
  run_field_session.ps1                  ← 唯一入口（= 原 运行现场策略会话.ps1）
  logs\                                  ← 会话日志落地（必须存在，已含 README.txt 占位）
  RISE63_handover.md                     ← 本文件
  RISE63_S0_S3_report.md                 ← 离线验收报告（含全部数字与 SHA）
  deploy_package\cpp_deploy-as2w.tar.gz  ← 备份用：万一需要在 CPRO 重建
```

> 原项目里的 `部署测试\运行现场策略会话.ps1` 与本包内的 `run_field_session.ps1` 是同一脚本，只是把 HDC 与日志目录改成了上面的 ASCII 结构。回到原工作站时用项目里的那份。

要求：
- Windows 10/11，PowerShell 5.1 即可；**不需要** Python、不需要编译器、不需要联网。
- 不要用中文路径（HDC 与脚本的路径处理对中文不稳）。

---

## 2. 连上 CPRO（两种通道，优先 USB）

先在 PowerShell 里设一个变量方便复制：

```powershell
$hdc = "D:\as2w_field\hdc\hdc_6.1.1.280\hdc.exe"
& $hdc list targets -v
```

| 通道 | 目标串 | 说明 |
|---|---|---|
| **USB**（首选，最稳） | `0123456789ABCDEF` | 插线即用；CPRO 重启后通常自动恢复 |
| WiFi | `<CPRO当前IP>:8710` | 路由器会重分地址，**以当次 `list targets` 或路由器管理页为准** |

- WiFi 未列在 targets 时只需重连一次：`& $hdc tconn <CPRO当前IP>:8710`（**不要** `hdc kill`、**不要** `hdc tmode`，会把通道弄掉）。
- 判断好坏：`& $hdc -t <目标> shell id` 返回 `uid=0(root)` 即通。
- 只读确认狗链路（狗必须上电）：会话脚本的 dry-run 会自动查 LowState；手工可看 `packets` 是否 ≥100、`mode_machine=2`。

---

## 3. 现场前置（每次会话前）

1. 狗上电，CPRO 与狗之间 eth0 有载波（上一轮就卡在这里：LowState 0 包）。
2. 场地平整；人员握**原厂遥控器**与**物理急停**。
3. 吊装测试（推荐第一级）：四轮**必须着地**；吊带松量 **≥0.35 m**（起身后机身升高约 0.19 m，松量不足会在最高点被拉死）；挂点在**机身重心正上方**，不要挂腿/轮。
4. 确认没有别的发布端在跑：会话脚本 step4 会检查 16 轴 mode 是否全 0，非 0 会 FATAL（说明还有别的 LowCmd 写入者）。

---

## 4. 两级动作（逐级取得当次同意）

### 4.1 第一级：只读预检（不动狗，随时可跑）

```powershell
cd D:\as2w_field
powershell -NoProfile -ExecutionPolicy Bypass -File ".\run_field_session.ps1" `
  -Policy rise63 -Mode rise63-dry-run -MonitorSeconds 30 -Target 0123456789ABCDEF
```

期望：
- `step1 audit: ... AUDIT PASS`、`RISE63 AUDIT PASS`、`step2 probe: low packets=... mode_machine=2`
- 只读检查通过后脚本在 step3（工厂模式 `name=ai`）或 step4 止步并退出：**不动狗**。
- 若 `FATAL: LowState not flowing (got 0 pkts/s)` → 狗未上电或 eth0 不通，先解决链路。

### 4.2 第二级：吊装下只验证“能不能起”（`-RiseExit x`）

> 需要**当次明确同意**，且现场人员握遥控器与物理急停。

```powershell
cd D:\as2w_field
powershell -NoProfile -ExecutionPolicy Bypass -File ".\run_field_session.ps1" `
  -Policy rise63 -Mode rise63 -RiseExit x -MonitorSeconds 120 -Target 0123456789ABCDEF -ConfirmMotion
```

动作序列（脚本会自动做控制权交接：工厂 `ai` → 趴下确认 → `ReleaseMode`）：

1. 发布端进入 `Passive`，约 3–5 s 后就绪，**此时把遥控器交给操作手**。
2. **A**：85/3.2 固定 PD 起立到"四足预备姿态"（hips 0 / thighs 0.65 / calves −1.3）。
3. **B（只按一次）**：通过门控后进入 **0.5 s 交接**，随后策略抬前轮到**双后轮支撑**，固定零命令，**自动上限 10 s**。
4. 到 10 s（或你按 **A**）→ `--rise-exit x` = **EmergencyDamping 被动**，机身由吊带接住。
5. 任何时候按 **X** = 立即被动。

**观察清单（当场记录）**
- B 后 **0.5 s 内**是否开始抬前轮；**1 s 内**是否到后轮支撑；
- 两个后轮是否保持支撑、前轮离地且不甩；机身是否前后点头/左右晃；
- 是否出现异响、电机闷响、明显过流声；
- 保持段是否稳定（预期到 10 s 上限自动退出）；
- X 退出是否干脆（吊带是否吃住）。

**正常日志（publisher.log 里应出现）**
```
[FixedStand] → PolicyHandoff (B accepted: pose_error=... upright=...)
[RISE63] warning only: no auto-brake in the rise/hold ...      ← 出现警告也不是故障，只记录
[PolicyHandoff] → PolicyWalk (zero-speed handoff complete)
[PolicyWalk] rise-exit=x → EmergencyDamping (passive)
RISE63_REMOTE_RC=0
DEBUG_LOWCMD_RESULT rows=... dropped=0 file_ok=1
DEBUG_POLICY_RESULT rows=... file_ok=1
```

### 4.3 第三级（第一级通过后另议）：无吊装、走下法

```powershell
... -Policy rise63 -Mode rise63 -RiseExit descend -MonitorSeconds 180 -Target <目标> -ConfirmMotion
```
- 必须有 **≥1.5 m 空场**（下法时机身后退最多约 0.32 m）+ 护垫；
- 下法序列：前腿先落地（1.5 s）→ 四轮 −2.5 rad/s（1 s 斜坡）→ 后腿回站姿（2.5 s）→ 固定站姿 → 20 s 后自动 Y 趴下；
- 前轮落地冲击实测约 **2.2 kN**（仿真），现场注意听/看冲击，异常按 **X**；
- `-RiseExit squat`（直接折叠趴下）与 `descend` 的差别：squat 会让机身砸地（仿真 2.1–2.7 kN），**仅作下法失败时的备选**。

---

## 5. 结束后必须做的事

1. 会话目录已自动回拉到 `D:\as2w_field\logs\field_sessions\<时间>_rise63\`（含 `session.log`、`publisher.log`、`low_capture_fast.log`、`low_protection.log`、`debug_policy.csv`、`debug_lowcmd.csv`），另有 `logs\<时间>_windows_rise63_*_field_session.log` 转录。
2. **核对四件事**：`RISE63_REMOTE_RC=0`；`DEBUG_LOWCMD_RESULT ... dropped=0 file_ok=1`；`DEBUG_POLICY_RESULT ... file_ok=1`；保护流里没有 `motorstate`/`warning` 非 0。
3. 把现场观感（抬轮是否干脆、后轮支撑是否稳、有无异响/点头/侧晃、X 退出是否干脆）写在同一个目录的 `NOTES.md` 里。
4. 把整个 `field_sessions\<时间>_rise63\` + 转录 + NOTES 发回给我（或合到项目 `部署测试/04_调试记录/logs/`），我会补 INDEX 与下一步判断。
5. 离开现场前：狗趴下、断使能；不要长期保留 WiFi 上的调试通道。

---

## 6. 故障对照

| 现象 | 原因 | 处理 |
|---|---|---|
| `FATAL: LowState not flowing (got 0 pkts/s)` | 狗未上电 / CPRO→狗 eth0 断 | 上电、查网线/载波；用 dry-run 复测 |
| `FATAL: mode_machine=...` | 固件状态异常 | 不要强行继续，记录数值 |
| `FATAL: CheckMode rc=...` | 主控不可达 | 查链路；不要反复重试 |
| `FATAL: 16-axis mode={...}` | 还有别的 LowCmd 写入者 | 先杀掉别的发布端再试 |
| `FATAL: ReleaseMode failed` | 控制权未释放 | 保持趴下，重试一次；仍失败则停 |
| `FATAL: offline audit FAILED` | CPRO 上的二进制与合同不匹配 | 核对下面 §7 的 SHA；必要时用包重建 |
| `FATAL: debug diagnostics missing` | 录制没落盘 | 查 §7 的二进制 SHA 与磁盘余量（需 ≥1 GB） |
| 发布端自停 `STARTUP FAULT: stale/invalid LowState` | CPRO 高负载 DDS 抖动 | 查负载；**不要**放宽看门狗 |
| WiFi 目标 `Offline` | 地址变了/本机重连无线 | `hdc tconn <新地址>:8710`；或直接插 USB |

---

## 7. CPRO 侧“已验证版本”核对（只读）

在 PowerShell 里（`$hdc` 见 §2）：

```powershell
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes(
  "cd /root/as2w_deploy && sha256sum build-rise63/deploy/robots/as2w/rise63/as2w_rise63_publish models/rise63/policy.onnx models/rise63/deploy.yaml deploy/robots/as2w/common/as2w_publish.cpp"))
& $hdc -t 0123456789ABCDEF shell "/data/local/tmp/as2w_field/cpro_ubuntu.sh /bin/bash -lc 'echo $b64 | base64 -d > /tmp/s.sh && bash /tmp/s.sh'"
```

应得到（2026-11-20 已验证）：

```
ac8d64907103442d1659ce8f4f7a6ef39a067ba546505bde5117e14f7441ce60  build-rise63/deploy/robots/as2w/rise63/as2w_rise63_publish
1f09372919d82d35b32aed032517571f03b48b67c99bfcc30760ea20db4c0031  models/rise63/policy.onnx
aaa96f821d4fcc82dcdb7f288916ec453ff28dd6ada87f6c4732f2d405474764  models/rise63/deploy.yaml
74b9f1f4676f760c6faba8e355909c57dac6180847331ccaeb5b191a00d1eeac  deploy/robots/as2w/common/as2w_publish.cpp
```

不一致 → **不要跑运动模式**，先用 `deploy_package\cpp_deploy-as2w.tar.gz` 在 CPRO 重建（见 §8）。

---

## 8. 万一必须在 CPRO 重建（备用）

包里有 `deploy_package\cpp_deploy-as2w.tar.gz`（SHA `734c6750bc3b9ea461f362cefd30b2e3c4016f2428a32cbd95f4b624b40ae8af`）。

1. 传到 CPRO 的 rootfs 可见路径（必须走英文名）：
   `& $hdc -t 0123456789ABCDEF file send "D:\as2w_field\deploy_package\cpp_deploy-as2w.tar.gz" /data/ubuntu_in_hm/ubuntu_rootfs/root/rise63_src.tar.gz`
2. 解包并构建（base64 脚本法，避免多层引号）：

```powershell
$cmd = @'
set -e
cd /root/as2w_deploy
tar -xzf /root/rise63_src.tar.gz
cmake -S . -B build-rise63 -DAS2W_RISE63=ON -DAS2W_RISE63_CANDIDATE=ON -DAS2W_RISE63_PUBLISH=ON -DONNXRUNTIME_ROOT=/root/ort122 -DCMAKE_PREFIX_PATH=/root/unitree_sdk2_install -DBUILD_TESTING=ON
cmake --build build-rise63 -j4
ctest --test-dir /root/as2w_deploy/build-rise63 -R rise63 --output-on-failure
'@
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($cmd))
& $hdc -t 0123456789ABCDEF shell "/data/local/tmp/as2w_field/cpro_ubuntu.sh /bin/bash -lc 'echo $b64 | base64 -d > /tmp/b.sh && bash /tmp/b.sh'"
```
3. 看到 `100% tests passed` 并回到 §7 核对 SHA，才算可上狗。

---

## 9. 硬性禁止（外场同样有效）

- 不取得当次明确同意，不跑 `-Mode rise63`（运动模式）。
- 不绕过 `run_field_session.ps1` 手敲 CPRO 上的发布端二进制。
- 不在第一级就用 `-RiseExit descend/squat`（先吊装验证起身）。
- 不改源码、不改 `deploy.yaml`、不加 `--vx/--vy/--wz-limit`（本策略无命令通道，二进制会直接拒绝）。
- 不用 `hdc kill` / `hdc tmode`；不要往 UDC 写空值。
- 不在吊带松量 <0.35 m、挂点不在重心正上方时起立。
- 出现任何异常（异响、明显下沉、前轮甩打、机身侧倾 >约 30° 且持续）→ **按 X**，并记录时间点。

---

## 10. 一页速查

```
连接：      & $hdc list targets -v      （USB: 0123456789ABCDEF）
只读预检：  run_field_session.ps1 -Policy rise63 -Mode rise63-dry-run -MonitorSeconds 30 -Target <目标>
吊装起立：  ... -Mode rise63 -RiseExit x        -MonitorSeconds 120 -Target <目标> -ConfirmMotion
空地走下法：... -Mode rise63 -RiseExit descend  -MonitorSeconds 180 -Target <目标> -ConfirmMotion
按键：      A 站姿 / B 起立（按一次，10 s 上限）/ Y 折叠趴下 / X 立即被动
判成功：    B 后 ≤0.5 s 抬前轮、≤1 s 到后轮支撑；RC=0；dropped=0；保护位全 0
交回：      field_sessions/<时间>_rise63/ + windows 转录 + NOTES.md
```
