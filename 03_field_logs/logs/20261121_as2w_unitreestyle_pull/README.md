# 2026-11-21 as2w-unitreestyle 仓库落盘与训练服务器镜像选型（阶段原始记录）

- 日期：2026-11-21 14:01–14:45（Asia/Shanghai）
- 设备/环境：Windows 工作站（zoubu）、DSH 会话；**未连接 caolin、CPRO 或机器狗**
- 性质：代码/文档归档 + 外部平台调研，**无编译、无仿真、无实机动作**

## 1. 拉取

```powershell
# 首选 git clone 失败（本会话沙箱令牌导致 schannel 无凭证）
git clone --depth 1 https://github.com/YuTaoV5/as2w-unitreestyle.git $env:TEMP\as2w-unitreestyle-clone
# fatal: unable to access '...': schannel: AcquireCredentialsHandle failed: SEC_E_NO_CREDENTIALS (0x8009030E)
git -c http.sslBackend=openssl clone --depth 1 ...   # 换成 openssl 后：Recv failure: Connection was reset
curl.exe -sS -I https://example.com                  # curl: (35) schannel: AcquireCredentialsHandle failed —— 与 GitHub 无关，TLS 通道整体不可用
```

改用 Python（OpenSSL 通道，绕开 schannel）：

```python
urllib.request.urlopen("https://codeload.github.com/YuTaoV5/as2w-unitreestyle/zip/refs/heads/main").read()
# downloaded bytes: 694603224
# root: ['as2w-unitreestyle-main']   file count: 1900
```

镜像通道测速（各读前 8.1 MB）：direct 1.5 Mbps、ghfast.top 1.2 Mbps、ghproxy.net 握手超时、hub.gitmirror.com 握手 EOF。全程约 60 分钟。

## 2. 落盘

```powershell
robocopy <temp>\as2w-unitreestyle-main "D:\...\部署参考\as2w-unitreestyle" /E /NFL /NDL /NJH /NJS /NP /MT:16 /R:1 /W:1
# robocopy exit=1  seconds=0.6
# src files=1900 sizeMB=798   dst files=1900 sizeMB=798
# has .git = False
```

逐文件 SHA256 清单：本目录 `SHA256SUMS.txt`（1900 行，20.0 s），清单自身
`96c4b26c30956ae37d46751c500e828062d98c96cf497c6b7331844161ccb56d`。

**权限事故（与本项目 2026-11-20 合并时同类）**：在 `部署参考\` 下 `New-Item -ItemType Directory` /
`cmd mkdir` 均 `Access is denied`；`部署测试\` 同样如此，而工作区根目录可写。官方排查脚本
`diagnose-windows-sandbox-acl.ps1 -Path ...\部署参考 -AllowRoot ...\AS2-W_CPRO` 判定
`NOT_THIS_CLASS`（DACL 无缺项，`WRITE_DAC`/`WRITE_OWNER` 均在，`FIXED=0`，未改动任何权限），
报告 `D:\Projects\Athena\运动控制\Collie\_dsh_acl_recovery\acl-report-99d39ab944e24b57973f9547997b0b6d.jsonl`。
结论与 11-20 记录一致：属沙箱低完整性标签限制（这两个目录为 Medium 完整性），不是 DACL 损坏；
本次靠 DSH 写入通道建出目录、随后会话切到完全访问完成复制。

## 3. 仓库结构与环境事实（详见 `06_训练协作\训练服务器租用与镜像选型_20261121.md`）

- 1900 文件 / 798 MB；`docs/session-20261121` 一项占 557 MB（mp4/npz/CSV 实机产物）。
- 训练栈记录：Isaac Sim `6.2.0-rc.14+release.52805.02189a84.gl`、Isaac Lab `v3.0.0`（pip `isaaclab==28.0.0`）、
  Python 3.12.3、torch `2.11.0+cu130`、`rsl-rl-lib==5.5.1`、驱动 580.126.09、GPU L40 46,068 MiB。
- 仓库 README 自述：原始实机数据约 16 GB 未上传，GitHub 只放索引与 SHA。

## 4. gpufree 镜像调研

页面为 SPA，清单取自 `GET https://www.gpufree.cn/api/v1/images/public/display_list`
（10 分类、约 90 镜像；“具身机器人” 48 个）。标签组接口
`/api/v1/icomm/images/tags/group-by-categories` 给出 `IsaacSim/Lab`、`Mujoco`、`locomotion`、`Unitree` 等分类。
商品接口 `GET /api/v1/products/list`（价格字段单位为分）用于机型/价格。

结论：首选 image_id **234 `Isaacsim6.2.0+IsaacLab3.0.0 workstation+ROS2 jazzy`（54 G）**，
机型 L40 48G（或 L40S 48G），数据盘 ≥200 G；次选 229（5.1+2.3.2 pip 底座自己升级）。
不建议 A100（无 RT 核心）。

## 5. 边界

- 未编译、未跑 `check_runtime.sh`/`replay.sh`/`smoke.sh`、未与 1220 基线比数值。
- 镜像能否直接对上 Isaac Lab 3.0.0、驱动是否 ≥580、`workstation版` headless 是否顺利，均**待开机器实测**。
- 价格/体积/版本为 2026-11-21 接口快照。
