# AS2-W 实机记录日志归档（2026-11-20）

从 CPRO 拉取的实机运行日志全量归档。源：CPRO `192.168.10.75:8710`（USB 通道 `0123456789ABCDEF`），
经 `tar czf` 压缩后 `hdc file recv` 传回本机，SHA256 已与服务器端逐一核对一致。

## 内容

| 路径 | 大小 | 内容 |
|---|---|---|
| `field_logs_bundle.tar.gz` | 40,373,521 B | 原始压缩包：`field_logs/` + 最后一次运行的 `debug_lowcmd.csv` / `debug_policy.csv` |
| `field_sessions.tar.gz` | 386,302,856 B | 原始压缩包：CPRO `/root/field_sessions/` 全部会话目录（154 个） |
| `field_logs/` | 257 个文件 | 解包后的 field.sh 会话日志（low-state / low-protection / motion-release 等）与 `INDEX.md` |
| `sessions/field_sessions/` | 154 个目录 | 解包后的历史会话（r1199 / r214 / r249 / r270 / r284 / r4000 / getup1199 / himloco / rise63 / unitree58 等） |
| `debug_lowcmd.csv` | 18,980,241 B | 最后一次运行（mu025 权重摇杆轮）的最终下发帧（500 Hz） |
| `debug_policy.csv` | 2,005,648 B | 最后一次运行的策略帧（相位、ticks、观测、动作、目标） |

## 校验

```
5c09c4e00b17cfb486e5060f4eb7d50de0152c130e74d5225b92d953e6b65a22  field_logs_bundle.tar.gz
7f3a237c85139ce6b3e0e5f74a61225b9ad6383e3ff20bc30e1fdb657d6a3576  field_sessions.tar.gz
```

两个哈希均与 CPRO 端 `sha256sum` 输出一致（本机用 `Get-FileHash` 复核）。

## 时间范围

- `field_logs/`：2026-11-09 至 2026-11-20 20:10 的 field.sh 会话
- `field_sessions/`：2026-11 中旬至 2026-11-20 的发布端会话（含其他策略线：himloco / rise63 / r2000a / r1199 / r214 等）

## 说明

- 会话目录内一般包含 `publisher.log`、`debug_lowcmd.csv`、`debug_policy.csv`、`low_capture_fast.log`、`low_protection.log`。
- 本归档是 CPRO 本地日志的全量快照；更早的 CPRO 日志若已被轮转覆盖，不在其中。
- 分析脚本与结论见 `AS2W-v2-stage-reports/`（如 `field-zero-b-jit-20261120.md`、`field-20261120-mu025/`、`friction-unmodeled-20261120.md`）。
