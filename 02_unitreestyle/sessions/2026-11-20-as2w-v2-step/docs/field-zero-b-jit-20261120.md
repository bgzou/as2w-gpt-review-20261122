# 实机首飞结果：抖动权重零命令接管（2026-11-20 18:50）

对象：`as2w-v2-step-jit-20261120`（权重 `9999512641a16f20…c41c33`，部署 ONNX `7a0367e5…435b715`）
现场：CPRO（192.168.10.75）→ 狗 eth0 192.168.123.161，battery 48.2 V，保护流全程 `motorstate_or=0x0 / warning_or=0x0 / modes=1111111111111111`。
授权：现场当次同意本次零命令首飞（A 固定 PD 站 → B 零命令 → 自动刹轮/回站 → 急停被动），全程 `vx=vy=wz=0`。

## 流程（发布端原始输出）

```
MotionSwitcher ReleaseMode rc=0  form=1
T2 固定站（interp 3 s） → B 零命令 WALK 10 s → brake（bounded zero hold cap）
→ ReturnStand → FixedStand → EmergencyDamping → 停止
DEBUG_LOWCMD rows=45582 dropped=0；DEBUG_POLICY rows=675
Publisher exit=0，UNITREE58_V2STEP_ZERO_B_REMOTE_RC=0
```

WALK 段 501 帧 @20 ms（10.0 s），命令恒为 0；中途操作员推杆 `lx=-1` 被发布端正确拒绝（`requested=0,0,0`）。

## 关键指标（对比 2026-11-19 旧 clamp 权重 10 Hz 实机记录）

| 指标 | 11-19 旧权重 | **本次抖动权重** | 变化 |
|---|---|---|---|
| 腿目标 p2p（均值） | 0.46–0.50 rad | **0.360 rad** | **−25%** |
| 腿目标符号翻转率 | **30%** | **8.2%** | **−73%** |
| 目标主导频率 | 4.5 Hz | **1.2 Hz** | −73% |
| 腿跟踪误差（均值） | 0.161 rad | **0.101 rad** | −37% |
| 机身 pitch 均值 | −0.53° | −0.51° | 持平 |
| 机身 pitch 范围 | −4.91…+2.86° | **−3.41…+0.74°** | 摆幅约减半 |
| 机身 roll | 未记录 | −2.49°（−4.40…−0.03） | 略左倾，待观察 |

单关节 p2p 最大 0.454 rad（仍有单个关节接近用满 ±0.25 行程），但整体已从"满幅翻符号"变为"小幅低频"。

## 判读

1. **tanh 化的效果在实机成立**：旧权重 `|raw|` 达 34 倍、等价 bang-bang；本权重 8.2% 翻转、1.2 Hz 主导，抖动显著下降。
2. **跟踪误差同步改善**（0.161→0.101 rad），说明不是"把抖动滤掉、跟踪变差"，而是策略本身更平滑。
3. **姿态没有变差**：pitch 均值与旧版相同，范围更窄；roll 有 −2.5° 静态左倾，需要下一轮确认是否为配重/零位偏差。
4. 保护流全程零告警、零丢帧（dropped=0）。

## 当前机器状态

试验按设计以 EmergencyDamping（被动）结束，狗处于被动阻尼姿态；控制权仍在低层（`ReleaseMode` 已释放）。恢复原厂运控需要一条 gated RPC `motion-select ai`（需现场当次同意）。

## 下一步建议

1. 现场确认姿态与声音（p2p 0.36 是否仍有可闻抖动）；
2. 视需要恢复原厂控制，或继续有界侧移测试（`vy ±0.10`、`wz ±0.10`，单次 ≤30 s）；
3. 回传的 `debug_policy.csv` / `debug_lowcmd.csv` 已归档到 `X:/Items/as2w/.stage58/field20261120/`，分析与 JSON 在 `AS2W-v2-stage-reports/`。
