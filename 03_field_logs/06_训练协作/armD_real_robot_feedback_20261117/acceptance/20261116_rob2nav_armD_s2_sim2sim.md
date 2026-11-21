# 2026-11-16 Rob2Nav armD S2 sim2sim 复现原始记录

- 环境：caolin 服务器，MuJoCo 3.17.0 CPU（包证据为单次运行、states=3.0，同口径）。
- 命令（与包 reproduce.mujoco 一致 + 合同必带 gyro 参数，manifest 漏写已补）：
  `python3 tape_r11_mj_matrix_v2.py --xml simulation/r11_mjcf_excl.xml --ckpt weights/armD_turn65_final.pt --out s2_out/ --contract contracts/wheel_contract_v1.json --contract-name r11_legacy --states 3.0 --yaw-kp 60 --ff-cap 4 --ff-gain 0`
- 耗时约 90 秒（后台 nohup，日志 s2_matrix.log，逐例 JSON+逐帧 trace 已存 s2_out/）。

## 逐例（我方复现 vs 包内 evidence）

| 用例 | 我方 | 包内 |
|---|---|---|
| hold | PASS | PASS |
| turn_left_0.05 | INVALID_STAGE1 sup=0.720 | INVALID sup=0.747 |
| turn_right_0.05 | FAIL trk 0.42 | FAIL 0.66 |
| turn_left_0.10 | INVALID sup=0.433 term=1 nw=15 | INVALID sup=0.387 term=1 nw=51 |
| turn_right_0.10 | INVALID sup=0.680 | INVALID sup=0.447 |
| straight_fwd_0.25 | PASS vx 1.08 | PASS 1.09 |
| straight_bwd_0.25 | PASS vx 1.05 | PASS |
| arc_left_0.25 | FAIL yaw 0.50 | FAIL 0.63 |
| arc_right_0.25 | PASS yaw 1.05 / vx 0.95 | PASS 1.09 |
| brake_fwd_0.25 | PASS | PASS |
| brake_bwd_0.25 | PASS | PASS |
| **合计** | **6/11** | **6/11** |

## 判定

- **复现 PASS**：同一通过集 {hold, straight±0.25, brake±0.25, arc_right}，失败模式与量级一致（纯转=支撑塌陷为主，跟踪次之；数值差为 MuJoCo 版本接触抖动）。
- **双仿真门槛本身未过**（包自述已知）：4 项纯转 Isaac/MuJoCo 双端败且方向相反（欠转 vs 超调），arc_left 仅 MuJoCo 单端败——按 02 文档分流属策略/包装侧问题，反馈训练侧，本项目不调参硬凑。
- **由此确定实机有界包络**：hold、前后直线 ≤0.25 m/s、刹车、（后续若许可）arc_right；**纯原地转与 arc_left 永不进实机清单**。

## 下一阶段

S3 C++ 适配（cpp_deploy-as2w 唯一源码树新增 r-armD 控制器 + CMake + models/manifest + 现场入口），速度包络 vx_limit=0.25、wz_limit 仅 arc_right 所需（−0.05）且默认锁定。
