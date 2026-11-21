# AS2-W 原厂风格部署预检

本目录保存导出和 CPRO 控制链的参考实现，以及用基础训练 `model_300.pt` 完成的预检。数字是 x86_64 上的离线测试结果，最终交接以域随机化后选中的权重及对应回放为准。

| 测试 | 平地 | 斜坡 |
| --- | ---: | ---: |
| 传感器至目标逐帧对拍 | 14,400 帧 | 9,600 帧 |
| 观测最大绝对误差 | 8.35e-7 | 8.35e-7 |
| ONNX 腿目标最大误差 | 1.38e-6 rad | 1.08e-6 rad |
| ONNX 轮目标最大误差 | 4.77e-7 rad/s | 4.77e-7 rad/s |
| C++ 控制数学层最大目标误差 | 4.77e-7 | 4.77e-7 |

斜坡数据已用修正后的地形重跑。48 个试次在约 4.29° 至 5.95° 的坡面起步，逐试次实测坡度见 `slope-replay-context.json`。早期预检的出生点误落在水平坡顶，表格采用修正后结果。

C++ 原生 ONNX Runtime 1.22.0 推理也已在 x86_64 上完成上述 24,000 帧对拍。最终目标最大误差为平地约 1.38e-6、斜坡约 1.10e-6，结果见 `flat-cpro-onnx.json` 和 `slope-cpro-onnx.json`。依赖的现场 OrtRunner 头文件保存在 `include/isaaclab/algorithms/algorithms.h`。最终选中权重会重新验证，CPRO ARM64 与现场状态机接入仍待执行。

策略以 50 Hz 推理。输入 58 维，网络输出 12 个腿动作。外部包装做腿目标低通和轮速映射，生成 12 个腿位置目标与 4 个轮速目标。纯侧移和转向时，轮速目标在滤波衰减后为零。当前训练名义增益是腿 55/2、轮 0/2。

- `contract.json` 记录每个输入维度、16 轴顺序和 CPRO 状态槽位。
- `runtime.py` 包含 NumPy 控制包装、ONNX Runtime 推理和 LowState 只读记录。
- `cpro_policy.hpp` 和 `cpro_ort_adapter.hpp` 提供 C++ 控制实现及现场 OrtRunner 接口。外层状态机负责 LowCmd、已标定的发送方向和控制权交接。
- `export.py` 和 `verify.py` 用于单文件 ONNX 导出及传感器逐帧对拍。

训练资产为校准后的 `as2w-load-v0.urdf`，SHA-256 为 `7c9740a95042bad7b73cadfda90bdc7c1873ac766afbb7c0e7e492367d45afb0`。最终训练包会附 URDF 和网格。CPRO ARM64、第二仿真器和真机执行结果当前均为 `not_run`。
