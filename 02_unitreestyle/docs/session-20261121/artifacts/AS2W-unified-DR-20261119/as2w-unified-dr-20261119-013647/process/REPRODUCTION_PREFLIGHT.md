# 训练复现入口预检

交付构建脚本已配置单独附带每阶段实际使用的初始化 checkpoint，并将它们与最终选中输出区分。对应路径、SHA 和源码入口写入最终 manifest 的 stage_initializations。实际交付包将在第三阶段筛选完成后生成。

阶段复跑入口使用各阶段 source-snapshot 中训练时的源码，依照本次记录的输入 checkpoint、seed、base_update、随机化级别和优化配置接续。基础训练另有从零开始 10000 次更新的入口。

已将第二阶段的完整源码快照迁移到独立目录，并补齐其相对路径依赖，在 16 个环境、48 步 rollout 下实际完成 1 次 PPO 更新，日志含 AS2W_TRAIN_SMOKE_OK。迁移后生成的 19 个源文件 SHA256 均核对通过，哈希索引不自引用。该预检验证迁移后的入口与依赖，完整训练结果仍以正式运行和固定回放为准。

依赖记录见 training-environment.json，包含本机 27 个相关包的版本、Isaac Lab 和 Unitree 源码 commit 及工作区状态。最终包另附 ONNX 与 C++ 推理版本、回放和实际误差报告。

本目录 reproduce.sh 是交付包中的复跑入口预览。运行时需要最终交付包的模型、源码和资产目录，以及相应 Isaac Lab 训练环境。
