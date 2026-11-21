# 原厂风格 v2 平地续训

部署目录建议 /root/as2w/unitree-flat-v2-2000。保留整目录相对结构。

入口 launch_v2.sh，配置8192环境、48步采样、2000新增更新，20ms控制、腿轮5Hz滤波，DR level 0，纯平地。58维观测、12维腿动作，轮速独立映射 vx/0.086。vy/wz通过四轮抬落覆盖奖励学习踏步，vx通过四轮支撑项引导滚动。最终需关节曲线、轮载和视频验收。

资产 assets/as2w-t2-v2.urdf 为27.512kg（含 JT128 的 0.5kg），底盘质心相对load-v0前移20mm。v2_collision.py 设置底盘与两前大腿的碰撞过滤。保留URDF关节轴，DDS到MuJoCo的符号转换不能直接当成训练关节轴改动。

启动命令（本次没有执行）：

```bash
cd /root/as2w/unitree-flat-v2-2000
export RESUME=/root/gpufree-data/as2w-runs/unitree-unified-reward-20261119-010208/train/model_1000.pt
export OUT=/root/gpufree-data/as2w-runs/unitree-flat-v2-baseline/train
/root/as2w/preflight/isaac-python prepare_v2_usd.py
bash launch_v2.sh
```

RESUME为历史服务器路径，入口会检查文件存在，可换成兼容58观测12动作的原厂权重。不能使用HIMLoco model_7836。加载actor/critic后新增2000轮，编号从0开始。

run.py参数入口；task.py地形与附加奖励；environment.py动作与基础奖励；swing_reward.py抬轮事件与覆盖；reward_config.json奖励权重；control.toml滤波；training.py PPO；best_checkpoint.py候选保留。

已做本地Python语法检查。依赖原服务器Isaac Lab、isaaclab_physx和isaaclab_rl。Isaac场景构建、PhysX碰撞过滤生效、小环境GPU冒烟及训练均未运行。首次正式训练前须在服务器核对这些运行项。best.pt仍需独立步态验收。

保留整个 generated-v2 目录（含 USD 依赖层）。正式配置为 8192/48/2000/flat。优先用固定 SHA 的 model_1000.pt。先用 4096 环境做 1 更新冒烟（独立输出），再测 8192 显存。若边界占比仍接近 1，先评估确定性 actor 均值与步态再提交长跑。

4096 环境冒烟命令，在上面的 USD 生成后执行，输出路径必须尚不存在。

    /root/as2w/preflight/isaac-python run.py --out /root/gpufree-data/as2w-runs/v2-smoke-4096/train --asset generated-v2/as2w-t2-v2/as2w-t2-v2.usd --terrain flat --num-envs 4096 --rollout-steps 48 --iterations 1 --dr-level 0 --resume "$RESUME" --resume-update 0
