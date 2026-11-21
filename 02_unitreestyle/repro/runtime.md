# 服务器环境

完整单轮验证使用 Linux x86_64、Python 3.12、L40 (46,068 MiB)、NVIDIA 驱动 580.126.09、Isaac Sim `6.2.0-rc.14+release.52805.02189a84.gl` 和 Isaac Lab `v3.0.0`。对训练主机 `source/` 和 `apps/` 下的 Python、TOML、Kit、YAML 文件逐个计算 SHA-256，与官方 `v3.0.0` 标签相同，共 2,454 个文件。具体软件包版本在 [runtime-packages.txt](runtime-packages.txt)，这个清单用于环境比对。Isaac Sim 和 GPU 驱动从 NVIDIA 单独安装。

## 使用与训练服务器相同的环境

配置同一发行构建和 Isaac Lab 环境，设置路径后执行 [README](../README.md) 的预检、起点回放和完整单轮。默认路径对应原服务器镜像，任务代码与输出可以放在任意目录。

```bash
export ISAACSIM_ROOT=/isaac-sim
export AS2W_PYTHON=/opt/isaaclab-venv/bin/python
./repro/check_runtime.sh
```

## 从公开版构建兼容环境

如果拿不到相同的 RC 镜像，下面使用官方发行的 Isaac Sim 6.2.0 和与原服务器一致的 Isaac Lab 源码标签。发行版 PhysX 构建与实测 RC 可能不同。先运行预检、起点回放和完整单轮，确认轨迹和验收结果；不同 GPU、驱动或构建的训练结果可能有数值差异。安装参见 [Isaac Lab v3.0.0 安装说明](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html) 和 [NVIDIA 容器说明](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html)。

```bash
# 在仓库外准备 Python 3.12 虚拟环境和 Isaac Lab 源码。
export AS2W_VENV="$HOME/as2w-isaaclab-v3"
uv venv --python 3.12 --seed "$AS2W_VENV"
source "$AS2W_VENV/bin/activate"
uv pip install 'isaacsim[all,extscache]==6.2.0.0' --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match --prerelease=allow
uv pip install 'torch==2.11.0' 'torchvision==0.26.0' --index-url https://download.pytorch.org/whl/cu130
git clone --branch v3.0.0 --depth 1 https://github.com/isaac-sim/IsaacLab.git "$HOME/IsaacLab-v3.0.0"
(cd "$HOME/IsaacLab-v3.0.0" && ./isaaclab.sh -i 'rl[rsl-rl]')
uv pip install 'rsl-rl-lib==5.5.1' 'tensordict==0.14.2' 'scipy==1.17.0' 'trimesh==4.11.1' 'matplotlib==3.10.8'
export AS2W_RUNTIME=pip
export AS2W_PYTHON="$AS2W_VENV/bin/python"
```

首次启动按 NVIDIA 安装说明完成许可交互。若遇到 `libcarb.so` 静态 TLS 报错，在启动前应用 NVIDIA 安装说明给出的 `LD_PRELOAD` 配置。然后回到任务仓库执行 README 中的预检和运行命令。
