#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
sha256sum -c repro/inputs.sha256
nvidia-smi --query-gpu=name,memory.total,memory.free,driver_version --format=csv
sim_root=${ISAACSIM_ROOT:-/isaac-sim}
if [[ -f "$sim_root/VERSION" ]]; then
    printf 'Isaac Sim build '
    cat "$sim_root/VERSION"
    printf "\n"
    if ! cmp -s repro/isaac-sim-version.txt "$sim_root/VERSION"; then
        printf 'Isaac Sim build differs from the recorded training build. Run replay and smoke before training.\n' >&2
    fi
fi
./preflight/analysis-python - <<'PY'
from importlib.metadata import version
import sys
print(sys.version)
for package in ('isaaclab','isaaclab_physx','isaaclab_rl','isaaclab_tasks',
                'rsl-rl-lib','torch','tensordict','numpy','scipy','trimesh'):
    print(f'{package}=={version(package)}')
PY
