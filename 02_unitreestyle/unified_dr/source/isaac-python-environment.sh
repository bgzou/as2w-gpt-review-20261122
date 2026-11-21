#!/usr/bin/env bash
# Set ISAAC_SIM_ROOT to the simulator and ISAACLAB_PYTHON to the venv executable.
set -euo pipefail
test -n "$ISAAC_SIM_ROOT"
test -n "$ISAACLAB_PYTHON"
test -x "$ISAAC_SIM_ROOT/python.sh"
test -x "$ISAACLAB_PYTHON"
unset LD_LIBRARY_PATH PYTHONHOME PYTHONPATH
export PYTHONEXE="$ISAACLAB_PYTHON"
exec "$ISAAC_SIM_ROOT/python.sh" --no-ros-env "$@"
