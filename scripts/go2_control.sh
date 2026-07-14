#!/usr/bin/env bash

set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash

if [[ ! -f "${REPOSITORY_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${REPOSITORY_DIR}/install/setup.bash" >&2
  exit 1
fi

source "${REPOSITORY_DIR}/install/setup.bash"
set -u

if ! ros2 control list_controllers 2>/dev/null | grep -q 'unitree_guide_controller.*active'; then
  echo "unitree_guide_controller is not active; start Gazebo first." >&2
  exit 1
fi

exec python3 "${SCRIPT_DIR}/go2_control.py"
