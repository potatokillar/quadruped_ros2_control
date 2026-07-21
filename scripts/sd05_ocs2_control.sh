#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_native"

source /opt/ros/humble/setup.bash
source "${INSTALL_DIR}/setup.bash"

exec python3 "${ROOT_DIR}/scripts/sd05_ocs2_control.py"
