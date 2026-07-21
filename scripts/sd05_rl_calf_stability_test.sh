#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_native"
LAUNCH_LOG="/tmp/sd05_rl_calf_stability_launch.log"

source /opt/ros/humble/setup.bash
source "${INSTALL_DIR}/setup.bash"
set -u

cleanup() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

pkill -TERM -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch gz_quadruped_playground gazebo.launch.py .*controller:=joint_diagnostic' 2>/dev/null || true
pkill -TERM -f '^ign gazebo' 2>/dev/null || true
sleep 1

setsid ros2 launch gz_quadruped_playground gazebo.launch.py \
  pkg_description:=sd05_description \
  controller:=joint_diagnostic \
  height:=0.5 \
  fixed_base:=true \
  rviz:=false \
  external_sensors:=false \
  paused:=true >"${LAUNCH_LOG}" 2>&1 &
launch_pid=$!

for _ in {1..300}; do
  if ign service -s /world/empty/control \
      --reqtype ignition.msgs.WorldControl \
      --reptype ignition.msgs.Boolean \
      --timeout 100 \
      --req 'pause: true, multi_step: 1' >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

for _ in {1..600}; do
  ign service -s /world/empty/control \
    --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean \
    --timeout 100 \
    --req 'pause: true, multi_step: 1' >/dev/null 2>&1 || true
  controllers="$(timeout 1 ros2 control list_controllers 2>/dev/null || true)"
  if [[ "${controllers}" == *"rl_hip_diagnostic_controller"*"active"* ]] && \
     [[ "${controllers}" == *"rl_thigh_diagnostic_controller"*"active"* ]] && \
     [[ "${controllers}" == *"rl_calf_diagnostic_controller"*"active"* ]]; then
    break
  fi
  sleep 0.05
done

controllers="$(ros2 control list_controllers)"
for name in rl_hip_diagnostic_controller rl_thigh_diagnostic_controller rl_calf_diagnostic_controller; do
  if ! grep -Eq "${name}.*active" <<<"${controllers}"; then
    echo "${name} did not become active. See ${LAUNCH_LOG}" >&2
    exit 1
  fi
done

ign service -s /world/empty/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: false' >/dev/null

python3 "${ROOT_DIR}/scripts/sd05_rl_calf_stability_test.py"
