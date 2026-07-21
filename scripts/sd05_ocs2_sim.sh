#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_native"

setup_environment() {
  source /opt/ros/humble/setup.bash
  source "${INSTALL_DIR}/setup.bash"
  set -u
  export PATH="/opt/openrobots/bin:${PATH}"
  export LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
  export ROS2CLI_NO_DAEMON=1
}

stop_simulation() {
  pkill -f "ros2 launch gz_quadruped_playground gazebo.launch.py" 2>/dev/null || true
  pkill -f "ruby.*gz sim" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ruby.*/usr/bin/ign gazebo" 2>/dev/null || true
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "parameter_bridge /clock@" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "controller_manager.*spawner" 2>/dev/null || true

  for _ in {1..50}; do
    if ! pgrep -f "ign gazebo" >/dev/null; then
      return
    fi
    sleep 0.1
  done

  echo "Failed to stop an existing Ignition Gazebo process." >&2
  return 1
}

wait_for_ocs2_controller() {
  local launch_pid="$1"
  step_simulation_while_paused "${launch_pid}" &
  local step_pid=$!
  local wait_status

  if wait_for_ocs2_controller_service "${launch_pid}"; then
    wait_status=0
  else
    wait_status=$?
  fi
  kill "${step_pid}" 2>/dev/null || true
  wait "${step_pid}" 2>/dev/null || true

  case "${wait_status}" in
    0)
      return
      ;;
    2)
      echo "Gazebo launch exited before the OCS2 controller became active." >&2
      ;;
    *)
      echo "Timed out waiting for the OCS2 controller to become active." >&2
      ;;
  esac
  return 1
}

wait_for_ocs2_controller_service() {
  local launch_pid="$1"

  python3 - "${launch_pid}" <<'PY'
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


launch_pid = int(sys.argv[1])
deadline = time.monotonic() + 120.0
rclpy.init()
node = Node("sd05_ocs2_controller_waiter")
client = node.create_client(ListControllers, "/controller_manager/list_controllers")

try:
    while rclpy.ok() and time.monotonic() < deadline:
        try:
            os.kill(launch_pid, 0)
        except ProcessLookupError:
            raise SystemExit(2)

        if not client.wait_for_service(timeout_sec=0.2):
            continue

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if not future.done():
            continue

        try:
            controllers = future.result().controller
        except Exception:
            time.sleep(0.1)
            continue

        if any(controller.name == "ocs2_quadruped_controller" and
               controller.state == "active" for controller in controllers):
            raise SystemExit(0)

        time.sleep(0.1)

    raise SystemExit(3)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

step_simulation_while_paused() {
  local launch_pid="$1"
  local stop_requested=false
  trap 'stop_requested=true' TERM

  while [[ "${stop_requested}" == false ]] && kill -0 "${launch_pid}" 2>/dev/null; do
    ign service -s /world/empty/control \
      --reqtype ignition.msgs.WorldControl \
      --reptype ignition.msgs.Boolean \
      --timeout 500 \
      --req 'pause: true, multi_step: 1' >/dev/null 2>&1 || true
    sleep 0.02
  done

  trap - TERM
}

start_physics_in_passive() {
  ign service -s /world/empty/control \
    --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean \
    --timeout 3000 \
    --req 'pause: false' >/dev/null
}

case "${1:-start}" in
  start)
    setup_environment
    stop_simulation
    ros2 launch gz_quadruped_playground gazebo.launch.py \
      pkg_description:=sd05_description \
      controller:=ocs2 \
      height:=0.14 \
      rviz:=false \
      external_sensors:=false \
      paused:=true &
    launch_pid=$!

    trap 'kill -INT "${launch_pid}" 2>/dev/null || true' INT TERM
    if ! wait_for_ocs2_controller "${launch_pid}"; then
      kill -INT "${launch_pid}" 2>/dev/null || true
      wait "${launch_pid}" || true
      exit 1
    fi
    start_physics_in_passive
    echo "SD05 is in passive down pose. Send command 2 to stand up and enter OCS2 stance."
    wait "${launch_pid}"
    ;;
  stop)
    stop_simulation
    ;;
  *)
    echo "Usage: $0 [start|stop]" >&2
    exit 2
    ;;
esac
