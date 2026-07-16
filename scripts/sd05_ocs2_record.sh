#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="${ROOT_DIR}/install_native"
OUTPUT_ROOT="${HOME}/.ros/sd05_diagnostics"
DURATION=""

usage() {
  cat <<'EOF'
Usage: sd05_ocs2_record.sh [--duration SECONDS] [--output DIRECTORY]

Records until Ctrl+C by default. Use --duration only when a fixed recording
window is needed.
EOF
}

while (($# > 0)); do
  case "$1" in
    --duration)
      DURATION="${2:-}"
      shift 2
      ;;
    --output)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "${DURATION}" ]] && ! [[ "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--duration must be a positive number of seconds." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${INSTALL_DIR}/setup.bash"
export ROS2CLI_NO_DAEMON=1
BRIDGE_EXECUTABLE="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

timestamp="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUTPUT_ROOT}"
bag_path="${OUTPUT_ROOT}/sd05_ocs2_${timestamp}"
bridge_log="${OUTPUT_ROOT}/sd05_ocs2_${timestamp}_bridge.log"

bridge_pid=""
bag_pid=""
timer_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${timer_pid}" ]]; then
    kill "${timer_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bag_pid}" ]]; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]]; then
    kill -INT "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"${BRIDGE_EXECUTABLE}" \
  '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  >"${bridge_log}" 2>&1 &
bridge_pid=$!

sleep 1
if ! kill -0 "${bridge_pid}" 2>/dev/null; then
  echo "Gazebo pose bridge failed to start. See ${bridge_log}" >&2
  exit 1
fi

echo "Waiting for Gazebo /clock; the recorder may be started before the simulation."
clock_ready=false
for _ in {1..600}; do
  if ros2 topic list 2>/dev/null | grep -qx '/clock'; then
    clock_ready=true
    break
  fi
  sleep 0.2
done
if [[ "${clock_ready}" != true ]]; then
  echo "Timed out waiting 120 seconds for Gazebo /clock." >&2
  exit 1
fi

topics=(
  /clock
  /world/empty/pose/info
  /world/empty/dynamic_pose/info
  /joint_states
  /dynamic_joint_states
  /imu_sensor_broadcaster/imu
  /odom
  /pose
  /control_input
  /cmd_vel
  /ocs2_quadruped_controller/joint_commands
  /ocs2_quadruped_controller/measured_observation
  /ocs2_quadruped_controller/desired_observation
  /ocs2_quadruped_controller/wbc_diagnostics
  /rosout
)

echo "Recording SD05 Gazebo and OCS2 data to:"
echo "  ${bag_path}"
if [[ -n "${DURATION}" ]]; then
  echo "Duration: ${DURATION} seconds"
else
  echo "Duration: until Ctrl+C"
fi

ros2 bag record --output "${bag_path}" "${topics[@]}" &
bag_pid=$!

if [[ -n "${DURATION}" ]]; then
  (
    sleep "${DURATION}"
    kill -INT "${bag_pid}" 2>/dev/null || true
  ) &
  timer_pid=$!
fi

set +e
wait "${bag_pid}"
bag_status=$?
set -e
bag_pid=""

if [[ ${bag_status} -ne 0 && ${bag_status} -ne 130 ]]; then
  echo "rosbag recording exited with status ${bag_status}." >&2
  exit "${bag_status}"
fi

echo "Recording complete: ${bag_path}"
