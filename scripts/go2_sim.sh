#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPOSITORY_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Stopping previous Go2 simulation..."
launch_pids="$(
  pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch unitree_guide_controller gazebo_classic.launch.py' || true
)"
if [[ -n "${launch_pids}" ]]; then
  kill -TERM ${launch_pids} 2>/dev/null || true
fi
pkill -TERM -x gzclient 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true

for _ in $(seq 1 10); do
  if ! pgrep -x gzserver >/dev/null && ! pgrep -x gzclient >/dev/null; then
    break
  fi
  sleep 0.5
done

pkill -KILL -x gzclient 2>/dev/null || true
pkill -KILL -x gzserver 2>/dev/null || true

echo "Starting Go2 simulation..."
exec ros2 launch unitree_guide_controller gazebo_classic.launch.py \
  pkg_description:=go2_description \
  height:="${GO2_SPAWN_HEIGHT:-0.5}"
