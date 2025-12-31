#!/usr/bin/env bash
set -euo pipefail

# One-click: build workspace (Debug) + start TB3 Gazebo + Nav2 + RViz.
# Designed to be used from VS Code as a background preLaunchTask.
#
# Usage:
#   ./ros2_ws/scripts/oneclick_tb3_nav2_rviz_debug.sh [ros_distro] [tb3_model]
#
# Defaults:
#   ros_distro=jazzy
#   tb3_model=waffle_pi

ros_distro="${1:-jazzy}"
tb3_model="${2:-waffle_pi}"

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

prefix() {
  local tag="$1"
  shift
  printf '[tb3_oneclick][%s] %s\n' "$tag" "$*"
}

cleanup() {
  prefix STOP "Stopping Gazebo/Nav2/RViz..."
  if [[ -n "${gazebo_pid:-}" ]] && kill -0 "$gazebo_pid" 2>/dev/null; then kill "$gazebo_pid" 2>/dev/null || true; fi
  if [[ -n "${nav2_pid:-}" ]] && kill -0 "$nav2_pid" 2>/dev/null; then kill "$nav2_pid" 2>/dev/null || true; fi
  if [[ -n "${rviz_pid:-}" ]] && kill -0 "$rviz_pid" 2>/dev/null; then kill "$rviz_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

prefix START "workspace=${workspace_dir} distro=${ros_distro} model=${tb3_model}"

if [[ ! -d "/opt/ros/${ros_distro}" ]]; then
  prefix ERROR "/opt/ros/${ros_distro} not found. Install ROS 2 ${ros_distro} first."
  exit 2
fi

# Base + workspace overlay
# shellcheck disable=SC1090
source "/opt/ros/${ros_distro}/setup.bash"
cd "${workspace_dir}"

prefix BUILD "colcon build (Debug)"
colcon build --symlink-install \
  --packages-select ros2_learning_cpp ros2_learning_turtlebot3_teleop \
  --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# shellcheck disable=SC1091
source "${workspace_dir}/install/setup.bash"

export TURTLEBOT3_MODEL="${tb3_model}"

prefix LAUNCH "Gazebo turtlebot3_world.launch.py"
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
gazebo_pid=$!

prefix LAUNCH "Nav2 navigation2.launch.py (use_sim_time=true + map.yaml)"
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:="/opt/ros/${ros_distro}/share/turtlebot3_navigation2/map/map.yaml" &
nav2_pid=$!

prefix LAUNCH "RViz2 tb3_navigation2.rviz"
rviz2 -d "/opt/ros/${ros_distro}/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz" &
rviz_pid=$!

prefix READY "Stack started. (Terminate this task to stop all.)"

wait
