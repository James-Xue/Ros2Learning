#!/usr/bin/env bash
set -euo pipefail

# Install ROS 2 debug symbol packages (dbgsym) required to step into ROS 2 libraries with source.
# This repo intentionally keeps these out of the default setup to avoid large installs.
#
# By default installs a small, high-value set (rclcpp/rcutils).
# Use --all for a broader set across the ROS 2 stack.
#
# Usage:
#   ./ros2_ws/scripts/setup_ros2_dbgsym.sh [ros_distro]
#   ./ros2_ws/scripts/setup_ros2_dbgsym.sh [ros_distro] --all
# Example:
#   ./ros2_ws/scripts/setup_ros2_dbgsym.sh jazzy

ROS_DISTRO_DEFAULT="jazzy"
ROS_DISTRO_ARG="${1:-}"
MODE_ARG="${2:-}"
ROS_DISTRO_ENV="${ROS_DISTRO:-}"
ROS_DISTRO="${ROS_DISTRO_ARG:-${ROS_DISTRO_ENV:-$ROS_DISTRO_DEFAULT}}"

install_all=0
if [[ "${MODE_ARG}" == "--all" ]]; then
  install_all=1
fi

pkgs_basic=(
  "ros-${ROS_DISTRO}-rclcpp-dbgsym"
  "ros-${ROS_DISTRO}-rcutils-dbgsym"
  "ros-${ROS_DISTRO}-rclcpp-action-dbgsym"
)

# A broader set that helps when stepping beyond rclcpp into rcl/rmw layers.
pkgs_extra=(
  "ros-${ROS_DISTRO}-rcl-dbgsym"
  "ros-${ROS_DISTRO}-rcl-action-dbgsym"
  "ros-${ROS_DISTRO}-rcl-logging-spdlog-dbgsym"
  "ros-${ROS_DISTRO}-rcl-yaml-param-parser-dbgsym"
  "ros-${ROS_DISTRO}-rmw-dbgsym"
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp-dbgsym"
  "ros-${ROS_DISTRO}-rcpputils-dbgsym"
)

echo "[setup_ros2_dbgsym] ROS_DISTRO=${ROS_DISTRO} all=${install_all}"

echo "[setup_ros2_dbgsym] Updating apt index"
sudo apt-get update

echo "[setup_ros2_dbgsym] Installing dbgsym packages (needed for stepping into rclcpp::init and friends)"

pkgs=("${pkgs_basic[@]}")
if [[ "${install_all}" -eq 1 ]]; then
  pkgs+=("${pkgs_extra[@]}")
fi

# Best-effort install: keep going if some packages are unavailable.
for pkg in "${pkgs[@]}"; do
  echo "[setup_ros2_dbgsym] Installing: ${pkg}"
  sudo apt-get install -y --no-install-recommends "${pkg}" || \
    echo "[setup_ros2_dbgsym] WARN: failed to install ${pkg} (package may not exist on this distro)"
done

echo "[setup_ros2_dbgsym] Done. Tip: if you also want glibc source-level stepping, install:"
echo "  sudo apt-get install -y libc6-dbg"
