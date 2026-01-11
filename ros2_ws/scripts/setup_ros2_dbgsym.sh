#!/usr/bin/env bash
set -euo pipefail

# Install ROS 2 debug symbol packages (dbgsym) required to step into rclcpp/rcutils with source.
# This repo intentionally keeps these out of the default setup to avoid large installs,
# but for source-level debugging you usually want them.
#
# Usage:
#   ./ros2_ws/scripts/setup_ros2_dbgsym.sh [ros_distro]
# Example:
#   ./ros2_ws/scripts/setup_ros2_dbgsym.sh jazzy

ROS_DISTRO_DEFAULT="jazzy"
ROS_DISTRO_ARG="${1:-}"
ROS_DISTRO_ENV="${ROS_DISTRO:-}"
ROS_DISTRO="${ROS_DISTRO_ARG:-${ROS_DISTRO_ENV:-$ROS_DISTRO_DEFAULT}}"

pkgs=(
  "ros-${ROS_DISTRO}-rclcpp-dbgsym"
  "ros-${ROS_DISTRO}-rcutils-dbgsym"
  "ros-${ROS_DISTRO}-rclcpp-action-dbgsym"
)

echo "[setup_ros2_dbgsym] ROS_DISTRO=${ROS_DISTRO}"

echo "[setup_ros2_dbgsym] Updating apt index"
sudo apt-get update

echo "[setup_ros2_dbgsym] Installing dbgsym packages (needed for stepping into rclcpp::init)"
sudo apt-get install -y "${pkgs[@]}"

echo "[setup_ros2_dbgsym] Done. Tip: if you also want glibc source-level stepping, install:"
echo "  sudo apt-get install -y libc6-dbg"
