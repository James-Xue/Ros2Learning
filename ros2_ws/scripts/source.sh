#!/usr/bin/env bash
set -euo pipefail

# 在当前 shell 中 source ROS 2 与工作空间 overlay
# 用法：
#   source ./scripts/source.sh [ros_distro]

# 优先级：参数 > 环境变量 ROS_DISTRO > 默认 jazzy
ROS_DISTRO_DEFAULT="jazzy"
ROS_DISTRO_ARG="${1:-}"
ROS_DISTRO_ENV="${ROS_DISTRO:-}"
ROS_DISTRO="${ROS_DISTRO_ARG:-${ROS_DISTRO_ENV:-$ROS_DISTRO_DEFAULT}}"

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[source] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found. Please install ROS 2 (${ROS_DISTRO}) first."
  return 1
fi

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "[source] NOTE: install/setup.bash not found yet. Run ./scripts/build.sh first."
fi

echo "[source] Sourced ROS 2 (${ROS_DISTRO}) and workspace overlay: ${WORKSPACE_DIR}"
