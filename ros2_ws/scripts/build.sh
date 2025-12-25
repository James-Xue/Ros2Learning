#!/usr/bin/env bash
set -euo pipefail

# 构建当前 colcon 工作空间
# 用法：
#   ./scripts/build.sh [ros_distro]
# 例如：
#   ./scripts/build.sh humble

ROS_DISTRO_DEFAULT="humble"
ROS_DISTRO="${1:-$ROS_DISTRO_DEFAULT}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[build] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found. Please install ROS 2 (${ROS_DISTRO}) first."
  exit 1
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

colcon build --symlink-install

echo "[build] Build finished. To use:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
