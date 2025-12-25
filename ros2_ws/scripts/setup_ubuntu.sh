#!/usr/bin/env bash
set -euo pipefail

# 这个脚本用于在 Ubuntu 上为本工作空间安装常用依赖。
# 用法：
#   ./scripts/setup_ubuntu.sh [ros_distro]
# 例如：
#   ./scripts/setup_ubuntu.sh jazzy

# 优先级：参数 > 环境变量 ROS_DISTRO > 默认 jazzy
ROS_DISTRO_DEFAULT="jazzy"
ROS_DISTRO_ARG="${1:-}"
ROS_DISTRO_ENV="${ROS_DISTRO:-}"
ROS_DISTRO="${ROS_DISTRO_ARG:-${ROS_DISTRO_ENV:-$ROS_DISTRO_DEFAULT}}"

if ! command -v lsb_release >/dev/null 2>&1; then
  echo "lsb_release not found; please install: sudo apt-get update && sudo apt-get install -y lsb-release"
  exit 1
fi

echo "[setup] ROS_DISTRO=${ROS_DISTRO}"

echo "[setup] Updating apt index"
sudo apt-get update

echo "[setup] Installing base tools"
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete

# rosdep init may fail if already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "[setup] rosdep init"
  sudo rosdep init
fi

echo "[setup] rosdep update"
rosdep update

# Install dependencies for packages under src
if [ -d "$(pwd)/src" ]; then
  echo "[setup] Installing ROS package dependencies via rosdep (if any)"
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    echo "[setup] WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found. Make sure ROS 2 is installed."
  fi

  # --ignore-src: don't try to install dependencies for packages in src themselves
  rosdep install --from-paths src --ignore-src -r -y || true
fi

echo "[setup] Done. Next: ./scripts/build.sh"
