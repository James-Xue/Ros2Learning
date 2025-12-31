#!/usr/bin/env bash
set -euo pipefail

# Fetch rclcpp sources matching the installed ROS 2 Debian package version.
# This enables stepping into rclcpp code in VS Code + gdb via sourceFileMap.
#
# Usage:
#   ./ros2_ws/scripts/setup_rclcpp_source.sh [ros_distro] [tag]
#
# Examples:
#   ./ros2_ws/scripts/setup_rclcpp_source.sh jazzy
#   ./ros2_ws/scripts/setup_rclcpp_source.sh jazzy 28.1.13

ros_distro="${1:-jazzy}"
requested_tag="${2:-}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dest_dir="${repo_root}/third_party/rclcpp_repo"

if [[ -n "${requested_tag}" ]]; then
  tag="${requested_tag}"
else
  tag=""
  if command -v dpkg-query >/dev/null 2>&1; then
    # Example version: 28.1.13-1noble.20251025.081428
    pkg_version="$(dpkg-query -W -f='${Version}' "ros-${ros_distro}-rclcpp" 2>/dev/null || true)"
    if [[ -n "${pkg_version}" ]]; then
      tag="${pkg_version%%-*}"
    fi
  fi

  # Safe default for this repo's current setup.
  tag="${tag:-28.1.13}"
fi

echo "[setup_rclcpp_source] ROS_DISTRO=${ros_distro} tag=${tag}"

echo "[setup_rclcpp_source] Destination: ${dest_dir}"
mkdir -p "${repo_root}/third_party"

if [[ -d "${dest_dir}/.git" ]]; then
  echo "[setup_rclcpp_source] Existing clone detected; updating tags..."
  git -C "${dest_dir}" fetch --tags --force
  git -C "${dest_dir}" checkout -q "${tag}" || git -C "${dest_dir}" checkout -q "refs/tags/${tag}"
else
  echo "[setup_rclcpp_source] Cloning ros2/rclcpp..."
  rm -rf "${dest_dir}"
  git clone --depth 1 --branch "${tag}" https://github.com/ros2/rclcpp.git "${dest_dir}"
fi

needed_file="${dest_dir}/rclcpp/src/rclcpp/init_options.cpp"
if [[ ! -f "${needed_file}" ]]; then
  echo "[setup_rclcpp_source] ERROR: expected file missing: ${needed_file}" >&2
  exit 2
fi

echo "[setup_rclcpp_source] OK: ${needed_file}"
