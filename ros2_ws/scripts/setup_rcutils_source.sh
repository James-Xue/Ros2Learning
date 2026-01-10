#!/usr/bin/env bash
set -euo pipefail

# Fetch rcutils sources matching the installed ROS 2 Debian package version.
# This enables stepping into rcutils code in VS Code + gdb via sourceFileMap.
#
# Usage:
#   ./ros2_ws/scripts/setup_rcutils_source.sh [ros_distro] [tag]
#
# Examples:
#   ./ros2_ws/scripts/setup_rcutils_source.sh jazzy
#   ./ros2_ws/scripts/setup_rcutils_source.sh jazzy 6.7.4

ros_distro="${1:-jazzy}"
requested_tag="${2:-}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dest_dir="${repo_root}/third_party/rcutils_repo"

if [[ -n "${requested_tag}" ]]; then
  tag="${requested_tag}"
else
  tag=""
  if command -v dpkg-query >/dev/null 2>&1; then
    # Example version: 6.7.4-1noble.20250822.213450
    pkg_version="$(dpkg-query -W -f='${Version}' "ros-${ros_distro}-rcutils" 2>/dev/null || true)"
    if [[ -n "${pkg_version}" ]]; then
      tag="${pkg_version%%-*}"
    fi
  fi

  # Safe default for this repo's current setup.
  tag="${tag:-6.7.4}"
fi

echo "[setup_rcutils_source] ROS_DISTRO=${ros_distro} tag=${tag}"
echo "[setup_rcutils_source] Destination: ${dest_dir}"

mkdir -p "${repo_root}/third_party"

if [[ -d "${dest_dir}/.git" ]]; then
  echo "[setup_rcutils_source] Existing clone detected; updating tags..."
  git -C "${dest_dir}" fetch --tags --force
  git -C "${dest_dir}" checkout -q "${tag}" || git -C "${dest_dir}" checkout -q "refs/tags/${tag}"
else
  echo "[setup_rcutils_source] Cloning ros2/rcutils..."
  rm -rf "${dest_dir}"
  git clone --depth 1 --branch "${tag}" https://github.com/ros2/rcutils.git "${dest_dir}"
fi

needed_file="${dest_dir}/src/filesystem.c"
if [[ ! -f "${needed_file}" ]]; then
  echo "[setup_rcutils_source] ERROR: expected file missing: ${needed_file}" >&2
  exit 2
fi

echo "[setup_rcutils_source] OK: ${needed_file}"
