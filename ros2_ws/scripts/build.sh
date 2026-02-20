#!/usr/bin/env bash
set -euo pipefail

# 构建当前 colcon 工作空间
# 用法：
#   ./scripts/build.sh [ros_distro]
# 例如：
#   ./scripts/build.sh jazzy

# 优先级：参数 > 环境变量 ROS_DISTRO > 默认 jazzy
ROS_DISTRO_DEFAULT="jazzy"
ROS_DISTRO_ARG="${1:-}"
ROS_DISTRO_ENV="${ROS_DISTRO:-}"
ROS_DISTRO="${ROS_DISTRO_ARG:-${ROS_DISTRO_ENV:-$ROS_DISTRO_DEFAULT}}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # ROS setup scripts may reference unset vars; don't let `set -u` break sourcing.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "[build] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found. Please install ROS 2 (${ROS_DISTRO}) first."
  exit 1
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

MERGED_COMPILE_COMMANDS="${WORKSPACE_DIR}/compile_commands.json"

merge_compile_commands() {
  local output_file="$1"
  local first=1

  {
    echo "["
    while IFS= read -r file; do
      # Strip outer [] and merge entries from each package compile_commands.
      local entries
      entries="$(sed -e '1d' -e '$d' "${file}")"
      if [ -z "${entries}" ]; then
        continue
      fi
      if [ "${first}" -eq 0 ]; then
        echo ","
      fi
      printf "%s" "${entries}"
      first=0
    done < <(find "${WORKSPACE_DIR}/build" -mindepth 2 -maxdepth 2 -name compile_commands.json -type f | sort)
    echo
    echo "]"
  } > "${output_file}"
}

colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

merge_compile_commands "${MERGED_COMPILE_COMMANDS}"

echo "[build] Build finished. To use:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo "[build] compile_commands merged to: ${MERGED_COMPILE_COMMANDS}"
