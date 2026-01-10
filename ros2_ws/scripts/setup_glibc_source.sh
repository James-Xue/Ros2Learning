#!/usr/bin/env bash
set -euo pipefail

# Downloads and extracts the glibc source that matches the host's glibc version.
# This enables stepping into libc startup code (e.g. __libc_start_main, libc_start_call_main.h)
# and the dynamic loader (ld.so) when debugging ROS2 nodes with gdb/VS Code.
#
# Usage:
#   ./ros2_ws/scripts/setup_glibc_source.sh

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
third_party_dir="${repo_root}/third_party"
src_dir="${third_party_dir}/glibc-src"
link_dir="${third_party_dir}/glibc"

# Prefer getconf (stable), fall back to ldd output.
glibc_version="$(getconf GNU_LIBC_VERSION 2>/dev/null | awk '{print $2}' || true)"
if [[ -z "${glibc_version}" ]]; then
  glibc_version="$(ldd --version 2>/dev/null | head -n 1 | sed -n 's/.* //p' || true)"
fi

if [[ -z "${glibc_version}" ]]; then
  echo "[setup_glibc_source] ERROR: unable to detect glibc version (getconf/ldd)." >&2
  exit 1
fi

tarball="glibc-${glibc_version}.tar.xz"
url="https://ftp.gnu.org/gnu/libc/${tarball}"

echo "[setup_glibc_source] glibc version: ${glibc_version}"

mkdir -p "${src_dir}"
cd "${src_dir}"

if [[ ! -f "${tarball}" ]]; then
  echo "[setup_glibc_source] Downloading ${url}"
  curl -L -o "${tarball}" "${url}"
fi

if [[ ! -d "glibc-${glibc_version}" ]]; then
  echo "[setup_glibc_source] Extracting ${tarball}"
  tar -xf "${tarball}"
fi

# Create/update stable symlink used by VS Code launch.json: ${repo_root}/third_party/glibc
rm -f "${link_dir}"
ln -s "glibc-src/glibc-${glibc_version}" "${link_dir}"

echo "[setup_glibc_source] OK: ${link_dir} -> $(readlink -f "${link_dir}")"
