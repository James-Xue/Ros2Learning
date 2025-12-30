#!/usr/bin/env bash
set -euo pipefail

# Downloads and extracts the glibc source that matches the host's glibc version.
# This enables stepping into libc startup code (e.g. libc_start_call_main.h) in GDB/VS Code.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP_DIR="$ROOT_DIR/third_party"
SRC_DIR="$TP_DIR/glibc-src"
LINK_DIR="$TP_DIR/glibc"

# Prefer getconf (stable), fall back to ldd output.
GLIBC_VERSION="$(getconf GNU_LIBC_VERSION 2>/dev/null | awk '{print $2}' || true)"
if [[ -z "${GLIBC_VERSION}" ]]; then
  GLIBC_VERSION="$(ldd --version 2>/dev/null | head -n 1 | sed -n 's/.* //p' || true)"
fi

if [[ -z "${GLIBC_VERSION}" ]]; then
  echo "Unable to detect glibc version (getconf/ldd)." >&2
  exit 1
fi

TARBALL="glibc-${GLIBC_VERSION}.tar.xz"
URL="https://ftp.gnu.org/gnu/libc/${TARBALL}"

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

if [[ ! -f "$TARBALL" ]]; then
  echo "Downloading $URL"
  curl -L -o "$TARBALL" "$URL"
fi

if [[ ! -d "glibc-${GLIBC_VERSION}" ]]; then
  echo "Extracting $TARBALL"
  tar -xf "$TARBALL"
fi

# Create/update stable symlink used by launch.json: ${workspaceFolder}/third_party/glibc
rm -f "$LINK_DIR"
ln -s "glibc-src/glibc-${GLIBC_VERSION}" "$LINK_DIR"

echo "glibc source ready: $LINK_DIR -> $(readlink -f "$LINK_DIR")"
