#!/usr/bin/env bash
# Build workspace on the host (native).
# Note: Native mode is optional and may be brittle if your host has mixed ROS distros or Snap conflicts.

set -euo pipefail

# Absolute dir of THIS file: .../tools/native
NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# tools/ directory
TOOLS_DIR="$(cd "${NATIVE_DIR}/.." && pwd)"

# Repo root: parent of tools/
REPO_ROOT="$(cd "${TOOLS_DIR}/.." && pwd)"

# Common utilities
source "${TOOLS_DIR}/lib/common.sh"

# Native env helpers (native_source_ros, etc.)
source "${NATIVE_DIR}/env.sh"

native_build_clean() {
  info "Building workspace natively (clean build)..."
  cd "${REPO_ROOT}"
  native_source_ros

  rm -rf build install log
  sudo apt update
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y --skip-keys ${BGR_SKIP_KEYS}
  colcon build --cmake-clean-cache

  info "[OK] Native build finished."
}

ensure_ws_built_native_if_needed() {
  if ws_built_host && ! ws_needs_rebuild_host; then
    info "Workspace already built and up-to-date -> skipping build."
    return 0
  fi

  if ! ws_built_host; then
    warn "Workspace not built (install/setup.bash missing) -> building now..."
  else
    warn "Workspace changed (src newer than install/) -> rebuilding now..."
  fi

  native_build_clean
}
