#!/usr/bin/env bash
# Build the ROS 2 workspace inside Docker (clean build).
# Runs: rm -rf build install log; rosdep; colcon
# Mounts the repo into the container at $BGR_WS_IN_CONTAINER.

set -euo pipefail
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=../lib/common.sh
source "${TOOLS_DIR}/../lib/common.sh"
# shellcheck source=image.sh
source "${TOOLS_DIR}/image.sh"

build_ws_docker_clean() {
  ensure_image
  info "Building workspace in Docker (clean build)..."

  docker run --rm -it \
    -v "${REPO_ROOT}:${BGR_WS_IN_CONTAINER}" \
    "${BGR_IMAGE}" \
    bash -lc "
      set -e
      cd '${BGR_WS_IN_CONTAINER}'
      source /opt/ros/${BGR_ROS_DISTRO}/setup.bash

      rm -rf build install log

      apt update
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y --skip-keys ${BGR_SKIP_KEYS}

      colcon build --cmake-clean-cache
      echo '[OK] Docker build finished.'
    "
}

ensure_ws_built_docker_if_needed() {
  if ws_built_host && ! ws_needs_rebuild_host; then
    info "Workspace already built and up-to-date -> skipping build."
    return 0
  fi

  if ! ws_built_host; then
    warn "Workspace not built (install/setup.bash missing) -> building now..."
  else
    warn "Workspace changed (src newer than install/) -> rebuilding now..."
  fi

  build_ws_docker_clean
}
