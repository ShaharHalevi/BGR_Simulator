#!/usr/bin/env bash
# Native run helpers: RViz / Gazebo on the host.

set -euo pipefail
CONTAINING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
source "${CONTAINING_DIR}/../lib/common.sh"
# shellcheck source=env.sh
source "${CONTAINING_DIR}/env.sh"
# shellcheck source=build_ws.sh
source "${CONTAINING_DIR}/build_ws.sh"

native_rviz() {
  ensure_ws_built_native_if_needed
  info "Launching RViz natively..."
  cd "${REPO_ROOT}"
  native_source_ros
  # shellcheck disable=SC1091
  source install/setup.bash
  ros2 launch bgr_description display.launch.py
}

native_gazebo() {
  ensure_ws_built_native_if_needed
  info "Launching Gazebo natively..."
  cd "${REPO_ROOT}"
  native_source_ros
  # shellcheck disable=SC1091
  source install/setup.bash
  ros2 launch bgr_description gazebo.launch.py
}

native_all() {
  warn "Native ALL will open Gazebo then RViz. Usually you want two terminals."
  native_gazebo
}
