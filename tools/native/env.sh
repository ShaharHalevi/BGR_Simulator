#!/usr/bin/env bash
# Native ROS environment loader (host, not Docker).
# Goal: always source Jazzy if available, and avoid mixing distros (humble/jazzy).

set -euo pipefail
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=../lib/common.sh
source "${TOOLS_DIR}/../lib/common.sh"

native_source_ros() {
  # Prefer explicit BGR_ROS_DISTRO. Fallback to /opt/ros/jazzy.
  if [[ -f "/opt/ros/${BGR_ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${BGR_ROS_DISTRO}/setup.bash"
    return 0
  fi

  if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    return 0
  fi

  err "ROS not found under /opt/ros (expected ${BGR_ROS_DISTRO} or jazzy)"
  return 1
}
