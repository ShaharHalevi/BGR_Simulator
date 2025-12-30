#!/usr/bin/env bash
# Entry point for the simulator launcher.
# Supports:
#   - Interactive menu: ./tools/run_gui.sh
#   - CLI mode:
#       ./tools/run_gui.sh docker all
#       ./tools/run_gui.sh docker gazebo
#       ./tools/run_gui.sh docker rviz
#       ./tools/run_gui.sh docker build-image
#       ./tools/run_gui.sh docker build-ws
#       ./tools/run_gui.sh docker shell
#       ./tools/run_gui.sh native all
#       ./tools/run_gui.sh native gazebo
#       ./tools/run_gui.sh native rviz
#       ./tools/run_gui.sh native build-ws

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/common.sh
source "${TOOLS_DIR}/lib/common.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=gui/menu.sh
source "${TOOLS_DIR}/gui/menu.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=docker/image.sh
source "${TOOLS_DIR}/docker/image.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=docker/build_ws.sh
source "${TOOLS_DIR}/docker/build_ws.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=docker/run.sh
source "${TOOLS_DIR}/docker/run.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=native/build_ws.sh
source "${TOOLS_DIR}/native/build_ws.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=native/run.sh
source "${TOOLS_DIR}/native/run.sh"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


usage() {
  cat <<EOF
Usage:
  ${REPO_ROOT}/tools/run_gui.sh              # interactive menu

CLI:
  ${REPO_ROOT}/tools/run_gui.sh docker all
  ${REPO_ROOT}/tools/run_gui.sh docker gazebo
  ${REPO_ROOT}/tools/run_gui.sh docker rviz
  ${REPO_ROOT}/tools/run_gui.sh docker build-image
  ${REPO_ROOT}/tools/run_gui.sh docker build-ws
  ${REPO_ROOT}/tools/run_gui.sh docker shell
  ${REPO_ROOT}/tools/run_gui.sh docker list-images

  ${REPO_ROOT}/tools/run_gui.sh native all
  ${REPO_ROOT}/tools/run_gui.sh native gazebo
  ${REPO_ROOT}/tools/run_gui.sh native rviz
  ${REPO_ROOT}/tools/run_gui.sh native build-ws

Environment overrides (optional):
  BGR_IMAGE=bgr-env:jazzy
  BGR_ROS_DISTRO=jazzy
  BGR_WS_IN_CONTAINER=/ws
  BGR_PERSISTENT_CONTAINER=0|1
  BGR_CONTAINER_NAME=bgr_sim
EOF
}

main() {
  need docker

  if [[ $# -eq 0 ]]; then
    menu_loop
    exit 0
  fi

  local mode="${1:-}"
  local action="${2:-}"

  case "${mode}" in
    -h|--help|help) usage; exit 0 ;;
    docker)
      case "${action}" in
        all) docker_all ;;
        gazebo) docker_gazebo ;;
        rviz) docker_rviz ;;
        build-image) ensure_image ;;
        build-ws) build_ws_docker_clean ;;
        shell) docker_open_shell ;;
        list-images) list_images ;;
        *) usage; err "Unknown docker action: ${action}"; exit 1 ;;
      esac
      ;;
    native)
      case "${action}" in
        all) native_all ;;
        gazebo) native_gazebo ;;
        rviz) native_rviz ;;
        build-ws) native_build_clean ;;
        *) usage; err "Unknown native action: ${action}"; exit 1 ;;
      esac
      ;;
    *)
      usage
      err "Unknown mode: ${mode}"
      exit 1
      ;;
  esac
}

main "$@"
