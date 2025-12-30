#!/usr/bin/env bash
# Text menu UI and dispatch. Keep business logic OUT of here.

set -euo pipefail
MENU_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
source "${MENU_DIR}/../lib/common.sh"
# shellcheck source=term.sh
source "${MENU_DIR}/term.sh"

# shellcheck source=../docker/image.sh
source "${MENU_DIR}/../docker/image.sh"
# shellcheck source=../docker/build_ws.sh
source "${MENU_DIR}/../docker/build_ws.sh"
# shellcheck source=../docker/run.sh
source "${MENU_DIR}/../docker/run.sh"

# shellcheck source=../native/build_ws.sh
source "${MENU_DIR}/../native/build_ws.sh"
# shellcheck source=../native/run.sh
source "${MENU_DIR}/../native/run.sh"

docker_rviz() {
  ensure_ws_built_docker_if_needed
  info "Launching RViz via Docker..."
  docker_gui_run "
    set -e
    cd '${BGR_WS_IN_CONTAINER}'
    source /opt/ros/${BGR_ROS_DISTRO}/setup.bash
    source install/setup.bash
    ros2 launch bgr_description display.launch.py
  "
}

docker_gazebo() {
  ensure_ws_built_docker_if_needed
  info "Launching Gazebo via Docker..."
  docker_gui_run "
    set -e
    cd '${BGR_WS_IN_CONTAINER}'
    source /opt/ros/${BGR_ROS_DISTRO}/setup.bash
    source install/setup.bash
    ros2 launch bgr_description gazebo.launch.py
  "
}

docker_all() {
  info "Start ALL (Docker): image + build (if needed) + Gazebo + RViz"

  ensure_image
  ensure_ws_built_docker_if_needed

  warn "We will run Gazebo and RViz in TWO terminals (recommended)."

  local self="${REPO_ROOT}/tools/run_gui.sh"

  if open_new_terminal "cd '${REPO_ROOT}' && '${self}' docker gazebo"; then
    info "Gazebo started in a new terminal."
  else
    warn "Couldn't open a new terminal automatically."
    warn "Run Gazebo in another terminal with:"
    warn "  ${self} docker gazebo"
  fi

  # RViz in current terminal
  docker_rviz
}

native_all() {
  info "Start ALL (Native): build (if needed) + Gazebo + RViz"
  warn "We will run Gazebo and RViz in TWO terminals (recommended)."

  local self="${REPO_ROOT}/tools/run_gui.sh"

  if open_new_terminal "cd '${REPO_ROOT}' && '${self}' native gazebo"; then
    info "Native Gazebo started in a new terminal."
  else
    warn "Couldn't open a new terminal automatically."
    warn "Run native Gazebo in another terminal with:"
    warn "  ${self} native gazebo"
  fi

  native_rviz
}

print_menu() {
  cat <<EOF

BGR Simulator launcher (text menu)

--- Quick start ---
1) Start ALL (Docker): build if needed + Gazebo + RViz
2) Start ALL (Native): build if needed + Gazebo + RViz

--- Docker tools ---
3) Check / Build Docker image (${BGR_IMAGE})
4) Build workspace (Docker) [clean: rosdep + colcon]
5) Open shell inside Docker
6) List Docker images

--- Run with Docker ---
7) Run Gazebo (Docker)
8) Run RViz  (Docker)

--- Native tools / run ---
9)  Build workspace (Native) [clean: rosdep + colcon]
10) Run Gazebo (Native)
11) Run RViz  (Native)

0) Exit

EOF
}

menu_loop() {
  while true; do
    print_menu
    read -rp "Choose [0-11]: " choice
    case "${choice}" in
      1) docker_all ;;
      2) native_all ;;

      3) ensure_image ;;
      4) build_ws_docker_clean ;;
      5) docker_open_shell ;;
      6) list_images ;;

      7) docker_gazebo ;;
      8) docker_rviz ;;

      9) native_build_clean ;;
      10) native_gazebo ;;
      11) native_rviz ;;

      0) exit 0 ;;
      *) warn "Invalid choice: ${choice}" ;;
    esac
  done
}
