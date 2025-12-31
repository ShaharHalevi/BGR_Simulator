#!/usr/bin/env bash
# Text menu UI and dispatch.
# Single GUI: first choose MODE (Docker/Native), then show only relevant actions.

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

MODE=""  # docker|native

# -------------------------
# Menus
# -------------------------
print_mode_menu() {
  cat <<EOF

BGR Simulator launcher

Choose mode:
1) Docker (recommended)
2) Native
0) Exit

EOF
}

print_docker_menu() {
  cat <<EOF

BGR Simulator launcher (Docker)

--- Quick start ---
1) Start ALL (Docker): build if needed + Gazebo + RViz

--- Docker tools ---
2) Check / Build Docker image (${BGR_IMAGE})
3) Build workspace (Docker) [clean: rosdep + colcon]
4) Open shell inside Docker
5) List Docker images

--- Run with Docker ---
6) Run Gazebo (Docker)
7) Run RViz  (Docker)

8) Back (choose mode again)
0) Exit

EOF
}

print_native_menu() {
  cat <<EOF

BGR Simulator launcher (Native)

--- Quick start ---
1) Start ALL (Native): build if needed + Gazebo + RViz

--- Native tools / run ---
2) Build workspace (Native) [clean: rosdep + colcon]
3) Run Gazebo (Native)
4) Run RViz  (Native)

5) Back (choose mode again)
0) Exit

EOF
}

# -------------------------
# Mode selection
# -------------------------
choose_mode_loop() {
  while true; do
    print_mode_menu
    read -rp "Choose [0-2]: " c
    case "$c" in
      1) MODE="docker"; return 0 ;;
      2) MODE="native"; return 0 ;;
      0) exit 0 ;;
      *) warn "Invalid choice: $c" ;;
    esac
  done
}

# -------------------------
# Docker loop
# -------------------------
docker_loop() {
  while true; do
    print_docker_menu
    read -rp "Choose [0-8]: " choice
    case "${choice}" in
      1) docker_all ;;

      2) ensure_image ;;
      3) build_ws_docker_clean ;;
      4) docker_open_shell ;;
      5) list_images ;;

      6) docker_gazebo ;;
      7) docker_rviz ;;

      8) MODE=""; return 0 ;;
      0) exit 0 ;;
      *) warn "Invalid choice: ${choice}" ;;
    esac
  done
}

# -------------------------
# Native loop
# -------------------------
native_loop() {
  while true; do
    print_native_menu
    read -rp "Choose [0-5]: " choice
    case "${choice}" in
      1) native_all ;;

      2) native_build_clean ;;
      3) native_gazebo ;;
      4) native_rviz ;;

      5) MODE=""; return 0 ;;
      0) exit 0 ;;
      *) warn "Invalid choice: ${choice}" ;;
    esac
  done
}

# -------------------------
# Public entry
# -------------------------
menu_loop() {
  while true; do
    if [[ -z "${MODE}" ]]; then
      choose_mode_loop
    fi

    case "${MODE}" in
      docker) docker_loop ;;
      native) native_loop ;;
      *)
        MODE=""
        warn "Unknown MODE '${MODE}', resetting..."
        ;;
    esac
  done
}
