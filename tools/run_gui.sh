#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

IMAGE="bgr-env:jazzy"
WS_IN_CONTAINER="/ws"

has() { command -v "$1" >/dev/null 2>&1; }
need() { has "$1" || { echo "[ERROR] Missing dependency: $1"; exit 1; }; }

info() { echo -e "\033[1;34m[run_gui]\033[0m $*"; }
warn() { echo -e "\033[1;33m[run_gui]\033[0m $*"; }
err()  { echo -e "\033[1;31m[run_gui]\033[0m $*" >&2; }

# -------------------------
# Docker image
# -------------------------
image_exists() { docker image inspect "$IMAGE" >/dev/null 2>&1; }

ensure_image() {
  if image_exists; then
    info "Docker image exists: $IMAGE"
  else
    info "Docker image NOT found: $IMAGE"
    info "Building now: docker build -t $IMAGE ."
    docker build -t "$IMAGE" .
    info "Build done: $IMAGE"
  fi
}

# -------------------------
# Workspace build (Docker)
# -------------------------
ws_built_host() { [[ -f "$REPO_ROOT/install/setup.bash" ]]; }

build_ws_docker_clean() {
  ensure_image
  info "Building workspace in Docker (clean build)..."

  docker run --rm -it \
    --net=host \
    -v "$REPO_ROOT":"$WS_IN_CONTAINER" \
    "$IMAGE" \
    bash -lc "
      set -e
      cd '$WS_IN_CONTAINER'
      source /opt/ros/jazzy/setup.bash
      rm -rf build install log
      apt update
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y --skip-keys robot_state_publisher_gui
      colcon build --cmake-clean-cache
      echo '[OK] Docker build finished.'
    "
}

ensure_ws_built_docker_if_needed() {
  if ws_built_host; then
    info "Workspace already built (install/setup.bash exists) -> skipping build."
  else
    warn "Workspace not built (install/setup.bash missing) -> building now..."
    build_ws_docker_clean
  fi
}

# -------------------------
# X11 for Docker GUI
# -------------------------
x11_hint() {
  cat <<'EOF'
[X11 NOTE] To run RViz/Gazebo GUI from Docker:
  Host (once): xhost +local:docker
  Then run with:
    --net=host
    -e DISPLAY=$DISPLAY
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
If DISPLAY is empty, Docker GUI will not work.
EOF
}

docker_gui_run() {
  ensure_image

  if [[ -z "${DISPLAY:-}" ]]; then
    err "DISPLAY is empty. Cannot launch GUI apps from Docker."
    x11_hint
    exit 1
  fi

  docker run --rm -it \
    --net=host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$REPO_ROOT":"$WS_IN_CONTAINER" \
    "$IMAGE" \
    bash -lc "$*"
}

# -------------------------
# Docker run commands
# -------------------------
docker_shell() {
  ensure_image
  info "Opening shell inside Docker (repo mounted at $WS_IN_CONTAINER)"
  docker run --rm -it --net=host -v "$REPO_ROOT":"$WS_IN_CONTAINER" "$IMAGE" bash
}

docker_images_list() {
  info "Docker images:"
  docker images
}

rviz_docker() {
  ensure_ws_built_docker_if_needed
  info "Launching RViz via Docker..."
  x11_hint
  docker_gui_run "
    set -e
    cd '$WS_IN_CONTAINER'
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch bgr_description display.launch.py
  "
}

gazebo_docker() {
  ensure_ws_built_docker_if_needed
  info "Launching Gazebo via Docker..."
  x11_hint
  docker_gui_run "
    set -e
    cd '$WS_IN_CONTAINER'
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch bgr_description gazebo.launch.py
  "
}

start_all_docker() {
  info "Start ALL (Docker): ensure image + build (if needed) + Gazebo + RViz"
  ensure_image
  ensure_ws_built_docker_if_needed

  warn "This will open Gazebo and RViz as separate Docker runs."
  warn "You will need TWO terminals/windows. We'll launch Gazebo first, then RViz."
  warn "Tip: if GUI doesn't open, run on host: xhost +local:docker"

  # Gazebo blocks the terminal; so we launch Gazebo in a new terminal if possible
  if has gnome-terminal; then
    gnome-terminal -- bash -lc "
      cd '$REPO_ROOT'
      $0 __run_gazebo_docker
      exec bash
    " >/dev/null 2>&1 || true
  else
    warn "gnome-terminal not found -> Gazebo will run in current terminal."
    gazebo_docker
    return
  fi

  # RViz in current terminal (or user can open separately)
  rviz_docker
}

# internal helper (so we can spawn Gazebo in a new terminal easily)
if [[ "${1:-}" == "__run_gazebo_docker" ]]; then
  gazebo_docker
  exit 0
fi

# -------------------------
# Native build + run
# -------------------------
native_build_clean() {
  info "Building workspace natively (clean build)..."
  bash -lc "
    set -e
    cd '$REPO_ROOT'

    if [ -n \"\${ROS_DISTRO:-}\" ] && [ -f \"/opt/ros/\$ROS_DISTRO/setup.bash\" ]; then
      source \"/opt/ros/\$ROS_DISTRO/setup.bash\"
    elif [ -f \"/opt/ros/jazzy/setup.bash\" ]; then
      source /opt/ros/jazzy/setup.bash
    else
      echo '[ERROR] ROS not found under /opt/ros'
      exit 1
    fi

    rm -rf build install log
    sudo apt update
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y --skip-keys robot_state_publisher_gui
    colcon build --cmake-clean-cache
    echo '[OK] Native build finished.'
  "
}

native_rviz() {
  info "Launching RViz natively..."
  if [[ ! -f "$REPO_ROOT/install/setup.bash" ]]; then
    err "Missing install/setup.bash. Run: Build workspace (Native) first."
    exit 1
  fi
  bash -lc "
    set -e
    cd '$REPO_ROOT'
    if [ -n \"\${ROS_DISTRO:-}\" ] && [ -f \"/opt/ros/\$ROS_DISTRO/setup.bash\" ]; then
      source \"/opt/ros/\$ROS_DISTRO/setup.bash\"
    elif [ -f \"/opt/ros/jazzy/setup.bash\" ]; then
      source /opt/ros/jazzy/setup.bash
    else
      echo '[ERROR] ROS not found under /opt/ros'
      exit 1
    fi
    source install/setup.bash
    ros2 launch bgr_description display.launch.py
  "
}

native_gazebo() {
  info "Launching Gazebo natively..."
  if [[ ! -f "$REPO_ROOT/install/setup.bash" ]]; then
    err "Missing install/setup.bash. Run: Build workspace (Native) first."
    exit 1
  fi
  bash -lc "
    set -e
    cd '$REPO_ROOT'
    if [ -n \"\${ROS_DISTRO:-}\" ] && [ -f \"/opt/ros/\$ROS_DISTRO/setup.bash\" ]; then
      source \"/opt/ros/\$ROS_DISTRO/setup.bash\"
    elif [ -f \"/opt/ros/jazzy/setup.bash\" ]; then
      source /opt/ros/jazzy/setup.bash
    else
      echo '[ERROR] ROS not found under /opt/ros'
      exit 1
    fi
    source install/setup.bash
    ros2 launch bgr_description gazebo.launch.py
  "
}

# -------------------------
# Menu
# -------------------------
menu() {
  cat <<EOF

BGR Simulator launcher (text menu)

--- Quick start ---
1) Start ALL (Docker): build if needed + Gazebo + RViz

--- Docker tools ---
2) Check / Build Docker image ($IMAGE)
3) Build workspace (Docker) [clean: rosdep + colcon]
4) Open shell inside Docker
5) List Docker images

--- Run with Docker ---
6) Run Gazebo (Docker)
7) Run RViz  (Docker)

--- Native tools / run ---
8) Build workspace (Native) [clean: rosdep + colcon]
9) Run Gazebo (Native)
10) Run RViz  (Native)

0) Exit

EOF
  read -rp "Choose [0-10]: " choice
  case "$choice" in
    1) start_all_docker ;;
    2) ensure_image ;;
    3) build_ws_docker_clean ;;
    4) docker_shell ;;
    5) docker_images_list ;;
    6) gazebo_docker ;;
    7) rviz_docker ;;
    8) native_build_clean ;;
    9) native_gazebo ;;
    10) native_rviz ;;
    0) exit 0 ;;
    *) warn "Invalid choice" ;;
  esac
}

main() {
  need docker
  while true; do menu; done
}

main
