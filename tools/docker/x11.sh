#!/usr/bin/env bash
# X11/GUI glue for running Qt apps (RViz/Gazebo) from inside Docker.
# Notes:
# - We default to the "xhost" approach (no Xauthority file mount) because it's the simplest for teams.
# - If GUI fails, we print actionable hints.

set -euo pipefail
# shellcheck source=../lib/common.sh
source "${TOOLS_DIR}/../lib/common.sh"

x11_hint() {
  cat <<'EOF'
[X11 NOTE] To run RViz/Gazebo GUI from Docker (host side):
  1) Make sure DISPLAY is set:
       echo $DISPLAY
  2) Allow local docker connections:
       xhost +local:docker
  3) Then Docker should run with:
       --net=host
       -e DISPLAY=$DISPLAY
       -e QT_X11_NO_MITSHM=1
       -e QT_QPA_PLATFORM=xcb
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw

If DISPLAY is empty, GUI apps cannot open.
If you're on Wayland, this still works via XWayland in most setups.
EOF
}

x11_prepare_host() {
  # Only try if DISPLAY exists.
  if [[ -z "${DISPLAY:-}" ]]; then
    err "DISPLAY is empty on host. GUI from Docker will not work."
    x11_hint
    return 1
  fi

  # Try to allow Docker to connect to X server.
  if has xhost; then
    # This command is safe and idempotent (it just adds a rule).
    xhost +local:docker >/dev/null 2>&1 || true
    info "xhost rule applied: +local:docker"
  else
    warn "xhost not found on host. GUI might fail. Install x11-xserver-utils or run via another method."
  fi

  return 0
}

docker_x11_args() {
  # Echo shell-escaped args for `docker run $(docker_x11_args) ...`
  local args=()
  args+=(--net=host)
  args+=(-e "DISPLAY=${DISPLAY}")
  args+=(-e "QT_X11_NO_MITSHM=1")
  # Force Qt to use XCB even on Wayland sessions
  args+=(-e "QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}")
  args+=(-v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
  printf "%q " "${args[@]}"
}
