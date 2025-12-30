#!/usr/bin/env bash
# Docker run helpers: run GUI commands, open shell, optional persistent container.

set -euo pipefail
CONTAINING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
source "${CONTAINING_DIR}/../lib/common.sh"
# shellcheck source=image.sh
source "${CONTAINING_DIR}/image.sh"
# shellcheck source=x11.sh
source "${CONTAINING_DIR}/x11.sh"

container_exists() { docker ps -a --format '{{.Names}}' | grep -qx "${BGR_CONTAINER_NAME}"; }
container_running() { docker ps --format '{{.Names}}' | grep -qx "${BGR_CONTAINER_NAME}"; }

ensure_persistent_container() {
  ensure_image

  if container_running; then
    info "Persistent container running: ${BGR_CONTAINER_NAME}"
    return 0
  fi

  if container_exists; then
    info "Starting existing container: ${BGR_CONTAINER_NAME}"
    docker start "${BGR_CONTAINER_NAME}" >/dev/null
    return 0
  fi

  info "Creating persistent container: ${BGR_CONTAINER_NAME}"
  # Run idle; commands will be executed via docker exec.
  docker run -d \
    --name "${BGR_CONTAINER_NAME}" \
    -v "${REPO_ROOT}:${BGR_WS_IN_CONTAINER}" \
    "${BGR_IMAGE}" \
    bash -lc "tail -f /dev/null" >/dev/null

  info "Container created and running: ${BGR_CONTAINER_NAME}"
}

docker_run_oneoff() {
  # Usage: docker_run_oneoff "bash -lc '...'"
  ensure_image
  docker run --rm -it \
    -v "${REPO_ROOT}:${BGR_WS_IN_CONTAINER}" \
    "${BGR_IMAGE}" \
    bash -lc "$1"
}

docker_exec_persistent() {
  # Usage: docker_exec_persistent "bash -lc '...'"
  ensure_persistent_container
  docker exec -it "${BGR_CONTAINER_NAME}" bash -lc "$1"
}

docker_run_cmd() {
  # Decides between one-off and persistent container mode.
  local cmd="$1"
  if [[ "${BGR_PERSISTENT_CONTAINER}" == "1" ]]; then
    docker_exec_persistent "${cmd}"
  else
    docker_run_oneoff "${cmd}"
  fi
}

docker_open_shell() {
  ensure_image
  if [[ "${BGR_PERSISTENT_CONTAINER}" == "1" ]]; then
    ensure_persistent_container
    info "Opening shell in persistent container: ${BGR_CONTAINER_NAME}"
    docker exec -it "${BGR_CONTAINER_NAME}" bash
  else
    info "Opening shell in one-off container (repo mounted at ${BGR_WS_IN_CONTAINER})"
    docker run --rm -it -v "${REPO_ROOT}:${BGR_WS_IN_CONTAINER}" "${BGR_IMAGE}" bash
  fi
}

docker_gui_run() {
  # Runs GUI apps (RViz/Gazebo) with X11 args. Always one-off to avoid GUI state headaches.
  ensure_image
  x11_prepare_host

  if [[ -z "${DISPLAY:-}" ]]; then
    err "DISPLAY is empty. Cannot launch GUI apps from Docker."
    x11_hint
    exit 1
  fi

  local x11_args
  x11_args="$(docker_x11_args)"

  # shellcheck disable=SC2086
  docker run --rm -it \
    ${x11_args} \
    -v "${REPO_ROOT}:${BGR_WS_IN_CONTAINER}" \
    "${BGR_IMAGE}" \
    bash -lc "$1"
}
