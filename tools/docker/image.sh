#!/usr/bin/env bash
# Docker image helpers: check / build / list

set -euo pipefail
FATHER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
source "${FATHER_DIR}/../lib/common.sh"

image_exists() { docker image inspect "${BGR_IMAGE}" >/dev/null 2>&1; }

ensure_image() {
  if image_exists; then
    info "Docker image exists: ${BGR_IMAGE}"
    return 0
  fi

  warn "Docker image NOT found: ${BGR_IMAGE}"
  info "Building image from repo root Dockerfile:"
  info "  docker build -t ${BGR_IMAGE} ."

  (cd "${REPO_ROOT}" && docker build -t "${BGR_IMAGE}" .)

  info "Build done: ${BGR_IMAGE}"
}

list_images() {
  info "Docker images:"
  docker images
}
