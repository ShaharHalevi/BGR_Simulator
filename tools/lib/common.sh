#!/usr/bin/env bash
# Common utilities shared by all launcher scripts.
# Keep this file small and boring: logging, checks, repo root discovery, constants.

set -euo pipefail

# Repo root = parent of "tools/"
COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${COMMON_DIR}/../.." && pwd)"

# ---- Defaults (override via env vars if needed) ----
: "${BGR_IMAGE:=bgr-env:jazzy}"
: "${BGR_ROS_DISTRO:=jazzy}"
: "${BGR_WS_IN_CONTAINER:=/ws}"
: "${BGR_CONTAINER_NAME:=bgr_sim}"         # used only if you choose persistent container mode
: "${BGR_PERSISTENT_CONTAINER:=0}"         # 0 = use `docker run --rm` (recommended), 1 = keep a named container
: "${BGR_SKIP_KEYS:=robot_state_publisher_gui}"  # rosdep skip keys (can be comma-separated, used as-is)

# ---- Helpers ----
has() { command -v "$1" >/dev/null 2>&1; }

need() {
  local bin="$1"
  has "$bin" || { echo "[ERROR] Missing dependency: $bin" >&2; exit 1; }
}

info() { echo -e "\033[1;34m[bgr]\033[0m $*"; }
warn() { echo -e "\033[1;33m[bgr]\033[0m $*"; }
err()  { echo -e "\033[1;31m[bgr]\033[0m $*" >&2; }

# Prints a quoted command-line (useful for debug logs)
q() { printf "%q " "$@"; }

# --- Workspace helpers ---
ws_install_file() { echo "${REPO_ROOT}/install/setup.bash"; }

ws_built_host() { [[ -f "$(ws_install_file)" ]]; }

# True if any file under src/ is newer than install/setup.bash (very lightweight "needs rebuild" check).
ws_needs_rebuild_host() {
  local install="$(ws_install_file)"
  [[ -f "$install" ]] || return 0

  # If src is missing, assume no rebuild needed (but your builds won't do anything).
  [[ -d "${REPO_ROOT}/src" ]] || return 1

  # find newest mtime under src (ignore dot dirs)
  local newest
  newest="$(find "${REPO_ROOT}/src" -type f -not -path '*/\.*' -printf '%T@\n' 2>/dev/null | sort -nr | head -n1 || true)"
  [[ -n "$newest" ]] || return 1

  local install_mtime
  install_mtime="$(stat -c '%Y' "$install" 2>/dev/null || echo 0)"

  # newest is float seconds; compare as int
  local newest_i="${newest%.*}"
  [[ "$newest_i" -gt "$install_mtime" ]]
}
