#!/usr/bin/env bash
# Terminal helpers (host side): spawn a new terminal window if available.

set -euo pipefail
TERM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
source "${TERM_DIR}/../lib/common.sh"

open_new_terminal() {
  # Usage: open_new_terminal "command to run"
  local cmd="$1"

  if has gnome-terminal; then
    gnome-terminal -- bash -lc "${cmd}; exec bash" >/dev/null 2>&1 || return 1
    return 0
  fi

  if has x-terminal-emulator; then
    x-terminal-emulator -e bash -lc "${cmd}; exec bash" >/dev/null 2>&1 || return 1
    return 0
  fi

  if has konsole; then
    konsole -e bash -lc "${cmd}; exec bash" >/dev/null 2>&1 || return 1
    return 0
  fi

  return 1
}
