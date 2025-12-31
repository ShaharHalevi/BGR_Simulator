#!/usr/bin/env bash
# tools/run_gui_.sh
# Entry point for BGR simulator launcher.
# - No business logic here.
# - Loads modules from tools/lib, tools/docker, tools/native, tools/gui.
# - Supports:
#     ./tools/run_gui_.sh                # interactive menu
#     ./tools/run_gui_.sh -h|--help      # print usage
#     ./tools/run_gui_.sh docker <cmd>   # CLI docker commands
#     ./tools/run_gui_.sh native <cmd>   # CLI native commands

set -euo pipefail




declare -r BGR_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# -------------------------
# Load core libs (keep these lightweight)
# -------------------------
# shellcheck source=lib/constants.sh
source "${BGR_TOOLS_DIR}/lib/constants.sh"
# shellcheck source=lib/common.sh
source "${BGR_TOOLS_DIR}/lib/common.sh"
# shellcheck source=lib/validation.sh
source "${BGR_TOOLS_DIR}/lib/validation.sh"
# shellcheck source=lib/usage.sh
source "${BGR_TOOLS_DIR}/lib/usage.sh"
# shellcheck source=lib/args.sh
source "${BGR_TOOLS_DIR}/lib/args.sh"

# -------------------------
# Help MUST run before menu/dispatch
# -------------------------
case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac

# -------------------------
# Load feature modules
# -------------------------
# GUI
# shellcheck source=gui/menu.sh
source "${BGR_TOOLS_DIR}/gui/menu.sh"

# Docker
# shellcheck source=docker/image.sh
source "${BGR_TOOLS_DIR}/docker/image.sh"
# shellcheck source=docker/build_ws.sh
source "${BGR_TOOLS_DIR}/docker/build_ws.sh"
# shellcheck source=docker/x11.sh
source "${BGR_TOOLS_DIR}/docker/x11.sh"
# shellcheck source=docker/executor.sh
#source "${BGR_TOOLS_DIR}/docker/executor.sh"
# shellcheck source=docker/run.sh
source "${BGR_TOOLS_DIR}/docker/run.sh"
# Native
# shellcheck source=native/env.sh
source "${BGR_TOOLS_DIR}/native/env.sh"
# shellcheck source=native/build_ws.sh
source "${BGR_TOOLS_DIR}/native/build_ws.sh"
# shellcheck source=native/executor.sh
#source "${BGR_TOOLS_DIR}/native/executor.sh"
# shellcheck source=native/run.sh
source "${BGR_TOOLS_DIR}/native/run.sh"
# -------------------------
# Dispatch
# -------------------------
dispatch_cli() {
  ###########################################################################
  # dispatch_cli
  #
  # Purpose:
  #   This function is the “traffic cop” for your script.
  #   It looks at what the user typed in the terminal (the CLI arguments),
  #   figures out which *mode* they want (docker/native/menu/help),
  #   then calls the correct function to do the requested action.
  #
  # High-level flow:
  #   1) Parse command-line arguments (MODE, ACTION, EXTRA)
  #   2) If user asked for help -> print usage
  #   3) If user asked for menu (or gave no mode) -> open interactive menu
  #   4) If user chose docker mode -> check docker requirements, then run action
  #   5) If user chose native mode -> check native requirements, then run action
  #   6) If mode/action is unknown -> show error + usage, exit with failure
  #
  # Terms:
  #   - MODE:   how we run things (inside Docker, on the host machine, etc.)
  #   - ACTION: what we want to do (run gazebo, run rviz, build image, etc.)
  #   - EXTRA:  extra leftover arguments (optional; not used here directly)
  #
  ###########################################################################

  # parse_args is expected to read "$@" (all user arguments)
  # and set these variables:
  #   MODE   -> one of: docker | native | menu | help
  #   ACTION -> the sub-command, for example:
  #             all / gazebo / rviz / build-ws / build-image / shell / list-images
  #   EXTRA  -> any remaining args not consumed by the parser
  #
  # Example:
  #   ./script.sh docker gazebo
  #   MODE="docker"
  #   ACTION="gazebo"
  #
  # Note:
  #   If you do not have parse_args implemented yet, you can replace this line
  #   with a simple manual parser (e.g., MODE="$1"; ACTION="$2"; shift 2; EXTRA=("$@")).
  parse_args "$@"

  ###########################################################################
  # Now we decide what to do based on MODE.
  #
  # The `case` statement is like a multi-branch if/else:
  #   case "$MODE" in
  #     value1) ... ;;
  #     value2) ... ;;
  #     *) default ... ;;
  #   esac
  ###########################################################################
  case "${MODE}" in

    #########################################################################
    # MODE = help
    # The user explicitly asked for help.
    # So we print the usage message and finish.
    #########################################################################
    help)
      usage
      ;;

    #########################################################################
    # MODE = menu OR MODE is empty ("")
    #
    # - "menu" means: show an interactive menu (probably a loop with options)
    # - "" (empty) means: user didn’t specify a mode at all, so we default
    #   to menu mode for friendliness.
    #########################################################################
    menu|"")
      menu_loop
      ;;

    # menu_loop is defined in tools/gui/menu.sh:

    #########################################################################
    # MODE = docker
    #
    # This means we want to run everything inside Docker containers.
    # Before doing any docker action, we verify Docker is available and
    # dependencies are satisfied (e.g., docker installed, daemon running).
    #########################################################################
    docker)
      validate_docker_deps

      #######################################################################
      # Inside docker mode, we choose what to do based on ACTION.
      #######################################################################
      case "${ACTION}" in

        # ACTION = all
        # Run the “everything” docker workflow (whatever docker_all does).
        all)
          docker_all
          ;;

        # ACTION = gazebo
        # Start Gazebo in docker environment.
        gazebo)
          docker_gazebo
          ;;

        # ACTION = rviz
        # Start RViz in docker environment.
        rviz)
          docker_rviz
          ;;

        # ACTION = build-image
        # Build the docker image if needed (or ensure it exists).
        build-image)
          ensure_image
          ;;

        # ACTION = build-ws
        # Clean-build the workspace inside docker.
        build-ws)
          build_ws_docker_clean
          ;;

        # ACTION = shell
        # Open an interactive shell inside the docker container.
        shell)
          docker_open_shell
          ;;

        # ACTION = list-images
        # Show the docker images relevant to this project.
        list-images)
          list_images
          ;;

        #####################################################################
        # Default case for docker actions:
        # If the action doesn’t match any known command, print an error,
        # show usage, and return failure (exit code 1).
        #####################################################################
        *)
          err "Unknown docker action: ${ACTION}"
          usage
          return 1
          ;;
      esac
      ;;

    #########################################################################
    # MODE = native
    #
    # This means we want to run everything directly on the machine
    # (not using Docker).
    #
    # Before running any native action, we verify dependencies exist:
    # - correct ROS install?
    # - gazebo/rviz installed?
    # - environment variables set?
    #########################################################################
    native)
      validate_native_deps

      #######################################################################
      # Inside native mode, choose what to do based on ACTION.
      #######################################################################
      case "${ACTION}" in

        # ACTION = all
        # Run the “everything” workflow on the host machine.
        all)
          native_all
          ;;

        # ACTION = gazebo
        # Start Gazebo natively (on the host machine).
        gazebo)
          native_gazebo
          ;;

        # ACTION = rviz
        # Start RViz natively.
        rviz)
          native_rviz
          ;;

        # ACTION = build-ws
        # Clean-build the workspace natively.
        build-ws)
          native_build_clean
          ;;

        #####################################################################
        # Default case for native actions:
        # Unknown action -> error + usage + return failure.
        #####################################################################
        *)
          err "Unknown native action: ${ACTION}"
          usage
          return 1
          ;;
      esac
      ;;

    #########################################################################
    # Default case for MODE:
    # If MODE wasn’t recognized (not help/menu/docker/native),
    # show an error, print usage, and fail.
    #########################################################################
    *)
      err "Unknown mode: ${MODE}"
      usage
      return 1
      ;;
  esac
}


main() {
  # If no args -> interactive menu
  if [[ $# -eq 0 ]]; then
    menu_loop
    exit 0
  fi

  dispatch_cli "$@"
}

main "$@"
