#!/bin/bash

# ==========================================
# BGR Simulation General Launch Manager
# ==========================================

# Restrict ROS 2 and Gazebo discovery to the local machine to avoid network crosstalk
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1

WORLDS_DIR="src/bgr_description/worlds"

# 1. Choose Simulation Mode
echo -e "\e[1;36m====================================================\e[0m"
echo -e "\e[1;36m           BGR Simulation Launch Manager            \e[0m"
echo -e "\e[1;36m====================================================\e[0m"
echo -e "\e[1;32mSelect Simulation Mode:\e[0m"
echo "  1) Keyboard Teleoperation (Manual Control)"
echo "  2) Simulation Only (External Controller Required)"
echo

read -rp "Choose mode (1-2): " mode_choice
echo

case "$mode_choice" in
    1)
        mode="keyboard"
        ;;
    2)
        mode="sim"
        ;;
    *)
        echo -e "\e[1;31mInvalid mode selection. Exiting.\e[0m" >&2
        exit 1
        ;;
esac

# 2. Scan and list available world maps
if [ ! -d "$WORLDS_DIR" ]; then
    echo -e "\e[1;31mError: Worlds directory '$WORLDS_DIR' not found!\e[0m" >&2
    exit 1
fi

# Load world files containing 'Opt' into an array (sorted alphabetically)
mapfile -t WORLDS < <(find "$WORLDS_DIR" -maxdepth 1 -name "*Opt*.world" -printf "%f\n" | sort)

if [ ${#WORLDS[@]} -eq 0 ]; then
    echo -e "\e[1;31mError: No optimized (*Opt*.world) files found in '$WORLDS_DIR'!\e[0m" >&2
    exit 1
fi

echo -e "\e[1;32mSelect World Map:\e[0m"
for i in "${!WORLDS[@]}"; do
    printf "  %2d) %s\n" $((i + 1)) "${WORLDS[i]}"
done
echo

read -rp "Enter choice (1-${#WORLDS[@]}): " world_choice
echo

if [[ ! "$world_choice" =~ ^[0-9]+$ ]] || [ "$world_choice" -lt 1 ] || [ "$world_choice" -gt "${#WORLDS[@]}" ]; then
    echo -e "\e[1;31mInvalid map selection. Exiting.\e[0m" >&2
    exit 1
fi

SELECTED_WORLD="${WORLDS[$((world_choice - 1))]}"

# 3. Build Workspace if required
CLEAN_BUILD=false

check_build_required() {
    CLEAN_BUILD=false

    # 1. If install/setup.bash is missing, build is mandatory and must be clean
    if [ ! -f install/setup.bash ]; then
        CLEAN_BUILD=true
        return 0
    fi

    # 2. Check if git commit hash has changed since the last build
    if [ -d .git ] && command -v git &>/dev/null; then
        local current_commit=$(git rev-parse HEAD 2>/dev/null)
        if [ -f install/.last_built_commit ]; then
            local last_commit=$(cat install/.last_built_commit 2>/dev/null)
            if [ "$current_commit" != "$last_commit" ]; then
                echo -e "\e[1;33mGit commit mismatch detected (switched branch/commit). Clean build required.\e[0m"
                CLEAN_BUILD=true
                return 0
            fi
        else
            echo -e "\e[1;33mLast built commit tracking file missing. Clean build required.\e[0m"
            CLEAN_BUILD=true
            return 0
        fi
    fi

    # 3. Check if any source files are newer than the last build setup.bash file
    if [ -d src ]; then
        if [ -n "$(find src -type f -newer install/setup.bash -print -quit 2>/dev/null)" ]; then
            echo -e "\e[1;33mSource code modification detected. Quick incremental build required.\e[0m"
            CLEAN_BUILD=false
            return 0
        fi
    fi

    return 1 # Build not required
}

if check_build_required; then
    echo -e "\e[1;33mBuilding Workspace...\e[0m"
    if [ "$CLEAN_BUILD" = "true" ]; then
        echo "Performing clean build..."
        rm -rf build/ install/ log/
    else
        echo "Performing incremental build..."
    fi
    # Reset environment variables pointing to deleted install paths to prevent colcon warnings
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    source /opt/ros/jazzy/setup.bash
    colcon build || { echo -e "\e[1;31mError: Workspace build failed!\e[0m" >&2; exit 1; }
    
    # Save the commit hash of the successful build
    if [ -d .git ] && command -v git &>/dev/null; then
        git rev-parse HEAD > install/.last_built_commit 2>/dev/null
    fi
else
    echo -e "\e[1;32mWorkspace is up to date. Skipping build.\e[0m"
fi

# 4. Launch selected mode and map
echo -e "\e[1;32mLaunching $mode mode on map '$SELECTED_WORLD'...\e[0m"
sleep 1

if [ "$mode" = "keyboard" ]; then
    exec ./scripts/keyboard_launch.sh "$SELECTED_WORLD"
else
    exec ./scripts/sim_launch.sh "$SELECTED_WORLD"
fi
