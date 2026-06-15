#!/bin/bash

# ==============================================================================
# BGR Simulation Docker Launch Manager
# ==============================================================================
#
# Environment Variables Options:
#
#   1. MODE (Simulation type)
#      - sim       : Simulation only (default)
#      - keyboard  : Manual keyboard teleoperation
#
#   2. HEADLESS (Gazebo GUI mode)
#      - true      : Headless mode (no Gazebo UI, default)
#      - false     : Headed mode (renders Gazebo UI)
#
#   3. WORLD (Map selection)
#      - Map1Opt.world             (Default - optimized Map 1)
#      - Map2Opt.world             (Optimized Map 2)
#      - Map3Opt.world             (Optimized Map 3)
#      - SkidpadOpt.world          (Optimized Skidpad)
#      - AccelerationOpt.world     (Optimized Acceleration track)
#      - TrainingMapOpt.world      (Optimized Training Map)
#      - MapTestday1Opt.world      (Optimized Test Day Map 1)
#      - MapTestday2Opt.world      (Optimized Test Day Map 2)
#      - MapTestday3Opt.world      (Optimized Test Day Map 3)
#
# ==============================================================================

# Restrict ROS 2 and Gazebo discovery to the local machine to avoid network crosstalk
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1

WORLDS_DIR="src/bgr_description/worlds"

# Load environment variables with defaults
HEADLESS="${HEADLESS:-true}"
WORLD="${WORLD:-Map1Opt.world}"
MODE="${MODE:-sim}" # Options: 'sim' (simulation only) or 'keyboard' (manual teleop)

# Validate selected mode
if [ "$MODE" != "keyboard" ] && [ "$MODE" != "sim" ]; then
    echo -e "\e[1;31mError: Invalid MODE '$MODE'. Allowed values are 'sim' or 'keyboard'.\e[0m" >&2
    exit 1
fi

# Validate world file exists
if [ ! -f "$WORLDS_DIR/$WORLD" ] && [ ! -f "$WORLD" ]; then
    echo -e "\e[1;31mError: World file '$WORLD' not found in $WORLDS_DIR!\e[0m" >&2
    exit 1
fi

# Clean and rebuild the workspace in the current directory
rm -rf build/ install/ log/
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
colcon build

# Source the ROS 2 setup and the workspace setup
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch selected mode
if [ "$MODE" = "keyboard" ]; then
    exec ./scripts/keyboard_launch.sh "$WORLD" "$HEADLESS"
else
    exec ./scripts/sim_launch.sh "$WORLD" "$HEADLESS"
fi
