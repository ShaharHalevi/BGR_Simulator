#!/bin/bash

# =====================================================
# Automated Simulation Startup Without Keyboard Control
# =====================================================

# Restrict ROS 2 and Gazebo discovery to the local machine to avoid network crosstalk with other systems
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1

# World selection parameter (defaults to Map1Opt.world if not provided as $1)
WORLDS_DIR="src/bgr_description/worlds"
WORLD_NAME="${1:-Map1Opt.world}"
HEADLESS_MODE="${2:-false}"

# Verify if the world file actually exists
if [ ! -f "$WORLDS_DIR/$WORLD_NAME" ] && [ ! -f "$WORLD_NAME" ]; then
    echo -e "\e[1;31mError: World file '$WORLD_NAME' not found in $WORLDS_DIR!\e[0m" >&2
    exit 1
fi

# Helper function to forcefully terminate all simulator processes and clear shared memory
nuclear_sweep() {
    docker stop bgr_simulator 2>/dev/null || true
    killall -9 ros2 ros2-daemon gz ruby rviz2 parameter_bridge 2>/dev/null
    killall -9 gz-sim-server gz-sim-gui ign-gazebo-server ign-gazebo-gui 2>/dev/null
    killall -9 robot_state_publisher static_transform_publisher ros2_control_node 2>/dev/null
    pkill -9 -f "bgr_description" 2>/dev/null
    pkill -9 -f "bgr_controller" 2>/dev/null
    pkill -9 -f "controller_manager/spawner" 2>/dev/null
    pkill -9 -f "car_dashboard.py" 2>/dev/null
    pkill -9 -f "keyboard_teleop" 2>/dev/null
    rm -rf ~/.ignition/ ~/.gz/ /tmp/ignition_* /tmp/gz_* /tmp/gazebo_* /dev/shm/rtps* /dev/shm/fastdds*
}

# Define clean closure handler
cleanup() {
    echo -e "\n\e[1;31mInitiating Graceful Shutdown...\e[0m"
    echo "Waiting for ROS 2 nodes to complete saving telemetry..."
    sleep 3
    echo "Running final nuclear sweep for zombie processes..."
    nuclear_sweep
    echo -e "\e[1;36mGraceful shutdown complete.\e[0m"
}

# Bind cleanup handler to script exit (handles Ctrl+C and normal exit)
trap cleanup EXIT

# 1. Killing all previous processes to verify startup is secure
echo -e "\e[1;36m[1/2] Running Nuclear Cleanup...\e[0m"
ros2 daemon stop 2>/dev/null || true
nuclear_sweep
echo -e "\e[1;36mCleanup complete.\e[0m"
sleep 1

# 2. Launching Gazebo Sim in Foreground
echo -e "\e[1;32m[2/2] Launching Gazebo in Foreground...\e[0m"
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo -e "\e[1;31mError: install/setup.bash not found!\e[0m" >&2
    exit 1
fi
clear
echo -e "\e[1;32m[STAGE 1] Launching Gazebo (World: $WORLD_NAME) (Press Ctrl+C to close)...\e[0m"
ros2 launch bgr_description gazebo.launch.py world_name:="$WORLD_NAME" headless:="$HEADLESS_MODE"
