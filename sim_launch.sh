#!/bin/bash

# ==========================================
# BGR Simulation Automated Startup Script
# (No Keyboard Edition - Native Bash)
# ==========================================

# Restrict ROS 2 discovery to the local machine to avoid network crosstalk with other systems
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

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
echo -e "\e[1;36m[1/3] Running Nuclear Cleanup...\e[0m"
ros2 daemon stop 2>/dev/null || true
nuclear_sweep
echo -e "\e[1;36mCleanup complete.\e[0m"
sleep 1

# 2. Build Workspace (Clearing the build, install, log folders to avoid conflicts)
echo -e "\e[1;33m[2/3] Building Workspace...\e[0m"
rm -rf build/ install/ log/
# Reset environment variables pointing to deleted install paths to prevent colcon warnings
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
colcon build || { echo -e "\e[1;31mError: Workspace build failed!\e[0m" >&2; exit 1; }

# 3. Launching Gazebo Sim in Foreground
echo -e "\e[1;32m[3/3] Launching Gazebo in Foreground...\e[0m"
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo -e "\e[1;31mError: install/setup.bash not found!\e[0m" >&2
    exit 1
fi
clear
echo -e "\e[1;32m[STAGE 1] Launching Gazebo (Press Ctrl+C to close)...\e[0m"
ros2 launch bgr_description gazebo.launch.py world_name:=Map1Opt.world
