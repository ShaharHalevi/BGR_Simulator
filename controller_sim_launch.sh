#!/bin/bash

# ==========================================
# BGR Simulation Automated Startup Script
# (No Keyboard Edition - Tmux 1-Pane)
# ==========================================

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 1. Killing all previous processes to verify startup is secure
echo "[1/3] Running Nuclear Cleanup..."
ros2 daemon stop 2>/dev/null || true
# Stop clashing Docker containers if running on host network
docker stop bgr_simulator 2>/dev/null || true
# Kill wrapper scripts, multiplexers, and bridges
killall -9 ros2 ros2-daemon gz ruby rviz2 parameter_bridge 2>/dev/null
# Kill actual Gazebo Sim C++ engines (server and GUI)
killall -9 gz-sim-server gz-sim-gui ign-gazebo-server ign-gazebo-gui 2>/dev/null
# Kill core ROS 2 C++ nodes that easily get orphaned
killall -9 robot_state_publisher static_transform_publisher ros2_control_node 2>/dev/null
# Kill specific python scripts and package nodes
pkill -9 -f "bgr_description" 2>/dev/null
pkill -9 -f "bgr_controller" 2>/dev/null
pkill -9 -f "controller_manager/spawner" 2>/dev/null
pkill -9 -f "car_dashboard.py" 2>/dev/null
pkill -9 -f "keyboard_teleop" 2>/dev/null
rm -rf ~/.ignition/ ~/.gz/ /tmp/ignition_* /tmp/gz_* /tmp/gazebo_* /dev/shm/rtps* /dev/shm/fastdds*
echo "Cleanup complete."
sleep 1

# 2. Build Workspace (Clearing the build, install, log folders to avoid conflicts)
echo "[2/3] Building Workspace..."
rm -rf build/ install/ log/
# Reset environment variables pointing to deleted install paths to prevent colcon warnings
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
colcon build

# 3. Launching Tmux Session...
echo "[3/3] Launching Tmux Session..."

SESSION="bgr_sim"
tmux kill-session -t $SESSION 2>/dev/null

# Create a new detached tmux session
tmux new-session -d -s $SESSION

# Layout:
# +----------------+
# |     Pane 0     |
# |    (Gazebo)    |
# +----------------+

# -------------------------
# PANE 0: GAZEBO
# -------------------------
tmux send-keys -t $SESSION:0.0 "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && source /opt/ros/jazzy/setup.bash && source \$(pwd)/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "clear; echo -e '\e[1;32m[STAGE 1] Launching Gazebo...\e[0m'" C-m
tmux send-keys -t $SESSION:0.0 "ros2 launch bgr_description gazebo.launch.py world_name:=Map1Opt.world" C-m

# Define clean closure handler
cleanup() {
    echo -e "\n\e[1;31mInitiating Graceful Shutdown...\e[0m"
    tmux kill-session -t $SESSION 2>/dev/null
    
    echo "Running final nuclear sweep..."
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
    echo "Graceful shutdown complete."
}

# Attach to the tmux session
tmux attach-session -t $SESSION

# Run cleanup as soon as the tmux session exits or detaches
cleanup
