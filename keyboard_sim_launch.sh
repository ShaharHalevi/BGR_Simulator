#!/bin/bash

# ==========================================
# BGR Simulation Automated Startup Script
# (Tmux Edition)
# ==========================================

# Tmux must be installed to run this script
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Attempting to install..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get update && sudo apt-get install -y tmux
    else
        echo "Error: tmux is required but not installed, and apt-get is not available."
        exit 1
    fi
fi



# NOTE: If for any reason you do not close the simulator down gracefully, please enter this into the terminal to kill all zombie processes in the background!!
# killall -9 ros2 ros2-daemon gz ruby rviz2 2>/dev/null && pkill -9 -f "bgr_description|bgr_controller"


# ==============================================================================
# 1. NUCLEAR CLEANUP
# This step ensures a clean, secure startup by terminating any orphaned processes,
# container conflicts, or leftover cache files from previous runs. Specifically:
# - Stops conflicting Docker containers (e.g., bgr_simulator) running on the host network.
# - Kills wrapper scripts, multiplexers, and bridges (ros2, tmux, parameter_bridge).
# - Terminates all Gazebo Sim / Ignition Gazebo server and GUI processes.
# - Cleans up core ROS 2 nodes (state publishers, control nodes) that easily get orphaned.
# - Stops specific simulation nodes and Python utilities (bgr_description, bgr_controller, car_dashboard, keyboard_teleop).
# - Clears temporary directory caches, Ignition/Gazebo configuration, and FastDDS shared memory.
# ==============================================================================
echo "[1/3] Running Nuclear Cleanup..."
docker stop bgr_simulator 2>/dev/null || true
killall -9 ros2 ros2-daemon gz ruby rviz2 parameter_bridge 2>/dev/null
killall -9 gz-sim-server gz-sim-gui ign-gazebo-server ign-gazebo-gui 2>/dev/null
killall -9 robot_state_publisher static_transform_publisher ros2_control_node 2>/dev/null
pkill -9 -f "bgr_description" 2>/dev/null
pkill -9 -f "bgr_controller" 2>/dev/null
pkill -9 -f "car_dashboard.py" 2>/dev/null
pkill -9 -f "keyboard_teleop" 2>/dev/null
rm -rf ~/.ignition/ ~/.gz/ /tmp/ignition_* /tmp/gz_* /tmp/gazebo_* /dev/shm/rtps* /dev/shm/fastdds*
echo "Cleanup complete."
sleep 1

# 2. Build Workspace (Clearing the build, install, log folders to avoid conflicts)
echo "[2/3] Building Workspace..."
rm -rf build/ install/ log/
colcon build

echo "[3/3] Launching Tmux Session..."

SESSION="bgr_sim"
tmux kill-session -t $SESSION 2>/dev/null

# Create a new detached tmux session
tmux new-session -d -s $SESSION

# Split the window into 4 panes
# Layout:
# +----------------+----------------+
# | Pane 0         | Pane 2         |
# | (Gazebo)       | (Controllers)  |
# +----------------+----------------+
# | Pane 1         | Pane 3         |
# | (Bridge)       | (Drive Console)|
# +----------------+----------------+

tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# -------------------------
# PANE 0: GAZEBO (Top-Left)
# -------------------------
tmux send-keys -t $SESSION:0.0 "source /opt/ros/jazzy/setup.bash && source \$(pwd)/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "clear; echo -e '\e[1;32m[STAGE 1] Launching Gazebo...\e[0m'" C-m
tmux send-keys -t $SESSION:0.0 "ros2 launch bgr_description gazebo.launch.py world_name:=Map1Opt.world" C-m

# -------------------------
# PANE 2: CONTROLLERS (Top-Right)
# -------------------------
tmux send-keys -t $SESSION:0.2 "source /opt/ros/jazzy/setup.bash && source \$(pwd)/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "clear; echo -e '\e[1;33mWaiting for Gazebo to fully spawn...\e[0m'" C-m
tmux send-keys -t $SESSION:0.2 "until ros2 topic list 2>/dev/null | grep -q '/robot/full_state'; do sleep 1; done" C-m
tmux send-keys -t $SESSION:0.2 "echo -e '\e[1;32mGazebo active! Delaying 2s...\e[0m'; sleep 2" C-m
tmux send-keys -t $SESSION:0.2 "echo -e '\e[1;32m[STAGE 2] Launching Controllers...\e[0m'" C-m
tmux send-keys -t $SESSION:0.2 "ros2 launch bgr_controller controller.launch.py" C-m

# -------------------------
# PANE 1: KEYBOARD BRIDGE (Bottom-Left)
# -------------------------
tmux send-keys -t $SESSION:0.1 "source /opt/ros/jazzy/setup.bash && source \$(pwd)/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "clear; echo -e '\e[1;33mWaiting for Controllers...\e[0m'" C-m
tmux send-keys -t $SESSION:0.1 "until ros2 control list_controllers 2>/dev/null | grep -q 'forward_velocity_controller.*active' && ros2 control list_controllers 2>/dev/null | grep -q 'forward_position_controller.*active'; do sleep 1; done" C-m
tmux send-keys -t $SESSION:0.1 "echo -e '\e[1;32mControllers active! Delaying 2s...\e[0m'; sleep 2" C-m
tmux send-keys -t $SESSION:0.1 "echo -e '\e[1;32m[STAGE 3] Launching Keyboard Bridge...\e[0m'" C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch bgr_controller keyboard_teleop.launch.py" C-m

# -------------------------
# PANE 3: DRIVE CONSOLE (Bottom-Right)
# -------------------------
tmux send-keys -t $SESSION:0.3 "source /opt/ros/jazzy/setup.bash && source \$(pwd)/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.3 "clear; echo -e '\e[1;33mWaiting for Bridge...\e[0m'" C-m
tmux send-keys -t $SESSION:0.3 "until ros2 node list 2>/dev/null | grep -q 'joy_array_bridge'; do sleep 1; done" C-m
tmux send-keys -t $SESSION:0.3 "echo -e '\e[1;32mBridge active! Delaying 2s...\e[0m'; sleep 2" C-m
tmux send-keys -t $SESSION:0.3 "clear" C-m
tmux send-keys -t $SESSION:0.3 "echo '=========================================='" C-m
tmux send-keys -t $SESSION:0.3 "echo '  Simulation is fully launched & ready!   '" C-m
tmux send-keys -t $SESSION:0.3 "echo '                                          '" C-m
tmux send-keys -t $SESSION:0.3 "echo '  -> Press Ctrl+C HERE to shut down <-    '" C-m
tmux send-keys -t $SESSION:0.3 "echo '=========================================='" C-m

# Chain the teleop node with the automated shutdown sequence.
# When teleop exits (via Ctrl+C), it kills the tmux session to trigger the host-side cleanup.
tmux send-keys -t $SESSION:0.3 "ros2 run bgr_controller keyboard_teleop.py; tmux kill-session -t $SESSION" C-m

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
    pkill -9 -f "car_dashboard.py" 2>/dev/null
    pkill -9 -f "keyboard_teleop" 2>/dev/null
    rm -rf ~/.ignition/ ~/.gz/ /tmp/ignition_* /tmp/gz_* /tmp/gazebo_* /dev/shm/rtps* /dev/shm/fastdds*
    echo "Graceful shutdown complete."
}

# Attach to the tmux session
tmux attach-session -t $SESSION

# Run cleanup as soon as the tmux session exits or detaches
cleanup
