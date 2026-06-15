#!/bin/bash

# ==================================================
# Automated Simulation Startup With Keyboard Control
# ==================================================

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
    
    # 1. Stop background ROS 2 nodes gracefully
    if [ -n "$GAZEBO_PID" ] && kill -0 $GAZEBO_PID 2>/dev/null; then
        echo "Sending stop signal to Gazebo..."
        kill -INT $GAZEBO_PID 2>/dev/null
    fi
    
    if [ -n "$BRIDGE_PID" ] && kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "Sending stop signal to Keyboard Bridge..."
        kill -INT $BRIDGE_PID 2>/dev/null
    fi

    # 2. Wait for telemetry logger and nodes to write final data
    if [ -n "$GAZEBO_PID" ] || [ -n "$BRIDGE_PID" ]; then
        echo "Waiting for ROS 2 nodes to complete saving telemetry..."
        sleep 3
    fi
    
    # 3. Nuclear sweep to clean up zombie or hung processes
    echo "Running final nuclear sweep for zombie processes..."
    nuclear_sweep
    echo -e "\e[1;36mGraceful shutdown complete.\e[0m"
}

# Bind cleanup handler to script exit
trap cleanup EXIT

# 1. Killing all previous processes to verify startup is secure
echo -e "\e[1;36m[1/2] Running Nuclear Cleanup...\e[0m"
ros2 daemon stop 2>/dev/null || true
nuclear_sweep
echo -e "\e[1;36mCleanup complete.\e[0m"
sleep 1

# 2. Sourcing and Starting Background Jobs
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo -e "\e[1;31mError: install/setup.bash not found!\e[0m" >&2
    exit 1
fi

echo -e "\e[1;32m[2/2] Starting Background Simulation...\e[0m"

# Launch Gazebo in the background, redirecting logs to /tmp/gazebo.log
echo -e "-> Starting Gazebo Sim (World: $WORLD_NAME) (Logs redirected to \e[1;36m/tmp/gazebo.log\e[0m)..."
ros2 launch bgr_description gazebo.launch.py world_name:="$WORLD_NAME" headless:="$HEADLESS_MODE" > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

# 4. Polling check to verify Gazebo controllers are active before launching keyboard bridge
echo -e "\e[1;32mWaiting for controllers to activate...\e[0m"
until ros2 control list_controllers 2>/dev/null | grep -q 'forward_velocity_controller.*active' && \
      ros2 control list_controllers 2>/dev/null | grep -q 'forward_position_controller.*active'; do
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        echo -e "\e[1;31mError: Gazebo failed to start. Check /tmp/gazebo.log.\e[0m" >&2
        exit 1
    fi
    sleep 1
done
echo -e "\e[1;32mControllers active! Delaying 1s...\e[0m"
sleep 1

# Launch Keyboard Bridge in the background, suppressing stdout and stderr logs
echo -e "-> Starting Keyboard Bridge..."
ros2 launch bgr_controller keyboard_teleop.launch.py > /dev/null 2>&1 &
BRIDGE_PID=$!

# Polling check to verify keyboard bridge node is active before launching teleop script
echo -e "\e[1;32mWaiting for keyboard bridge node...\e[0m"
until ros2 node list 2>/dev/null | grep -q 'joy_array_bridge'; do
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo -e "\e[1;31mError: Keyboard bridge failed to start.\e[0m" >&2
        exit 1
    fi
    sleep 1
done

clear
echo -e "\e[1;32m====================================================\e[0m"
echo -e "\e[1;32m  Simulation has loaded and is ready!              \e[0m"
echo -e "\e[1;32m  - Standard Gazebo logs are piped to /tmp/gazebo.log\e[0m"
echo -e "\e[1;32m  - Run 'tail -f /tmp/gazebo.log' in another tab to view\e[0m"
echo -e "\e[1;32m====================================================\e[0m"

# 5. Run keyboard teleop in the foreground
ros2 run bgr_controller keyboard_teleop.py
