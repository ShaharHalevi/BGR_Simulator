# BGR_Simulator
BGRacing Formula Student – ROS 2 Ackermann vehicle for GZ Sim: URDF/Xacro, ros2_control, Gazebo/RViz launches.

BUILD
# workspace
mkdir -p ~/ws_bgr/src
cd ~/ws_bgr/src
# place `bgr_description` and `bgr_controller` here (from this repo)
cd ~/ws_bgr
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# 1) Gazebo world + robot
ros2 launch bgr_description gz.launch.py

# 2) Controllers (after the sim is up)
ros2 launch bgr_controller controllers.launch.py

RVIZ
ros2 launch bgr_description display.launch.py
