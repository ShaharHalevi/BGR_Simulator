FROM osrf/ros:jazzy-desktop

# Install basic dependencies and tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-matplotlib \
    git \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# Install simulation / ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Set working directory to the ROS workspace
WORKDIR /ros2_ws

# Create workspace src folder.
# The actual project source code is mounted by docker-compose at runtime.
RUN mkdir -p /ros2_ws/src/bgr_simulator

# Source ROS automatically when entering the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Default command.
# docker-compose usually overrides this with its own command.
CMD ["bash"]