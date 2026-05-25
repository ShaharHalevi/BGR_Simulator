FROM osrf/ros:jazzy-desktop

# Add NVIDIA GPU support for hardware acceleration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Install basic dependencies and tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-matplotlib \
    git \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# Install all simulation dependencies listed in the README
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-* \
    ros-jazzy-*-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Set working directory to the ROS workspace
WORKDIR /ros2_ws

# Copy the entire BGR_Simulator repository into the workspace src folder
COPY . /ros2_ws/src/bgr_simulator

# Convert all copied files to Unix line endings
RUN find /ros2_ws/src/bgr_simulator -type f -exec dos2unix {} +

# Install and initialize rosdep
RUN apt-get update && rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the workspace and run the default simulator launch file
# This command runs Step 1 from the README
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py headless:=True"]