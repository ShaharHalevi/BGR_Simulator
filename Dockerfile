# ============================================================
# BGR Simulator Dev Image (Ubuntu 24.04 + ROS 2 Jazzy)
#
# This Dockerfile is designed to avoid the folloing issues:
# 1) Missing build tools (cmake/build-essential) -> colcon fails
# 2) rosdep failing due to outdated apt indexes -> we install core deps here,
#    but still recommend `apt update` before rosdep at runtime (because we clean apt lists)
# 3) GUI apps (RViz, joint_state_publisher_gui) failing in Docker -> install X11/Qt libs
# 4) Missing editors in container -> install nano
# 5) Common simulator deps: ros2_control, controllers, ros_gz, rviz2, etc.
# ============================================================

ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

# Avoid interactive prompts during apt installs.
ENV DEBIAN_FRONTEND=noninteractive

# Choose ROS distro. Jazzy matches Ubuntu 24.04.
ENV ROS_DISTRO=jazzy

# Qt/RViz in Docker often needs this to avoid shared memory issues on X11.
# We'll also pass it at `docker run`, but having it here doesn't hurt.
ENV QT_X11_NO_MITSHM=1

# ------------------------------------------------------------
# 0) Locale (UTF-8) - ROS tools behave best with proper locale.
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ------------------------------------------------------------
# 1) Base utilities + build tools + editors
# - build-essential + cmake: required for CMake/colcon builds
# - curl/gnupg/lsb-release: for adding ROS apt repo
# - python3/pip/git: standard dev tooling
# - nano: so we can edit package.xml / configs inside container
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    git \
    nano \
  && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 2) X11/Qt runtime libraries for GUI apps in Docker
# These help RViz + joint_state_publisher_gui run when DISPLAY is forwarded.
# (They are often present indirectly, but installing explicitly avoids surprises.)
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxcomposite1 \
    libxi6 \
    libxtst6 \
    libxkbcommon-x11-0 \
    libgl1 \
    mesa-utils \
  && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 3) Add ROS 2 apt repository and key
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ------------------------------------------------------------
# 4) Install ROS 2 + packages (your requested list + simulator needs)
#
# Notes:
# - ros-base: ROS core CLI/libraries
# - colcon + rosdep: build + dependency management
# - rviz2 + joint_state_publisher_gui: display.launch.py
# - ros-gz-sim + ros-gz-bridge: Gazebo / GZ Sim integration
# - gz-ros2-control: integration between GZ and ros2_control
# - ros2-control + ros2-controllers: controllers stack
# - robot-state-publisher + xacro: robot description pipeline
# - extras: robot-localization, joy, joy-teleop, tf-transformations, turtlesim
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    \
    python3-colcon-common-extensions \
    python3-rosdep \
    \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control \
    \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-tf-transformations \
  && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 5) Initialize rosdep
# - rosdep init creates the system-level rosdep config
# - rosdep update fetches the dependency rules
#
# We allow rosdep init to fail if it was already initialized in a cached layer.
# ------------------------------------------------------------
RUN rosdep init || true
RUN rosdep update

# ------------------------------------------------------------
# 6) Convenience: auto-source ROS in every shell
# This makes `ros2 ...` available immediately inside the container.
# ------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# ------------------------------------------------------------
# 7) Workspace location inside container
# We'll mount your repository/workspace to /ws at runtime.
# ------------------------------------------------------------
WORKDIR /ws

CMD ["bash"]
