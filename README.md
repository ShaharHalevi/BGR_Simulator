# BGR_Simulator

This package contains the Gazebo harmonic simulation environment for the BGRacing car. It includes the URDF/Xacro robot descriptions, Gazebo worlds, custom dashboards, and the ROS 2 bridge configurations necessary to expose simulated hardware to the rest of the autonomy stack.

## Overview

The simulator runs inside its own Docker container and is built on **ROS 2 Jazzy**. It relies on `ros_gz` packages to bridge Gazebo topics into standard ROS 2 messages (e.g., `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `sensor_msgs/JointState`).

It also launches several custom GUI components:
- `track_gui.py`: Visualizes the track and race progress.
- `car_dashboard.py`: Displays speed, steering commands, and other telemetry.

---

## 🏎️ Running the Simulator (Standalone)

If you wish to test or drive the simulator manually without launching the `planning`, `mapping`, and `perception` containers, follow these instructions.

### 1. Configure the Display (X11)
Since the simulator relies heavily on GUI applications, ensure your host machine is configured to render Docker displays:

**If on Windows**:
1. Install and run [VcXsrv](https://sourceforge.net/projects/vcxsrv/) via XLaunch.
2. Ensure you check **"Disable access control"** during setup.
3. Ensure the simulator's environment variable is set to `DISPLAY=host.docker.internal:0`.

**If on Linux**:
1. Run `xhost +local:root` on your host terminal.
2. Ensure the simulator's environment variable is set to `DISPLAY=${DISPLAY}` and the `/tmp/.X11-unix` volume is mounted.

### 2. Build and Launch
Navigate to the root directory of the repository (where `docker-compose.yml` is located) and explicitly start *only* the simulator service:

```bash
docker compose up simulator --build
```

Gazebo will launch with the car spawned at the starting position, alongside your telemetry dashboards.

---

## Manual Control (Teleop)

You can manually drive the car around the track to test sensors and vehicle dynamics.

Open a new terminal, and connect to the running simulator container:

```bash
docker exec -it bgr_simulator bash
```

Source the workspace:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Launch the keyboard teleop node to drive the car with your arrow keys or W/A/S/D:
```bash
ros2 launch bgr_controller keyboard_teleop.launch.py
```

## Topics Exposed

The simulator bridges the following key Gazebo topics to standard ROS 2 topics:

*   `/lidar/points` (`sensor_msgs/PointCloud2`): Pointcloud data from the onboard LiDAR.
*   `/model/bgr/odometry` (`nav_msgs/Odometry`): Perfect odometry ground truth from Gazebo.
*   `/tf` & `/tf_static`: Robot transforms published by `robot_state_publisher`.
