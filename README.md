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

Navigate to the root of the `BGR_Simulator` repository (where the `Dockerfile` is located).

> [!NOTE]
> **Windows Users:** Ensure that **Docker Desktop** is open and running in the background before attempting to build or run the image.

Build the image:
```bash
docker build -t bgr_simulator .
```

Run the container with GUI support and Live Volume Mounting (so edits to Python scripts instantly take effect!):

**For Windows (PowerShell):**
```powershell
docker run --rm -it `
  --name bgr_simulator `
  -e DISPLAY=host.docker.internal:0 `
  -e QT_X11_NO_MITSHM=1 `
  -v ${PWD}/src:/ros2_ws/src/bgr_simulator/src:rw `
  bgr_simulator `
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py"
```

**For Linux / WSL:**
```bash
docker run --rm -it \
  --name bgr_simulator \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/src:/ros2_ws/src/bgr_simulator/src:rw \
  bgr_simulator \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py"
```

> [!WARNING]
> **Windows/Git Users (Line Endings Bug):** 
> If you edit Python files on Windows, your editor might save them with Windows line endings (`CRLF`). When ROS 2 tries to run these scripts inside the Linux container, you will get a fatal crash like `python3\r: No such file or directory`. 
> 
> **To fix**: Ensure your code editor (e.g. VS Code) is set to save Python/Bash files as **`LF`** (Linux format), not `CRLF`. If a file crashes due to this, open it, change it to `LF`, save it, and then explicitly run the `docker exec ... colcon build` command again to update the ROS 2 cache.

Because of `--symlink-install` in the command above, any edits you make to Python scripts (like those in `bgr_controller`) will instantly take effect on your host machine without needing a rebuild! 

### 🔄 When to Rebuild the ROS 2 Workspace?
Even with `--symlink-install`, there are specific cases where you **must** manually rebuild the ROS 2 cache inside the running container. 

You need to rebuild if you:
*   Add a **new** Python file or delete an existing one.
*   Modify `setup.py`, `CMakeLists.txt`, or `package.xml`.
*   Modify any `C++` source code.
*   Modify `.launch.py` files.
*   Change Python line endings (`CRLF` -> `LF`) to fix crash bugs.

**How to rebuild without restarting Docker:**
Open a new terminal on your host, execute into the running simulator, and run `colcon build`:
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
```

### 🖥️ Opening a New Terminal
To interact with the running simulator (to launch nodes or echo topics), open a new terminal on your host machine and copy-paste these 3 commands to enter the container and source ROS 2:
```bash
docker exec -it bgr_simulator bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

**Docker Compose (for running multiple services together):**

Refer to the main `README.md` for instructions on running the simulator as part of the full autonomy stack.

---

## Manual Control (Teleop)

You can manually drive the car around the track to test sensors and vehicle dynamics.

> [!IMPORTANT]
> Because these scripts run continuously, **you must open a new terminal tab for every command block below**. 

**Terminal 1 (Controllers & Bridge):** Connects to the simulator, sources the workspace, and spawns the wheel/steering controllers along with the command bridge.
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bgr_controller controller.launch.py"
```

**Terminal 2 (Keyboard Input):** Connects, sources, and launches the interactive keyboard script. *(Keep this terminal focused to capture your `w, a, s, d, x, q` keystrokes)*
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run bgr_controller keyboard_teleop.py"
```

## Topics Exposed

The simulator bridges the following key Gazebo topics to standard ROS 2 topics:

*   `/lidar/points` (`sensor_msgs/PointCloud2`): Pointcloud data from the onboard LiDAR.
*   `/model/bgr/odometry` (`nav_msgs/Odometry`): Perfect odometry ground truth from Gazebo.
*   `/tf` & `/tf_static`: Robot transforms published by `robot_state_publisher`.


