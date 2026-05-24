# 🏎️ BGRacing Driverless Simulator (ROS 2 Jazzy)

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Gazebo Sim](https://img.shields.io/badge/Simulator-Gazebo_Sim-orange?logo=gazebo&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

A high-fidelity simulation environment for the **BGRacing Formula Student Team**.
Built on **ROS 2 Jazzy** and **Gazebo Sim (GZ)**, utilizing `ros2_control` for accurate vehicle dynamics and Ackermann steering.

---

## 📸 Gallery & Demos

### Simulation Environment
![Simulation View](/doc/Car1.png)


### Driving Demo + LiDAR Sim


https://github.com/user-attachments/assets/a613b591-cd07-49bc-b930-c8c4ba2a1f91



### Driving Demo + Localization Sim

https://github.com/user-attachments/assets/27b3cb61-808b-4af2-99aa-c85f0a2a9ecf




---

## 🚀 Features

* **Custom Ackermann Steering:** Realistic geometry with mimic joints for visualization.
* **ROS 2 Control Integration:** Velocity and Position controllers for precise wheel command.
* **Dynamic Tracks:** Load CSV based tracks (cones) directly into the simulation.
* **Interactive Tools:**
    * `car_dashboard.py`: Real-time vehicle telemetry.
* **Sensor Simulation:** Simulated Odometry, IMU (via Gazebo plugins), and Cameras.

---

## 🛠️ Installation & Prerequisites

### 1. System Requirements
* **OS:** Ubuntu 24.04 LTS (Noble Numbat)
* **ROS:** ROS 2 Jazzy Jalisco

### 2. Install Dependencies
Run the following commands to install all necessary packages for ROS 2 Jazzy and Gazebo:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-* \
    ros-jazzy-*-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-turtlesim \
    ros-jazzy-robot-localization \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-tf-transformations
```

### 3. Clone & Build
Create a workspace and clone the repository:

```bash
mkdir -p ~/bgr_ws/src
cd ~/bgr_ws/src
git clone https://github.com/shaharhalevi/bgr_simulator.git .
```

Install python dependencies (if any) and build the workspace:

```bash
cd ~/bgr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---

## 🏁 How to Run

Open separate terminals for each step (or use functionality like Tmux/Terminator). Always source your workspace in every new terminal:

```bash
source install/setup.bash
```

### Step 1: Launch the Simulation
This loads the track, the robot model (bgr_description), and the Gazebo environment.
Open a new terminal and enter:
```bash
source install/setup.bash && ros2 launch bgr_description gazebo.launch.py
```
**Note on Worlds:** You can select different tracks by passing the `world_name` argument. Available worlds:
* `Acceleration.world` (Default)
* `Skidpad.world`
* `TrainingMap.world`
* `CompetitionMap1.world`, `CompetitionMap2.world`, `CompetitionMap3.world`
* `CompetitionMapTestday1.world`, `CompetitionMapTestday2.world`, `CompetitionMapTestday3.world`

**Command to launch a specific world:**
```bash
source install/setup.bash && ros2 launch bgr_description gazebo.launch.py world_name:=CompetitionMap1.world
```

**Headless Mode:**
If you want to run the simulation without the GUI (e.g., in Docker or on a server), use:
```bash
source install/setup.bash && ros2 launch bgr_description gazebo.launch.py headless:=True
```


**Tip:** To run both headless and world, combine both arguments above into the same line (with a space inbetween). Additionally, this also starts the car_dashboard automatically. 

### Step 2: Activate Controllers
Once the simulation is running, spawn the ros2_control managers:
Open a new terminal and enter:
```bash
source install/setup.bash && ros2 launch bgr_controller controller.launch.py
```

You should see output confirming `joint_state_broadcaster`, `forward_velocity_controller`, and `forward_position_controller` are active.

### Step 3: Drive the Car (Keyboard Teleop)
To drive the car using your keyboard, run the teleoperation script in order:
Keyboard setup launcher:
Open a new terminal and enter:
```bash
source install/setup.bash && ros2 launch bgr_controller keyboard_teleop.launch.py
```
Keyboard activation:
Open a new terminal and enter:
```bash
source install/setup.bash && ros2 run bgr_controller keyboard_teleop.py
```

**Controls:**
- Use **Arrow Keys** or **WASD** to control the car
- **Space** to brake/stop
- **Q** to quit the teleop node
- Ensure the terminal running the script has focus for keyboard input to work

---

---

## 🔧 Troubleshooting

**Robot not moving?** Ensure you ran Step 2 (Controllers). The robot won't respond to commands if the controllers aren't spawned.

**Missing Models?** Ensure the environment variable `GZ_SIM_RESOURCE_PATH` is set correctly. The launch file handles this, but if you moved folders manually, check `gazebo.launch.py`.

**Gazebo crashes on VM?** Ensure "Accelerate 3D Graphics" is enabled in your VM settings, or try running with `LIBGL_ALWAYS_SOFTWARE=1` if you lack a GPU.


> [!IMPORTANT]
> **Graceful Shutdown:** To ensure all nodes and the simulation state are saved/closed correctly, always shut down your terminals in the **reverse order** they were opened (LIFO/Stack order). Close Step 3 (Teleop), then Step 2 (Controllers), and finally Step 1 (Gazebo).

---

**Maintained by:** BGRacing Simulator Team

---

#  🏎️ Docker Setup

This package contains the Gazebo harmonic simulation environment for the BGRacing car. It includes the URDF/Xacro robot descriptions, Gazebo worlds, custom dashboards, and the ROS 2 bridge configurations necessary to expose simulated hardware to the rest of the autonomy stack.

## Overview

The simulator runs inside its own Docker container and is built on **ROS 2 Jazzy**. It relies on `ros_gz` packages to bridge Gazebo topics into standard ROS 2 messages (e.g., `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `sensor_msgs/JointState`).

It also launches several custom GUI components:
- `track_gui.py`: Visualizes the track and race progress.
- `car_dashboard.py`: Displays speed, steering commands, and other telemetry.

---

## 🛠️ Prerequisites & Setup

Before running the simulator, you must install Docker and configure your host machine's display server to allow GUI applications to run from the container. 

Follow the complete setup track below for your operating system.

---

### 🐧 Linux Setup

**Part 1 — Install Docker Engine:**

On Linux, you install the Docker Engine directly.

> [!NOTE]
> These instructions are for **Ubuntu/Debian-based** distributions. For other distros, refer to the [official Docker docs](https://docs.docker.com/engine/install/).

1. Remove any old/conflicting Docker packages:
   ```bash
   sudo apt-get remove docker docker-engine docker.io containerd runc
   ```
2. Set up the Docker apt repository and install the Engine:
   ```bash
   sudo apt-get update
   sudo apt-get install -y ca-certificates curl gnupg
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   sudo chmod a+r /etc/apt/keyrings/docker.gpg
   
   echo \
     "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
     "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   
   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```
3. Run Docker without `sudo` (Post-install):
   ```bash
   sudo usermod -aG docker $USER
   ```
   **Log out and log back in** for this change to take effect.
4. Enable Docker to start on boot:
   ```bash
   sudo systemctl enable docker
   sudo systemctl start docker
   ```

**Part 2 — Configure the Display (xhost):**

Since the simulator relies heavily on GUI applications, ensure your host machine is configured to render Docker displays. Run the following command in your host terminal:

> [!CAUTION]
> **Security Warning:** Running `xhost +local:root` in Linux weakens the security of your display server. It is recommended to only use these settings on trusted private networks.

```bash
xhost +local:root
```

---

### 🪟 Windows Setup

**Part 1 — Install Docker Desktop:**

Docker Desktop is the easiest way to run Docker on Windows. It bundles the Docker Engine, Docker CLI, and a GUI dashboard.

> [!IMPORTANT]
> Docker Desktop on Windows uses **WSL 2** (Windows Subsystem for Linux 2) as its backend. Make sure WSL 2 is enabled before installing. Open PowerShell as Administrator and run `wsl --install`, then restart your PC.

1. Go to [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop) and download the **Docker Desktop for Windows** installer.
2. Run the installer and ensure **"Use WSL 2 instead of Hyper-V"** is selected.
3. Launch Docker Desktop from the Start Menu. Wait for the engine to initialize.

**Part 2 — Configure the Display (VcXsrv):**

Since the simulator relies heavily on GUI applications, ensure your host machine is configured to render Docker displays:

> [!CAUTION]
> **Security Warning:** Proceeding with "Disable access control" in VcXsrv weakens the security of your display server. It is recommended to only use these settings on trusted private networks.

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
2. Open **XLaunch** from your Start Menu to configure the server:
   - **Display Settings:** Select **"Multiple windows"** and leave the Display number as `-1` (default). Click Next.
   - **Client Startup:** Select **"Start no client"**. Click Next.
   - **Extra Settings:** Ensure **"Clipboard"** is checked. **CRITICAL:** You must check the **"Disable access control"** box so Docker can communicate with it. Click Next.
   - Click **Finish**. (You should see an 'X' icon appear in your Windows system tray).

> [!NOTE]
> Ensure that both **Docker Desktop** and **VcXsrv** (with access control disabled) are open and running in the background before proceeding to launch the simulator.

---

## 🏎️ Running the Simulator (Standalone)

If you wish to test or drive the simulator manually without launching the `perception` and `localization` containers, follow these instructions.

### Build and Launch

Navigate to the root of the `BGR_Simulator` repository. You can run the simulator in two modes: **Headed** (with Gazebo GUI) or **Headless** (background physics and logic only, for lower PC resource usage).

#### Option A: Headed Mode (With GUI)

```bash
docker compose up --build
```

#### Option B: Headless Gazebo (Python GUIs Only)
Runs Gazebo headlessly (to save PC resources) but still renders your Python GUIs (track selector, speed dashboard).

```bash
HEADLESS=True docker compose up --build
```

> [!TIP]
> After the first build, you can omit `--build` from subsequent runs to start faster.

> [!WARNING]
> **Windows/Git Users (Line Endings Bug):** 
> If you edit Python files on Windows, your editor might save them with Windows line endings (`CRLF`). When ROS 2 tries to run these scripts inside the Linux container, you will get a fatal crash like `python3\r: No such file or directory`. 
> 
> **To fix**: Ensure your code editor (e.g. VS Code) is set to save Python/Bash files as **`LF`** (Linux format), not `CRLF`. If a file crashes due to this, open it, change it to `LF`, save it, and then explicitly run the `docker exec ... colcon build` command again to update the ROS 2 cache.

Any edits you make to files will require a rebuild of the workspace to take effect inside the simulator.

### 🔄 Rebuilding the ROS 2 Workspace
You need to rebuild if you modify any code, URDF, launch files, or configuration: 

You need to rebuild if you:
*   Add a **new** Python file or delete an existing one.
*   Modify `setup.py`, `CMakeLists.txt`, or `package.xml`.
*   Modify any `C++` source code.
*   Modify `.launch.py` files.
*   Change Python line endings (`CRLF` -> `LF`) to fix crash bugs.

**How to rebuild without restarting Docker:**
Open a new terminal on your host, execute into the running simulator, and run `colcon build`:
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && colcon build"
```

### 🖥️ Exploring the Container Manually (Optional)
If you want to manually poke around the files or just echo raw ROS 2 topics, you can open a new terminal on your host machine and drop directly into the container's shell:
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
> The commands below are **all-in-one** convenience scripts designed to be pasted entirely into a **fresh Windows/Host terminal**. 
> Do **NOT** paste these if you are already inside the container's `root@...:/ros2_ws#` bash shell, or they will crash!
> Because these scripts run continuously, **you must open a separate, new Host terminal tab for each command block below**. 

**Terminal 1 (Controllers):** Spawns the joint state broadcaster, velocity controller, and steering position controller.
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bgr_controller controller.launch.py"
```

> [!NOTE]
> Wait for the controller spawners to finish (you'll see `[INFO] ... Successfully loaded controller ...` messages) before launching the bridge in Terminal 2.

**Terminal 2 (Bridge):** Starts the `joy_array_bridge` — translates high-level speed/steer topics into joint-controller array commands.
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run bgr_controller joy_array_bridge.py"
```

**Terminal 3 (Keyboard Launch):** Launches the keyboard controller.
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bgr_controller keyboard_teleop.launch.py"
```

**Terminal 4 (Keyboard Input):** Launches the interactive keyboard script. *(Keep this terminal focused to capture your `w, a, s, d, x, q` keystrokes)*
```bash
docker exec -it bgr_simulator bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run bgr_controller keyboard_teleop.py"
```

## Topics Exposed

The simulator bridges the following key Gazebo topics to standard ROS 2 topics:

*   `/scan/points` (`sensor_msgs/PointCloud2`): Pointcloud data from the onboard LiDAR.
    *   *To securely view the LiDAR data stream without freezing your terminal with numbers, use the `--no-arr` filter:*
        `ros2 topic echo /scan/points --no-arr`
    *   **Understanding the Output**: The `--no-arr` flag prevents ROS 2 from printing the massive list of point coordinates (which is nearly 3 million bytes of `data` per frame!). Instead, you'll see the core metadata of the LiDAR stream:
        *   `header.frame_id`: `bgr/base_footprint/lidar` (the sensor's frame of reference).
        *   `height` & `width`: The resolution of the scan (e.g., 256 vertical channels by 360 horizontal points).
        *   `point_step` & `row_step`: Byte sizes defining how the raw data array is laid out in memory.
*   `/imu` (`sensor_msgs/Imu`): Inertial Measurement Unit data (orientation, angular velocity, and linear acceleration).
    *   *To view the real-time IMU data stream, open a fresh terminal inside the container and run:*
        `ros2 topic echo /imu`


## Services Exposed

**Getting Track / Cone Poses:**
To retrieve the ground-truth poses of all cones for a specific track, you can call the `/get_track` service. 

For example, to get the cone positions for the `Acceleration` track, run the following command in a fresh terminal (make sure to be inside the container environment):
```bash
ros2 service call /get_track bgr_description/srv/GetTrack "{track_name: 'Acceleration'}"
```
This will return an array of `Cone` messages containing the `(x, y)` coordinates and `color` of each cone.

**Missing Models?** Ensure the environment variable is set correctly. The launch file handles this, but if you moved folders manually, check `gazebo.launch.py`.


**Vehicle doesn't spawn or is invisible in GUI?** 
1. Sometimes Gazebo takes longer than expected to load, causing a race condition where the vehicle spawns before the world is ready. The `gazebo.launch.py` now includes a 10-second delay for spawning to prevent this.
2. If it still happens, it may be due to "zombie" background processes. Kill them and try again:
   ```bash
   pkill -9 -f "gz sim"
   pkill -9 -f "ros_gz"
   pkill -9 -f "robot_state_publisher"
   killall -9 gz ros2 rviz2 ruby
   ```

---
