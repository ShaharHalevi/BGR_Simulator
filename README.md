# BGR_Simulator

This package contains the Gazebo harmonic simulation environment for the BGRacing car. It includes the URDF/Xacro robot descriptions, Gazebo worlds, custom dashboards, and the ROS 2 bridge configurations necessary to expose simulated hardware to the rest of the autonomy stack.

## Overview

The simulator runs inside its own Docker container and is built on **ROS 2 Jazzy**. It relies on `ros_gz` packages to bridge Gazebo topics into standard ROS 2 messages (e.g., `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `sensor_msgs/JointState`).

It also launches several custom GUI components:
- `track_gui.py`: Visualizes the track and race progress.
- `car_dashboard.py`: Displays speed, steering commands, and other telemetry.

---

## 🛠️ Prerequisites: Installing Docker

Before running the simulator, you must have Docker installed on your machine. Follow the instructions for your operating system below.

### 🪟 Windows — Docker Desktop

Docker Desktop is the easiest way to run Docker on Windows. It bundles the Docker Engine, Docker CLI, and a GUI dashboard.

> [!IMPORTANT]
> Docker Desktop on Windows uses **WSL 2** (Windows Subsystem for Linux 2) as its backend. Make sure WSL 2 is enabled before installing.

**Step 1 — Enable WSL 2 (if not already enabled):**

Open PowerShell as Administrator and run:
```powershell
wsl --install
```
This installs WSL 2 and the default Ubuntu distribution. **Restart your PC** when prompted.

You can verify WSL 2 is active at any time with:
```powershell
wsl --list --verbose
```
Make sure your distro shows `VERSION 2`.

**Step 2 — Download & Install Docker Desktop:**

1. Go to [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop) and download the **Docker Desktop for Windows** installer.
2. Run the installer. When prompted, ensure **"Use WSL 2 instead of Hyper-V"** is selected (it usually is by default).
3. Follow the on-screen prompts and complete the installation.
4. **Restart your PC** if asked.

**Step 3 — Start Docker Desktop:**

Launch Docker Desktop from the Start Menu. Wait for the whale 🐳 icon in your system tray to stop animating — this means the Docker Engine is ready.

> [!NOTE]
> Docker Desktop must be **running in the background** every time you use a `docker` command. If it is not open, commands like `docker build` or `docker run` will fail with a connection error.

**Step 4 — Verify the installation:**

Open a new PowerShell or Command Prompt terminal and run:
```powershell
docker --version
docker run hello-world
```
If you see a version number and a "Hello from Docker!" message, your setup is complete.

---

### 🐧 Linux — Docker Engine

On Linux, you install the Docker Engine directly (no desktop GUI app is needed).

> [!NOTE]
> These instructions are for **Ubuntu/Debian-based** distributions (e.g., Ubuntu 22.04, Ubuntu 24.04). For other distros (Fedora, Arch, etc.), refer to the [official Docker docs](https://docs.docker.com/engine/install/).

**Step 1 — Remove any old/conflicting Docker packages:**
```bash
sudo apt-get remove docker docker-engine docker.io containerd runc
```

**Step 2 — Set up the Docker apt repository:**
```bash
# Add Docker's official GPG key
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the Docker repository to apt sources
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
```

**Step 3 — Install Docker Engine:**
```bash
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**Step 4 — Run Docker without `sudo` (Post-install):**

By default, Docker requires `sudo`. To avoid typing `sudo` before every `docker` command, add your user to the `docker` group:
```bash
sudo usermod -aG docker $USER
```
**Log out and log back in** for this change to take effect.

**Step 5 — Enable Docker to start on boot:**
```bash
sudo systemctl enable docker
sudo systemctl start docker
```

**Step 6 — Verify the installation:**
```bash
docker --version
docker run hello-world
```
If you see a version number and a "Hello from Docker!" message, your setup is complete.

---

## 🏎️ Running the Simulator (Standalone)

If you wish to test or drive the simulator manually without launching the `perception` and `localization` containers, follow these instructions.

### 1. Configure the Display (X11)

> [!CAUTION]
> **Security Warning:** Proceeding with "Disable access control" in VcXsrv or running `xhost +local:root` in Linux weakens the security of your display server. This allows containerized (and potentially external network) applications to connect to your screen. It is highly recommended to only use these settings on trusted private networks.

Since the simulator relies heavily on GUI applications, ensure your host machine is configured to render Docker displays:

**If on Windows**:
1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
2. Open **XLaunch** from your Start Menu to configure the server:
   - **Display Settings:** Select **"Multiple windows"** and leave the Display number as `-1` (default). Click Next.
   - **Client Startup:** Select **"Start no client"**. Click Next.
   - **Extra Settings:** Ensure **"Clipboard"** is checked. **CRITICAL:** You must check the **"Disable access control"** box so Docker can communicate with it. Click Next.
   - Click **Finish**. (You should see an 'X' icon appear in your Windows system tray).
3. The `docker run` commands below already include `-e DISPLAY=host.docker.internal:0` which routes the simulation's GUI to this local X server.

**If on Linux**:
1. Run `xhost +local:root` on your host terminal.
2. Ensure the simulator's environment variable is set to `DISPLAY=${DISPLAY}` and the `/tmp/.X11-unix` volume is mounted.

### 2. Build and Launch

Navigate to the root of the `BGR_Simulator` repository (where the `Dockerfile` is located).

> [!NOTE]
> **Windows Users:** Ensure that **Docker Desktop** and **VcXsrv** (with access control disabled) are open and running in the background before attempting to build or run the image.

Build the image:
```bash
docker build -t bgr_simulator .
```

You can run the simulator in two modes: **Headed** (with gazibo GUI) or **Headless** (background physics and logic only, for lower PC resource usage).

#### Option A: Headed Mode (With GUI)
Run the container with full GUI support and Live Volume Mounting (so edits to Python scripts instantly take effect!).

**For Windows (PowerShell):**
```powershell
$GPU_FLAG = if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) { "--gpus all" } else { "" }
docker run --rm -it `
  --name bgr_simulator `
  $GPU_FLAG `
  -e DISPLAY=host.docker.internal:0 `
  -e QT_X11_NO_MITSHM=1 `
  -v ${PWD}/src:/ros2_ws/src/bgr_simulator/src:rw `
  bgr_simulator `
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py"
```

**For Linux / WSL:**
```bash
GPU_FLAG=$(command -v nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "")
docker run --rm -it \
  --name bgr_simulator \
  --network host \
  $GPU_FLAG \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/src:/ros2_ws/src/bgr_simulator/src:rw \
  bgr_simulator \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py"
```

#### Option B: Headless Gazebo (Python GUIs Only)
If you want to run Gazebo headlessly (to save PC resources) but still want your Python GUIs (like the track selector and speed dashboard) to appear, use this option. We still map the `DISPLAY` environment variables here so your Python apps can render, but we inject a headless flag (`-s`) on-the-fly specifically for Gazebo.

**For Windows (PowerShell):**
```powershell
$GPU_FLAG = if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) { "--gpus all" } else { "" }
docker run --rm -it `
  --name bgr_simulator `
  $GPU_FLAG `
  -e DISPLAY=host.docker.internal:0 `
  -e QT_X11_NO_MITSHM=1 `
  -v ${PWD}/src:/ros2_ws/src/bgr_simulator/src:rw `
  bgr_simulator `
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py headless:=True"
```

**For Linux / WSL:**
```bash
GPU_FLAG=$(command -v nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "")
docker run --rm -it \
  --name bgr_simulator \
  --network host \
  $GPU_FLAG \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/src:/ros2_ws/src/bgr_simulator/src:rw \
  bgr_simulator \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch bgr_description gazebo.launch.py headless:=True"
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

**Terminal 3 (Keyboard Input):** Launches the interactive keyboard script. *(Keep this terminal focused to capture your `w, a, s, d, x, q` keystrokes)*
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




