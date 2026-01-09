# 🏎️ BGRacing Driverless Simulator (ROS 2 Jazzy)

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Gazebo Sim](https://img.shields.io/badge/Simulator-Gazebo_Sim-orange?logo=gazebo&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

A high-fidelity simulation environment for the **BGRacing Formula Student Team**.
Built on **ROS 2 Jazzy** and **Gazebo Sim (GZ)**, utilizing `ros2_control` for accurate vehicle dynamics and Ackermann steering.

---

## 📸 Gallery & Demos

### Simulation Environment
![Simulation View](src/TracksV0/models/asphalt_plane/thumbnails/1.png)
*Real-time simulation environment with custom tracks and cones.*

### Dashboard & GUI
![Dashboard View](https://placeholder.com/dashboard_screenshot.png)

### Driving Demo
[![Watch the video](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://youtu.be/YOUR_VIDEO_ID)

---

## 🚀 Features

* **Custom Ackermann Steering:** Realistic geometry with mimic joints for visualization.
* **ROS 2 Control Integration:** Velocity and Position controllers for precise wheel command.
* **Dynamic Tracks:** Load CSV based tracks (cones) directly into the simulation.
* **Interactive Tools:**
    * `track_gui.py`: Visual track selection and management.
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
colcon build --symlink-install
```

---

## 🏁 How to Run

Open separate terminals for each step (or use functionality like Tmux/Terminator). Always source your workspace in every new terminal:

```bash
source ~/bgr_ws/install/setup.bash
```

### Step 1: Launch the Simulation
This loads the track, the robot model (bgr_description), and the Gazebo environment.

```bash
ros2 launch bgr_description gazebo.launch.py
```

**Tip:** This also starts the car_dashboard and track_gui automatically.

### Step 2: Activate Controllers
Once the simulation is running, spawn the ros2_control managers:

```bash
ros2 launch bgr_controller controller.launch.py
```

You should see output confirming `joint_state_broadcaster`, `forward_velocity_controller`, and `forward_position_controller` are active.

### Step 3: Drive the Car (Keyboard Teleop)
To drive the car using your keyboard, run the teleoperation script:

```bash
ros2 run bgr_controller keyboard_teleop.py
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

---

**Maintained by:** BGRacing Simulator Team
