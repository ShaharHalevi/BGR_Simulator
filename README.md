# BGRacing – Driverless Simulator (ROS 2 + GZ Sim)

*A clear, stable Ackermann robot model for our Formula Student simulator team.*
*Includes URDF/Xacro, Gazebo (GZ Sim) tuning, `ros2_control` controllers, and ready-to-run launch files.*

![ROS 2](https://img.shields.io/badge/ROS%202-Iron%20%7C%20Humble-blue)
![GZ Sim](https://img.shields.io/badge/GZ%20Sim-8%2B-brightgreen)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

---

## 1) What’s inside

* **`bgr_description`** – URDF/Xacro model, meshes, Gazebo + RViz launch files

  * Ackermann layout (front steering with **mimic**), sane inertials, wheel friction/slip tuning
* **`bgr_controller`** – `ros2_control` YAML + controller spawner launch

  * **Velocity** command for wheels; **Position** command for the **right** steering joint (left follows via mimic)
* All files include **simple, plain-English comments** for fast onboarding.

---

## 2) Repository layout

```
bgr_robot/
├─ .gitignore
├─ LICENSE
├─ README.md
└─ src/
   ├─ bgr_description/
   │  ├─ package.xml
   │  ├─ CMakeLists.txt
   │  ├─ urdf/
   │  │  ├─ bgr.urdf.xacro
   │  │  ├─ bgr_gazebo.xacro
   │  │  └─ bgr_ros2_control.xacro
   │  ├─ meshes/
   │  │  └─ visual/ (base_link.STL, Wheel_fl.STL, Wheel_fr.STL, Wheel_rl.STL, Wheel_rr.STL)
   │  ├─ launch/
   │  │  ├─ gz.launch.py          # GZ Sim world + spawn
   │  │  └─ display.launch.py     # RViz display (no sim)
   │  └─ rviz/
   │     └─ display.rviz
   └─ bgr_controller/
      ├─ package.xml
      ├─ CMakeLists.txt
      ├─ config/
      │  └─ bgr_controllers.yaml  # ros2_control config
      └─ launch/
         └─ controllers.launch.py # spawns controllers
```

> If you already have a workspace `~/ws_bgr`, place the two packages (`bgr_description`, `bgr_controller`) in `~/ws_bgr/src/`.

---

## 3) Requirements

* ROS 2 **Humble** (Ignition Gazebo) **or** ROS 2 **Iron/Jazzy** (GZ Sim)
* Packages: `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `controller_manager`, `joint_state_publisher_gui`, `xacro`

Install (replace `${ROS_DISTRO}` with `humble` / `iron` / `jazzy`):

```bash
sudo apt update
sudo apt install \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-xacro
```

---

## 4) Quick Start (VS Code + Terminal)

Follow these steps exactly. Commands are for Linux/macOS terminals (Bash/Zsh).

### 4.1 Clone the repo and open it in VS Code

**Option A – inside VS Code**

1. Open **VS Code**.
2. `Ctrl+Shift+P` (macOS: `Cmd+Shift+P`) → **Git: Clone** → paste your repo URL.
3. Choose a folder (e.g., `~/ws_bgr/src`) and click **Open** when VS Code asks.

**Option B – from a normal terminal**

```bash
mkdir -p ~/ws_bgr/src
cd ~/ws_bgr/src
git clone <YOUR_REPO_URL>
code .   # opens VS Code in the workspace
```

### 4.2 Build with colcon

Open a terminal (**Terminal → New Terminal** in VS Code) and run:

```bash
# Ensure you are in the workspace root (~/ws_bgr)
cd ~/ws_bgr

# First time only: install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source the workspace (applies to THIS terminal only)
source install/setup.bash
```

### 4.3 Start the simulator (Gazebo / GZ Sim) OR RViz (display only)

**Option A – Gazebo (GZ Sim)**

```bash
# Terminal A (already sourced)
ros2 launch bgr_description gz.launch.py
```

**Option B – RViz (no simulator)**

```bash
# Terminal A (already sourced)
ros2 launch bgr_description display.launch.py
```

> If the robot appears too high/low in Gazebo, adjust `-z` in `bgr_description/launch/gz.launch.py`.

### 4.4 Start controllers (new terminal)

Open a **new terminal** and **source** again:

```bash
# Terminal B
cd ~/ws_bgr
source install/setup.bash

# Start controllers: joint_state_broadcaster, velocity, position
ros2 launch bgr_controller controllers.launch.py
```

Expected state:

```
joint_state_broadcaster         active
forward_velocity_controller     active
forward_position_controller     active
```

Check:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

### 4.5 Drive the robot (new terminal)

Open **another** new terminal and **source** again:

```bash
# Terminal C
cd ~/ws_bgr
source install/setup.bash
```

**Topics**

* **Velocity (wheels)**: `/forward_velocity_controller/commands` (`std_msgs/Float64MultiArray`)
* **Position (right steering)**: `/forward_position_controller/commands` (`std_msgs/Float64MultiArray`)
* **Left steering is a mimic** of the right with multiplier **-1.0** (do not command it directly).

**Drive straight**

```bash
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/Float64MultiArray "data:
- 5.0  # Wheel_rr
- 5.0  # Wheel_rl
- 5.0  # Wheel_fr (remove if RWD only)
- 5.0  # Wheel_fl (remove if RWD only)
"
```

**Turn right, then go forward**

```bash
# Steering right (~0.3 rad)
ros2 topic pub --once /forward_position_controller/commands std_msgs/Float64MultiArray "data: [0.3]"

# Forward at moderate speed
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/Float64MultiArray "data: [2.0, 2.0, 2.0, 2.0]"
```

**Center steering + stop**

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/Float64MultiArray "data: [0.0]"
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
```

> **RWD only?** Remove `Wheel_fr_joint` and `Wheel_fl_joint` from the YAML velocity controller, rebuild, then publish only **2** values (RR, RL).

---

## 5) Troubleshooting

**Wheels spin but robot doesn’t move**

* Let the robot settle; ensure wheels touch ground (spawn at `-z ~ 1.0–1.2`)
* Wheel joint origins: **Z = -wheel_radius**
* Wheel **collision** must be a cylinder rolled **+90°** (axis = Y)
* Check `bgr_gazebo.xacro` friction/slip (`mu1/mu2 ~ 1.2`, slip ~ `0.01`)

**Robot sinks / is stuck**

* Base collision origin should be **`+body_size_z/2`** so the body sits above ground
* Keep contact `kp` high and `max_vel` small (already set)

**Steering only on one side**

* Right steering (`Steering_fr_joint`) is commanded (position)
* Left steering (`Steering_fl_joint`) is **mimic** with multiplier **-1.0** (URDF + physics plugin)
* Names in launch/YAML must match exactly

**Controllers won’t activate**

* `ros2 control list_hardware_interfaces` → joints must export `velocity` / `position`
* Start `joint_state_broadcaster` **before** other controllers (our launch does)
* Ensure controller names in launch equal YAML names

**RViz shows nothing**

* Set **Fixed Frame** = `base_link`
* Ensure `robot_state_publisher` runs with `robot_description`

---

## 6) Common changes

* **RWD only** – remove the front wheel joints from the velocity controller YAML; rebuild.
* **Different world** – change `empty.sdf` in `bgr_description/launch/gz.launch.py`.
* **Steering range** – edit `steer_limit` (radians) in `bgr.urdf.xacro` and keep controller min/max consistent.
* **Mesh orientation off** – adjust `<visual><origin rpy="...">` in the wheel macro (collision already aligned).

---

## 7) Team context (why)

* **Team:** BGRacing – Driverless Simulator (BGU Formula Student)
* **Goal:** Smooth, realistic Ackermann dynamics for controller testing before track days
* **Next:** add Innoviz LiDAR + camera via GZ plugins, perception topics, and a `cmd_vel` → wheel/steering translator node

---

## 8) Contributing

* Branches: `feat/<area>`, `fix/<area>`, `docs/<area>`
* PR checklist:

  * Launches run clean (no critical warnings)
  * Robot spawns on ground; wheels touch ground
  * Steering mimic works
  * `ros2 control list_controllers` shows 3 active controllers
  * Keep comments in **simple English**

---

## 9) License

Released under the **MIT License**. See `LICENSE`.

---

## 10) Quick start (3 commands)

```bash
# Build
cd ~/ws_bgr && colcon build && source install/setup.bash

# Sim + robot
ros2 launch bgr_description gz.launch.py

# Controllers
ros2 launch bgr_controller controllers.launch.py
```
