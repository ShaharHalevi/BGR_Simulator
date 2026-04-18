# 🚀 BGR Simulator Launch Architecture

## Overview
The BGR Simulator launch sequence (`gazebo.launch.py`) uses a **deterministic, event-driven pipeline**. Unlike traditional ROS 2 launch files that rely on `time.sleep()`, this architecture uses **Readiness Verification Gates** to ensure each stage triggers only when the system is actually ready.

---

## 🏗️ Stage 1: Core Simulation Initialization
**Trigger:** Immediate.
- **Components:** Gazebo Server, Robot State Publisher, ROS-GZ Bridge.
- **Verification Gate:** `ros2 topic echo --once /clock`.
- **Result:** Confirms the physics engine is responsive and the bridge is active.

## 🚗 Stage 2: Vehicle Spawning
**Trigger:** Successful completion of Stage 1 (+ 7s physics buffer).
- **Components:** `gz_spawn_entity` (ros_gz_sim create).
- **Verification Gate:** `ros2 topic echo --once /model/bgr/odometry`.
- **Buffer Note:** We use a 7s buffer here to allow heavy track collision meshes to fully "cook" in real-time before dropping the car, preventing physics explosions.
- **Result:** Confirms the vehicle URDF has been successfully injected into physics and is broadcasting motion data.

## 🎮 Stage 3: Tooling & Logic
**Trigger:** Successful completion of Stage 2.
- **Components:** Controllers, Sensor noise publishers, Dashboards, Cone mapping.
- **Verification Gate:** Bash retry-loop for `gz service -s /gui/follow`.
- **Result:** Confirms secondary logic is active and the camera has snapped to the vehicle.

---

## 🛠️ Performance & Stability
- **Zero Race Conditions:** Controllers never start before the car exists.
- **Hardware Agnostic:** Automatically adjusts to host processor speed.
- **Headless Optimized:** The readiness checkers work perfectly in CI (GitHub Actions) without a GPU.

---

## 📋 Verification Checklist

### Functional Tests
To verify LiDAR, IMU, and Camera connectivity:
```bash
colcon test --packages-select bgr_description --event-handlers console_direct+
```

### Manual Verification
To see the stages happening in real-time, run:
```bash
ros2 launch bgr_description gazebo.launch.py
```
Look for:
- `[STAGE 1 COMPLETE] Gazebo /clock is active.`
- `[STAGE 2 START] Clock detected. Buffering for physics meshes...`
- `[STAGE 3 START] Vehicle is responsive.`
- `[GUI TRACKER] Successfully locked onto the vehicle!`
