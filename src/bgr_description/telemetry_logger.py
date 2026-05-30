#!/usr/bin/env python3
import os
import sys
import csv
import json
import math
import threading
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from bgr_description.msg import Cone, ConeArray
from bgr_description.srv import GetTrack

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger')

        # 1. Declare Parameters
        self.declare_parameter('world_name', 'CompetitionMap1Opt')

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value

        self.get_logger().info(f"Initializing Telemetry Logger for world: {self.world_name}")

        # 2. World Filtering
        world_lower = self.world_name.lower()
        self.enabled = True
        if "skidpad" in world_lower or "training" in world_lower or "opt" not in world_lower:
            self.get_logger().warn(f"Telemetry logging disabled for world '{self.world_name}'.")
            self.enabled = False

        # 3. Setup Directory for Logs
        self.wall_start_time = datetime.now()
        now_str = self.wall_start_time.strftime("%Y%m%d_%H%M%S")
        
        # Dynamically find the ROS 2 workspace root by traversing up
        # to find a directory named src, install, build, or log.
        workspace_root = None
        try:
            current_dir = os.path.abspath(__file__)
            while True:
                parent_dir = os.path.dirname(current_dir)
                if parent_dir == current_dir: # Reached filesystem root '/'
                    break
                base = os.path.basename(parent_dir)
                if base in ["src", "install", "build", "log"]:
                    workspace_root = os.path.dirname(parent_dir)
                    break
                current_dir = parent_dir
        except Exception:
            pass

        if workspace_root:
            self.log_dir = os.path.join(workspace_root, "sim-logs", f"run_{now_str}")
        else:
            self.log_dir = os.path.join(os.getcwd(), "sim-logs", f"run_{now_str}")

        if self.enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            self.get_logger().info(f"Logging telemetry to: {self.log_dir}")

        # 4. State variables for telemetry history
        self.lock = threading.Lock()
        
        self.time_history = []
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.roll_history = []
        self.pitch_history = []
        self.yaw_history = []
        self.vx_history = []
        self.vy_history = []
        self.wz_history = []
        self.ax_history = []
        self.ay_history = []

        self.steer_cmd_history = []
        self.steer_actual_history = []
        self.speed_cmd_history = []

        self.x_noisy_history = []
        self.y_noisy_history = []
        self.yaw_noisy_history = []

        self.rpm_fl_history = []
        self.rpm_fr_history = []
        self.rpm_rl_history = []
        self.rpm_rr_history = []

        self.loc_err_history = []

        # Current cached state of slow/asynchronous topics (for forward-fill)
        self.latest_noisy_state = [0.0] * 12
        self.latest_wheels_status = [0.0] * 5
        self.latest_steer_cmd = 0.0
        self.latest_speed_cmd = 0.0
        
        self.latest_vx = 0.0
        self.latest_vy = 0.0

        # Calculations for angular velocity wz, distance sum, and lap tracking
        self.last_full_state_time = None
        self.last_yaw = 0.0
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        self.distance_since_last_crossing = 0.0

        self.last_tracked_x = None
        self.last_tracked_y = None
        self.lap_start_time = None
        self.last_crossing_time = None
        self.lap_times = []
        
        # Cone collision tracking
        self.collision_events = []
        self.unique_cones_hit = set()

        # Gate cones coordinates (defaults to fallback)
        self.gate_p1 = None
        self.gate_p2 = None
        self.track_cones = []

        # 5. Service Client to Fetch Track Map
        if self.enabled:
            self.track_client = self.create_client(GetTrack, 'get_track')
            self.fetch_track_cones()

        # 6. QoS Configuration (Matching Best Effort of Gazebo/Aggregators)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 7. Subscriptions
        if self.enabled:
            self.full_state_sub = self.create_subscription(
                Float64MultiArray,
                '/robot/full_state',
                self.full_state_callback,
                qos_profile
            )

            self.noisy_state_sub = self.create_subscription(
                Float64MultiArray,
                '/robot/noisy_state',
                self.noisy_state_callback,
                qos_profile
            )

            self.wheels_status_sub = self.create_subscription(
                Float64MultiArray,
                '/robot/wheels_status',
                self.wheels_status_callback,
                qos_profile
            )

            self.steer_cmd_sub = self.create_subscription(
                Float64MultiArray,
                '/forward_position_controller/commands',
                self.steer_cmd_callback,
                10
            )

            self.speed_cmd_sub = self.create_subscription(
                Float64MultiArray,
                '/forward_velocity_controller/commands',
                self.speed_cmd_callback,
                10
            )

            self.collision_sub = self.create_subscription(
                Cone,
                'cone_collision',
                self.cone_collision_callback,
                10
            )

            self.accumulated_collision_sub = self.create_subscription(
                ConeArray,
                'collided_cones',
                self.collided_cones_callback,
                10
            )

        # 8. Register Signal Handlers for SIGHUP and SIGTERM
        try:
            signal.signal(signal.SIGTERM, self._handle_shutdown_signal)
            signal.signal(signal.SIGHUP, self._handle_shutdown_signal)
        except (ValueError, OSError):
            pass

    def _handle_shutdown_signal(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Propagating graceful exit...")
        raise SystemExit

    # --- Geometric helpers for gate crossing check ---
    def ccw(self, A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def intersect(self, A, B, C, D):
        return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)

    # --- Callbacks ---
    def noisy_state_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 12:
                self.latest_noisy_state = list(msg.data)

    def wheels_status_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 5:
                self.latest_wheels_status = list(msg.data)

    def steer_cmd_callback(self, msg):
        with self.lock:
            if len(msg.data) > 0:
                self.latest_steer_cmd = msg.data[0]

    def speed_cmd_callback(self, msg):
        with self.lock:
            if len(msg.data) > 0:
                self.latest_speed_cmd = msg.data[0]

    def cone_collision_callback(self, msg):
        with self.lock:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            vx = self.latest_vx
            vy = self.latest_vy
            v_mag = math.sqrt(vx**2 + vy**2)

            collision_info = {
                "timestamp_sim": current_time,
                "name": msg.id,
                "color": msg.color,
                "position": [msg.x, msg.y],
                "velocity": [vx, vy]
            }
            self.collision_events.append(collision_info)
            self.unique_cones_hit.add(msg.id)
            self.get_logger().warn(f"[COLLISION] Hit {msg.id} ({msg.color}) at {v_mag:.2f} m/s.")

    def collided_cones_callback(self, msg):
        # We can optionally keep sync with the list from visible_cones
        with self.lock:
            for cone in msg.cones:
                self.unique_cones_hit.add(cone.id)

    def full_state_callback(self, msg):
        with self.lock:
            if len(msg.data) < 12:
                return

            current_time = self.get_clock().now().nanoseconds * 1e-9

            # Ground Truth Values
            x, y, z = msg.data[0], msg.data[1], msg.data[2]
            roll, pitch, yaw = msg.data[3], msg.data[4], msg.data[5]
            vx, vy, vz = msg.data[6], msg.data[7], msg.data[8]
            ax, ay, az = msg.data[9], msg.data[10], msg.data[11]

            # Detect simulation backward time jump (e.g. world reset)
            if self.last_full_state_time is not None and current_time < self.last_full_state_time:
                self.get_logger().warn("Simulation time jump backwards detected! Resetting tracking history.")
                self.last_full_state_time = None
                self.last_x = None
                self.last_y = None
                self.last_tracked_x = None
                self.last_tracked_y = None
                self.lap_start_time = None
                self.last_crossing_time = None
                self.distance_since_last_crossing = 0.0

            self.latest_vx = vx
            self.latest_vy = vy

            # Calculate wz (yaw rate)
            wz = 0.0
            if self.last_full_state_time is not None:
                dt = current_time - self.last_full_state_time
                if dt > 0:
                    diff_yaw = yaw - self.last_yaw
                    diff_yaw = (diff_yaw + math.pi) % (2 * math.pi) - math.pi
                    wz = diff_yaw / dt

            # Calculate total distance driven & detect teleports
            dist_step = 0.0
            is_teleport = False
            if self.last_x is not None and self.last_y is not None:
                dist_step = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if self.last_full_state_time is not None:
                    dt = current_time - self.last_full_state_time
                    if dt > 0 and (dist_step / dt) > 150.0:
                        is_teleport = True
                        self.get_logger().warn(f"Teleportation detected (apparent speed {dist_step/dt:.1f} m/s). Resetting step state.")

            if is_teleport:
                # Reset tracking history to current state to prevent distance spikes or false crossings
                self.last_full_state_time = current_time
                self.last_yaw = yaw
                self.last_x = x
                self.last_y = y
                self.last_tracked_x = x
                self.last_tracked_y = y
                dist_step = 0.0
                self.lap_start_time = None
                self.last_crossing_time = None
                self.distance_since_last_crossing = 0.0
            else:
                self.total_distance += dist_step
                self.distance_since_last_crossing += dist_step
                self.last_full_state_time = current_time
                self.last_yaw = yaw
                self.last_x = x
                self.last_y = y

            # Get latest values from other topics
            x_noisy = self.latest_noisy_state[0]
            y_noisy = self.latest_noisy_state[1]
            yaw_noisy = self.latest_noisy_state[5]

            steer_actual = self.latest_wheels_status[0]
            rpm_fl = self.latest_wheels_status[1]
            rpm_fr = self.latest_wheels_status[2]
            rpm_rl = self.latest_wheels_status[3]
            rpm_rr = self.latest_wheels_status[4]

            steer_cmd = self.latest_steer_cmd
            speed_cmd = self.latest_speed_cmd

            # Calculate localization error (distance on XY ground-plane)
            loc_err = math.sqrt((x - x_noisy)**2 + (y - y_noisy)**2)
            self.loc_err_history.append(loc_err)

            # Check Lap Crossing
            if not is_teleport:
                self.check_lap_crossing(current_time, x, y)

            # Append to lists
            self.time_history.append(current_time)
            self.x_history.append(x)
            self.y_history.append(y)
            self.z_history.append(z)
            self.roll_history.append(roll)
            self.pitch_history.append(pitch)
            self.yaw_history.append(yaw)
            self.vx_history.append(vx)
            self.vy_history.append(vy)
            self.wz_history.append(wz)
            self.ax_history.append(ax)
            self.ay_history.append(ay)

            self.steer_cmd_history.append(steer_cmd)
            self.steer_actual_history.append(steer_actual)
            self.speed_cmd_history.append(speed_cmd)

            self.x_noisy_history.append(x_noisy)
            self.y_noisy_history.append(y_noisy)
            self.yaw_noisy_history.append(yaw_noisy)

            self.rpm_fl_history.append(rpm_fl)
            self.rpm_fr_history.append(rpm_fr)
            self.rpm_rl_history.append(rpm_rl)
            self.rpm_rr_history.append(rpm_rr)

    def check_lap_crossing(self, current_time, x, y):
        if self.gate_p1 is None or self.gate_p2 is None:
            return

        # Auto-start timer if we spawn near the gate and start moving
        if self.lap_start_time is None:
            gate_mid_x = (self.gate_p1[0] + self.gate_p2[0]) / 2.0
            gate_mid_y = (self.gate_p1[1] + self.gate_p2[1]) / 2.0
            dist_to_gate = math.sqrt((x - gate_mid_x)**2 + (y - gate_mid_y)**2)
            if dist_to_gate < 5.0:
                v_mag = math.sqrt(self.latest_vx**2 + self.latest_vy**2)
                if v_mag > 0.1:
                    self.lap_start_time = current_time
                    self.last_crossing_time = current_time
                    self.distance_since_last_crossing = 0.0
                    self.get_logger().info(f"[LAP TRACKER] Movement detected (speed={v_mag:.2f} m/s). Lap timer started at sim time: {current_time:.3f} s")
                    self.last_tracked_x = x
                    self.last_tracked_y = y
                    return
                else:
                    return

        if self.last_tracked_x is None or self.last_tracked_y is None:
            self.last_tracked_x = x
            self.last_tracked_y = y
            return

        A = (self.last_tracked_x, self.last_tracked_y)
        B = (x, y)
        C = self.gate_p1
        D = self.gate_p2

        if self.intersect(A, B, C, D):
            if self.lap_start_time is None:
                # First crossing starts the timer
                self.lap_start_time = current_time
                self.last_crossing_time = current_time
                self.distance_since_last_crossing = 0.0
                self.get_logger().info(f"[LAP TRACKER] Start/Finish line crossed. Lap timer started at sim time: {current_time:.3f} s")
            else:
                # Subsequent crossing (check 30m distance rule)
                if self.distance_since_last_crossing >= 30.0:
                    lap_time = current_time - self.last_crossing_time
                    self.lap_times.append(lap_time)
                    self.last_crossing_time = current_time
                    self.distance_since_last_crossing = 0.0
                    self.get_logger().info(f"[LAP TRACKER] Lap {len(self.lap_times)} completed: {lap_time:.3f} s. (Best: {min(self.lap_times):.3f} s)")
                else:
                    self.get_logger().warn(f"[LAP TRACKER] Start/Finish crossing ignored. Distance since last crossing ({self.distance_since_last_crossing:.2f} m) is less than 30m.")

        self.last_tracked_x = x
        self.last_tracked_y = y

    # --- Service Client ---
    def fetch_track_cones(self):
        self.get_logger().info("Requesting track cones from /get_track...")
        if not self.track_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /get_track not available! Using default gate fallback.")
            self.set_default_gate()
            return

        req = GetTrack.Request()
        req.track_name = self.world_name
        future = self.track_client.call_async(req)
        future.add_done_callback(self.track_service_callback)

    def track_service_callback(self, future):
        try:
            res = future.result()
            if res and res.success:
                with self.lock:
                    self.track_cones = res.cones
                    self.get_logger().info(f"Successfully cached {len(self.track_cones)} track cones.")
                    self.find_gate_cones()
            else:
                self.get_logger().error(f"Track service failed: {res.message}. Using default gate fallback.")
                self.set_default_gate()
        except Exception as e:
            self.get_logger().error(f"Track service call failed: {e}. Using default gate fallback.")
            self.set_default_gate()

    def find_gate_cones(self):
        # Assumes lock is held by caller (track_service_callback)
        p1, p2 = None, None
        for cone in self.track_cones:
            if cone.id == "cone_orange_big_001":
                p1 = (cone.x, cone.y)
            elif cone.id == "cone_orange_big_002":
                p2 = (cone.x, cone.y)

        if p1 is not None and p2 is not None:
            self.gate_p1 = p1
            self.gate_p2 = p2
            self.get_logger().info(f"Start/Finish gate configured using cone coordinates: {p1} <-> {p2}")
        else:
            self.get_logger().warn("Gate cones 'cone_orange_big_001'/'cone_orange_big_002' not found. Using default gate fallback.")
            self.set_default_gate()

    def set_default_gate(self):
        with self.lock:
            self.gate_p1 = (0.0, -1.726)
            self.gate_p2 = (0.0, 1.726)
            self.get_logger().info(f"Fallback gate set: {self.gate_p1} <-> {self.gate_p2}")

    # --- Data Serialization on Shutdown ---
    def save_data(self):
        if not self.enabled:
            return
        # Shallow copy historical lists under the lock to minimize hold time
        with self.lock:
            if not self.time_history:
                self.get_logger().warn("No telemetry data recorded (empty run). Saving minimal session summary.")
                self.save_minimal_summary()
                return

            time_hist = list(self.time_history)
            x_hist = list(self.x_history)
            y_hist = list(self.y_history)
            z_hist = list(self.z_history)
            roll_hist = list(self.roll_history)
            pitch_hist = list(self.pitch_history)
            yaw_hist = list(self.yaw_history)
            vx_hist = list(self.vx_history)
            vy_hist = list(self.vy_history)
            wz_hist = list(self.wz_history)
            ax_hist = list(self.ax_history)
            ay_hist = list(self.ay_history)

            steer_cmd_hist = list(self.steer_cmd_history)
            steer_actual_hist = list(self.steer_actual_history)
            speed_cmd_hist = list(self.speed_cmd_history)

            x_noisy_hist = list(self.x_noisy_history)
            y_noisy_hist = list(self.y_noisy_history)
            yaw_noisy_hist = list(self.yaw_noisy_history)

            rpm_fl_hist = list(self.rpm_fl_history)
            rpm_fr_hist = list(self.rpm_fr_history)
            rpm_rl_hist = list(self.rpm_rl_history)
            rpm_rr_hist = list(self.rpm_rr_history)

            loc_err_hist = list(self.loc_err_history)

            total_dist = self.total_distance
            lap_times = list(self.lap_times)
            collision_events = list(self.collision_events)
            unique_cones = set(self.unique_cones_hit)
            track_cones = list(self.track_cones)
            world_name = self.world_name
            wall_start = self.wall_start_time
            log_dir = self.log_dir

        # Perform I/O and visualization rendering outside the lock
        self.get_logger().info("Saving session records...")
        self._write_csv(log_dir, time_hist, x_hist, y_hist, z_hist, roll_hist, pitch_hist, yaw_hist,
                       vx_hist, vy_hist, wz_hist, ax_hist, ay_hist, steer_cmd_hist, steer_actual_hist,
                       speed_cmd_hist, x_noisy_hist, y_noisy_hist, yaw_noisy_hist,
                       rpm_fl_hist, rpm_fr_hist, rpm_rl_hist, rpm_rr_hist, loc_err_hist)
        
        self._write_full_summary(log_dir, time_hist, vx_hist, vy_hist, loc_err_hist, total_dist,
                                lap_times, collision_events, unique_cones, world_name, wall_start)
        
        self._write_plot(log_dir, track_cones, x_hist, y_hist, vx_hist, vy_hist, collision_events, world_name)
        self.get_logger().info(f"Session data successfully saved to: {log_dir}")

    def _write_csv(self, log_dir, time_hist, x_hist, y_hist, z_hist, roll_hist, pitch_hist, yaw_hist,
                   vx_hist, vy_hist, wz_hist, ax_hist, ay_hist, steer_cmd_hist, steer_actual_hist,
                   speed_cmd_hist, x_noisy_hist, y_noisy_hist, yaw_noisy_hist,
                   rpm_fl_hist, rpm_fr_hist, rpm_rl_hist, rpm_rr_hist, loc_err_hist):
        csv_path = os.path.join(log_dir, "telemetry.csv")
        try:
            with open(csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "timestamp_sim", "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "wz", "ax", "ay",
                    "steer_cmd", "steer_actual", "speed_cmd", "x_noisy", "y_noisy", "yaw_noisy",
                    "rpm_fl", "rpm_fr", "rpm_rl", "rpm_rr", "localization_error"
                ])
                for i in range(len(time_hist)):
                    writer.writerow([
                        time_hist[i], x_hist[i], y_hist[i], z_hist[i],
                        roll_hist[i], pitch_hist[i], yaw_hist[i],
                        vx_hist[i], vy_hist[i], wz_hist[i], ax_hist[i], ay_hist[i],
                        steer_cmd_hist[i], steer_actual_hist[i], speed_cmd_hist[i],
                        x_noisy_hist[i], y_noisy_hist[i], yaw_noisy_hist[i],
                        rpm_fl_hist[i], rpm_fr_hist[i], rpm_rl_hist[i], rpm_rr_hist[i],
                        loc_err_hist[i]
                    ])
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def _write_full_summary(self, log_dir, time_hist, vx_hist, vy_hist, loc_err_hist, total_dist,
                            lap_times, collision_events, unique_cones, world_name, wall_start):
        summary_path = os.path.join(log_dir, "session_summary.json")
        wall_end_time = datetime.now()
        duration = time_hist[-1] - time_hist[0] if len(time_hist) > 1 else 0.0

        # Calculate speed stats
        speeds = [math.sqrt(vx**2 + vy**2) for vx, vy in zip(vx_hist, vy_hist)]
        max_speed = max(speeds) if speeds else 0.0
        avg_speed = sum(speeds) / len(speeds) if speeds else 0.0

        # Calculate localization error stats
        max_loc_err = max(loc_err_hist) if loc_err_hist else 0.0
        avg_loc_err = sum(loc_err_hist) / len(loc_err_hist) if loc_err_hist else 0.0

        summary = {
            "metadata": {
                "wall_clock_start": wall_start.strftime("%Y-%m-%d %H:%M:%S"),
                "wall_clock_end": wall_end_time.strftime("%Y-%m-%d %H:%M:%S"),
                "simulation_duration_sec": duration,
                "track_name": world_name,
                "total_distance_driven_meters": total_dist
            },
            "performance_metrics": {
                "total_laps": len(lap_times),
                "lap_times_sec": lap_times,
                "best_lap_time_sec": min(lap_times) if lap_times else None,
                "average_lap_time_sec": sum(lap_times) / len(lap_times) if lap_times else None,
                "max_speed_mps": max_speed,
                "average_speed_mps": avg_speed
            },
            "incident_log": {
                "total_cone_collisions": len(collision_events),
                "total_unique_cones_hit": len(unique_cones),
                "collisions": collision_events
            },
            "localization_diagnostics": {
                "average_localization_error_meters": avg_loc_err,
                "max_localization_error_meters": max_loc_err
            }
        }
        try:
            with open(summary_path, 'w') as file:
                json.dump(summary, file, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to write session summary JSON: {e}")

    def save_minimal_summary(self):
        summary_path = os.path.join(self.log_dir, "session_summary.json")
        wall_end_time = datetime.now()
        summary = {
            "metadata": {
                "wall_clock_start": self.wall_start_time.strftime("%Y-%m-%d %H:%M:%S"),
                "wall_clock_end": wall_end_time.strftime("%Y-%m-%d %H:%M:%S"),
                "simulation_duration_sec": 0.0,
                "track_name": self.world_name,
                "total_distance_driven_meters": 0.0
            },
            "performance_metrics": {
                "total_laps": 0,
                "lap_times_sec": [],
                "best_lap_time_sec": None,
                "average_lap_time_sec": None,
                "max_speed_mps": 0.0,
                "average_speed_mps": 0.0
            },
            "incident_log": {
                "total_cone_collisions": 0,
                "total_unique_cones_hit": 0,
                "collisions": []
            },
            "localization_diagnostics": {
                "average_localization_error_meters": 0.0,
                "max_localization_error_meters": 0.0
            }
        }
        try:
            with open(summary_path, 'w') as file:
                json.dump(summary, file, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to write minimal JSON: {e}")

    def _write_plot(self, log_dir, track_cones, x_hist, y_hist, vx_hist, vy_hist, collision_events, world_name):
        try:
            plt.figure(figsize=(10, 8))

            # 1. Plot Track Cones (colored appropriately)
            cone_xs = {"blue": [], "yellow": [], "orange": [], "orange_big": [], "other": []}
            cone_ys = {"blue": [], "yellow": [], "orange": [], "orange_big": [], "other": []}
            for cone in track_cones:
                color = cone.color
                if color in cone_xs:
                    cone_xs[color].append(cone.x)
                    cone_ys[color].append(cone.y)
                else:
                    if "orange" in color:
                        cone_xs["orange"].append(cone.x)
                        cone_ys["orange"].append(cone.y)
                    else:
                        cone_xs["other"].append(cone.x)
                        cone_ys["other"].append(cone.y)

            plt.scatter(cone_xs["blue"], cone_ys["blue"], color='royalblue', marker='o', s=25, label='Blue Cones')
            plt.scatter(cone_xs["yellow"], cone_ys["yellow"], color='gold', marker='o', s=25, label='Yellow Cones')
            plt.scatter(cone_xs["orange"], cone_ys["orange"], color='darkorange', marker='o', s=25, label='Orange Cones')
            plt.scatter(cone_xs["orange_big"], cone_ys["orange_big"], color='orangered', marker='^', s=50, label='Big Orange Cones (Start)')
            plt.scatter(cone_xs["other"], cone_ys["other"], color='gray', marker='o', s=20, label='Other Cones')

            # 2. Downsample Trajectory to optimize rendering speed (limit to 10k points)
            speeds = [math.sqrt(vx**2 + vy**2) for vx, vy in zip(vx_hist, vy_hist)]
            downsample_factor = max(1, len(x_hist) // 10000)
            x_plot = x_hist[::downsample_factor]
            y_plot = y_hist[::downsample_factor]
            speeds_plot = speeds[::downsample_factor]

            sc = plt.scatter(x_plot, y_plot, c=speeds_plot, cmap='jet', s=3, label='Trajectory', alpha=0.8)
            cbar = plt.colorbar(sc)
            cbar.set_label('Speed (m/s)')

            # 3. Plot Red 'X' for cone collisions
            if collision_events:
                col_xs = [col["position"][0] for col in collision_events]
                col_ys = [col["position"][1] for col in collision_events]
                plt.scatter(col_xs, col_ys, color='red', marker='x', s=80, linewidths=2.5, zorder=5, label='Cone Hits')

            # Set plot details
            plt.title(f"Spatial Trajectory Plot - {world_name}")
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")
            plt.axis('equal')
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.legend(loc='upper right')

            plt.savefig(os.path.join(log_dir, "trajectory_plot.png"), dpi=300, bbox_inches='tight')
            plt.close()
        except Exception as e:
            self.get_logger().error(f"Failed to generate trajectory plot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
