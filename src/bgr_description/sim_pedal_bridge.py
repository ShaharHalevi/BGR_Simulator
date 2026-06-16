#!/usr/bin/env python3
"""
sim_pedal_bridge
================
Drop-in simulation equivalent of pedal_node/bridge_node.py.

Publishes the same vehicle/* topics with IDENTICAL:
  - topic names
  - message types (fs_msgs/VehicleStatus, geometry_msgs/TwistWithCovarianceStamped, etc.)
  - field semantics (units, covariance values, frame_ids)
  - QoS profiles (BEST_EFFORT sensor_qos / RELIABLE+TRANSIENT_LOCAL status_qos)

Performance Optimization:
  - Calculates and publishes the data that 'car_wheel_publisher.py' and 'car_state_publisher.py' previously worked on.
  - Publishes legacy '/robot/wheels_status' and '/robot/full_state' topics at 50 Hz.

Published Topics:
---------------------------------------------------------------------------------------------------------------------
Topic                       | Rate   | Message Type                            | Data Description
---------------------------------------------------------------------------------------------------------------------
vehicle/status              | 20 Hz  | fs_msgs/VehicleStatus                   | Heartbeat (AS_STATE_DRIVING [2])
vehicle/odom                | 50 Hz  | geometry_msgs/TwistWithCovarianceStamped| Twist-only body speed (x = avg rear wheels in m/s)
vehicle/wheel_speeds        | 50 Hz  | sensor_msgs/JointState                  | 4 wheels linear velocity (m/s)
vehicle/gps/fix             | 50 Hz  | sensor_msgs/NavSatFix                   | Geoposition (latitude, longitude, altitude)
vehicle/steering_angle      | 50 Hz  | std_msgs/Float32                        | Average front steering angle (degrees)
/robot/wheels_status        | 50 Hz  | std_msgs/Float64MultiArray              | Legacy: [steering (deg), RPM_FL, FR, RL, RR]
/robot/full_state           | 50 Hz  | std_msgs/Float64MultiArray              | Legacy: [Pos_x,y,z, Roll,Pitch,Yaw, Vel_x,y,z, Acc_x,y,z]
---------------------------------------------------------------------------------------------------------------------

Design:
  - vehicle/odom is TWIST-ONLY. No pose. linear.x = avg(rear wheel speeds).
  - vehicle/gps/fix is forwarded from the Gazebo NavSat sensor (/gps/fix).
  - vehicle/wheel_speeds are in m/s (rad/s * wheel_radius), same as real bridge.
  - vehicle/steering_angle is in DEGREES, same as real bridge.
  - vehicle/status is published at 20 Hz with a simulated DRIVING state.

"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Float64MultiArray, Header
from sensor_msgs.msg import JointState, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

from fs_msgs.msg import VehicleStatus


# ── Vehicle constants ─────────────────────────────────────────────────────────
# Must match URDF and planning_control/params.py.
WHEEL_RADIUS = 0.24  # m


def wheels_to_longitudinal(rl_mps: float, rr_mps: float) -> float:
    """Average rear-wheel speeds → longitudinal body speed. Mirrors bridge_node.py."""
    return 0.5 * (rl_mps + rr_mps)


class SimPedalBridge(Node):
    """
    Simulated ECU bridge — publishes vehicle/* topics from Gazebo simulator data.
    Also acts as the single source for legacy /robot/wheels_status and /robot/full_state
    topics to keep CPU deserialization minimal.
    """

    # Joint names as they appear in /joint_states (Gazebo → ros2_control)
    _WHEEL_JOINTS = ['Wheel_fl_joint', 'Wheel_fr_joint',
                     'Wheel_rl_joint', 'Wheel_rr_joint']
    _STEER_JOINTS = ['Steering_fl_joint', 'Steering_fr_joint']

    # Names published in vehicle/wheel_speeds — must match bridge_node.py's
    # wheel_joint_names parameter: ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']
    _WHEEL_NAMES = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']

    # Frame IDs — must match bridge_node.py's frame_id_base / frame_id_gps
    FRAME_BASE = 'base_link'
    FRAME_GPS  = 'gps_link'

    # Status timer rate — matches bridge_node.py's watchdog_rate_hz default (20 Hz)
    STATUS_HZ = 20.0
    # Telemetry timer rate — matches real car STM32 telemetry publish rate (50 Hz)
    TELEMETRY_HZ = 50.0

    def __init__(self):
        super().__init__('sim_pedal_bridge')

        # ── Cached states ─────────────────────────────────────────────────────
        self._latest_joint_state = None
        self._latest_gps = None
        self._latest_odom = None

        # ── State Variables for Acceleration Calculation ──────────────────────
        self.last_time = None
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0

        # ── QoS profiles — identical to bridge_node.py ────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # 1. Official vehicle/* topics
        self.pub_status = self.create_publisher(
            VehicleStatus, 'vehicle/status', status_qos)
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped, 'vehicle/odom', sensor_qos)
        self.pub_wheels = self.create_publisher(
            JointState, 'vehicle/wheel_speeds', sensor_qos)
        self.pub_gps = self.create_publisher(
            NavSatFix, 'vehicle/gps/fix', sensor_qos)
        self.pub_steering = self.create_publisher(
            Float32, 'vehicle/steering_angle', sensor_qos)

        # 2. Legacy /robot/* topics (now processed here to save CPU)
        self.pub_legacy_wheels = self.create_publisher(
            Float64MultiArray, '/robot/wheels_status', 10)
        self.pub_legacy_full_state = self.create_publisher(
            Float64MultiArray, '/robot/full_state', 10)

        # ── Subscribers — fast caching, no immediate serialization ────────────
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, sensor_qos)
        self.create_subscription(
            NavSatFix, '/gps/fix', self._on_gps, sensor_qos)
        self.create_subscription(
            Odometry, '/model/bgr/odometry', self._on_odom, sensor_qos)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.STATUS_HZ, self._publish_status)
        self.create_timer(1.0 / self.TELEMETRY_HZ, self._publish_telemetry)

        self.get_logger().info(
            'sim_pedal_bridge started — mirroring pedal_node vehicle/* topics and legacy robot/* topics.'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _now_header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """Helper to convert Quaternion (x,y,z,w) to Euler Angles (Roll, Pitch, Yaw) in Radians."""
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        """Cache incoming joint state (200 Hz)."""
        self._latest_joint_state = msg

    def _on_gps(self, msg: NavSatFix):
        """Cache incoming GPS NavSatFix."""
        self._latest_gps = msg

    def _on_odom(self, msg: Odometry):
        """Cache incoming ground-truth Odometry (50 Hz)."""
        self._latest_odom = msg

    def _publish_telemetry(self):
        """
        Processes and publishes all telemetry topics at 50 Hz.
        Matches STM32 telemetry update frequency on the physical vehicle.
        """
        # ── 1. Process Joint States if available ──────────────────────────────
        msg = self._latest_joint_state
        if msg is not None:
            stamp     = self.get_clock().now().to_msg()
            name_list = list(msg.name)
            vel_list  = list(msg.velocity)
            pos_list  = list(msg.position)

            def vel(joint: str) -> float:
                try:
                    return float(vel_list[name_list.index(joint)])
                except ValueError:
                    return 0.0

            def pos(joint: str) -> float:
                try:
                    return float(pos_list[name_list.index(joint)])
                except ValueError:
                    return 0.0

            # Linear speeds (m/s)
            fl_mps = vel('Wheel_fl_joint') * WHEEL_RADIUS
            fr_mps = vel('Wheel_fr_joint') * WHEEL_RADIUS
            rl_mps = vel('Wheel_rl_joint') * WHEEL_RADIUS
            rr_mps = vel('Wheel_rr_joint') * WHEEL_RADIUS

            # Official topic: vehicle/wheel_speeds
            js = JointState()
            js.header.stamp    = stamp
            js.header.frame_id = self.FRAME_BASE
            js.name     = self._WHEEL_NAMES
            js.velocity = [fl_mps, fr_mps, rl_mps, rr_mps]
            self.pub_wheels.publish(js)

            # Official topic: vehicle/odom (twist only)
            tw = TwistWithCovarianceStamped()
            tw.header.stamp    = stamp
            tw.header.frame_id = self.FRAME_BASE
            tw.twist.twist.linear.x = wheels_to_longitudinal(rl_mps, rr_mps)
            tw.twist.covariance = [0.0] * 36
            tw.twist.covariance[0] = 0.05 ** 2
            for idx in (7, 14, 21, 28, 35):
                tw.twist.covariance[idx] = -1.0
            self.pub_twist.publish(tw)

            # Official topic: vehicle/steering_angle (degrees)
            angles_rad = [pos('Steering_fl_joint'), pos('Steering_fr_joint')]
            avg_deg = math.degrees(sum(angles_rad) / len(angles_rad)) if angles_rad else 0.0
            st = Float32()
            st.data = float(avg_deg)
            self.pub_steering.publish(st)

            # ── 1c. Legacy topic: /robot/wheels_status (replacing car_wheel_publisher.py) ──
            # Maps joint velocities (rad/s) to RPMs using the conversion: 1 rad/s = 60 / (2 * pi) RPM
            # Pack array format: [SteeringAngle (deg), RPM_FL, RPM_FR, RPM_RL, RPM_RR]
            rpms = [
                vel('Wheel_fl_joint') * 9.549296596425384,
                vel('Wheel_fr_joint') * 9.549296596425384,
                vel('Wheel_rl_joint') * 9.549296596425384,
                vel('Wheel_rr_joint') * 9.549296596425384
            ]
            legacy_ws = Float64MultiArray()
            legacy_ws.data = [avg_deg] + rpms
            self.pub_legacy_wheels.publish(legacy_ws)

        # ── 2. Process GPS if available ───────────────────────────────────────
        gps_msg = self._latest_gps
        if gps_msg is not None:
            fwd = NavSatFix()
            fwd.header.stamp    = self.get_clock().now().to_msg()
            fwd.header.frame_id = self.FRAME_GPS
            fwd.status.status   = NavSatStatus.STATUS_FIX
            fwd.status.service  = NavSatStatus.SERVICE_GPS
            fwd.latitude        = gps_msg.latitude
            fwd.longitude       = gps_msg.longitude
            fwd.altitude        = gps_msg.altitude
            fwd.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub_gps.publish(fwd)

        # ── 3. Process Odometry for Legacy /robot/full_state (replacing car_state_publisher.py) ──
        odom_msg = self._latest_odom
        if odom_msg is not None:
            # Time stamp conversion (seconds)
            current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
            
            # Linear body velocities
            vel_x = odom_msg.twist.twist.linear.x
            vel_y = odom_msg.twist.twist.linear.y
            vel_z = odom_msg.twist.twist.linear.z
            
            # Calculate linear acceleration components via numerical differentiation: a = dv / dt
            acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
            if self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 0:
                    acc_x = (vel_x - self.last_vel_x) / dt
                    acc_y = (vel_y - self.last_vel_y) / dt
                    acc_z = (vel_z - self.last_vel_z) / dt
            
            # Cache velocities and timestamp for the next derivative step
            self.last_time = current_time
            self.last_vel_x = vel_x
            self.last_vel_y = vel_y
            self.last_vel_z = vel_z

            # Extract 3D position
            pos_x = odom_msg.pose.pose.position.x
            pos_y = odom_msg.pose.pose.position.y
            pos_z = odom_msg.pose.pose.position.z
            
            # Convert orientation from Quaternion (x, y, z, w) to Euler angles (Roll, Pitch, Yaw)
            qx = odom_msg.pose.pose.orientation.x
            qy = odom_msg.pose.pose.orientation.y
            qz = odom_msg.pose.pose.orientation.z
            qw = odom_msg.pose.pose.orientation.w
            roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)

            # Package and publish the 12-element legacy full state array
            legacy_fs = Float64MultiArray()
            legacy_fs.data = [
                pos_x, pos_y, pos_z,    # Indices 0-2: Position (meters)
                roll, pitch, yaw,       # Indices 3-5: Euler orientation (radians)
                vel_x, vel_y, vel_z,    # Indices 6-8: Linear velocity (m/s)
                acc_x, acc_y, acc_z     # Indices 9-11: Linear acceleration (m/s^2)
            ]
            self.pub_legacy_full_state.publish(legacy_fs)

    def _publish_status(self):
        """Publish vehicle/status at 20 Hz with a simulated DRIVING state."""
        status = VehicleStatus()
        status.header       = self._now_header(self.FRAME_BASE)
        status.as_state     = VehicleStatus.AS_STATE_DRIVING
        status.mission_id   = 1
        status.ebs_armed    = False
        status.sensor_error = False
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimPedalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
