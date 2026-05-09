#!/usr/bin/env python3

"""
EKF publisher signal conventions
================================

Published topics
----------------
/robot/datalogger_gt
/robot/datalogger_noisy
  Layout:
    [0] abs_speed    [m/s]
    [1] pos_x        [m]
    [2] pos_y        [m]
    [3] pos_z        [m]
    [4] acc_x        [m/s^2]
    [5] acc_y        [m/s^2]
    [6] acc_z        [m/s^2]
    [7] roll_rate    [rad/s]
    [8] pitch_rate   [rad/s]
    [9] yaw_rate     [rad/s]

/robot/mcu_gt
/robot/mcu_noisy
  Layout:
    [0] steering_angle [rad]
    [1] rpm_fl         [rpm]
    [2] rpm_fr         [rpm]
    [3] rpm_rl         [rpm]
    [4] rpm_rr         [rpm]

Sign conventions
----------------
pos_x:
  Positive when driving forward from the nominal spawn direction.

pos_y:
  Positive according to the simulator map Y direction.
  At the default spawn used in current tests, forward motion primarily changes pos_x.

pos_z:
  Positive upward.
  Note: when the car is standing on the ground, pos_z may be around 0.48 m
  because the published reference point is above the ground plane.

abs_speed:
  Always nonnegative. This is speed magnitude, not signed longitudinal velocity.

acc_x:
  Positive for forward acceleration.

acc_y:
  Positive for lateral acceleration to the left.

acc_z:
  Positive upward.

roll_rate:
  Positive by simulator right-hand-rule convention.

pitch_rate:
  Positive by simulator right-hand-rule convention.

yaw_rate:
  Positive for left turn, negative for right turn.

steering_angle:
  Positive for left steer, negative for right steer.

rpm_fl / rpm_fr / rpm_rl / rpm_rr:
  Positive for forward wheel rotation, negative for reverse.

Timing model
------------
The publisher runs at 50 Hz.
The datalogger topic is published at 50 Hz.
GPS-like fields [abs_speed, pos_x, pos_y, pos_z] are refreshed every 5 ticks
(10 Hz effective) and held constant between updates.
IMU-like fields are refreshed every tick (50 Hz).
MCU fields are refreshed every tick (50 Hz).

Noise / bias model
------------------
GPS:
  abs_speed, pos_x, pos_y, pos_z use additive white measurement noise.

IMU:
  acc_x_meas    = acc_x_true    + b_ax + white_noise
  acc_y_meas    = acc_y_true    + b_ay + white_noise
  yaw_rate_meas = yaw_rate_true + b_gz + white_noise

  Biases evolve as random walks:
    b_ax(k+1) = b_ax(k) + w_bax
    b_ay(k+1) = b_ay(k) + w_bay
    b_gz(k+1) = b_gz(k) + w_bgz

MCU:
  steering_meas = steering_true + b_delta + white_noise
  rpm_meas      = (1 + s_ws + s_common + s_wheel_i + s_dyn_i(t)) * rpm_true + white_noise
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(float(value), float(lo)), float(hi))


class EkfPublisher(Node):
    RAD_S_TO_RPM = 9.549296596425384

    def __init__(self):
        super().__init__("ekf_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ===== Timing =====
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("random_seed", 12345)
        self.declare_parameter("gps_update_every_n_ticks", 5)
        self.declare_parameter("enable_noise", True)
        self.declare_parameter("publish_stamped_topics", True)
        self.declare_parameter("max_joint_odom_skew_s", 0.05)

        # ===== Sign convention mapping =====
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("yaw_rate_sign", 1.0)
        self.declare_parameter("rpm_fl_sign", 1.0)
        self.declare_parameter("rpm_fr_sign", 1.0)
        self.declare_parameter("rpm_rl_sign", 1.0)
        self.declare_parameter("rpm_rr_sign", 1.0)

        # ===== GPS measurement noise =====
        self.declare_parameter("gps_speed_noise_std", 0.05)
        self.declare_parameter("gps_pos_x_noise_std", 0.20)
        self.declare_parameter("gps_pos_y_noise_std", 0.20)
        self.declare_parameter("gps_pos_z_noise_std", 0.10)

        # ===== IMU initial biases =====
        self.declare_parameter("acc_x_bias_initial", 0.0)
        self.declare_parameter("acc_y_bias_initial", 0.0)
        self.declare_parameter("gyro_z_bias_initial", 0.0)

        # ===== IMU white noise =====
        self.declare_parameter("acc_x_noise_std", 0.05)
        self.declare_parameter("acc_y_noise_std", 0.05)
        self.declare_parameter("acc_z_noise_std", 0.05)
        self.declare_parameter("roll_rate_noise_std", 0.001)
        self.declare_parameter("pitch_rate_noise_std", 0.001)
        self.declare_parameter("gyro_z_noise_std", 0.001)

        # ===== IMU bias random walk =====
        self.declare_parameter("acc_x_bias_rw_std", 0.0003)
        self.declare_parameter("acc_y_bias_rw_std", 0.0003)
        self.declare_parameter("gyro_z_bias_rw_std", 0.0005)

        # ===== MCU sensor model =====
        self.declare_parameter("steering_bias", 0.0)
        self.declare_parameter("steering_noise_std", 0.005)
        self.declare_parameter("wheel_speed_scale", 0.0)
        self.declare_parameter("rpm_fl_noise_std", 3.0)
        self.declare_parameter("rpm_fr_noise_std", 3.0)
        self.declare_parameter("rpm_rl_noise_std", 3.0)
        self.declare_parameter("rpm_rr_noise_std", 3.0)
        self.declare_parameter("wheel_speed_common_scale_bias_std", 0.01)
        self.declare_parameter("wheel_speed_per_wheel_scale_bias_std", 0.005)
        self.declare_parameter("wheel_speed_dynamic_scale_base_std", 0.0015)
        self.declare_parameter("wheel_speed_dynamic_scale_ax_gain", 0.004)
        self.declare_parameter("wheel_speed_dynamic_scale_yaw_gain", 0.02)
        self.declare_parameter("wheel_speed_dynamic_scale_tau_s", 0.35)
        self.declare_parameter("wheel_speed_dynamic_scale_max_abs", 0.05)

        self.odom_sub = self.create_subscription(
            Odometry, "/model/bgr/odometry", self.odom_callback, qos
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, qos
        )

        self.pub_datalogger_gt = self.create_publisher(Float64MultiArray, "/robot/datalogger_gt", 10)
        self.pub_datalogger_noisy = self.create_publisher(Float64MultiArray, "/robot/datalogger_noisy", 10)
        self.pub_mcu_gt = self.create_publisher(Float64MultiArray, "/robot/mcu_gt", 10)
        self.pub_mcu_noisy = self.create_publisher(Float64MultiArray, "/robot/mcu_noisy", 10)
        self.pub_datalogger_gt_stamped = self.create_publisher(Float64MultiArray, "/robot/datalogger_gt_stamped", 10)
        self.pub_datalogger_noisy_stamped = self.create_publisher(Float64MultiArray, "/robot/datalogger_noisy_stamped", 10)
        self.pub_mcu_gt_stamped = self.create_publisher(Float64MultiArray, "/robot/mcu_gt_stamped", 10)
        self.pub_mcu_noisy_stamped = self.create_publisher(Float64MultiArray, "/robot/mcu_noisy_stamped", 10)

        self.steering_joints = ["Steering_fl_joint", "Steering_fr_joint"]
        self.wheel_joints = [
            "Wheel_fl_joint",
            "Wheel_fr_joint",
            "Wheel_rl_joint",
            "Wheel_rr_joint",
        ]

        self.have_odom = False
        self.have_joint = False
        self.last_joint_time = None

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0

        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        self.steering_angle = 0.0
        self.rpms = [0.0, 0.0, 0.0, 0.0]

        self.last_odom_time = None
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0

        self.gps_gt_cache = [0.0, 0.0, 0.0, 0.0]
        self.gps_noisy_cache = [0.0, 0.0, 0.0, 0.0]

        self.bias_acc_x = self.get_parameter("acc_x_bias_initial").value
        self.bias_acc_y = self.get_parameter("acc_y_bias_initial").value
        self.bias_gyro_z = self.get_parameter("gyro_z_bias_initial").value

        self.tick = 0
        self._last_published_odom_time = None
        self._last_published_joint_time = None

        seed_param = self.get_parameter("random_seed").value
        if seed_param is None or int(seed_param) < 0:
            self.rng = random.Random()
            self.random_seed = None
        else:
            self.random_seed = int(seed_param)
            self.rng = random.Random(self.random_seed)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.dt_nominal = 1.0 / rate

        common_std = float(self.get_parameter("wheel_speed_common_scale_bias_std").value)
        per_wheel_std = float(self.get_parameter("wheel_speed_per_wheel_scale_bias_std").value)
        common_bias = self.gaussian(common_std) if common_std > 0.0 else 0.0
        self.static_wheel_scale_offsets = [
            common_bias + (self.gaussian(per_wheel_std) if per_wheel_std > 0.0 else 0.0)
            for _ in range(4)
        ]
        self.dynamic_wheel_scale_offsets = [0.0, 0.0, 0.0, 0.0]
        # Keep a lightweight timer so publishing stays serialized in one callback,
        # but only emit packets when the underlying source timestamp actually
        # advanced. This avoids duplicate stamped packets with identical source
        # times, which otherwise creates zero-dt EKF propagations.
        self.timer = self.create_timer(self.dt_nominal, self.publish_callback)

        seed_str = "system entropy" if self.random_seed is None else str(self.random_seed)
        self.get_logger().info(
            "ekf_publisher started: /robot/datalogger_*, /robot/mcu_* (+ optional *_stamped topics), "
            f"random_seed={seed_str}, static_wheel_scale_offsets={[round(v, 4) for v in self.static_wheel_scale_offsets]}"
        )

    def odom_callback(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        values = [px, py, pz, vx, vy, vz, wx, wy, wz]
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn("Dropping odometry sample with non-finite values")
            return

        self.pos_x = px
        self.pos_y = py
        self.pos_z = pz
        self.vel_x = vx
        self.vel_y = vy
        self.vel_z = vz
        self.roll_rate = wx
        self.pitch_rate = wy
        self.yaw_rate = wz

        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        if self.last_odom_time is not None:
            dt = t - self.last_odom_time
            if dt > 1e-6:
                dvx = (self.vel_x - self.last_vel_x) / dt
                dvy = (self.vel_y - self.last_vel_y) / dt
                dvz = (self.vel_z - self.last_vel_z) / dt
                # Odometry twist is expressed in the child frame (base_link).
                # Convert body-frame velocity derivatives into body-frame acceleration
                # using a = dv/dt + w x v. With yaw-only motion this becomes:
                self.acc_x = dvx - self.yaw_rate * self.vel_y
                self.acc_y = dvy + self.yaw_rate * self.vel_x
                self.acc_z = dvz

        self.last_odom_time = t
        self.last_vel_x = self.vel_x
        self.last_vel_y = self.vel_y
        self.last_vel_z = self.vel_z

        self.have_odom = True

    def joint_callback(self, msg: JointState):
        try:
            self.last_joint_time = _stamp_to_sec(msg.header.stamp) if msg.header.stamp is not None else None

            steering_values = []
            for joint_name in self.steering_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.position) and math.isfinite(msg.position[idx]):
                        steering_values.append(msg.position[idx])
            if steering_values:
                self.steering_angle = sum(steering_values) / len(steering_values)

            rpms = [0.0, 0.0, 0.0, 0.0]
            for i, joint_name in enumerate(self.wheel_joints):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.velocity) and math.isfinite(msg.velocity[idx]):
                        vel_rad_s = msg.velocity[idx]
                        rpms[i] = vel_rad_s * self.RAD_S_TO_RPM

            self.rpms = rpms
            self.have_joint = True

        except Exception as e:
            self.get_logger().warn(f"Joint parsing error: {e}")

    def gaussian(self, std_dev: float) -> float:
        return self.rng.gauss(0.0, float(std_dev))

    def update_bias_random_walk(self, dt: float):
        self.bias_acc_x += self.gaussian(
            self.get_parameter("acc_x_bias_rw_std").value * math.sqrt(dt)
        )
        self.bias_acc_y += self.gaussian(
            self.get_parameter("acc_y_bias_rw_std").value * math.sqrt(dt)
        )
        self.bias_gyro_z += self.gaussian(
            self.get_parameter("gyro_z_bias_rw_std").value * math.sqrt(dt)
        )

    def update_wheel_dynamic_scale(self, dt: float):
        dt = max(float(dt), 1e-3)
        tau_s = max(float(self.get_parameter("wheel_speed_dynamic_scale_tau_s").value), 1e-3)
        alpha = math.exp(-dt / tau_s)
        sqrt_drive = math.sqrt(max(1.0 - alpha * alpha, 0.0))
        base_std = float(self.get_parameter("wheel_speed_dynamic_scale_base_std").value)
        ax_gain = float(self.get_parameter("wheel_speed_dynamic_scale_ax_gain").value)
        yaw_gain = float(self.get_parameter("wheel_speed_dynamic_scale_yaw_gain").value)
        max_abs = float(self.get_parameter("wheel_speed_dynamic_scale_max_abs").value)
        dyn_std = base_std + ax_gain * abs(self.acc_x) + yaw_gain * abs(self.yaw_rate)
        for i, prev in enumerate(self.dynamic_wheel_scale_offsets):
            candidate = alpha * prev + self.gaussian(dyn_std * sqrt_drive)
            self.dynamic_wheel_scale_offsets[i] = _clamp(candidate, -max_abs, max_abs)

    def publish_callback(self):
        publish_datalogger = (
            self.have_odom
            and self.last_odom_time is not None
            and (
                self._last_published_odom_time is None
                or self.last_odom_time > self._last_published_odom_time + 1e-9
            )
        )
        publish_mcu = (
            self.have_joint
            and self.last_joint_time is not None
            and (
                self._last_published_joint_time is None
                or self.last_joint_time > self._last_published_joint_time + 1e-9
            )
        )

        if not (publish_datalogger or publish_mcu):
            return

        enable_noise = bool(self.get_parameter("enable_noise").value)
        publish_stamped_topics = bool(self.get_parameter("publish_stamped_topics").value)
        gps_update_every = int(self.get_parameter("gps_update_every_n_ticks").value)

        steering_sign = float(self.get_parameter("steering_sign").value)
        yaw_rate_sign = float(self.get_parameter("yaw_rate_sign").value)
        rpm_signs = [
            float(self.get_parameter("rpm_fl_sign").value),
            float(self.get_parameter("rpm_fr_sign").value),
            float(self.get_parameter("rpm_rl_sign").value),
            float(self.get_parameter("rpm_rr_sign").value),
        ]

        if publish_datalogger:
            self.tick += 1

            abs_speed = math.sqrt(self.vel_x ** 2 + self.vel_y ** 2)
            gps_gt = [
                abs_speed,
                self.pos_x,
                self.pos_y,
                self.pos_z,
            ]
            imu_gt = [
                self.acc_x,
                self.acc_y,
                self.acc_z,
                self.roll_rate,
                self.pitch_rate,
                yaw_rate_sign * self.yaw_rate,
            ]

            if self.tick % gps_update_every == 1:
                self.gps_gt_cache = list(gps_gt)
                if enable_noise:
                    self.gps_noisy_cache = [
                        gps_gt[0] + self.gaussian(self.get_parameter("gps_speed_noise_std").value),
                        gps_gt[1] + self.gaussian(self.get_parameter("gps_pos_x_noise_std").value),
                        gps_gt[2] + self.gaussian(self.get_parameter("gps_pos_y_noise_std").value),
                        gps_gt[3] + self.gaussian(self.get_parameter("gps_pos_z_noise_std").value),
                    ]
                else:
                    self.gps_noisy_cache = list(gps_gt)

            if enable_noise:
                if self._last_published_odom_time is None:
                    dt_rw = self.dt_nominal
                else:
                    dt_rw = max(self.last_odom_time - self._last_published_odom_time, self.dt_nominal)
                self.update_bias_random_walk(dt_rw)
                imu_noisy = [
                    imu_gt[0] + self.bias_acc_x + self.gaussian(self.get_parameter("acc_x_noise_std").value),
                    imu_gt[1] + self.bias_acc_y + self.gaussian(self.get_parameter("acc_y_noise_std").value),
                    imu_gt[2] + self.gaussian(self.get_parameter("acc_z_noise_std").value),
                    imu_gt[3] + self.gaussian(self.get_parameter("roll_rate_noise_std").value),
                    imu_gt[4] + self.gaussian(self.get_parameter("pitch_rate_noise_std").value),
                    imu_gt[5] + self.bias_gyro_z + self.gaussian(self.get_parameter("gyro_z_noise_std").value),
                ]
            else:
                imu_noisy = list(imu_gt)

            datalogger_gt = self.gps_gt_cache + imu_gt
            datalogger_noisy = self.gps_noisy_cache + imu_noisy

            msg = Float64MultiArray()
            msg.data = datalogger_gt
            self.pub_datalogger_gt.publish(msg)

            msg = Float64MultiArray()
            msg.data = datalogger_noisy
            self.pub_datalogger_noisy.publish(msg)

            if publish_stamped_topics:
                source_t = self.last_odom_time if self.last_odom_time is not None else self.get_clock().now().nanoseconds * 1e-9
                msg = Float64MultiArray()
                msg.data = [source_t] + datalogger_gt
                self.pub_datalogger_gt_stamped.publish(msg)

                msg = Float64MultiArray()
                msg.data = [source_t] + datalogger_noisy
                self.pub_datalogger_noisy_stamped.publish(msg)

            self._last_published_odom_time = self.last_odom_time

        if publish_mcu:
            wheel_speed_scale = float(self.get_parameter("wheel_speed_scale").value)
            steering_bias = float(self.get_parameter("steering_bias").value)

            mcu_gt = [
                steering_sign * self.steering_angle,
                rpm_signs[0] * self.rpms[0],
                rpm_signs[1] * self.rpms[1],
                rpm_signs[2] * self.rpms[2],
                rpm_signs[3] * self.rpms[3],
            ]

            if enable_noise:
                if self._last_published_joint_time is None:
                    dt_wheel = self.dt_nominal
                else:
                    dt_wheel = max(self.last_joint_time - self._last_published_joint_time, self.dt_nominal)
                self.update_wheel_dynamic_scale(dt_wheel)
                wheel_scales = [
                    1.0 + wheel_speed_scale + self.static_wheel_scale_offsets[i] + self.dynamic_wheel_scale_offsets[i]
                    for i in range(4)
                ]
                mcu_noisy = [
                    mcu_gt[0] + steering_bias + self.gaussian(self.get_parameter("steering_noise_std").value),
                    wheel_scales[0] * mcu_gt[1] + self.gaussian(self.get_parameter("rpm_fl_noise_std").value),
                    wheel_scales[1] * mcu_gt[2] + self.gaussian(self.get_parameter("rpm_fr_noise_std").value),
                    wheel_scales[2] * mcu_gt[3] + self.gaussian(self.get_parameter("rpm_rl_noise_std").value),
                    wheel_scales[3] * mcu_gt[4] + self.gaussian(self.get_parameter("rpm_rr_noise_std").value),
                ]
            else:
                mcu_noisy = list(mcu_gt)

            msg = Float64MultiArray()
            msg.data = mcu_gt
            self.pub_mcu_gt.publish(msg)

            msg = Float64MultiArray()
            msg.data = mcu_noisy
            self.pub_mcu_noisy.publish(msg)

            if publish_stamped_topics:
                joint_t = self.last_joint_time if self.last_joint_time is not None else self.get_clock().now().nanoseconds * 1e-9
                msg = Float64MultiArray()
                msg.data = [joint_t] + mcu_gt
                self.pub_mcu_gt_stamped.publish(msg)

                msg = Float64MultiArray()
                msg.data = [joint_t] + mcu_noisy
                self.pub_mcu_noisy_stamped.publish(msg)

            self._last_published_joint_time = self.last_joint_time


def main(args=None):
    rclpy.init(args=args)
    node = EkfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()