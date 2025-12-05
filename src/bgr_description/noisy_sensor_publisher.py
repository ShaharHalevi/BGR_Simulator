#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math
import random

class NoisySensorPublisher(Node):
    def __init__(self):
        super().__init__('noisy_sensor_publisher')

        # --- QoS Configuration ---
        # Must match Gazebo's "Best Effort" policy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber (The Truth) ---
        # We subscribe to the perfect data from Gazebo
        self.subscription = self.create_subscription(
            Odometry,
            '/model/bgr/odometry',
            self.listener_callback,
            qos_profile)

        # --- Publisher (The Simulation) ---
        # This publishes the NOISY data that simulates real-world sensors
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/robot/noisy_state', 
            10)

        # --- Noise Parameters (Based on Ublox M8N & Generic IMU) ---
        # These values represent one Standard Deviation (Sigma)
        
        # GPS Position Noise (Meters) - Ublox M8N is approx 2.0m CEP
        self.NOISE_POS_XY = 2.0  
        self.NOISE_POS_Z = 3.0   # Vertical accuracy is usually worse
        
        # GPS Speed Noise (m/s) - Ublox M8N is very accurate (0.05 m/s)
        self.NOISE_VEL = 0.05    
        
        # IMU Orientation Noise (Degrees)
        self.NOISE_YAW = 0.5     
        
        # IMU Acceleration Noise (m/s^2)
        self.NOISE_ACCEL = 0.15  

        # --- Internal State for Acceleration Calculation ---
        self.last_time = None
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0

        self.get_logger().info('Noisy Sensor Node Started. Simulating Ublox M8N & IMU imperfections.')

    def listener_callback(self, msg):
        """
        1. Reads perfect data.
        2. Adds realistic noise.
        3. Publishes to /robot/noisy_state.
        """
        
        # --- 1. Get Time & Calculate DT ---
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0
        if self.last_time is not None:
            dt = current_time - self.last_time
        
        # --- 2. Extract Perfect Velocities ---
        true_vel_x = msg.twist.twist.linear.x
        true_vel_y = msg.twist.twist.linear.y
        true_vel_z = msg.twist.twist.linear.z
        
        # --- 3. Calculate Perfect Acceleration ---
        true_acc_x, true_acc_y, true_acc_z = 0.0, 0.0, 0.0
        if dt > 0:
            true_acc_x = (true_vel_x - self.last_vel_x) / dt
            true_acc_y = (true_vel_y - self.last_vel_y) / dt
            true_acc_z = (true_vel_z - self.last_vel_z) / dt

        # Update history
        self.last_time = current_time
        self.last_vel_x = true_vel_x
        self.last_vel_y = true_vel_y
        self.last_vel_z = true_vel_z

        # --- 4. Extract Perfect Position & Orientation ---
        true_pos_x = msg.pose.pose.position.x
        true_pos_y = msg.pose.pose.position.y
        true_pos_z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        true_roll, true_pitch, true_yaw = self.euler_from_quaternion(qx, qy, qz, qw)

        # ==========================================================
        #      NOISE INJECTION (The Magic Happens Here)
        # ==========================================================
        
        # 1. Add Noise to Position (GPS Simulation)
        noisy_pos_x = true_pos_x + random.gauss(0, self.NOISE_POS_XY)
        noisy_pos_y = true_pos_y + random.gauss(0, self.NOISE_POS_XY)
        noisy_pos_z = true_pos_z + random.gauss(0, self.NOISE_POS_Z)

        # 2. Add Noise to Velocity (GPS Doppler Simulation)
        noisy_vel_x = true_vel_x + random.gauss(0, self.NOISE_VEL)
        noisy_vel_y = true_vel_y + random.gauss(0, self.NOISE_VEL)
        noisy_vel_z = true_vel_z + random.gauss(0, self.NOISE_VEL)

        # 3. Add Noise to Orientation (IMU/Compass Simulation)
        # Convert degrees noise to radians for math
        noise_yaw_rad = math.radians(random.gauss(0, self.NOISE_YAW))
        noisy_yaw = true_yaw + noise_yaw_rad
        
        # We typically don't add much noise to roll/pitch in this simple model, 
        # but you can add it if needed.
        noisy_roll = true_roll 
        noisy_pitch = true_pitch

        # 4. Add Noise to Acceleration (IMU Accelerometer Simulation)
        noisy_acc_x = true_acc_x + random.gauss(0, self.NOISE_ACCEL)
        noisy_acc_y = true_acc_y + random.gauss(0, self.NOISE_ACCEL)
        noisy_acc_z = true_acc_z + random.gauss(0, self.NOISE_ACCEL)

        # ==========================================================

        # --- 5. Publish the Noisy Data ---
        out_msg = Float64MultiArray()
        out_msg.data = [
            noisy_pos_x, noisy_pos_y, noisy_pos_z,  # 0-2 (Noisy GPS)
            noisy_roll, noisy_pitch, noisy_yaw,     # 3-5 (Noisy IMU Angle)
            noisy_vel_x, noisy_vel_y, noisy_vel_z,  # 6-8 (Noisy GPS Vel)
            noisy_acc_x, noisy_acc_y, noisy_acc_z   # 9-11 (Noisy IMU Accel)
        ]
        self.publisher_.publish(out_msg)

        # --- 6. Print Comparison (For you to see the effect) ---
        # We print True X vs Noisy X to show the drift
        print(f"\rREAL X: {true_pos_x:6.2f}m  VS  NOISY GPS X: {noisy_pos_x:6.2f}m (Err: {abs(true_pos_x - noisy_pos_x):.2f}m)   ", end="")

    def euler_from_quaternion(self, x, y, z, w):
        """ Helper for quaternion conversion """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z 

def main(args=None):
    rclpy.init(args=args)
    node = NoisySensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping Noisy Sensor Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()