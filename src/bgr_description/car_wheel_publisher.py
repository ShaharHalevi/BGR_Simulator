#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class WheelsMonitor(Node):
    def __init__(self):
        super().__init__('wheels_monitor')

        # --- QoS Configuration ---
        # Using Best Effort to ensure we catch high-frequency updates from Gazebo
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber ---
        # Listening to the joint states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            qos_profile)

        # --- Publisher ---
        # Publishes an array containing [Steering, RPM_FL, RPM_FR, RPM_RL, RPM_RR]
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/robot/wheels_status', 
            10)

        # --- Joint Names Configuration (From your URDF) ---
        self.steering_joint = "Steering_fl_joint"
        
        # List of all 4 wheel joints in specific order: FL, FR, RL, RR
        self.wheel_joints = [
            "Wheel_fl_joint",  # Front Left
            "Wheel_fr_joint",  # Front Right
            "Wheel_rl_joint",  # Rear Left
            "Wheel_rr_joint"   # Rear Right
        ]

        self.get_logger().info(' Wheels Monitor Started. Tracking all 4 wheels.')

    def listener_callback(self, msg):
        """
        Callback to process joint data and calculate RPMs.
        """
        try:
            steering_angle = 0.0
            
            # List to store RPMs: [FL, FR, RL, RR]
            rpms = [0.0, 0.0, 0.0, 0.0]

            # --- 1. Get Steering Angle ---
            if self.steering_joint in msg.name:
                idx = msg.name.index(self.steering_joint)
                steering_angle = math.degrees(msg.position[idx])

            # --- 2. Get RPM for each wheel ---
            # Loop through our defined wheel names
            for i, joint_name in enumerate(self.wheel_joints):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    vel_rad_sec = msg.velocity[idx]
                    
                    # Convert rad/s to RPM
                    rpms[i] = vel_rad_sec * 9.549296596425384

            # --- 3. Publish Data ---
            # Array Format: [Steering, FL, FR, RL, RR]
            out_msg = Float64MultiArray()
            out_msg.data = [steering_angle] + rpms
            self.publisher_.publish(out_msg)

            # --- 4. Console Display ---
            # Formatting for clean output
            print(
                f"\r"
                f"Steer: {steering_angle:5.1f}° | "
                f"FL: {rpms[0]:5.1f} | "
                f"FR: {rpms[1]:5.1f} | "
                f"RL: {rpms[2]:5.1f} | "
                f"RR: {rpms[3]:5.1f} (RPM)   ", 
                end=""
            )

        except Exception as e:
            self.get_logger().warn(f"Data error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WheelsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping Wheels Monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()