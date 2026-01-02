#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math

class SuperStateSpy(Node):
    def __init__(self):
        super().__init__('super_state_spy')
        
        # --- QoS Configuration ---
        # Gazebo usually publishes data using "Best Effort" reliability.
        # We must set our subscriber to match, otherwise we might not receive messages.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Subscriber ---
        # Listening to the ground-truth odometry topic from Gazebo
        self.subscription = self.create_subscription(
            Odometry,
            '/model/bgr/odometry',
            self.listener_callback,
            qos_profile)
            
        # --- Publisher ---
        # We will publish a single array containing: [Position, Orientation, Velocity, Acceleration]
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/robot/full_state', 
            10)
            
        # --- State Variables for Acceleration Calculation ---
        # We need to store previous values to calculate the derivative (dv/dt)
        self.last_time = None
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0
            
        self.get_logger().info('Full State Publisher Started. Monitoring: Pos, RPY, Vel, Accel')

    def listener_callback(self, msg):
        """
        Callback function executed every time a new Odometry message arrives.
        """
        
        # 1. Extract Current Time (in seconds)
        # Combine seconds and nanoseconds
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 2. Extract Current Velocity (Linear)
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z
        
        # 3. Calculate Acceleration
        # Formula: a = (v_final - v_initial) / (t_final - t_initial)
        acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            # Prevent division by zero
            if dt > 0:
                acc_x = (vel_x - self.last_vel_x) / dt
                acc_y = (vel_y - self.last_vel_y) / dt
                acc_z = (vel_z - self.last_vel_z) / dt
        
        # 4. Update "Last" variables for the next iteration
        self.last_time = current_time
        self.last_vel_x = vel_x
        self.last_vel_y = vel_y
        self.last_vel_z = vel_z

        # 5. Extract Position
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        
        # 6. Extract Orientation (Quaternion) & Convert to Euler
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)

        # 7. Pack and Publish Data
        # Creating a 12-element array
        out_msg = Float64MultiArray()
        out_msg.data = [
            pos_x, pos_y, pos_z,    # Indices 0-2
            roll, pitch, yaw,       # Indices 3-5 (Radians)
            vel_x, vel_y, vel_z,    # Indices 6-8
            acc_x, acc_y, acc_z     # Indices 9-11
        ]
        self.publisher_.publish(out_msg)
        
        # 8. Console Logging (Optional - for debugging)
        # Using carriage return (\r) to update the same line
        print(
            f"\r"
            f"POS: [{pos_x:.2f}, {pos_y:.2f}] | "
            f"VEL: [{vel_x:.2f}, {vel_y:.2f}] | "
            f"ACC: [{acc_x:.2f}, {acc_y:.2f}] | "
            f"YAW: {math.degrees(yaw):.1f}°   ", 
            end=""
        )

    # Movement helper function
    def euler_from_quaternion(self, x, y, z, w):
        """
        Helper function to convert Quaternion (x,y,z,w) to Euler Angles (Roll, Pitch, Yaw).
        Returns angles in Radians.
        """
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        # Clamp value to avoid domain errors
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z 

def main(args=None):
    rclpy.init(args=args)
    node = SuperStateSpy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()