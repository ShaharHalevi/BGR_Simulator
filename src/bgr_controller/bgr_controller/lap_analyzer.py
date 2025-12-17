#!/usr/bin/env python3
"""
    ROS 2 node for analyzing vehicle performance and providing a lap summary.

    """




import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from math import sqrt
import time

class LapAnalyzer(Node):
    
    def __init__(self):
        super().__init__('lap_analyzer')
        
        # 1. Subscriptions
        # Subscribes to Odometry for position, velocity, and lap detection
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribes to JointState for steering angle and velocity metrics
        self.joint_state_subscription = self.create_subscription( JointState, '/joint_states', self.joint_state_callback,10)
        
        # 2. State Variables and Settings
        self.is_lap_active = False
        self.start_time = None
        self.lap_counter = 0

        # Metrics storage and initialization
        self.reset_metrics()

        self.get_logger().info('Lap Analyzer Node Started. Ready for first lap.')
        self.summary_pub = self.create_publisher(String, 'lap_summary_data', 10)


    def reset_metrics(self):
        """Resets all metrics for a new lap."""
        self.max_speed = 0.0
        self.min_speed = float('inf')
        self.total_speed = 0.0
        self.speed_samples = 0
        self.total_distance = 0.0
        self.max_steer_angle = 0.0
        self.max_steering_velocity = 0.0
        
        # Store last position for cumulative distance calculation
        self.last_x = 0.0
        self.last_y = 0.0


    def odom_callback(self, msg):
        """Processes odometry data (position and velocity)."""
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        linear_v = abs(msg.twist.twist.linear.x) # Magnitude of linear velocity (forward/backward)

        # Cumulative Distance Calculation (Euclidean distance between sequential points)
        if self.is_lap_active:
            dx = current_x - self.last_x
            dy = current_y - self.last_y
            segment_distance = sqrt(dx*dx + dy*dy)
            self.total_distance += segment_distance
            
            # 1. Update Speed Metrics
            self.max_speed = max(self.max_speed, linear_v)
            # Only update min speed if the car is actually moving
            if linear_v > 0.01: 
                self.min_speed = min(self.min_speed, linear_v)
                
            self.total_speed += linear_v
            self.speed_samples += 1

        self.last_x = current_x
        self.last_y = current_y

        self.check_lap_start_end(current_x, linear_v)


    def joint_state_callback(self, msg):
        """Processes joint state data (Steering angles/velocities)."""
        if not self.is_lap_active:
            return

        try:
            # Find the primary steering joint (Steering_fr_joint, as per your URDF)
            fr_steer_index = msg.name.index('Steering_fr_joint')

            # Steering position and velocity magnitude
            steer_pos = abs(msg.position[fr_steer_index])
            steer_vel = abs(msg.velocity[fr_steer_index])
            
            # 2. Update Steering Control Metrics
            self.max_steer_angle = max(self.max_steer_angle, steer_pos)
            self.max_steering_velocity = max(self.max_steering_velocity, steer_vel)
            
            # NOTE: Max Effort could be tracked here if 'msg.effort' was used/exposed.
            
        except ValueError:
            # Handle case where joint name is not found (initial startup)
            pass


    def check_lap_start_end(self, x, v):
        """Checks for crossing the virtual start/finish line (X=0)."""
        
        # Lap line crossing condition: X coordinate close to 0, while moving forward (v > 0.1)
        # This prevents triggering the lap immediately at startup or due to minor noise.
        if abs(x) < 0.1 and v > 0.1:
            
            if not self.is_lap_active:
                # *** Lap Start ***
                self.is_lap_active = True
                self.start_time = time.time()
                self.get_logger().info('*** Lap Started! ***')
                
            else:
                # *** Lap End (Crossing the line again) ***
                end_time = time.time()
                lap_duration = end_time - self.start_time
                self.lap_counter += 1
                
                self.print_summary(lap_duration)
                
                # Reset and immediately start the next lap
                self.reset_metrics()
                self.is_lap_active = True
                self.start_time = end_time


    def print_summary(self, duration):
        """Prints the calculated lap summary to the console."""
        
        if self.speed_samples == 0:
            avg_speed = 0.0
            self.min_speed = 0.0
        else:
            avg_speed = self.total_speed / self.speed_samples
            
        # Constant from your URDF (0.6 radians)
        STEER_LIMIT = 0.6 
        
        self.get_logger().info('=============================================')
        self.get_logger().info(f'LAP SUMMARY - LAP {self.lap_counter} 🏁')
        self.get_logger().info('---------------------------------------------')
        
        # 1. Base Metrics
        self.get_logger().info(f'Lap Time: {duration:.3f} seconds')
        self.get_logger().info(f'Total Distance: {self.total_distance:.2f} meters')
        
        # 2. Speed Metrics (Performance Analysis)
        self.get_logger().info('--- Velocity Performance ---')
        self.get_logger().info(f' Avg Speed: {avg_speed:.2f} m/s')
        self.get_logger().info(f'⬆Max Speed: {self.max_speed:.2f} m/s')
        self.get_logger().info(f'⬇Min Speed: {self.min_speed:.2f} m/s')
        
        # 3. Control Metrics (Algorithm Quality)
        self.get_logger().info('--- Steering & Control ---')
        self.get_logger().info(f'Max Steer Angle (abs): {self.max_steer_angle:.2f} rad')
        
        # Utilization calculation for optimization feedback
        steer_utilization = self.max_steer_angle / STEER_LIMIT * 100 if STEER_LIMIT else 0
        self.get_logger().info(f'Max Steer Utilization: {steer_utilization:.1f}% (vs. Max {STEER_LIMIT} rad)')
        self.get_logger().info(f'Max Steer Velocity: {self.max_steering_velocity:.2f} rad/s (Measures smoothness)')
        
        self.get_logger().info('=============================================')

        # Publish summary message to GUI
        msg = String()
        msg.data = (f"Lap: {self.lap_counter} | "
                    f"Time: {duration:.2f}s | "
                    f"Max Speed: {self.max_speed:.2f} m/s | "
                    f"Steer Util: {steer_utilization:.1f}%")

        self.summary_pub.publish(msg) # send to gui

def main(args=None):
    rclpy.init(args=args)
    lap_analyzer = LapAnalyzer()
    
    try:
        rclpy.spin(lap_analyzer)
    except KeyboardInterrupt:
        pass
        
    # Clean up and shutdown
    lap_analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()