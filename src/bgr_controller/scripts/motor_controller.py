#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Physics Parameters
        self.gear_ratio = 11.5 
        self.motor_inertia = 0.000274 
        self.reflected_inertia = self.motor_inertia * (self.gear_ratio ** 2) 
        self.gain = 100.0  
        
        # Subscriber - מאזין לפקודת המהירות של הצוות
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            self.drive_callback,
            10)
            
        # Publisher - שולח כוח לכל 4 הגלגלים המניעים
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            10)

    def drive_callback(self, msg):
        effort_msg = Float64MultiArray()
        # חישוב המומנט: מהירות X אינרציה X הגבר
        # המערך תואם ל-YAML: [RR, RL, FR, FL]
        effort_msg.data = [float(vel * self.reflected_inertia * self.gain) for vel in msg.data]
        self.publisher.publish(effort_msg)
        
        # תיעוד בטרמינל כדי לראות שהסקריפט עובד
        if len(msg.data) > 0:
            self.get_logger().info(f'Driving with gain {self.gain}. First wheel effort: {effort_msg.data[0]:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()