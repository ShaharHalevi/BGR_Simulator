#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class JoyArrayBridge(Node):
    def __init__(self):
        super().__init__('joy_array_bridge')

        # Publishers אל הבקרים שלך
        self.pub_wheels = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_steer  = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)

        # Subscribers מה-joy_teleop
        self.sub_speed = self.create_subscription(
            Float64, '/wheel_speed', self.on_speed, 10)
        self.sub_steer = self.create_subscription(
            Float64, '/steering_angle', self.on_steer, 10)

        # ערכים אחרונים (לשמירה)
        self.last_speed = 0.0
        self.last_angle = 0.0

    def on_speed(self, msg: Float64):
        self.last_speed = msg.data
        out = Float64MultiArray()
        out.data = [msg.data, msg.data, msg.data, msg.data]  # RL, RR, FL, FR
        self.pub_wheels.publish(out)

    def on_steer(self, msg: Float64):
        self.last_angle = msg.data
        out = Float64MultiArray()
        out.data = [msg.data]  # בקר ההיגוי שלך מצפה מערך בגודל 1
        self.pub_steer.publish(out)

def main():
    rclpy.init()
    node = JoyArrayBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
