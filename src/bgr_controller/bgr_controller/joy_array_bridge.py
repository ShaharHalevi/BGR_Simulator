#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class JoyArrayBridge(Node):
    def __init__(self):
        super().__init__('joy_array_bridge')

        self.wheelbase = 2.0368
        self.track = 1.3092
        self.steer_limit = 0.6
        self.max_center_steer = self._center_limit_from_wheel_limit()

        # Publishers to the ros2_control controllers
        # The forward_velocity_controller expects a Float64MultiArray of 4 values (for the 4 wheels)
        self.pub_wheels = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # The forward_position_controller expects [front_left, front_right] steering positions.
        self.pub_steer  = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)

        # Subscribers receiving input from the keyboard_teleop node
        self.sub_speed = self.create_subscription(
            Float64, '/wheel_speed', self.on_speed, 10)
        self.sub_steer = self.create_subscription(
            Float64, '/steering_angle', self.on_steer, 10)

        # Store the last received values
        self.last_speed = 0.0
        self.last_angle = 0.0

    def _center_limit_from_wheel_limit(self) -> float:
        inner_radius = self.wheelbase / math.tan(self.steer_limit)
        center_radius = inner_radius + self.track / 2.0
        return math.atan(self.wheelbase / center_radius)

    def _ackermann_angles(self, center_angle: float) -> tuple[float, float]:
        center_angle = max(-self.max_center_steer, min(self.max_center_steer, center_angle))
        if abs(center_angle) < 1e-6:
            return 0.0, 0.0

        turn_radius = self.wheelbase / math.tan(center_angle)
        left = math.atan(self.wheelbase / (turn_radius - self.track / 2.0))
        right = math.atan(self.wheelbase / (turn_radius + self.track / 2.0))
        left = max(-self.steer_limit, min(self.steer_limit, left))
        right = max(-self.steer_limit, min(self.steer_limit, right))
        return left, right

    def on_speed(self, msg: Float64):
        self.last_speed = msg.data
        out = Float64MultiArray()
        # Publish the target velocity to all four wheels: Rear-Left, Rear-Right, Front-Left, Front-Right
        out.data = [msg.data, msg.data, msg.data, msg.data]  
        self.pub_wheels.publish(out)

    def on_steer(self, msg: Float64):
        self.last_angle = msg.data
        out = Float64MultiArray()
        out.data = list(self._ackermann_angles(msg.data))
        self.pub_steer.publish(out)

def main():
    rclpy.init()
    node = JoyArrayBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
