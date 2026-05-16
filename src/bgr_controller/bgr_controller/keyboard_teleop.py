#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


msg = """
Reading from the keyboard!
---------------------------
Moving around:
   w
a  s  d

x : Force stop
q : Reset steering to 0

CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        
        self.pub_speed = self.create_publisher(Float64, '/wheel_speed', 10)
        self.pub_steer = self.create_publisher(Float64, '/steering_angle', 10)

        self.speed = 0.0
        self.steer = 0.0
        self.speed_step = 1.0   
        self.steer_step = 0.05
        self.max_speed = 20.0
        self.max_steer = 0.384

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)  
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                
                # Adjust speed based on user input
                if key == 'w':
                    self.speed = min(self.speed + self.speed_step, self.max_speed)
                elif key == 's':
                    self.speed = max(self.speed - self.speed_step, -self.max_speed)
                
                # Adjust steering based on user input
                elif key == 'a':
                    self.steer = min(self.steer + self.steer_step, self.max_steer)
                elif key == 'd':
                    self.steer = max(self.steer - self.steer_step, -self.max_steer)
                
                # Reset steering to center
                elif key == 'q':
                    self.steer = 0.0
                
                # Emergency stop
                elif key == 'x':
                    self.speed = 0.0
                    self.steer = 0.0
                
                # Exit on CTRL+C
                elif key == '\x03':
                    break

                # Create and publish the new speed message
                v_msg = Float64()
                v_msg.data = float(self.speed)
                self.pub_speed.publish(v_msg)

                # Create and publish the new steering message
                s_msg = Float64()
                s_msg.data = float(self.steer)
                self.pub_steer.publish(s_msg)

                # Print the current state to the terminal
                print(f"\rSpeed: {self.speed:.2f} | Steer: {self.steer:.2f}", end="")

        except Exception as e:
            print(e)

        finally:
            # When exiting, ensure to publish a stop message to halt the car
            stop_msg = Float64()
            stop_msg.data = 0.0
            self.pub_speed.publish(stop_msg)
            self.pub_steer.publish(stop_msg)
            
            # Restore the original terminal settings (undo raw mode)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()