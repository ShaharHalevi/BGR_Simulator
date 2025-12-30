#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import math

class CarStateListener(Node):
    def __init__(self, data_callback):
        super().__init__('car_dashboard_listener')
        self.data_callback = data_callback
        
        # Subscribe to the topic defined in car_state_publisher.py
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/robot/full_state',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.data_callback(msg.data)

class DashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("BGR Car Dashboard")
        self.root.geometry("300x400")
        
        # Data Variables
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.total_speed = 0.0

        # Layout
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 1. Speed Section
        ttk.Label(main_frame, text="SPEED (m/s)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W, pady=(0,5))
        self.lbl_speed = ttk.Label(main_frame, text="0.00", font=("Consolas", 20))
        self.lbl_speed.pack(anchor=tk.W, padx=20)
        
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # 2. Acceleration Section
        ttk.Label(main_frame, text="ACCELERATION (m/s²)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W, pady=(0,5))
        
        acc_frame = ttk.Frame(main_frame)
        acc_frame.pack(fill=tk.X, padx=10)
        
        ttk.Label(acc_frame, text="X:").grid(row=0, column=0, sticky=tk.W)
        self.lbl_acc_x = ttk.Label(acc_frame, text="0.00", font=("Consolas", 10))
        self.lbl_acc_x.grid(row=0, column=1, sticky=tk.W, padx=(5, 15))

        ttk.Label(acc_frame, text="Y:").grid(row=0, column=2, sticky=tk.W)
        self.lbl_acc_y = ttk.Label(acc_frame, text="0.00", font=("Consolas", 10))
        self.lbl_acc_y.grid(row=0, column=3, sticky=tk.W, padx=5)

    def update_data(self, data):
        # data indices: 6-8 is Vel, 9-11 is Acc
        if len(data) < 12: return

        self.vel = data[6:9]
        self.acc = data[9:12]
        self.total_speed = math.sqrt(self.vel[0]**2 + self.vel[1]**2) # Magnitude of X and Y velocity
        
        # Index 5 is Yaw in Radians. Convert to Degrees.
        yaw_deg = data[5] * (180 / math.pi)
        self.lbl_yaw.config(text=f"{yaw_deg:.1f}°")

        self.root.after(0, self.refresh_ui)

    def refresh_ui(self):
        self.lbl_speed.config(text=f"{self.total_speed:.2f}")
        self.lbl_acc_x.config(text=f"{self.acc[0]:.2f}")
        self.lbl_acc_y.config(text=f"{self.acc[1]:.2f}")

def main():
    rclpy.init()
    root = tk.Tk()
    app = DashboardApp(root)
    
    ros_node = CarStateListener(app.update_data)
    
    # Run ROS in background thread
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()