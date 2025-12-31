#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import math
from collections import deque

# --- Matplotlib imports ---
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =============================================================================
# ROS 2 Node Class
# =============================================================================
class CarStateListener(Node):
    def __init__(self, data_callback, wheels_callback):
        super().__init__('car_dashboard_listener')
        self.data_callback = data_callback
        self.wheels_callback = wheels_callback

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.data_subscription = self.create_subscription(
            Float64MultiArray, '/robot/full_state', self.data_listener_callback, qos_profile)
        
        self.wheels_subscription = self.create_subscription(
            Float64MultiArray, '/robot/wheels_status', self.wheels_listener_callback, qos_profile)

    def data_listener_callback(self, msg):
        self.data_callback(msg.data)

    def wheels_listener_callback(self, msg):
        self.wheels_callback(msg.data)


# =============================================================================
# Tkinter GUI Class
# =============================================================================
class DashboardApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("Car Dashboard")
        self.root.geometry("350x750-0+0") 
        
        # --- Internal Data Storage ---
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.yaw_rad = 0.0
        self.steering_angle = 0.0
        self.rpms = [0, 0, 0, 0]

        self._setup_ui()
        self.process_ros_events()

    def _setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Speed Section
        ttk.Label(main_frame, text="SPEED (m/s)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W, pady=(0,5))
        self.lbl_speed = ttk.Label(main_frame, text="0.00", font=("Consolas", 20))
        self.lbl_speed.pack(anchor=tk.W, padx=20)
        
        self.lbl_yaw = ttk.Label(main_frame, text="Yaw: 0.0°", font=("Consolas", 10))
        self.lbl_yaw.pack(anchor=tk.W, padx=20)

        # Acceleration Section
        ttk.Label(main_frame, text="ACCELERATION (m/s²)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W, pady=(0,5))
        acc_frame = ttk.Frame(main_frame)
        acc_frame.pack(fill=tk.X, padx=10)
        
        ttk.Label(acc_frame, text="X:").grid(row=0, column=0, sticky=tk.W)
        self.lbl_acc_x = ttk.Label(acc_frame, text="0.00", font=("Consolas", 10))
        self.lbl_acc_x.grid(row=0, column=1, sticky=tk.W, padx=(5, 15))

        ttk.Label(acc_frame, text="Y:").grid(row=0, column=2, sticky=tk.W)
        self.lbl_acc_y = ttk.Label(acc_frame, text="0.00", font=("Consolas", 10))
        self.lbl_acc_y.grid(row=0, column=3, sticky=tk.W, padx=5)

        # Steering & Wheels Section
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(main_frame, text="STEERING & WHEELS", font=("Helvetica", 12, "bold")).pack(anchor=tk.W)

        self.steer_canvas = tk.Canvas(main_frame, width=200, height=200, bg="black")
        self.steer_canvas.pack(pady=5)
        self.steer_canvas.create_oval(20, 20, 180, 180, outline="gray", width=4)
        self.needle = self.steer_canvas.create_line(100, 100, 100, 20, fill="red", width=4)

        self.lbl_rpms = ttk.Label(main_frame, text="FL: 0 | FR: 0\nRL: 0 | RR: 0", font=("Consolas", 10))
        self.lbl_rpms.pack()

        # --- GRAPH: Initialized here ---
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(main_frame, text="Velocity Graph", font=("Helvetica", 12, "bold")).pack(anchor=tk.W)
        self.velocity_graph = VelocityGraph(main_frame)

    def process_ros_events(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.update_display()
        self.root.after(50, self.process_ros_events)

    def update_data(self, data):
        if len(data) >= 12:
            self.vel = data[6:9]
            self.acc = data[9:12]
            self.yaw_rad = data[5]

    def update_wheels(self, data_list):
        if len(data_list) >= 5:
            self.steering_angle = data_list[0]
            self.rpms = data_list[1:5]

    def update_display(self):
        try:
            # --- Math ---
            total_speed = math.sqrt(self.vel[0]**2 + self.vel[1]**2)
            yaw_deg = self.yaw_rad * (180 / math.pi)

            # --- Update Text ---
            self.lbl_speed.config(text=f"{total_speed:.2f}")
            self.lbl_acc_x.config(text=f"{self.acc[0]:.2f}")
            self.lbl_acc_y.config(text=f"{self.acc[1]:.2f}")
            self.lbl_yaw.config(text=f"Yaw: {yaw_deg:.1f}°")
            self.lbl_rpms.config(text=f"FL: {self.rpms[0]:.0f} | FR: {self.rpms[1]:.0f}\nRL: {self.rpms[2]:.0f} | RR: {self.rpms[3]:.0f}")

            # --- Update Graph (Delegated to class) ---
            self.velocity_graph.update(total_speed)

            # --- Update Steering Canvas ---
            angle_rad = math.radians(self.steering_angle - 90)
            radius, cx, cy = 80, 100, 100
            nx = cx + radius * math.cos(angle_rad)
            ny = cy + radius * math.sin(angle_rad)
            self.steer_canvas.coords(self.needle, cx, cy, nx, ny)

        except Exception:
            pass 

# =============================================================================
# CLASS: Velocity Graph
# =============================================================================
class VelocityGraph:
    def __init__(self, parent_frame):
        """Initializes the graph visual elements and data storage."""
        # --- Data Config ---
        self.history_len = 100
        self.data = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.update_counter = 0  # Used to throttle redraws
        
        # --- Visual Config ---
        self.fig = Figure(figsize=(3, 2), dpi=100) # Small size
        self.ax = self.fig.add_subplot(111)
        
        # Style
        # self.ax.set_title("Velocity History", fontsize=8)
        self.ax.set_xlabel("Time[s]", fontsize=8)
        self.ax.set_ylabel("Speed [m/s]", fontsize=8)
        self.fig.tight_layout()
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_ylim(0, 10) # Default Y limit
        
        # Create the line object once
        self.line, = self.ax.plot(self.data, color='#00cec9', linewidth=2)
        
        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.X, pady=10)

    def update(self, new_value):
        """Adds new data and periodically redraws the plot."""
        # 1. Store Data (Fast)
        self.data.append(new_value)
        self.update_counter += 1

        # 2. Redraw (Slow) - Only every 4th update (approx 200ms)
        if self.update_counter % 4 == 0:
            self.line.set_ydata(self.data)
            
            # Dynamic Y-axis scaling
            max_val = max(self.data)
            if max_val > 1.0:
                self.ax.set_ylim(0, max_val * 1.2)
            
            self.canvas.draw()

# =============================================================================
# Main Execution
# =============================================================================
def main():
    rclpy.init()
    root = tk.Tk() 
    
    ros_node = CarStateListener(lambda x: None, lambda x: None)
    app = DashboardApp(root, ros_node)
    
    ros_node.data_callback = app.update_data
    ros_node.wheels_callback = app.update_wheels

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()