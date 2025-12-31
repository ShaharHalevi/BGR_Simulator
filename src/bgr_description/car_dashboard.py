#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import math

# =============================================================================
# ROS 2 Node Class
# =============================================================================
class CarStateListener(Node):
    """
    ROS 2 Node that subscribes to car telemetry topics.
    It forwards received data to the GUI via callback functions.
    """
    def __init__(self, data_callback, wheels_callback):
        super().__init__('car_dashboard_listener')
        
        # Callbacks provided by the GUI class to update the display
        self.data_callback = data_callback
        self.wheels_callback = wheels_callback

        # --- QoS Configuration ---
        # We use 'BEST_EFFORT' reliability to match standard simulation/sensor outputs.
        # If the publisher is 'Best Effort' and we listen as 'Reliable', 
        # we will NOT receive any messages. This setting prevents that issue.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriptions ---
        self.data_subscription = self.create_subscription(
            Float64MultiArray, 
            '/robot/full_state', 
            self.listener_callback, 
            qos_profile
        )
        
        self.wheels_subscription = self.create_subscription(
            Float64MultiArray, 
            '/robot/wheels_status', 
            self.wheels_listener_callback, 
            qos_profile
        )

    def listener_callback(self, msg):
        """
        Triggered when /robot/full_state receives data.
        msg.data structure expected: [x, y, z, roll, pitch, yaw, vx, vy, vz, ax, ay, az]
        """
        # Debug print to verify data flow in terminal
        # print(f"DEBUG: Full State received (len: {len(msg.data)})") 
        self.data_callback(msg.data)

    def wheels_listener_callback(self, msg):
        """
        Triggered when /robot/wheels_status receives data.
        msg.data structure expected: [steering_angle, rpm_fl, rpm_fr, rpm_rl, rpm_rr]
        """
        # print("DEBUG: Wheels status received")
        self.wheels_callback(msg.data)


# =============================================================================
# Tkinter GUI Class
# =============================================================================
class DashboardApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("BGR Car Dashboard")
        self.root.geometry("300x480") 
        
        # --- Internal Data Storage ---
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.total_speed = 0.0
        self.steering_angle = 0.0
        self.rpms = [0, 0, 0, 0]

        # --- GUI Layout Setup ---
        self._setup_ui()

        # --- Start the ROS Loop within Tkinter ---
        # This is the key to running without external threads.
        self.process_ros_events()

    def _setup_ui(self):
        """Builds all the visual elements (Labels, Canvas, etc.)"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 1. Speed Section
        ttk.Label(main_frame, text="SPEED (m/s)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W, pady=(0,5))
        self.lbl_speed = ttk.Label(main_frame, text="0.00", font=("Consolas", 20))
        self.lbl_speed.pack(anchor=tk.W, padx=20)
        
        # Added Yaw display
        self.lbl_yaw = ttk.Label(main_frame, text="Yaw: 0.0°", font=("Consolas", 10))
        self.lbl_yaw.pack(anchor=tk.W, padx=20)

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

        # 3. Steering & Wheels Section
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(main_frame, text="STEERING & WHEELS", font=("Helvetica", 12, "bold")).pack(anchor=tk.W)

        # Canvas for Steering Wheel visualization
        self.steer_canvas = tk.Canvas(main_frame, width=200, height=200, bg="black")
        self.steer_canvas.pack(pady=5)
        # Draw static wheel outline
        self.steer_canvas.create_oval(20, 20, 180, 180, outline="gray", width=4)
        # Draw dynamic needle (store reference in self.needle)
        self.needle = self.steer_canvas.create_line(100, 100, 100, 20, fill="red", width=4)

        # RPM Text Stats
        self.lbl_rpms = ttk.Label(main_frame, text="FL: 0 | FR: 0\nRL: 0 | RR: 0", font=("Consolas", 10))
        self.lbl_rpms.pack()

    def process_ros_events(self):
        """
        The 'Heartbeat' function.
        It asks ROS to process incoming messages once, then schedules itself
        to run again in 10ms. This keeps the GUI responsive.
        """
        # Check for new messages (non-blocking due to timeout_sec=0)
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        
        # Schedule next check in 10 milliseconds
        self.root.after(10, self.process_ros_events)

    def update_data(self, data):
        """Handles full_state data updates."""
        try:
            # Safety check: Ensure array has enough elements
            if len(data) < 12:
                print(f"Warning: Data array too short ({len(data)})")
                return

            # Parse Data
            # indices 6,7,8 -> Velocity X, Y, Z
            self.vel = data[6:9]
            # indices 9,10,11 -> Acceleration X, Y, Z
            self.acc = data[9:12]
            
            # Calculate total speed (magnitude of vector)
            self.total_speed = math.sqrt(self.vel[0]**2 + self.vel[1]**2)
            
            # Convert Yaw (index 5) from Radians to Degrees
            yaw_deg = data[5] * (180 / math.pi)

            # Update GUI Labels
            self.lbl_speed.config(text=f"{self.total_speed:.2f}")
            self.lbl_acc_x.config(text=f"{self.acc[0]:.2f}")
            self.lbl_acc_y.config(text=f"{self.acc[1]:.2f}")
            self.lbl_yaw.config(text=f"Yaw: {yaw_deg:.1f}°")

        except Exception as e:
            print(f"Error in update_data: {e}")

    def update_wheels(self, data_list):
        """Handles wheels_status data updates."""
        try:
            # Parse Data
            self.steering_angle = data_list[0]
            self.rpms = data_list[1:5] # [FL, FR, RL, RR]

            # Update RPM Text
            self.lbl_rpms.config(
                text=f"FL: {self.rpms[0]:.0f} | FR: {self.rpms[1]:.0f}\n"
                     f"RL: {self.rpms[2]:.0f} | RR: {self.rpms[3]:.0f}"
            )
            
            # Update Steering Wheel Graphics
            # Convert angle to radians for trig calculation (offset -90 for vertical zero)
            angle_rad = math.radians(self.steering_angle - 90)
            radius = 80 
            center_x, center_y = 100, 100

            # Calculate new tip position
            new_x = center_x + radius * math.cos(angle_rad)
            new_y = center_y + radius * math.sin(angle_rad)

            # Move the needle line
            self.steer_canvas.coords(self.needle, center_x, center_y, new_x, new_y)

        except Exception as e:
            print(f"Error in update_wheels: {e}")

# =============================================================================
# Main Execution
# =============================================================================
def main():
    # 1. Initialize ROS Client Library
    rclpy.init()
    
    # 2. Create the GUI Root
    root = tk.Tk()
    
    # 3. Create the ROS Node 
    # (We pass placeholder lambdas initially, connected later)
    ros_node = CarStateListener(lambda x: None, lambda x: None)
    
    # 4. Create the Dashboard App
    # This initializes the UI and starts the recursive event loop (process_ros_events)
    app = DashboardApp(root, ros_node)
    
    # 5. Connect the Real Callbacks
    # Now that 'app' exists, we can link its methods to the node
    ros_node.data_callback = app.update_data
    ros_node.wheels_callback = app.update_wheels

    # 6. Start the Main Loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup on exit
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()