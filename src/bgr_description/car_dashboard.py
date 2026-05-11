#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from bgr_description.msg import ConeArray
import tkinter as tk
from tkinter import ttk
import math
from collections import deque

# --- Matplotlib imports ---
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Polygon

# =============================================================================
# Modern Aesthetic Constants
# =============================================================================
COLOR_BG = "#0f0f0f"        # Deep Charcoal
COLOR_CARD = "#1a1a1a"      # Lighter surface
COLOR_ACCENT = "#00f2ff"    # Neon Cyan
COLOR_WARN = "#ff0055"      # Racing Red
COLOR_TEXT = "#ffffff"      # Pure White
COLOR_DIM = "#888888"       # Muted Gray

# =============================================================================
# ROS 2 Node Class
# =============================================================================
class CarStateListener(Node):
    def __init__(self, data_callback, wheels_callback, cones_callback):
        super().__init__('car_dashboard_listener')
        self.data_callback = data_callback
        self.wheels_callback = wheels_callback
        self.cones_callback = cones_callback

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.data_subscription = self.create_subscription(
            Float64MultiArray, '/robot/full_state', self.data_listener_callback, qos_profile)
        
        self.wheels_subscription = self.create_subscription(
            Float64MultiArray, '/robot/wheels_status', self.wheels_listener_callback, qos_profile)

        self.cone_subscription = self.create_subscription(
            ConeArray, 'visible_cones', self.cone_listener_callback, qos_profile)

    def data_listener_callback(self, msg):
        self.data_callback(msg.data)

    def wheels_listener_callback(self, msg):
        self.wheels_callback(msg.data)

    def cone_listener_callback(self, msg):
        self.cones_callback(msg.cones)


# =============================================================================
# Tkinter GUI Class
# =============================================================================
class DashboardApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("BGR Racing - Live Telemetry")
        # Thinner, taller window, spawning at Top Right (-0+0)
        self.root.geometry("330x1000-0+0")
        self.root.configure(bg=COLOR_BG)

        # --- Internal Data Storage ---
        self.pos = [0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.yaw_rad = 0.0
        self.steering_angle = 0.0
        self.rpms = [0, 0, 0, 0]
        self.cones = []

        self._setup_ui()
        self.process_ros_events()

    def _setup_ui(self):
        # Header
        header = tk.Frame(self.root, bg=COLOR_ACCENT, height=5)
        header.pack(fill=tk.X)
        
        title_lbl = tk.Label(self.root, text="BGR PERFORMANCE", font=("Helvetica", 14, "bold"), 
                             bg=COLOR_BG, fg=COLOR_TEXT, pady=10)
        title_lbl.pack()

        main_frame = tk.Frame(self.root, bg=COLOR_BG, padx=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- SPEED CARD ---
        speed_card = tk.Frame(main_frame, bg=COLOR_CARD, padx=15, pady=15, highlightthickness=1, highlightbackground="#333333")
        speed_card.pack(fill=tk.X, pady=5)
        
        tk.Label(speed_card, text="SPEED", font=("Helvetica", 10, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT).pack(anchor=tk.W)
        speed_info_frame = tk.Frame(speed_card, bg=COLOR_CARD)
        speed_info_frame.pack(fill=tk.X)
        self.lbl_speed = tk.Label(speed_info_frame, text="0.00", font=("Consolas", 36, "bold"), bg=COLOR_CARD, fg=COLOR_TEXT)
        self.lbl_speed.pack(side=tk.LEFT)
        tk.Label(speed_info_frame, text="m/s", font=("Helvetica", 12), bg=COLOR_CARD, fg=COLOR_DIM).pack(side=tk.LEFT, padx=5, pady=(15, 0))

        self.lbl_yaw = tk.Label(speed_card, text="YAW: 0.0°", font=("Consolas", 10), bg=COLOR_CARD, fg=COLOR_DIM)
        self.lbl_yaw.pack(anchor=tk.W)

        # --- ACCELERATION CARD ---
        acc_card = tk.Frame(main_frame, bg=COLOR_CARD, padx=15, pady=15, highlightthickness=1, highlightbackground="#333333")
        acc_card.pack(fill=tk.X, pady=5)
        tk.Label(acc_card, text="ACCELERATION", font=("Helvetica", 10, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT).pack(anchor=tk.W, pady=(0, 10))
        acc_grid = tk.Frame(acc_card, bg=COLOR_CARD)
        acc_grid.pack(fill=tk.X)
        
        # Long Accel
        tk.Label(acc_grid, text="LAT", font=("Helvetica", 9), bg=COLOR_CARD, fg=COLOR_DIM).grid(row=0, column=0, sticky=tk.W)
        self.lbl_acc_x = tk.Label(acc_grid, text="0.00", font=("Consolas", 14), bg=COLOR_CARD, fg=COLOR_TEXT)
        self.lbl_acc_x.grid(row=1, column=0, sticky=tk.W, padx=(0, 20))

        # Lat Accel
        tk.Label(acc_grid, text="LON", font=("Helvetica", 9), bg=COLOR_CARD, fg=COLOR_DIM).grid(row=0, column=1, sticky=tk.W)
        self.lbl_acc_y = tk.Label(acc_grid, text="0.00", font=("Consolas", 14), bg=COLOR_CARD, fg=COLOR_TEXT)
        self.lbl_acc_y.grid(row=1, column=1, sticky=tk.W)

        # --- STEERING & RPM CARD ---
        mid_card = tk.Frame(main_frame, bg=COLOR_CARD, padx=10, pady=10, highlightthickness=1, highlightbackground="#333333")
        mid_card.pack(fill=tk.X, pady=5)
        
        steer_col = tk.Frame(mid_card, bg=COLOR_CARD)
        steer_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.steer_canvas = tk.Canvas(steer_col, width=140, height=100, bg=COLOR_CARD, highlightthickness=0)
        self.steer_canvas.pack()
        self.steer_canvas.create_arc(10, 10, 130, 130, start=0, extent=180, outline="#333333", width=5, style=tk.ARC)
        self.needle = self.steer_canvas.create_line(70, 70, 70, 20, fill=COLOR_WARN, width=3, capstyle=tk.ROUND)
        
        rpm_col = tk.Frame(mid_card, bg=COLOR_CARD)
        rpm_col.pack(side=tk.LEFT, padx=10)
        tk.Label(rpm_col, text="WHEEL RPM", font=("Helvetica", 8, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT).pack(anchor=tk.W)
        self.lbl_rpms = tk.Label(rpm_col, text="FL: 0 | FR: 0\nRL: 0 | RR: 0", font=("Consolas", 9), bg=COLOR_CARD, fg=COLOR_TEXT, justify=tk.LEFT)
        self.lbl_rpms.pack()

        # --- MINIMAP CARD ---
        map_card = tk.Frame(main_frame, bg=COLOR_CARD, padx=10, pady=10, highlightthickness=1, highlightbackground="#333333")
        map_card.pack(fill=tk.BOTH, expand=True, pady=5)
        tk.Label(map_card, text="LIVE TRACK", font=("Helvetica", 10, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT).pack(anchor=tk.W)
        self.minimap = MiniMap(map_card)

        # --- VELOCITY GRAPH ---
        graph_card = tk.Frame(main_frame, bg=COLOR_CARD, padx=10, pady=10, highlightthickness=1, highlightbackground="#333333")
        graph_card.pack(fill=tk.BOTH, expand=True, pady=5)
        tk.Label(graph_card, text="VELOCITY HISTORY", font=("Helvetica", 10, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT).pack(anchor=tk.W)
        self.velocity_graph = VelocityGraph(graph_card)

    def process_ros_events(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.update_display()
        self.root.after(50, self.process_ros_events)

    def update_data(self, data):
        if len(data) >= 12:
            self.pos = data[0:2]
            self.vel = data[6:9]
            self.acc = data[9:12]
            self.yaw_rad = data[5]

    def update_wheels(self, data_list):
        if len(data_list) >= 5:
            self.steering_angle = data_list[0]
            self.rpms = data_list[1:5]

    def update_cones(self, cone_list):
        self.cones = cone_list

    def stable_format(self, val, decimals=2):
        """ Increased deadzone to completely eliminate sensor noise when idling """
        if abs(val) < 0.2:
            return f"{0.0:.{decimals}f}"
            
        formatted = f"{val:.{decimals}f}"
        # Catch any rogue negative zeros that bypass the deadzone
        if formatted.startswith("-") and float(formatted) == 0:
            return formatted[1:]
        return formatted

    def update_display(self):
        try:
            total_speed = math.sqrt(self.vel[0]**2 + self.vel[1]**2)
            yaw_deg = self.yaw_rad * (180 / math.pi)

            # --- Update Text ---
            self.lbl_speed.config(text=self.stable_format(total_speed))
            self.lbl_acc_x.config(text=self.stable_format(self.acc[0]))
            self.lbl_acc_y.config(text=self.stable_format(self.acc[1]))
            self.lbl_yaw.config(text=f"YAW: {self.stable_format(yaw_deg, 1)}°")
            self.lbl_rpms.config(text=f"FL:{self.rpms[0]:3.0f} FR:{self.rpms[1]:3.0f}\nRL:{self.rpms[2]:3.0f} RR:{self.rpms[3]:3.0f}")

            # --- Update Visuals ---
            self.velocity_graph.update(total_speed)
            self.minimap.update(self.pos, self.yaw_rad, self.cones)

            # --- Update Steering Needle ---
            angle_rad = math.radians(-self.steering_angle - 90)
            radius, cx, cy = 50, 70, 70
            nx = cx + radius * math.cos(angle_rad)
            ny = cy + radius * math.sin(angle_rad)
            self.steer_canvas.coords(self.needle, cx, cy, nx, ny)

        except Exception:
            pass 

# =============================================================================
# CLASS: MiniMap
# =============================================================================
class MiniMap:
    def __init__(self, parent_frame):
        # Reduced height from 3.5 to 2.5 to give space to velocity graph
        self.fig = Figure(figsize=(3.0, 2.5), dpi=100, facecolor=COLOR_CARD)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(COLOR_BG)
        self.ax.set_aspect('equal')
        
        # Style
        self.ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        self.ax.grid(True, linestyle='--', color='#222222', alpha=0.5)
        for spine in self.ax.spines.values():
            spine.set_color('#333333')
            
        # Objects
        self.trail_x = deque(maxlen=60)
        self.trail_y = deque(maxlen=60)
        self.trail_line, = self.ax.plot([], [], color=COLOR_ACCENT, linewidth=1.5, alpha=0.4, zorder=2)
        
        self.car_poly = Polygon([[0,0], [0,0], [0,0]], closed=True, facecolor=COLOR_WARN, edgecolor='white', linewidth=1, zorder=5)
        self.ax.add_patch(self.car_poly)
        
        self.cone_scatter = self.ax.scatter([], [], s=20, zorder=3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.configure(bg=COLOR_CARD, highlightthickness=0)
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)
        
        self.update_counter = 0

    def update(self, pos, yaw, cones):
        self.update_counter += 1
        if self.update_counter % 2 != 0: return # Throttle to 10Hz

        # 1. Update Trail
        self.trail_x.append(pos[0])
        self.trail_y.append(pos[1])
        self.trail_line.set_data(self.trail_x, self.trail_y)

        # 2. Update Car Polygon (Triangle)
        L, W = 2.5, 1.2
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        
        def transform(px, py):
            rx = px * cos_y - py * sin_y
            ry = px * sin_y + py * cos_y
            return [pos[0] + rx, pos[1] + ry]
            
        p1 = transform(L/2, 0)
        p2 = transform(-L/2, W/2)
        p3 = transform(-L/2, -W/2)
        self.car_poly.set_xy([p1, p2, p3])

        # 3. Update Cones
        if cones:
            cx = [c.x for c in cones]
            cy = [c.y for c in cones]
            colors = [c.color if c.color in ['blue', 'yellow', 'orange'] else 'white' for c in cones]
            self.cone_scatter.set_offsets(list(zip(cx, cy)))
            self.cone_scatter.set_facecolors(colors)
        else:
            self.cone_scatter.set_offsets(list(zip([], [])))

        # 3. Follow Car
        RANGE = 20
        self.ax.set_xlim(pos[0] - RANGE, pos[0] + RANGE)
        self.ax.set_ylim(pos[1] - RANGE, pos[1] + RANGE)
        
        self.canvas.draw_idle()

# =============================================================================
# CLASS: Velocity Graph
# =============================================================================
class VelocityGraph:
    def __init__(self, parent_frame):
        self.history_len = 100
        self.data = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.update_counter = 0 
        # Increased height from 2.0 to 2.5
        self.fig = Figure(figsize=(3.0, 2.5), dpi=100, facecolor=COLOR_CARD)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(COLOR_CARD)
        self.ax.tick_params(colors=COLOR_DIM, labelsize=7)
        self.ax.grid(True, linestyle='--', color='#222222', alpha=0.5)
        self.ax.set_ylim(0, 5) 
        self.line, = self.ax.plot(self.data, color=COLOR_ACCENT, linewidth=1.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.configure(bg=COLOR_CARD, highlightthickness=0)
        self.canvas_widget.pack(fill=tk.X)

    def update(self, new_value):
        self.data.append(new_value)
        self.update_counter += 1
        if self.update_counter % 5 == 0:
            self.line.set_ydata(self.data)
            max_val = max(self.data)
            if max_val > 0.5:
                self.ax.set_ylim(0, max_val * 1.3)
            else:
                self.ax.set_ylim(0, 5) # Prevent it from sticking to the bottom when stationary
            self.canvas.draw_idle()

# =============================================================================
# Main Execution
# =============================================================================
def main():
    rclpy.init()
    root = tk.Tk() 
    ros_node = CarStateListener(lambda x: None, lambda x: None, lambda x: None)
    app = DashboardApp(root, ros_node)
    ros_node.data_callback = app.update_data
    ros_node.wheels_callback = app.update_wheels
    ros_node.cones_callback = app.update_cones
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()