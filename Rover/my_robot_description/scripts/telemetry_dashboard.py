#!/usr/bin/env python3
"""
telemetry_dashboard.py
======================
Comprehensive Real-Time Visual Telemetry & Diagnostics Dashboard for 4-Wheel Rover.

Visualizations:
  1. IMU Attitude & Dynamics:
     - Artificial Horizon / Pitch & Roll Attitude Indicator Gauge (Canvas).
     - 3-Axis Linear Acceleration level bars (Ax, Ay, Az with gravity offset).
     - 3-Axis Gyroscope Angular Velocity rate bars (Wx, Wy, Wz).
  2. 4-Wheel Chassis & Encoders:
     - Top-down 4-wheel chassis diagram (FL, FR, RL, RR).
     - Per-wheel speeds (m/s and RPM) with bidirectional gauge bars.
     - Cumulative raw encoder tick counters and tick rates.
     - Direction-based dynamic color indicators (Forward, Reverse, Stopped, Slip).
  3. Chassis Kinematics & Health:
     - Aggregated linear speed (Vx) and angular velocity (Wz).
     - Real-time Odometry Pose (X, Y, Yaw).
     - Traction and Slip alert badge.
  4. Integrated Tank-Style Teleoperation:
     - Interactive keyboard and button controls for driving & testing live telemetry.
"""

import sys
import math
import threading
from typing import Dict, Optional, Tuple

import tkinter as tk
from tkinter import ttk

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Imu, JointState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray, Int64MultiArray
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object  # type: ignore


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert orientation quaternion to roll, pitch, yaw in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TelemetryROSNode(Node if ROS2_AVAILABLE else object):
    """ROS 2 Node subscribing to IMU, Encoders, JointStates, and Odometry."""

    def __init__(self, data_callback) -> None:
        if not ROS2_AVAILABLE:
            return
        super().__init__("telemetry_dashboard_node")
        self.data_callback = data_callback

        # Telemetry State
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.accel = [0.0, 0.0, 9.81]
        self.gyro = [0.0, 0.0, 0.0]

        self.wheel_names = ["left_front", "right_front", "left_rear", "right_rear"]
        self.wheel_speeds: Dict[str, float] = {w: 0.0 for w in self.wheel_names}
        self.wheel_ticks: Dict[str, int] = {w: 0 for w in self.wheel_names}
        self.prev_joint_pos: Dict[str, Optional[float]] = {w: None for w in self.wheel_names}
        self.ticks_per_rev = 1024

        self.odom_vx = 0.0
        self.odom_wz = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Subscriptions
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.sub_joint_states = self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.sub_wheel_speeds = self.create_subscription(Float64MultiArray, "/wheel/per_wheel_speeds", self._speeds_cb, 10)
        self.sub_wheel_ticks = self.create_subscription(Int64MultiArray, "/wheel/ticks", self._ticks_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/wheel/odom_raw", self._odom_cb, 10)

        # Publisher for Teleop
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        self.imu_roll, self.imu_pitch, self.imu_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.data_callback("imu")

    def _joint_cb(self, msg: JointState) -> None:
        for idx, name in enumerate(msg.name):
            for wheel in self.wheel_names:
                parts = wheel.split("_")
                inverted = f"{parts[1]}_{parts[0]}" if len(parts) == 2 else wheel
                if wheel in name or inverted in name:
                    if idx < len(msg.position):
                        rad = msg.position[idx]
                        self.wheel_ticks[wheel] = int((rad / (2.0 * math.pi)) * self.ticks_per_rev)
                    if idx < len(msg.velocity):
                        # Convert rad/s to linear speed m/s (radius = 0.06m)
                        self.wheel_speeds[wheel] = msg.velocity[idx] * 0.06
        self.data_callback("wheels")

    def _speeds_cb(self, msg: Float64MultiArray) -> None:
        for idx, speed in enumerate(msg.data):
            if idx < len(self.wheel_names):
                self.wheel_speeds[self.wheel_names[idx]] = speed
        self.data_callback("wheels")

    def _ticks_cb(self, msg: Int64MultiArray) -> None:
        for idx, ticks in enumerate(msg.data):
            if idx < len(self.wheel_names):
                self.wheel_ticks[self.wheel_names[idx]] = ticks
        self.data_callback("wheels")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.odom_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.data_callback("odom")

    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


class TelemetryDashboardApp:
    """Tkinter-based live telemetry and diagnostics visualizer."""

    def __init__(self, root: tk.Tk, ros_node: Optional[TelemetryROSNode] = None) -> None:
        self.root = root
        self.node = ros_node
        self.root.title("Rover 4-Wheel Telemetry & Diagnostics Center")
        self.root.geometry("1180x760")
        self.root.configure(bg="#0b0f19")

        # Commanded Teleop state
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.speed_scale_lin = 0.5
        self.speed_scale_ang = 1.0

        # Apply Styles
        self._setup_styles()

        # Build UI Panels
        self._build_header()
        self._build_main_panels()

        # Keyboard bindings
        self.root.bind("<Up>", lambda e: self.teleop_move(self.speed_scale_lin, 0.0))
        self.root.bind("<Down>", lambda e: self.teleop_move(-self.speed_scale_lin, 0.0))
        self.root.bind("<Left>", lambda e: self.teleop_move(0.0, self.speed_scale_ang))
        self.root.bind("<Right>", lambda e: self.teleop_move(0.0, -self.speed_scale_ang))
        self.root.bind("<space>", lambda e: self.teleop_stop())
        self.root.bind("<Escape>", lambda e: self.teleop_stop())

        # Continuous GUI update loop (30 Hz)
        self.root.after(33, self._gui_update_loop)

        # Continuous teleop publisher loop (10 Hz)
        if self.node:
            self.root.after(100, self._teleop_publish_loop)

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TProgressbar", thickness=14, troughcolor="#1e293b", background="#38bdf8")

    def _build_header(self) -> None:
        hdr = tk.Frame(self.root, bg="#111827", height=50, bd=1, relief="solid")
        hdr.pack(fill="x", side="top", padx=8, pady=(8, 4))

        title = tk.Label(
            hdr,
            text="🛸 AUTONOMOUS ROVER 4WD TELEMETRY & DIAGNOSTICS",
            font=("Helvetica", 14, "bold"),
            fg="#38bdf8",
            bg="#111827",
        )
        title.pack(side="left", padx=16, pady=8)

        self.status_badge = tk.Label(
            hdr,
            text="🟢 SYSTEM ACTIVE (4WD TANK STEER)",
            font=("Helvetica", 10, "bold"),
            fg="#10b981",
            bg="#1f2937",
            padx=10,
            pady=4,
        )
        self.status_badge.pack(side="right", padx=16, pady=8)

    def _build_main_panels(self) -> None:
        container = tk.Frame(self.root, bg="#0b0f19")
        container.pack(fill="both", expand=True, padx=8, pady=4)

        # 3 Column Layout: Left (IMU), Center (4-Wheel Chassis), Right (Odometry & Teleop)
        container.columnconfigure(0, weight=4)
        container.columnconfigure(1, weight=5)
        container.columnconfigure(2, weight=4)
        container.rowconfigure(0, weight=1)

        self._build_imu_panel(container)
        self._build_chassis_panel(container)
        self._build_dynamics_panel(container)

    # --------------------------------------------------------------------------
    # 1. IMU PANEL (Left Column)
    # --------------------------------------------------------------------------
    def _build_imu_panel(self, parent: tk.Frame) -> None:
        card = tk.LabelFrame(
            parent,
            text=" 🧭 IMU Telemetry & Attitude ",
            font=("Helvetica", 11, "bold"),
            fg="#f1f5f9",
            bg="#111827",
            padx=10,
            pady=10,
        )
        card.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        # Artificial Horizon Canvas
        self.horizon_size = 180
        self.horizon_canvas = tk.Canvas(
            card,
            width=self.horizon_size,
            height=self.horizon_size,
            bg="#0284c7",
            highlightthickness=2,
            highlightbackground="#334155",
        )
        self.horizon_canvas.pack(pady=(4, 8))

        # Attitude Numeric Display
        self.lbl_angles = tk.Label(
            card,
            text="Roll: 0.0°  |  Pitch: 0.0°  |  Yaw: 0.0°",
            font=("Courier", 10, "bold"),
            fg="#38bdf8",
            bg="#111827",
        )
        self.lbl_angles.pack(pady=(0, 10))

        # Linear Acceleration Bars
        acc_frame = tk.LabelFrame(card, text="Linear Acceleration (m/s²)", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#1e293b", padx=8, pady=6)
        acc_frame.pack(fill="x", pady=4)

        self.lbl_ax = tk.Label(acc_frame, text="Ax:  0.00 m/s²", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_ax.pack(anchor="w")
        self.bar_ax = ttk.Progressbar(acc_frame, orient="horizontal", length=200, mode="determinate", maximum=20.0)
        self.bar_ax.pack(fill="x", pady=(0, 4))

        self.lbl_ay = tk.Label(acc_frame, text="Ay:  0.00 m/s²", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_ay.pack(anchor="w")
        self.bar_ay = ttk.Progressbar(acc_frame, orient="horizontal", length=200, mode="determinate", maximum=20.0)
        self.bar_ay.pack(fill="x", pady=(0, 4))

        self.lbl_az = tk.Label(acc_frame, text="Az:  9.81 m/s²", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_az.pack(anchor="w")
        self.bar_az = ttk.Progressbar(acc_frame, orient="horizontal", length=200, mode="determinate", maximum=20.0)
        self.bar_az.pack(fill="x", pady=(0, 4))

        # Angular Velocity Display
        gyro_frame = tk.LabelFrame(card, text="Angular Rate / Gyro (rad/s)", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#1e293b", padx=8, pady=6)
        gyro_frame.pack(fill="x", pady=4)

        self.lbl_gx = tk.Label(gyro_frame, text="Wx (Roll Rate):   0.000 rad/s", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_gx.pack(anchor="w")
        self.lbl_gy = tk.Label(gyro_frame, text="Wy (Pitch Rate):  0.000 rad/s", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_gy.pack(anchor="w")
        self.lbl_gz = tk.Label(gyro_frame, text="Wz (Yaw Rate):    0.000 rad/s", font=("Courier", 9, "bold"), fg="#a855f7", bg="#1e293b")
        self.lbl_gz.pack(anchor="w")

    # --------------------------------------------------------------------------
    # 2. CHASSIS & 4-WHEEL ENCODERS PANEL (Center Column)
    # --------------------------------------------------------------------------
    def _build_chassis_panel(self, parent: tk.Frame) -> None:
        card = tk.LabelFrame(
            parent,
            text=" 🛞 4-Wheel Chassis & Encoders Telemetry ",
            font=("Helvetica", 11, "bold"),
            fg="#f1f5f9",
            bg="#111827",
            padx=8,
            pady=8,
        )
        card.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

        # 4 Quadrants for Front-Left, Front-Right, Rear-Left, Rear-Right
        quad_frame = tk.Frame(card, bg="#111827")
        quad_frame.pack(fill="both", expand=True)
        quad_frame.columnconfigure(0, weight=1)
        quad_frame.columnconfigure(1, weight=1)
        quad_frame.rowconfigure(0, weight=1)
        quad_frame.rowconfigure(1, weight=1)

        self.wheel_widgets: Dict[str, Dict[str, tk.Widget]] = {}

        wheels_meta = [
            ("left_front", "Front Left (FL)", 0, 0, "#059669"),
            ("right_front", "Front Right (FR)", 0, 1, "#0284c7"),
            ("left_rear", "Rear Left (RL)", 1, 0, "#059669"),
            ("right_rear", "Rear Right (RR)", 1, 1, "#0284c7"),
        ]

        for code, label_txt, r, c, theme_col in wheels_meta:
            w_box = tk.LabelFrame(
                quad_frame,
                text=f" {label_txt} ",
                font=("Helvetica", 10, "bold"),
                fg=theme_col,
                bg="#1e293b",
                padx=8,
                pady=6,
            )
            w_box.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")

            status_icon = tk.Label(w_box, text="⏹ STOPPED", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#1e293b")
            status_icon.pack(anchor="w")

            speed_lbl = tk.Label(w_box, text="Speed: 0.00 m/s (0 RPM)", font=("Courier", 10, "bold"), fg="#f8fafc", bg="#1e293b")
            speed_lbl.pack(anchor="w", pady=2)

            speed_bar = ttk.Progressbar(w_box, orient="horizontal", length=140, mode="determinate", maximum=2.0)
            speed_bar.pack(fill="x", pady=2)

            ticks_lbl = tk.Label(w_box, text="Ticks: 0", font=("Courier", 10), fg="#38bdf8", bg="#1e293b")
            ticks_lbl.pack(anchor="w", pady=2)

            self.wheel_widgets[code] = {
                "box": w_box,
                "status": status_icon,
                "speed": speed_lbl,
                "bar": speed_bar,
                "ticks": ticks_lbl,
            }

        # Central Visual Chassis Link
        center_info = tk.Frame(card, bg="#0f172a", bd=1, relief="solid", pady=8, padx=10)
        center_info.pack(fill="x", pady=(8, 0))

        tk.Label(
            center_info,
            text="Drivetrain: 4-Wheel Skid-Steer Tank Drive  |  Baseline Width: 0.49m",
            font=("Helvetica", 9, "italic"),
            fg="#94a3b8",
            bg="#0f172a",
        ).pack()

    # --------------------------------------------------------------------------
    # 3. CHASSIS DYNAMICS & TELEOP (Right Column)
    # --------------------------------------------------------------------------
    def _build_dynamics_panel(self, parent: tk.Frame) -> None:
        card = tk.LabelFrame(
            parent,
            text=" 🎮 Dynamics & Tank Drive Controls ",
            font=("Helvetica", 11, "bold"),
            fg="#f1f5f9",
            bg="#111827",
            padx=10,
            pady=10,
        )
        card.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)

        # Odometry / Motion Readout
        dyn_frame = tk.LabelFrame(card, text="Fused Odometry (/wheel/odom_raw)", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#1e293b", padx=8, pady=6)
        dyn_frame.pack(fill="x", pady=4)

        self.lbl_vx = tk.Label(dyn_frame, text="Linear Velocity (Vx):  0.00 m/s", font=("Courier", 10, "bold"), fg="#10b981", bg="#1e293b")
        self.lbl_vx.pack(anchor="w")

        self.lbl_wz = tk.Label(dyn_frame, text="Angular Rate (Wz):    0.00 rad/s", font=("Courier", 10, "bold"), fg="#a855f7", bg="#1e293b")
        self.lbl_wz.pack(anchor="w")

        self.lbl_pose = tk.Label(dyn_frame, text="Pose: X=0.00m  Y=0.00m  Yaw=0.0°", font=("Courier", 9), fg="#f8fafc", bg="#1e293b")
        self.lbl_pose.pack(anchor="w", pady=(4, 0))

        # Slip Checker Health Status
        self.lbl_slip_badge = tk.Label(
            card,
            text="TRACTION: NOMINAL (NO SLIP)",
            font=("Helvetica", 10, "bold"),
            fg="#10b981",
            bg="#1f2937",
            padx=8,
            pady=6,
        )
        self.lbl_slip_badge.pack(fill="x", pady=6)

        # Teleop Control Buttons
        btn_frame = tk.LabelFrame(card, text="Tank-Style Drive Controls", font=("Helvetica", 9, "bold"), fg="#94a3b8", bg="#1e293b", padx=8, pady=8)
        btn_frame.pack(fill="both", expand=True, pady=4)

        btn_opts = {"font": ("Helvetica", 9, "bold"), "width": 8, "height": 2, "bd": 0, "relief": "flat", "cursor": "hand2"}

        btn_fwd = tk.Button(btn_frame, text="▲ FWD\n(Up)", bg="#059669", fg="#ffffff", activebackground="#047857", command=lambda: self.teleop_move(self.speed_scale_lin, 0.0), **btn_opts)
        btn_fwd.grid(row=0, column=1, padx=4, pady=4)

        btn_rot_l = tk.Button(btn_frame, text="⟲ TANK L\n(Left)", bg="#4338ca", fg="#ffffff", activebackground="#3730a3", command=lambda: self.teleop_move(0.0, self.speed_scale_ang), **btn_opts)
        btn_rot_l.grid(row=1, column=0, padx=4, pady=4)

        btn_stop = tk.Button(btn_frame, text="⏹ STOP\n(Space)", bg="#dc2626", fg="#ffffff", activebackground="#b91c1c", command=self.teleop_stop, **btn_opts)
        btn_stop.grid(row=1, column=1, padx=4, pady=4)

        btn_rot_r = tk.Button(btn_frame, text="TANK R ⟳\n(Right)", bg="#4338ca", fg="#ffffff", activebackground="#3730a3", command=lambda: self.teleop_move(0.0, -self.speed_scale_ang), **btn_opts)
        btn_rot_r.grid(row=1, column=2, padx=4, pady=4)

        btn_rev = tk.Button(btn_frame, text="▼ REV\n(Down)", bg="#d97706", fg="#ffffff", activebackground="#b45309", command=lambda: self.teleop_move(-self.speed_scale_lin, 0.0), **btn_opts)
        btn_rev.grid(row=2, column=1, padx=4, pady=4)

        # Speed Sliders
        sliders_f = tk.Frame(card, bg="#111827")
        sliders_f.pack(fill="x", pady=4)

        tk.Label(sliders_f, text="Max Linear Speed (m/s):", font=("Helvetica", 8, "bold"), fg="#cbd5e1", bg="#111827").pack(anchor="w")
        self.scale_lin = tk.Scale(sliders_f, from_=0.1, to=2.0, resolution=0.1, orient="horizontal", bg="#1e293b", fg="#f8fafc", highlightthickness=0, command=self._update_speed_scales)
        self.scale_lin.set(self.speed_scale_lin)
        self.scale_lin.pack(fill="x", pady=(0, 4))

        tk.Label(sliders_f, text="Max Turn Rate (rad/s):", font=("Helvetica", 8, "bold"), fg="#cbd5e1", bg="#111827").pack(anchor="w")
        self.scale_ang = tk.Scale(sliders_f, from_=0.2, to=3.0, resolution=0.1, orient="horizontal", bg="#1e293b", fg="#f8fafc", highlightthickness=0, command=self._update_speed_scales)
        self.scale_ang.set(self.speed_scale_ang)
        self.scale_ang.pack(fill="x")

    # --------------------------------------------------------------------------
    # Teleop Helpers
    # --------------------------------------------------------------------------
    def _update_speed_scales(self, event=None) -> None:
        self.speed_scale_lin = float(self.scale_lin.get())
        self.speed_scale_ang = float(self.scale_ang.get())

    def teleop_move(self, vx: float, wz: float) -> None:
        self.target_vx = vx
        self.target_wz = wz

    def teleop_stop(self) -> None:
        self.target_vx = 0.0
        self.target_wz = 0.0

    def _teleop_publish_loop(self) -> None:
        if self.node:
            self.node.send_cmd_vel(self.target_vx, self.target_wz)
        self.root.after(100, self._teleop_publish_loop)

    # --------------------------------------------------------------------------
    # Live Graphic Drawing & GUI Updates
    # --------------------------------------------------------------------------
    def _draw_artificial_horizon(self, roll_deg: float, pitch_deg: float) -> None:
        c = self.horizon_canvas
        c.delete("all")
        w = self.horizon_size
        h = self.horizon_size
        cx = w / 2.0
        cy = h / 2.0
        r = (w / 2.0) - 4

        # Calculate pitch shift in pixels (e.g. 1 deg pitch = 2 px shift)
        pitch_shift = pitch_deg * 1.8
        roll_rad = math.radians(roll_deg)

        # Draw Ground (Brown polygon) and Sky (Blue polygon)
        # Using canvas clipping circle mask
        c.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#0284c7", outline="#475569", width=3)

        # Compute horizon line end points
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)

        # Horizon center offset by pitch
        hx = cx + pitch_shift * sin_r
        hy = cy + pitch_shift * cos_r

        # Ground points
        x1 = hx - 200 * cos_r
        y1 = hy + 200 * sin_r
        x2 = hx + 200 * cos_r
        y2 = hy - 200 * sin_r

        # Ground quadrant polygon
        gx3 = x2 - 300 * sin_r
        gy3 = y2 - 300 * cos_r
        gx4 = x1 - 300 * sin_r
        gy4 = y1 - 300 * cos_r

        c.create_polygon([x1, y1, x2, y2, gx3, gy3, gx4, gy4], fill="#b45309", outline="")

        # Horizon Line
        c.create_line(x1, y1, x2, y2, fill="#f8fafc", width=2)

        # Center Aircraft / Rover Reticle
        c.create_line(cx - 24, cy, cx - 8, cy, fill="#facc15", width=3)
        c.create_line(cx + 8, cy, cx + 24, cy, fill="#facc15", width=3)
        c.create_line(cx, cy - 8, cx, cy + 8, fill="#facc15", width=3)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#facc15", outline="")

        # Outer bezel overlay
        c.create_oval(cx - r, cy - r, cx + r, cy + r, fill="", outline="#38bdf8", width=3)

    def _gui_update_loop(self) -> None:
        if self.node:
            # 1. Update IMU Attitude & Horizon
            roll = self.node.imu_roll
            pitch = self.node.imu_pitch
            yaw = self.node.imu_yaw
            self._draw_artificial_horizon(roll, pitch)
            self.lbl_angles.config(text=f"Roll: {roll:+5.1f}°  |  Pitch: {pitch:+5.1f}°  |  Yaw: {yaw:+5.1f}°")

            # 2. Update Linear Accelerations
            ax, ay, az = self.node.accel
            self.lbl_ax.config(text=f"Ax: {ax:+6.2f} m/s²")
            self.bar_ax["value"] = min(20.0, abs(ax))

            self.lbl_ay.config(text=f"Ay: {ay:+6.2f} m/s²")
            self.bar_ay["value"] = min(20.0, abs(ay))

            self.lbl_az.config(text=f"Az: {az:+6.2f} m/s²")
            self.bar_az["value"] = min(20.0, abs(az))

            # 3. Update Gyro Rates
            gx, gy, gz = self.node.gyro
            self.lbl_gx.config(text=f"Wx (Roll Rate):   {gx:+6.3f} rad/s")
            self.lbl_gy.config(text=f"Wy (Pitch Rate):  {gy:+6.3f} rad/s")
            self.lbl_gz.config(text=f"Wz (Yaw Rate):    {gz:+6.3f} rad/s ({math.degrees(gz):+5.1f}°/s)")

            # 4. Update 4-Wheel Encoders & Speeds
            wheel_radius = 0.06
            for wheel_code, widgets in self.wheel_widgets.items():
                spd = self.node.wheel_speeds.get(wheel_code, 0.0)
                ticks = self.node.wheel_ticks.get(wheel_code, 0)
                rpm = (abs(spd) / (2.0 * math.pi * wheel_radius)) * 60.0

                widgets["speed"].config(text=f"Speed: {spd:+5.2f} m/s ({rpm:3.0f} RPM)")
                widgets["bar"]["value"] = min(2.0, abs(spd))
                widgets["ticks"].config(text=f"Ticks: {ticks:10d}")

                # Direction status & color
                if abs(spd) < 0.01:
                    widgets["status"].config(text="⏹ STOPPED", fg="#94a3b8")
                elif spd > 0.01:
                    widgets["status"].config(text="▲ FORWARD", fg="#10b981")
                else:
                    widgets["status"].config(text="▼ REVERSE", fg="#f59e0b")

            # 5. Update Odometry & Slip Indicator
            vx = self.node.odom_vx
            wz = self.node.odom_wz
            self.lbl_vx.config(text=f"Linear Velocity (Vx):  {vx:+6.2f} m/s")
            self.lbl_wz.config(text=f"Angular Rate (Wz):    {wz:+6.2f} rad/s ({math.degrees(wz):+5.1f}°/s)")
            self.lbl_pose.config(text=f"Pose: X={self.node.odom_x:+.2f}m  Y={self.node.odom_y:+.2f}m  Yaw={self.node.odom_yaw:+.1f}°")

            # Slip Detection Heuristic
            # If wheels are commanded/moving but linear acceleration is flat, or rotational disparity
            wheel_avg = sum(abs(s) for s in self.node.wheel_speeds.values()) / 4.0
            if wheel_avg > 0.25 and abs(vx) < 0.05:
                self.lbl_slip_badge.config(text="⚠️ WARNING: WHEEL SLIPPAGE DETECTED", fg="#ef4444", bg="#450a0a")
            else:
                self.lbl_slip_badge.config(text="✅ TRACTION: NOMINAL (NO SLIP)", fg="#10b981", bg="#1f2937")

        self.root.after(33, self._gui_update_loop)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)

    root = tk.Tk()

    ros_node = None
    if ROS2_AVAILABLE:
        ros_node = TelemetryROSNode(data_callback=lambda x: None)
        ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
        ros_thread.start()

    app = TelemetryDashboardApp(root, ros_node)

    try:
        root.mainloop()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if ROS2_AVAILABLE and ros_node:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
