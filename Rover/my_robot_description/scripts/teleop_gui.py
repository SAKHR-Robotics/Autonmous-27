#!/usr/bin/env python3
import sys
import threading
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist

class TeleopGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current commanded speeds
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        # Active velocities (streamed continuously via 10Hz timer)
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        self.root.title("Rover Teleop Control Center")
        self.root.geometry("420x460")
        self.root.configure(bg="#1e272e")
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Title Label
        title_label = tk.Label(root, text="🛸 Rover Control Center", font=("Helvetica", 16, "bold"), fg="#d2dae2", bg="#1e272e")
        title_label.pack(pady=12)
        
        # Controls Frame
        ctrl_frame = tk.Frame(root, bg="#1e272e")
        ctrl_frame.pack(pady=10)
        
        # Buttons layout
        btn_opts = {"font": ("Helvetica", 10, "bold"), "width": 10, "height": 2, "bd": 0, "relief": "flat", "cursor": "hand2"}
        
        self.btn_up = tk.Button(ctrl_frame, text="▲ Forward\n(Up Arrow)", bg="#05c46b", fg="#ffffff", activebackground="#04a75b", command=self.move_forward, **btn_opts)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        
        self.btn_left = tk.Button(ctrl_frame, text="◀ Left\n(Left Arrow)", bg="#3c40c6", fg="#ffffff", activebackground="#2f32a7", command=self.turn_left, **btn_opts)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        
        self.btn_stop = tk.Button(ctrl_frame, text="⏹ STOP\n(Space)", bg="#ff3f34", fg="#ffffff", activebackground="#e62e24", command=self.stop, **btn_opts)
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        
        self.btn_right = tk.Button(ctrl_frame, text="Right ▶\n(Right Arrow)", bg="#3c40c6", fg="#ffffff", activebackground="#2f32a7", command=self.turn_right, **btn_opts)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        
        self.btn_down = tk.Button(ctrl_frame, text="▼ Reverse\n(Down Arrow)", bg="#ffc048", fg="#1e272e", activebackground="#ffa801", command=self.move_backward, **btn_opts)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        
        # Keyboard bindings
        self.root.bind("<Up>", lambda event: self.move_forward())
        self.root.bind("<Down>", lambda event: self.move_backward())
        self.root.bind("<Left>", lambda event: self.turn_left())
        self.root.bind("<Right>", lambda event: self.turn_right())
        self.root.bind("<space>", lambda event: self.stop())
        self.root.bind("<Escape>", lambda event: self.stop())
        
        # Speed Sliders Frame
        speed_frame = tk.Frame(root, bg="#2f3542")
        speed_frame.pack(pady=12, fill="x", padx=25, ipady=8)
        
        lbl_lin = tk.Label(speed_frame, text="Linear Speed (m/s):", fg="#f1f2f6", bg="#2f3542", font=("Helvetica", 9, "bold"))
        lbl_lin.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.scale_lin = tk.Scale(speed_frame, from_=0.1, to=2.0, resolution=0.1, orient="horizontal", bg="#2f3542", fg="#f1f2f6", highlightthickness=0, command=self.update_speeds)
        self.scale_lin.set(self.linear_speed)
        self.scale_lin.grid(row=0, column=1, sticky="we", padx=10)
        
        lbl_ang = tk.Label(speed_frame, text="Angular Speed (rad/s):", fg="#f1f2f6", bg="#2f3542", font=("Helvetica", 9, "bold"))
        lbl_ang.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        self.scale_ang = tk.Scale(speed_frame, from_=0.2, to=3.0, resolution=0.1, orient="horizontal", bg="#2f3542", fg="#f1f2f6", highlightthickness=0, command=self.update_speeds)
        self.scale_ang.set(self.angular_speed)
        self.scale_ang.grid(row=1, column=1, sticky="we", padx=10)
        
        speed_frame.columnconfigure(1, weight=1)
        
        # Status Label
        self.lbl_status = tk.Label(root, text="Status: IDLE (Press Arrow keys to drive)", font=("Helvetica", 10, "italic"), fg="#808e9b", bg="#1e272e")
        self.lbl_status.pack(pady=6)
        
        # 10Hz Timer to continuously publish velocity commands to Gazebo
        self.timer = self.node.create_timer(0.1, self.timer_publish_callback)
        
    def update_speeds(self, event=None):
        self.linear_speed = float(self.scale_lin.get())
        self.angular_speed = float(self.scale_ang.get())
        
    def timer_publish_callback(self):
        twist = Twist()
        twist.linear.x = float(self.target_linear)
        twist.angular.z = float(self.target_angular)
        self.publisher.publish(twist)
        
    def move_forward(self):
        self.target_linear = self.linear_speed
        self.target_angular = 0.0
        self.lbl_status.config(text=f"Status: MOVING FORWARD ({self.linear_speed:.1f} m/s)", fg="#05c46b")
        
    def move_backward(self):
        self.target_linear = -self.linear_speed
        self.target_angular = 0.0
        self.lbl_status.config(text=f"Status: REVERSING ({-self.linear_speed:.1f} m/s)", fg="#ffc048")
        
    def turn_left(self):
        self.target_linear = 0.0
        self.target_angular = self.angular_speed
        self.lbl_status.config(text=f"Status: TURNING LEFT ({self.angular_speed:.1f} rad/s)", fg="#70a1ff")
        
    def turn_right(self):
        self.target_linear = 0.0
        self.target_angular = -self.angular_speed
        self.lbl_status.config(text=f"Status: TURNING RIGHT ({-self.angular_speed:.1f} rad/s)", fg="#70a1ff")
        
    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.lbl_status.config(text="Status: STOPPED", fg="#ff4757")

def ros2_spin(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass

def main():
    try:
        rclpy.init(args=sys.argv)
        node = Node('teleop_gui_node')
        
        # Threading to spin ROS 2 in the background
        spin_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
        spin_thread.start()
        
        root = tk.Tk()
        app = TeleopGUI(root, node)
        
        try:
            root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                # Publish final 0-velocity stop command before shutdown
                stop_pub = node.create_publisher(Twist, '/cmd_vel', 10)
                stop_pub.publish(Twist())
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
