#!/usr/bin/env python3
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from vision_msgs.msg import Detection3DArray, Detection3D


class InteractivePathTester(Node):
    def __init__(self):
        super().__init__('interactive_path_tester')

        # Publishers (Inputs to Path Planner)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.obstacle_pub = self.create_publisher(
            Detection3DArray, '/perception/obstacles_only', 10
        )

        # Subscribers (Outputs from Path Planner)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        self.latest_path = None
        self.latest_cmd = None
        self.path_received_event = threading.Event()
        self.cmd_received_event = threading.Event()

    def plan_callback(self, msg: Path):
        self.latest_path = msg
        self.path_received_event.set()

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg
        self.cmd_received_event.set()

    def send_initial_pose(self, x: float, y: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        self.init_pose_pub.publish(msg)

    def send_obstacle(self, x: float, y: float, size: float = 0.8):
        msg = Detection3DArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        det = Detection3D()
        det.id = 'rock_obstacle'
        det.bbox.center.position.x = float(x)
        det.bbox.center.position.y = float(y)
        det.bbox.center.position.z = 0.2
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x = float(size)
        det.bbox.size.y = float(size)
        det.bbox.size.z = 0.4
        msg.detections.append(det)

        self.obstacle_pub.publish(msg)

    def send_goal_and_wait(self, gx: float, gy: float, timeout_sec: float = 7.0):
        self.latest_path = None
        self.latest_cmd = None
        self.path_received_event.clear()
        self.cmd_received_event.clear()

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        print(f"\n[🚀] Sending Goal (x={gx}, y={gy}) to Planner...")
        print("[⏳] Waiting for planner output on /plan...")

        path_ok = self.path_received_event.wait(timeout=timeout_sec)
        self.cmd_received_event.wait(timeout=1.5)
        return path_ok, self.latest_path, self.latest_cmd


def calculate_path_length(path_msg: Path) -> float:
    total_len = 0.0
    poses = path_msg.poses
    for i in range(1, len(poses)):
        p1 = poses[i - 1].pose.position
        p2 = poses[i].pose.position
        total_len += math.hypot(p2.x - p1.x, p2.y - p1.y)
    return total_len


def interactive_loop(tester: InteractivePathTester):
    time.sleep(1.0)
    print("\n" + "=" * 55)
    print("   🤖 INTERACTIVE PATH PLANNING TESTER (ROS 2)   ")
    print("=" * 55)

    while rclpy.ok():
        try:
            print("\n" + "-" * 40)
            print("📝 [INPUTS]")
            print("-" * 40)

            start_in = input("Enter Start (X Y) [Press Enter for (0.0, 0.0)]: ").strip()
            if start_in:
                parts = start_in.split()
                sx, sy = float(parts[0]), float(parts[1])
            else:
                sx, sy = 0.0, 0.0
            tester.send_initial_pose(sx, sy)

            goal_in = input("Enter Goal (X Y) [e.g. 5.0 3.0]: ").strip()
            if not goal_in:
                print("⚠️ Goal cannot be empty!")
                continue
            g_parts = goal_in.split()
            gx, gy = float(g_parts[0]), float(g_parts[1])

            obs_in = input("Add Obstacle? (X Y) [Press Enter to skip]: ").strip()
            if obs_in:
                o_parts = obs_in.split()
                tester.send_obstacle(float(o_parts[0]), float(o_parts[1]))
                print(f"  -> Injected obstacle at: ({o_parts[0]}, {o_parts[1]})")
                time.sleep(0.5)

            path_ok, path_msg, cmd_msg = tester.send_goal_and_wait(gx, gy)

            print("\n" + "=" * 40)
            print("📊 [OUTPUTS FROM PLANNER]")
            print("=" * 40)

            if path_ok and path_msg and len(path_msg.poses) > 0:
                waypoints = path_msg.poses
                length = calculate_path_length(path_msg)
                print(f"  ✅ Status: PATH FOUND!")
                print(f"  🔢 Total Waypoints: {len(waypoints)} points")
                print(f"  📏 Total Path Length: {length:.2f} meters")
                print(f"  📍 Start Point: ({waypoints[0].pose.position.x:.2f}, {waypoints[0].pose.position.y:.2f})")
                print(f"  🏁 Goal Point:  ({waypoints[-1].pose.position.x:.2f}, {waypoints[-1].pose.position.y:.2f})")

                step = max(1, len(waypoints) // 4)
                print("\n  📍 Waypoint Samples:")
                for i in range(0, len(waypoints), step):
                    p = waypoints[i].pose.position
                    print(f"     Waypoint #{i:02d}: x = {p.x:5.2f}, y = {p.y:5.2f}")

                if cmd_msg:
                    print(f"\n  🏎️ Control Velocity Output (/cmd_vel):")
                    print(f"     vx: {cmd_msg.linear.x:.2f} m/s | wz: {cmd_msg.angular.z:.2f} rad/s")
            else:
                print("  ❌ No path found within timeout! Ensure goal is inside the map.")

            print("=" * 40)
            again = input("\nTest another Goal? (y/n) [y]: ").strip().lower()
            if again == 'n':
                break

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"⚠️ Error: {e}")

    rclpy.shutdown()
    sys.exit(0)


def main():
    rclpy.init()
    tester = InteractivePathTester()
    spin_thread = threading.Thread(target=rclpy.spin, args=(tester,), daemon=True)
    spin_thread.start()
    interactive_loop(tester)


if __name__ == '__main__':
    main()
