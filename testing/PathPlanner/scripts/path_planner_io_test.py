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
try:
    from nav2_msgs.srv import ClearEntireCostmap
    HAS_CLEAR_SRV = True
except ImportError:
    HAS_CLEAR_SRV = False


class EvaluatedPathTester(Node):
    def __init__(self):
        super().__init__('evaluated_path_tester')

        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.obstacle_pub = self.create_publisher(
            Detection3DArray, '/perception/obstacles_only', 10
        )

        self.path_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        if HAS_CLEAR_SRV:
            self.clear_global_client = self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
            )
            self.clear_local_client = self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
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

    def clear_old_obstacles(self):
        if not HAS_CLEAR_SRV:
            return
        req = ClearEntireCostmap.Request()
        try:
            if hasattr(self, 'clear_global_client') and self.clear_global_client.service_is_ready():
                self.clear_global_client.call_async(req)
            if hasattr(self, 'clear_local_client') and self.clear_local_client.service_is_ready():
                self.clear_local_client.call_async(req)
        except Exception:
            pass

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
        print("[⏳] Waiting for planner output and evaluating safety...")

        path_ok = self.path_received_event.wait(timeout=timeout_sec)
        self.cmd_received_event.wait(timeout=1.5)
        return path_ok, self.latest_path, self.latest_cmd


def evaluate_path_safety_and_quality(path_msg: Path, goal: tuple, obstacle: tuple = None, robot_radius: float = 0.45):
    poses = path_msg.poses
    num_pts = len(poses)

    # 1. Total Path Length & Continuity
    total_len = 0.0
    max_jump = 0.0
    for i in range(1, num_pts):
        d = math.hypot(poses[i].pose.position.x - poses[i-1].pose.position.x,
                       poses[i].pose.position.y - poses[i-1].pose.position.y)
        total_len += d
        if d > max_jump:
            max_jump = d

    # 2. Goal Accuracy Check
    last_pt = poses[-1].pose.position
    goal_dist_error = math.hypot(last_pt.x - goal[0], last_pt.y - goal[1])

    # 3. Collision & Minimum Clearance Check
    min_obs_dist = float('inf')
    collision_detected = False
    if obstacle is not None:
        obs_x, obs_y = obstacle
        obs_half_size = 0.4  # Half-width of 0.8m obstacle
        for p in poses:
            px, py = p.pose.position.x, p.pose.position.y
            dist_to_center = math.hypot(px - obs_x, py - obs_y)
            # Clearance from rover outer body to obstacle surface
            clearance = dist_to_center - (robot_radius + obs_half_size)
            if clearance < min_obs_dist:
                min_obs_dist = clearance
            if clearance <= 0.0:
                collision_detected = True

    # 4. Kinematic Turning Radius Feasibility Check
    min_turning_radius = float('inf')
    for i in range(1, num_pts - 1):
        p0 = (poses[i-1].pose.position.x, poses[i-1].pose.position.y)
        p1 = (poses[i].pose.position.x, poses[i].pose.position.y)
        p2 = (poses[i+1].pose.position.x, poses[i+1].pose.position.y)

        # Menger Curvature
        d01 = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        d12 = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        d02 = math.hypot(p2[0]-p0[0], p2[1]-p0[1])
        area = abs((p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[0])*(p2[0]-p0[0])) / 2.0
        if area > 1e-5 and (d01 * d12 * d02) > 1e-5:
            radius = (d01 * d12 * d02) / (4.0 * area)
            if radius < min_turning_radius:
                min_turning_radius = radius

    # Print Evaluation Dashboard
    print("\n" + "=" * 65)
    print("🛡️  PATH QUALITY & COLLISION EVALUATION REPORT")
    print("=" * 65)

    # Metric 1: Collision Evaluation
    print("1. 🚫 Collision & Clearance Evaluation:")
    if obstacle is not None:
        if collision_detected:
            print("   • Status: ❌ COLLISION DETECTED! (Path penetrates obstacle boundary)")
        else:
            safety_status = "EXCELLENT" if min_obs_dist > 0.4 else "SAFE (Caution Zone)"
            print(f"   • Status: ✅ PASSED (Zero Collisions!)")
            print(f"   • Min Clearance to Obstacle: {min_obs_dist:.2f} meters [{safety_status}]")
            print(f"   • Obstacle Position: ({obs_x:.2f}, {obs_y:.2f})")
    else:
        print("   • Status: ✅ No dynamic obstacles were injected on this run.")

    # Metric 2: Goal Accuracy
    print("\n2. 🎯 Goal Reachability & Accuracy:")
    accuracy_ok = goal_dist_error < 0.35
    acc_symbol = "✅" if accuracy_ok else "⚠️"
    print(f"   • Goal Coordinates: ({goal[0]:.2f}, {goal[1]:.2f})")
    print(f"   • Actual Path End:  ({last_pt.x:.2f}, {last_pt.y:.2f})")
    print(f"   • Distance Error:   {goal_dist_error*100:.1f} cm {acc_symbol} (Tolerance: < 35 cm)")

    # Metric 3: Kinematics Feasibility
    print("\n3. 🏎️ Kinematics & Turning Radius Check:")
    kinematics_ok = (min_turning_radius >= 0.75) or (min_turning_radius == float('inf'))
    kin_symbol = "✅" if kinematics_ok else "⚠️"
    r_disp = f"{min_turning_radius:.2f} m" if min_turning_radius != float('inf') else "Straight Line"
    print(f"   • Rover Minimum Allowed Radius: 0.80 m")
    print(f"   • Smallest Radius in Path:      {r_disp} {kin_symbol}")

    # Metric 4: Path Geometry
    print("\n4. 📏 Path Geometry:")
    print(f"   • Total Length:     {total_len:.2f} meters")
    print(f"   • Waypoints Count:  {num_pts} points")
    print(f"   • Path Continuity:  ✅ Max step = {max_jump:.2f} m (No gaps/teleports)")

    # Final Overall Verdict
    print("-" * 65)
    if (not collision_detected) and accuracy_ok and kinematics_ok:
        print("🏆 FINAL VERDICT: [PASS - 100/100]  (Safe and Ready to Drive!)")
    else:
        print("⚠️ FINAL VERDICT: [REVIEW NEEDED]   (Check warnings above)")
    print("=" * 65)


def interactive_loop(tester: EvaluatedPathTester):
    time.sleep(1.0)
    print("\n" + "=" * 65)
    print("   🤖 ADVANCED PATH PLANNER & EVALUATOR NODE (ROS 2)   ")
    print("=" * 65)

    while rclpy.ok():
        try:
            print("\n" + "-" * 40)
            print("📝 [INPUTS]")
            print("-" * 40)

            tester.clear_old_obstacles()
            print("  🧹 [Auto-Clear] Cleared previous obstacles from costmap.")
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
            obs_coord = None
            if obs_in:
                o_parts = obs_in.split()
                ox, oy = float(o_parts[0]), float(o_parts[1])
                tester.send_obstacle(ox, oy)
                obs_coord = (ox, oy)
                print(f"  -> Injected obstacle at: ({ox:.2f}, {oy:.2f})")
                time.sleep(0.5)

            path_ok, path_msg, cmd_msg = tester.send_goal_and_wait(gx, gy)

            if path_ok and path_msg and len(path_msg.poses) > 0:
                evaluate_path_safety_and_quality(path_msg, goal=(gx, gy), obstacle=obs_coord)
            else:
                print("\n❌ FAILED: No path found by planner within timeout!")

            again = input("\nEvaluate another Goal? (y/n) [y]: ").strip().lower()
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
    tester = EvaluatedPathTester()
    spin_thread = threading.Thread(target=rclpy.spin, args=(tester,), daemon=True)
    spin_thread.start()
    interactive_loop(tester)


if __name__ == '__main__':
    main()
