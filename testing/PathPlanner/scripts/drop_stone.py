#!/usr/bin/env python3
"""
drop_stone.py

Interactive Real-Time Obstacle Injection CLI Tool for ERC Path Planner & MPPI Controller.
Allows developers to dynamically drop rocks/obstacles directly onto the rover's
active path in real-time to observe online replanning and collision avoidance.

Usage:
  1. Auto-Drop: Drop a rock 2.0m ahead directly on the rover's active path:
     ./drop_stone.py
     or: ros2 run global_path_benchmarking drop_stone

  2. Custom Distance: Drop a rock 3.5m ahead on the path:
     ./drop_stone.py --distance 3.5

  3. Specific Coordinates: Drop a rock at (x, y):
     ./drop_stone.py 1.5 2.0
     or: ./drop_stone.py --x 1.5 --y 2.0

  4. Interactive Loop Mode: Press [Enter] anytime to drop a rock in the rover's path:
     ./drop_stone.py -i
"""

import sys
import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry


class DropStoneNode(Node):
    def __init__(self):
        super().__init__('drop_stone_cli')

        # Publisher to spawn dynamic obstacle
        self.pub = self.create_publisher(
            Point,
            '/mock_perception/spawn_obstacle',
            10
        )

        # Subscribers to rover state & current planned path
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.latest_odom = None
        self.latest_plan = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_cb,
            qos_volatile
        )

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_cb,
            qos_volatile
        )

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def plan_cb(self, msg: Path):
        if len(msg.poses) > 0:
            self.latest_plan = msg

    def wait_for_data(self, timeout_sec: float = 1.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_odom is not None and self.latest_plan is not None:
                break

    def get_rover_pose(self):
        if self.latest_odom is None:
            return 0.0, 0.0, 0.0
        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(pos.x), float(pos.y), yaw

    def find_target_point_on_path(self, target_distance_ahead: float = 2.0):
        rx, ry, yaw = self.get_rover_pose()

        # If a path is available, walk along it from the rover's position
        if self.latest_plan is not None and len(self.latest_plan.poses) > 1:
            poses = self.latest_plan.poses

            # 1. Find the index closest to the current rover position
            closest_idx = 0
            min_dist = float('inf')
            for i, p in enumerate(poses):
                d = math.hypot(p.pose.position.x - rx, p.pose.position.y - ry)
                if d < min_dist:
                    min_dist = d
                    closest_idx = i

            # 2. Accumulate distance forward along path from closest index
            accum_dist = 0.0
            chosen_idx = closest_idx
            for i in range(closest_idx + 1, len(poses)):
                prev = poses[i - 1].pose.position
                curr = poses[i].pose.position
                accum_dist += math.hypot(curr.x - prev.x, curr.y - prev.y)
                chosen_idx = i
                if accum_dist >= target_distance_ahead:
                    break

            target_x = float(poses[chosen_idx].pose.position.x)
            target_y = float(poses[chosen_idx].pose.position.y)
            return target_x, target_y, f"on path {accum_dist:.2f}m ahead of rover (index {chosen_idx}/{len(poses)})"

        # Fallback: project directly forward based on rover's current heading
        target_x = rx + target_distance_ahead * math.cos(yaw)
        target_y = ry + target_distance_ahead * math.sin(yaw)
        return target_x, target_y, f"{target_distance_ahead:.2f}m ahead along rover heading"

    def drop_stone(self, x: float, y: float, z: float = 0.20):
        msg = Point(x=x, y=y, z=z)
        # Publish multiple times to guarantee reception across DDS
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)


def run_drop(node: DropStoneNode, args):
    node.wait_for_data(timeout_sec=0.8)

    if args.x is not None and args.y is not None:
        target_x = float(args.x)
        target_y = float(args.y)
        desc = f"at specified coordinates ({target_x:.2f}, {target_y:.2f})"
    else:
        target_x, target_y, desc = node.find_target_point_on_path(args.distance)

    node.drop_stone(target_x, target_y, z=0.20)
    rx, ry, _ = node.get_rover_pose()

    print("\n" + "=" * 65)
    print("🪨 [OBSTACLE INJECTED SUCCESSFULLY]")
    print("=" * 65)
    print(f"  📍 Rock Location:     (X = {target_x:+.2f}, Y = {target_y:+.2f})")
    print(f"  🚜 Rover Location:    (X = {rx:+.2f}, Y = {ry:+.2f})")
    print(f"  📏 Relative Position: {desc}")
    print("-----------------------------------------------------------------")
    print("  👀 Watch RViz: Costmap will inflate a lethal obstacle zone,")
    print("     and Smac Hybrid A* & MPPI will dynamically reroute around it!")
    print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Drop an obstacle directly onto the rover's active path in real-time."
    )
    parser.add_argument('pos_args', nargs='*', help='Optional (x y) coordinates')
    parser.add_argument('--x', type=float, default=None, help='Target X coordinate')
    parser.add_argument('--y', type=float, default=None, help='Target Y coordinate')
    parser.add_argument('-d', '--distance', type=float, default=2.0,
                        help='Distance ahead along the path to drop the stone (default: 2.0m)')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='Keep running and drop stones interactively on [Enter]')

    args = parser.parse_args()

    # Parse positional x, y if provided
    if len(args.pos_args) >= 2:
        try:
            args.x = float(args.pos_args[0])
            args.y = float(args.pos_args[1])
        except ValueError:
            pass

    rclpy.init()
    node = DropStoneNode()

    try:
        if args.interactive:
            print("=" * 65)
            print("🎮 INTERACTIVE OBSTACLE INJECTION MODE ACTIVE")
            print("=" * 65)
            print("Commands:")
            print("  - Press [Enter]   -> Drop a rock ahead in the rover's path")
            print("  - Type 'd <dist>' -> Change drop distance (e.g. 'd 3.0')")
            print("  - Type 'x y'      -> Drop rock at specific coordinates (e.g. '1.5 2.0')")
            print("  - Type 'q'        -> Exit")
            print("=" * 65)

            while rclpy.ok():
                try:
                    user_input = input("👉 Press [Enter] to drop stone (or 'q' to quit): ").strip()
                except (KeyboardInterrupt, EOFError):
                    break

                if user_input.lower() in ['q', 'quit', 'exit']:
                    break
                elif user_input.startswith('d '):
                    try:
                        args.distance = float(user_input.split()[1])
                        print(f"Distance ahead updated to: {args.distance:.2f}m")
                        continue
                    except Exception:
                        pass
                elif len(user_input.split()) == 2:
                    parts = user_input.split()
                    try:
                        args.x = float(parts[0])
                        args.y = float(parts[1])
                        run_drop(node, args)
                        args.x = None
                        args.y = None
                        continue
                    except Exception:
                        pass

                run_drop(node, args)
        else:
            run_drop(node, args)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
