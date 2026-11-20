#!/usr/bin/env python3

import os
import sys
import time
import json
import yaml
import subprocess
import argparse
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
    NAV2_ACTIONS_AVAILABLE = True
except ImportError:
    NAV2_ACTIONS_AVAILABLE = False

# Import custom report generator functions
from global_path_benchmarking.report_generator import (
    format_metric,
    is_number,
    record_outcome,
    generate_html_report,
    generate_pdf_report
)


from pathplanner_mock_tools.scene_assets import (
    generate_synthetic_maps, generate_default_scenarios_yaml,
)


# Helper function to kill old processes
def clean_old_processes():
    raise RuntimeError('--clean is disabled: stop only processes owned by your validation run')

# Helper to generate maps if missing
class TestingNode(Node):
    def __init__(self, config_path):
        super().__init__("testing_node")
        self.config_path = config_path
        
        # Load scenarios
        with open(config_path, "r") as f:
            self.scenarios = yaml.safe_load(f).get("scenarios", [])
            
        # Publishers
        self.name_pub = self.create_publisher(String, "/test_name", 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/testing/pathplanner/scenario_truth", 10)
        self.std_map_pub = self.create_publisher(OccupancyGrid, "/map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.trav_pub = self.create_publisher(OccupancyGrid, "/traversability_map", 10)
        self.start_pub = self.create_publisher(PoseStamped, "/start_pose", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, "/waypoints_markers", 10)

        # Nav2 Action Clients
        if NAV2_ACTIONS_AVAILABLE:
            self._nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
            self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self._nav_through_client = None
            self._nav_to_pose_client = None

        # Subscriber to path to coordinate dynamic obstacles
        self.path_received = False
        self.path_sub = self.create_subscription(Path, "/plan", self.path_callback, 10)
        
        self.get_logger().info("Universal Testing Node Initialized.")

    def path_callback(self, msg):
        self.path_received = True

    def publish_map(self, scenario):
        resolution = scenario["resolution"]
        origin_coords = scenario["origin"]
        
        map_img_path = scenario["map_image"]
        img = Image.open(map_img_path).convert("L")
        width, height = img.size
        
        # Invert grayscale (255=free -> 0, 0=blocked -> 100)
        img_arr = np.array(img)
        cost_arr = np.zeros_like(img_arr)
        cost_arr[img_arr == 0] = 100
        cost_arr[img_arr == 255] = 0
        gray_mask = (img_arr > 0) & (img_arr < 255)
        cost_arr[gray_mask] = (100 - (img_arr[gray_mask] / 2.55)).astype(np.int8)
        
        cost_arr = np.flipud(cost_arr)
        cost_data = cost_arr.flatten().tolist()
        
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin_coords[0])
        grid.info.origin.position.y = float(origin_coords[1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = cost_data
        
        self.costmap_pub.publish(grid)
        self.std_map_pub.publish(grid)
        self.trav_pub.publish(grid)
        return grid, cost_arr

    def update_map_with_obstacle(self, base_grid, cost_arr, scenario, x, y, radius):
        resolution = scenario["resolution"]
        origin_x = scenario["origin"][0]
        origin_y = scenario["origin"][1]
        width = base_grid.info.width
        height = base_grid.info.height
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        grid_radius = int(radius / resolution)
        
        new_cost_arr = np.copy(cost_arr)
        for r in range(max(0, grid_y - grid_radius), min(height, grid_y + grid_radius + 1)):
            for c in range(max(0, grid_x - grid_radius), min(width, grid_x + grid_radius + 1)):
                if (r - grid_y)**2 + (c - grid_x)**2 <= grid_radius**2:
                    new_cost_arr[r, c] = 100
                    
        updated_grid = OccupancyGrid()
        updated_grid.header = base_grid.header
        updated_grid.header.stamp = self.get_clock().now().to_msg()
        updated_grid.info = base_grid.info
        updated_grid.data = new_cost_arr.flatten().tolist()
        
        self.costmap_pub.publish(updated_grid)
        self.std_map_pub.publish(updated_grid)
        return updated_grid, new_cost_arr

    def publish_start_goal(self, start, goal, waypoints=None):
        self.path_received = False
        
        # Publish Start Pose & InitialPose for SLAM/EKF/Sim
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = float(start[0])
        start_pose.pose.position.y = float(start[1])
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation.w = 1.0
        
        init_pose = PoseWithCovarianceStamped()
        init_pose.header = start_pose.header
        init_pose.pose.pose = start_pose.pose
        
        self.start_pub.publish(start_pose)
        self.initialpose_pub.publish(init_pose)
        
        # Publish Waypoint Markers in RViz if provided
        if waypoints:
            markers = MarkerArray()
            for idx, pt in enumerate(waypoints):
                m = Marker()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = "map"
                m.ns = "waypoints"
                m.id = idx
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = float(pt[0])
                m.pose.position.y = float(pt[1])
                m.pose.position.z = 0.3
                m.scale.x = 0.3
                m.scale.y = 0.3
                m.scale.z = 0.6
                m.color.r = 0.2
                m.color.g = 0.8
                m.color.b = 1.0
                m.color.a = 0.8
                markers.markers.append(m)
            self.waypoint_marker_pub.publish(markers)

        # Dispatch Goal via Nav2 Action (if available) or standard Topic
        action_sent = False
        if self._nav_through_client is not None and waypoints:
            if self._nav_through_client.wait_for_server(timeout_sec=0.2):
                goal_msg = NavigateThroughPoses.Goal()
                poses = []
                for pt in waypoints:
                    p = PoseStamped()
                    p.header.frame_id = "map"
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = float(pt[0])
                    p.pose.position.y = float(pt[1])
                    p.pose.orientation.w = 1.0
                    poses.append(p)
                goal_msg.poses = poses
                self._nav_through_client.send_goal_async(goal_msg)
                action_sent = True
                self.get_logger().info(f"Dispatched {len(poses)} waypoints via /navigate_through_poses Action.")

        if not action_sent:
            time.sleep(0.1)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(goal[0])
            goal_pose.pose.position.y = float(goal[1])
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)
            self.get_logger().info(f"Dispatched goal ({format_metric(goal[0], '.2f')}, {format_metric(goal[1], '.2f')}) via /goal_pose Topic.")

    def verify_scenarios(self):
        print("\n================ VERIFYING SCENARIOS ================")
        for s in self.scenarios:
            map_image = s["map_image"]
            ref_path = s["reference_path"]
            resolution = s["resolution"]
            origin = s["origin"]
            robot_radius = s.get("robot_radius", 0.3)
            
            img = Image.open(map_image).convert("L")
            img_arr = np.array(img)
            
            plt.figure(figsize=(8, 8))
            extent = [origin[0], origin[0] + img_arr.shape[1]*resolution, 
                      origin[1], origin[1] + img_arr.shape[0]*resolution]
            plt.imshow(img_arr, cmap='gray', origin='lower', extent=extent)
            
            ref_x = [p[0] for p in ref_path]
            ref_y = [p[1] for p in ref_path]
            plt.plot(ref_x, ref_y, 'g-o', label='Reference Path (Perfect)', linewidth=2)
            
            for px, py in ref_path:
                circle = plt.Circle((px, py), robot_radius, color='g', fill=True, alpha=0.1)
                plt.gca().add_patch(circle)
                
            plt.plot(s["start"][0], s["start"][1], 'bs', markersize=10, label='Start')
            plt.plot(s["goal"][0], s["goal"][1], 'r*', markersize=12, label='Goal')
            
            plt.title(f"Verify Scenario: {s['id']}")
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            out_file = f"reports/verify_scenario_{s['id']}.png"
            plt.savefig(out_file, bbox_inches='tight')
            plt.close()
            print(f"[VERIFIED] Scenario '{s['id']}' image saved to: {out_file}")
        print("=====================================================\n")

    def run_scenario(self, s, bench_config_file):
        sid = s["id"]
        self.get_logger().info(f"Setting up subprocesses for Scenario: {sid}")
        
        # Pre-clean temporary file if it exists
        temp_file = f"reports/temp_{sid}.json"
        if os.path.exists(temp_file):
            try:
                os.remove(temp_file)
            except Exception:
                pass
                
        # 1. Start target planner (if not external), benchmarking_node, and mock rover simulator
        target_pkg = "erc_path_planner"
        target_launch = "path_planning.launch.py"
        use_external = False
        
        if os.path.exists(bench_config_file):
            try:
                with open(bench_config_file, "r") as f:
                    b_cfg = yaml.safe_load(f) or {}
                    t_mod = b_cfg.get("target_module", {})
                    use_external = t_mod.get("use_external_module", False)
                    target_pkg = t_mod.get("target_package", target_pkg)
                    target_launch = t_mod.get("target_launch", target_launch)
            except Exception as e:
                self.get_logger().warn(f"Could not parse target_module config: {e}")

        if not use_external:
            planner_cmd = ["ros2", "launch", target_pkg, target_launch]
            planner_proc = subprocess.Popen(planner_cmd)
        else:
            self.get_logger().info("Target module configured as external: waiting for external node...")
            planner_proc = None

        bench_proc = subprocess.Popen([
            "ros2", "run", "global_path_benchmarking", "benchmarking_node",
            "--ros-args",
            "-p", f"config:={self.config_path}",
            "-p", f"benchmark_config:={bench_config_file}"
        ])
        mock_rover_proc = subprocess.Popen([
            "ros2", "run", "global_path_benchmarking", "mock_rover_sim",
            "--ros-args",
            "-p", f"initial_x:={float(s['start'][0])}",
            "-p", f"initial_y:={float(s['start'][1])}"
        ])
        
        # Wait for ROS 2 graph to initialize and nodes to discover each other
        time.sleep(2.0)
        
        try:
            # 2. Publish current test/scenario name
            name_msg = String()
            name_msg.data = sid
            self.name_pub.publish(name_msg)
            time.sleep(0.1)
            
            # 3. Publish base map
            grid, cost_arr = self.publish_map(s)
            time.sleep(1.0)
            
            # 4. Trigger planning (with waypoints support)
            self.publish_start_goal(s["start"], s["goal"], s.get("waypoints", []))
            
            # Wait for path response (spin testing_node to handle topic callback)
            t0 = time.time()
            while not self.path_received and (time.time() - t0) < 6.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                
            # 5. Handle dynamic obstacle if applicable
            dyn_obs = s.get("dynamic_obstacles", [])
            if len(dyn_obs) > 0 and self.path_received:
                obs = dyn_obs[0]
                self.get_logger().info(f"Injecting dynamic obstacle at ({obs['x']}, {obs['y']}) in {obs['trigger_time']}s...")
                time.sleep(obs["trigger_time"])
                
                # Update map with obstacle
                grid, cost_arr = self.update_map_with_obstacle(grid, cost_arr, s, obs["x"], obs["y"], obs["radius"])
                time.sleep(0.5)
                
                # Re-trigger planning
                self.publish_start_goal(s["start"], s["goal"], s.get("waypoints", []))
                
                # Wait for replan path
                t0 = time.time()
                while not self.path_received and (time.time() - t0) < 6.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    
            # Wait a brief moment to allow benchmarking node to output JSON
            t0 = time.time()
            while not os.path.exists(temp_file) and (time.time() - t0) < 3.0:
                time.sleep(0.1)
                
        finally:
            # 6. Terminate subprocesses
            self.get_logger().info(f"Scenario test completed. Terminating planner, evaluator, and mock nodes...")
            for proc in [planner_proc, bench_proc, mock_rover_proc]:
                if proc is not None:
                    try:
                        proc.terminate()
                        proc.wait(timeout=2.0)
                    except Exception:
                        try:
                            proc.kill()
                        except Exception:
                            pass
                
            # Brief cooldown to free topics/resources
            time.sleep(1.0)

    def compile_reports(self):
        results = {}
        for s in self.scenarios:
            temp_file = f"reports/temp_{s['id']}.json"
            if os.path.exists(temp_file):
                try:
                    with open(temp_file, "r") as f:
                        results[s["id"]] = json.load(f)
                    os.remove(temp_file)
                except Exception as e:
                    self.get_logger().error(f"Failed to read/delete temp file {temp_file}: {e}")
                    
        if not results:
            self.get_logger().error("No test results found to compile.")
            return False
            
        # Write JSON output
        out_json = "reports/results.json"
        with open(out_json, "w") as f:
            json.dump(results, f, indent=4, allow_nan=False)
        print(f"[INFO] Numerical results saved to: {out_json}")
        
        # Generate Markdown Report
        out_md = "reports/numerical_report.md"
        cwd = os.getcwd()
        md_content = []
        md_content.append("# 📈 ROS 2 Path Planning & MPPI Controller Benchmark Report\n")
        md_content.append("This report summarizes the performance evaluation of the **Smac Hybrid A* Global Planner** and **MPPI Local Controller** (`erc_path_planner`).\n\n")
        
        md_content.append("## 📊 Performance Summary Table\n")
        md_content.append("| Scenario ID | Outcome | Overall Score | Planner Score | MPPI Score | Mean CTE | Last Goal Distance | Plan Time |\n")
        md_content.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        
        for sid, res in results.items():
            outcome_emoji = record_outcome(res)
            p_score = res.get("planner_score")
            c_score = res.get("controller_score")
            cte = res.get("mean_cte")
            g_acc = res.get("goal_accuracy_m")
            md_content.append(
                f"| `{sid}` | **{outcome_emoji}** | `{format_metric(res['final_score'], '.1f')}/100` | "
                f"`{format_metric(p_score, '.1f')}` | `{format_metric(c_score, '.1f')}` | "
                f"`{format_metric(cte, '.3f')} m` | `{format_metric(g_acc, '.3f')} m` | `{format_metric(res['planning_time_s'], '.3f')} s` |\n"
            )
        md_content.append("\n---\n\n## 🔍 Detailed Scenario Analyses & Plots\n\n")
        
        for s in self.scenarios:
            sid = s["id"]
            if sid not in results:
                continue
            res = results[sid]
            outcome_emoji = record_outcome(res)
            plot_path = f"file://{os.path.join(cwd, 'reports', f'result_scenario_{sid}.png')}"
            
            md_content.append(f"### 📍 Scenario: `{sid}` ({outcome_emoji})\n")
            md_content.append(f"- **Overall Combined Score**: `{format_metric(res.get('final_score'), '.2f')} / 100`\n")
            md_content.append(f"- **Global Planner Score**: `{format_metric(res.get('planner_score'), '.2f')} / 100`\n")
            md_content.append(f"- **MPPI Controller Score**: `{format_metric(res.get('controller_score'), '.2f')} / 100`\n")
            md_content.append(f"- **Cross-Track Error (CTE)**: Mean `{format_metric(res.get('mean_cte'), '.3f')} m` | Max `{format_metric(res.get('max_cte'), '.3f')} m` | RMS `{format_metric(res.get('rms_cte'), '.3f')} m`\n")
            md_content.append(f"- **Commanded Velocity**: Mean Lin `{format_metric(res.get('mean_linear_vel'), '.2f')} m/s` | Mean Ang `{format_metric(res.get('mean_angular_vel'), '.2f')} rad/s`\n")
            md_content.append(f"- **Control Smoothness & Stability**: Lin Accel Std `{format_metric(res.get('linear_jerk_std'), '.2f')} m/s²` | Ang Accel Std `{format_metric(res.get('angular_jerk_std'), '.2f')} rad/s²`\n")
            md_content.append(f"- **Command Frequency**: `{format_metric(res.get('cmd_freq_hz'), '.1f')} Hz` | **Last Goal Distance**: `{format_metric(res.get('goal_accuracy_m'), '.3f')} m`\n")
            md_content.append(f"- **Planning Time**: `{format_metric(res.get('planning_time_s'), '.3f')} s`\n")
            md_content.append(f"- **Path Length / Ratio**: `{format_metric(res.get('path_length_m'), '.2f')} m` (Ratio: `{format_metric(res.get('length_ratio'), '.2f')}`)\n")
            md_content.append(f"- **Obstacle Clearance**: `{format_metric(res.get('safety_margin_m'), '.2f')} m` (Colliding samples: `{res.get('blocked_cells')}`)\n")
            md_content.append(f"- **Replanning Status**: Score `{format_metric(res.get('replanning_score'), '.1f')}/100` in `{format_metric(res.get('replanning_time_s'), '.3f')} s`\n\n")
            
            md_content.append(f"#### Path Visualizer:\n")
            md_content.append(f"![{sid} Plot]({plot_path})\n\n")
            
            if res.get("outcome") == "FAIL":
                md_content.append("> [!WARNING]\n")
                if is_number(res.get("blocked_cells")) and res.get("blocked_cells") > 0:
                    md_content.append(f"> **Collision Warning**: The path center line or its robot footprint ({s.get('robot_radius')}m) intersected wall obstacles. Implement obstacle inflation to fix this.\n")
                elif is_number(res.get("avg_cost")) and res.get("avg_cost") > 20:
                    md_content.append("> **High Cost Warning**: The path went directly through high-slope or rough terrain. Tune the planner weight to prioritize safer detours.\n")
                elif is_number(res.get("planning_time_s")) and res.get("planning_time_s") > 2.0:
                    md_content.append("> **Latency Warning**: Planning exceeded the preferred 2.0s boundary. Optimize search iterations.\n")
            md_content.append("\n---\n\n")
            
        with open(out_md, "w") as f:
            f.writelines(md_content)
        print(f"[INFO] Numerical Markdown report compiled at: {out_md}")
        
        # Convert results to run history format
        history = []
        for sid, res in results.items():
            history.append({
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
                "test_name": sid,
                "success": res.get("success"),
                "outcome": record_outcome(res),
                "planning_time_s": res.get("planning_time_s"),
                "path_length_m": res.get("path_length_m"),
                "blocked_cells": res.get("blocked_cells"),
                "avg_cost": res.get("avg_cost"),
                "safety_margin_m": res.get("safety_margin_m"),
                "length_ratio": res.get("length_ratio"),
                
                # New metrics
                "min_angle": res.get("min_angle"),
                "max_angle": res.get("max_angle"),
                "avg_angle": res.get("avg_angle"),
                "min_radius": res.get("min_radius"),
                "max_radius": res.get("max_radius"),
                "avg_radius": res.get("avg_radius"),
                "path_cost": res.get("path_cost")
            })
            
        # Compile HTML and PDF
        reports_ok = True
        out_html = "reports/numerical_report.html"
        out_pdf = "reports/numerical_report.pdf"
        
        try:
            if generate_html_report(history, out_html):
                self.get_logger().info(f"Numerical HTML report compiled at: {out_html}")
            else:
                reports_ok = False
                self.get_logger().warning("HTML report unavailable or export failed")
        except Exception as e:
            reports_ok = False
            self.get_logger().error(f"Failed to generate HTML report: {e}")
            
        try:
            if generate_pdf_report(history, out_pdf):
                self.get_logger().info(f"Numerical PDF report compiled at: {out_pdf}")
            else:
                reports_ok = False
                self.get_logger().warning("PDF report unavailable or export failed")
        except Exception as e:
            reports_ok = False
            self.get_logger().error(f"Failed to generate PDF report: {e}")
            
        return reports_ok


def main(args=None):
    from rclpy.utilities import remove_ros_args
    clean_args = remove_ros_args(args=sys.argv)[1:]
    
    parser = argparse.ArgumentParser(description="ROS 2 Global Path Planning Testing Orchestrator")
    parser.add_argument("--config", type=str, default="config/scenarios.yaml", help="Path to config/scenarios.yaml")
    parser.add_argument("--benchmark_config", type=str, default="config/benchmark_config.yaml", help="Path to config/benchmark_config.yaml")
    parser.add_argument("--verify", action="store_true", help="Generate pre-run verification PNGs and exit")
    parser.add_argument("--clean", action="store_true", help="Kill stale processes before running")
    parser.add_argument("--scenario_id", type=str, default=None, help="Select a specific scenario to run")
    
    cli_args = parser.parse_args(clean_args)
    
    # Clean up old processes if requested
    if cli_args.clean:
        clean_old_processes()
        
    # Generate synthetic map assets/yamls if missing
    generate_synthetic_maps()
    generate_default_scenarios_yaml()
    
    # Init ROS 2 and node
    rclpy.init(args=args)
    
    testing_node = TestingNode(cli_args.config)
    
    if cli_args.verify:
        testing_node.verify_scenarios()
        testing_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    # Select scenarios
    scenarios_to_run = []
    if cli_args.scenario_id:
        target = next((s for s in testing_node.scenarios if s["id"] == cli_args.scenario_id), None)
        if target:
            scenarios_to_run.append(target)
        else:
            print(f"[ERROR] Scenario ID '{cli_args.scenario_id}' not found in scenarios config.")
            testing_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    else:
        scenarios_to_run = testing_node.scenarios
        
    # Run tests
    try:
        for s in scenarios_to_run:
            testing_node.run_scenario(s, cli_args.benchmark_config)
            
        # Compile reports
        testing_node.compile_reports()
    except Exception as e:
        print(f"[ERROR] Exception occurred during testing orchestration: {e}")
    finally:
        testing_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
