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
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# Import custom report generator functions
from global_path_benchmarking.report_generator import (
    generate_html_report,
    generate_pdf_report
)


# Helper function to kill old processes
def clean_old_processes():
    print("[INFO] Cleaning up stale ROS 2 processes...")
    targets = ["algo_node", "benchmarking_node", "mock_planner", "benchmarker"]
    for target in targets:
        try:
            subprocess.run(["pkill", "-f", "-9", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
    time.sleep(1.0)

# Helper to generate maps if missing
def generate_synthetic_maps():
    os.makedirs("maps", exist_ok=True)
    os.makedirs("reports", exist_ok=True)
    
    # 1. Empty World
    empty_path = "maps/empty_world.png"
    if not os.path.exists(empty_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        img.save(empty_path)
        print(f"[INFO] Generated synthetic map: {empty_path}")
        
    # 2. Scattered Rocks
    rocks_path = "maps/scattered_rocks.png"
    if not os.path.exists(rocks_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        # Add rocks
        draw.ellipse([80, 80, 120, 120], fill=0)
        draw.ellipse([40, 50, 60, 70], fill=0)
        draw.ellipse([140, 130, 160, 150], fill=0)
        draw.rectangle([120, 40, 135, 60], fill=0)
        img.save(rocks_path)
        print(f"[INFO] Generated synthetic map: {rocks_path}")

    # 3. Canyon Gate
    canyon_path = "maps/canyon_gate.png"
    if not os.path.exists(canyon_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        # Vertical canyon walls with central gate gap (row 100, col 80 to 120)
        draw.rectangle([0, 95, 79, 105], fill=0)
        draw.rectangle([120, 95, 199, 105], fill=0)
        img.save(canyon_path)
        print(f"[INFO] Generated synthetic map: {canyon_path}")

    # 4. Slopes
    slopes_path = "maps/marsyard_slopes.png"
    if not os.path.exists(slopes_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([60, 60, 140, 140], fill=120)
        draw.ellipse([90, 90, 110, 110], fill=0)
        draw.rectangle([20, 80, 40, 120], fill=0)
        draw.rectangle([160, 80, 180, 120], fill=0)
        img.save(slopes_path)
        print(f"[INFO] Generated synthetic map: {slopes_path}")

    # 5. Labyrinth
    lab_path = "maps/marsyard_labyrinth.png"
    if not os.path.exists(lab_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([0, 50, 140, 60], fill=0)
        draw.rectangle([60, 100, 199, 110], fill=0)
        draw.rectangle([0, 150, 140, 160], fill=0)
        img.save(lab_path)
        print(f"[INFO] Generated synthetic map: {lab_path}")

    # 6. Snake Passage
    snake_path = "maps/snake_passage.png"
    if not os.path.exists(snake_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([0, 60, 140, 70], fill=0)
        draw.rectangle([60, 130, 199, 140], fill=0)
        img.save(snake_path)
        print(f"[INFO] Generated synthetic map: {snake_path}")

    # 7. Dead End Trap
    trap_path = "maps/dead_end_trap.png"
    if not os.path.exists(trap_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([60, 115, 140, 125], fill=0)
        draw.rectangle([60, 60, 70, 115], fill=0)
        draw.rectangle([130, 60, 140, 115], fill=0)
        img.save(trap_path)
        print(f"[INFO] Generated synthetic map: {trap_path}")

    # 8. Crater Field
    field_path = "maps/crater_field.png"
    if not os.path.exists(field_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.ellipse([80, 80, 120, 120], fill=180)
        draw.ellipse([36, 116, 84, 148], fill=140)
        draw.ellipse([116, 36, 148, 84], fill=150)
        img.save(field_path)
        print(f"[INFO] Generated synthetic map: {field_path}")

def generate_default_scenarios_yaml():
    os.makedirs("config", exist_ok=True)
    config_path = "config/scenarios.yaml"
    if not os.path.exists(config_path):
        scenarios = {
            "scenarios": [
                {
                    "id": "empty_straight",
                    "map_image": "maps/empty_world.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-3.0, -3.0],
                    "goal": [3.0, 3.0],
                    "reference_path": [
                        [-3.0, -3.0],
                        [-1.5, -1.5],
                        [0.0, 0.0],
                        [1.5, 1.5],
                        [3.0, 3.0]
                    ]
                },
                {
                    "id": "scattered_rocks_detour",
                    "map_image": "maps/scattered_rocks.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [-1.0, 1.0],
                        [1.0, 2.5],
                        [4.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 1.0,
                            "y": 2.5,
                            "radius": 0.4
                        }
                    ]
                },
                {
                    "id": "canyon_gate_passage",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, -1.0],
                        [0.0, 0.0],
                        [0.0, 2.0],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_rough_slopes",
                    "map_image": "maps/marsyard_slopes.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-3.5, 0.0],
                        [-3.0, 3.0],
                        [0.0, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_labyrinth",
                    "map_image": "maps/marsyard_labyrinth.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, -1.0],
                        [-3.5, -1.0],
                        [-3.5, 1.5],
                        [3.5, 1.5],
                        [3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "canyon_gate_blocked",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 0.0,
                            "y": 0.0,
                            "radius": 1.5
                        }
                    ]
                },
                {
                    "id": "marsyard_snake_passage",
                    "map_image": "maps/snake_passage.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, 0.0],
                        [-3.5, 0.0],
                        [-3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "dead_end_trap",
                    "map_image": "maps/dead_end_trap.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -1.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -1.0],
                        [0.0, -3.5],
                        [3.5, -3.5],
                        [3.5, 2.5],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "crater_field",
                    "map_image": "maps/crater_field.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [0.0, -2.5],
                        [2.5, 0.0],
                        [1.5, 2.5],
                        [4.0, 4.0]
                    ]
                }
            ]
        }
        with open(config_path, "w") as f:
            yaml.dump(scenarios, f, default_flow_style=False)
        print(f"[INFO] Generated default scenarios: {config_path}")

class TestingNode(Node):
    def __init__(self, config_path, use_astar=True):
        super().__init__("testing_node")
        self.config_path = config_path
        self.use_astar = use_astar
        
        # Load scenarios
        with open(config_path, "r") as f:
            self.scenarios = yaml.safe_load(f).get("scenarios", [])
            
        # Publishers
        self.name_pub = self.create_publisher(String, "/test_name", 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", 10)
        self.trav_pub = self.create_publisher(OccupancyGrid, "/traversability_map", 10)
        self.start_pub = self.create_publisher(PoseStamped, "/start_pose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        
        # Subscriber to path to coordinate dynamic obstacles
        self.path_received = False
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, 10)
        
        self.get_logger().info("Testing Node Initialized.")

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
        return updated_grid, new_cost_arr

    def publish_start_goal(self, start, goal):
        self.path_received = False
        
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = start[0]
        start_pose.pose.position.y = start[1]
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation.w = 1.0
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.start_pub.publish(start_pose)
        time.sleep(0.1)
        self.goal_pub.publish(goal_pose)

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
                
        # 1. Start algo_node and benchmarking_node directly from install directory
        algo_cmd = ["install/global_path_benchmarking/lib/global_path_benchmarking/algo_node"]
        if not self.use_astar:
            algo_cmd.extend(["--ros-args", "-p", "use_astar:=false"])
        algo_proc = subprocess.Popen(algo_cmd)
        bench_proc = subprocess.Popen([
            "install/global_path_benchmarking/lib/global_path_benchmarking/benchmarking_node",
            "--ros-args",
            "-p", f"config:={self.config_path}",
            "-p", f"benchmark_config:={bench_config_file}"
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
            
            # 4. Trigger planning
            self.publish_start_goal(s["start"], s["goal"])
            
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
                self.publish_start_goal(s["start"], s["goal"])
                
                # Wait for replan path
                t0 = time.time()
                while not self.path_received and (time.time() - t0) < 6.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    
            # Wait a brief moment to allow benchmarking node to output JSON
            t0 = time.time()
            while not os.path.exists(temp_file) and (time.time() - t0) < 3.0:
                time.sleep(0.1)
                
        finally:
            # 6. Terminate both subprocesses
            self.get_logger().info(f"Scenario test completed. Terminating algo and evaluator nodes...")
            algo_proc.terminate()
            bench_proc.terminate()
            
            # Wait and kill if necessary
            try:
                algo_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                algo_proc.kill()
            try:
                bench_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                bench_proc.kill()
                
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
            json.dump(results, f, indent=4)
        print(f"[INFO] Numerical results saved to: {out_json}")
        
        # Generate Markdown Report
        out_md = "reports/numerical_report.md"
        cwd = os.getcwd()
        md_content = []
        md_content.append("# 📈 ROS 2 Path Planning Benchmark Report\n")
        md_content.append("This report summarizes the performance evaluation of the global path planner. ")
        md_content.append("The target pass condition is a **Path Planning Score** >= threshold defined in config.\n\n")
        
        md_content.append("## 📊 Performance Summary Table\n")
        md_content.append("| Scenario ID | Outcome | Score | Success | Planning Time | Blocked Cells | Avg Cost | Replanning |\n")
        md_content.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        
        for sid, res in results.items():
            outcome_emoji = "✅ PASS" if res["outcome"] == "PASS" else "❌ FAIL"
            success_str = "YES" if res["success"] else "NO"
            md_content.append(
                f"| `{sid}` | **{outcome_emoji}** | `{res['final_score']:.1f}/100` | "
                f"{success_str} | `{res['planning_time_s']:.3f} s` | "
                f"`{res['blocked_cells']}` | `{res['avg_cost']:.1f}` | "
                f"`{res['replanning_score']:.1f}` |\n"
            )
        md_content.append("\n---\n\n## 🔍 Detailed Scenario Analyses & Plots\n\n")
        
        for s in self.scenarios:
            sid = s["id"]
            if sid not in results:
                continue
            res = results[sid]
            outcome_emoji = "✅ PASS" if res.get("outcome") == "PASS" else "❌ FAIL"
            plot_path = f"file://{os.path.join(cwd, 'reports', f'result_scenario_{sid}.png')}"
            
            md_content.append(f"### 📍 Scenario: `{sid}` ({outcome_emoji})\n")
            md_content.append(f"- **Final Score**: `{res.get('final_score', 0.0):.2f} / 100`\n")
            md_content.append(f"- **Planning Time**: `{res.get('planning_time_s', 0.0):.3f} s`\n")
            md_content.append(f"- **Path Length / Ratio**: `{res.get('path_length_m', 0.0):.2f} m` (Ratio: `{res.get('length_ratio', 0.0):.2f}`)\n")
            md_content.append(f"- **Footprint Collisions (Blocked Cells)**: `{res.get('blocked_cells', 0)}` cells\n")
            md_content.append(f"- **Average Traversed Cost**: `{res.get('avg_cost', 0.0):.1f}`\n")
            md_content.append(f"- **Safety Margin**: `{res.get('safety_margin_m', 0.0):.2f} m`\n")
            md_content.append(f"- **Replanning Status**: Score `{res.get('replanning_score', 0.0):.1f}/100` in `{res.get('replanning_time_s', 0.0):.3f} s`\n\n")
            
            md_content.append(f"#### Path Visualizer:\n")
            md_content.append(f"![{sid} Plot]({plot_path})\n\n")
            
            if res.get("outcome") == "FAIL":
                md_content.append("> [!WARNING]\n")
                if res.get("blocked_cells", 0) > 0:
                    md_content.append(f"> **Collision Warning**: The path center line or its robot footprint ({s.get('robot_radius')}m) intersected wall obstacles. Implement obstacle inflation to fix this.\n")
                elif res.get("avg_cost", 0) > 20:
                    md_content.append("> **High Cost Warning**: The path went directly through high-slope or rough terrain. Tune the planner weight to prioritize safer detours.\n")
                elif res.get("planning_time_s", 0) > 2.0:
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
                "success": res.get("success", False),
                "outcome": res.get("outcome", "FAIL"),
                "planning_time_s": res.get("planning_time_s", 0.0),
                "path_length_m": res.get("path_length_m", 0.0),
                "blocked_cells": res.get("blocked_cells", 0),
                "avg_cost": res.get("avg_cost", 0.0),
                "safety_margin_m": res.get("safety_margin_m", 0.0),
                "length_ratio": res.get("length_ratio", 1.0),
                
                # New metrics
                "min_angle": res.get("min_angle", 0.0),
                "max_angle": res.get("max_angle", 0.0),
                "avg_angle": res.get("avg_angle", 0.0),
                "min_radius": res.get("min_radius", 0.0),
                "max_radius": res.get("max_radius", 0.0),
                "avg_radius": res.get("avg_radius", 0.0),
                "path_cost": res.get("path_cost", 0.0)
            })
            
        # Compile HTML and PDF
        out_html = "reports/numerical_report.html"
        out_pdf = "reports/numerical_report.pdf"
        
        try:
            generate_html_report(history, out_html)
            self.get_logger().info(f"Numerical HTML report compiled at: {out_html}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate HTML report: {e}")
            
        try:
            generate_pdf_report(history, out_pdf)
            self.get_logger().info(f"Numerical PDF report compiled at: {out_pdf}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate PDF report: {e}")
            
        return True


def main(args=None):
    from rclpy.utilities import remove_ros_args
    clean_args = remove_ros_args(args=sys.argv)[1:]
    
    parser = argparse.ArgumentParser(description="ROS 2 Global Path Planning Testing Orchestrator")
    parser.add_argument("--config", type=str, default="config/scenarios.yaml", help="Path to config/scenarios.yaml")
    parser.add_argument("--benchmark_config", type=str, default="config/benchmark_config.yaml", help="Path to config/benchmark_config.yaml")
    parser.add_argument("--verify", action="store_true", help="Generate pre-run verification PNGs and exit")
    parser.add_argument("--clean", action="store_true", help="Kill stale processes before running")
    parser.add_argument("--scenario_id", type=str, default=None, help="Select a specific scenario to run")
    parser.add_argument("--use_astar", type=str, default="true", help="Set to 'false' to disable A* in algo_node")
    
    cli_args = parser.parse_args(clean_args)
    
    # Clean up old processes if requested
    if cli_args.clean:
        clean_old_processes()
        
    # Generate synthetic map assets/yamls if missing
    generate_synthetic_maps()
    generate_default_scenarios_yaml()
    
    # Init ROS 2 and node
    rclpy.init(args=args)
    
    use_astar_val = cli_args.use_astar.lower() == 'true'
    testing_node = TestingNode(cli_args.config, use_astar=use_astar_val)
    
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
