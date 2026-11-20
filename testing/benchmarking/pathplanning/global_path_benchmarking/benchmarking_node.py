#!/usr/bin/env python3

import os
import sys
import time
import json
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32

# Import custom report generator functions
from global_path_benchmarking.report_generator import (
    calculate_path_metrics,
    costmap_array,
    calculate_control_metrics,
    format_metric,
    is_number,
    record_outcome,
    generate_html_report,
    generate_pdf_report,
    generate_csv_report
)

class BenchmarkingNode(Node):
    def __init__(self):
        super().__init__("benchmarking_node")
        
        # Declare CLI / ROS Arguments for configs
        self.declare_parameter("config", "config/scenarios.yaml")
        self.declare_parameter("benchmark_config", "config/benchmark_config.yaml")
        
        config_path = self.get_parameter("config").value
        bench_config_path = self.get_parameter("benchmark_config").value
        
        # Load configs
        self.scenarios = []
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    self.scenarios = yaml.safe_load(f).get("scenarios", [])
                self.get_logger().info(f"Loaded scenarios from {config_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load scenarios: {e}")
        else:
            self.get_logger().warn(f"Scenarios file {config_path} not found.")

        # Default benchmark configs
        self.pass_threshold = 85.0
        self.weights = {
            "success": 0.25, "time": 0.15, "obstacle": 0.25,
            "cost": 0.15, "length": 0.10, "replan": 0.10
        }
        self.limits = {
            "target_planning_time": 2.0,
            "timeout_planning_time": 10.0,
            "target_length_ratio": 1.35
        }
        
        if os.path.exists(bench_config_path):
            try:
                with open(bench_config_path, "r") as f:
                    b_cfg = yaml.safe_load(f).get("benchmark", {})
                    self.pass_threshold = b_cfg.get("pass_threshold", self.pass_threshold)
                    self.weights.update(b_cfg.get("weights", {}))
                    self.limits.update(b_cfg.get("limits", {}))
                self.get_logger().info(f"Loaded benchmark configuration from {bench_config_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load benchmark configuration: {e}")
        else:
            self.get_logger().warn(f"Benchmark config {bench_config_path} not found. Using defaults.")

        # State Variables
        self.test_name = "general"
        self.costmap = None
        self.cost_arr = None
        self.start_pose = None
        self.goal_pose = None
        self.planner_internal_time = 0.0
        
        self.roundtrip_start_time = None
        self.history = []
        self.cmd_history = []
        self.odom_history = []
        
        # Path Tracking for dynamic scenarios
        self.path_count = 0
        self.static_eval = None
        self.static_time = 0.0
        self.replan_eval = None
        self.replan_time = 0.0
        
        # Subscribers
        self.test_name_sub = self.create_subscription(
            String, "/test_name", self.test_name_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.map_callback, 10
        )
        self.start_sub = self.create_subscription(
            PoseStamped, "/start_pose", self.start_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, "/plan", self.path_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.time_sub = self.create_subscription(
            Float32, "/planner_internal_time", self.time_callback, 10
        )
        
        self.get_logger().info("========================================")
        self.get_logger().info("Benchmarking Node Initialized (Planning & MPPI Control).")
        self.get_logger().info("Listening to inputs & outputs...")
        self.get_logger().info("========================================")

    def test_name_callback(self, msg):
        new_name = msg.data.strip()
        if not new_name:
            new_name = "general"
        if new_name != self.test_name:
            self.test_name = new_name
            self.get_logger().info(f"Test Context switched to: '{self.test_name}'")
            # Reset trackers for the new test case
            self.path_count = 0
            self.static_eval = None
            self.static_time = 0.0
            self.replan_eval = None
            self.replan_time = 0.0
            self.cmd_history = []
            self.odom_history = []

    def map_callback(self, msg):
        self.cost_arr = costmap_array(msg)
        self.costmap = msg if self.cost_arr is not None else None
        if self.costmap is None:
            self.get_logger().warning('Costmap unavailable or unsupported; safety metrics are unavailable')

    def start_callback(self, msg):
        self.start_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.roundtrip_start_time = self.get_clock().now()
        self.get_logger().info("Goal received. Timer started.")

    def cmd_vel_callback(self, msg):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_history.append((t_sec, msg.linear.x, msg.linear.y, msg.angular.z))

    def odom_callback(self, msg: Odometry):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        q = msg.pose.pose.orientation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        yaw = float('nan')
        if math.isfinite(norm) and norm > 0:
            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.odom_history.append((
            t_sec,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ))

    def time_callback(self, msg):
        self.planner_internal_time = msg.data

    def path_callback(self, msg):
        # Calculate compilation / planning time
        if self.planner_internal_time > 0.0:
            comp_time = self.planner_internal_time
        elif self.roundtrip_start_time is not None:
            comp_time = (self.get_clock().now() - self.roundtrip_start_time).nanoseconds / 1e9
        else:
            comp_time = None
            
        if not is_number(comp_time) or comp_time < 0:
            comp_time = None

        # Reset internal planner timer for the next run
        self.planner_internal_time = 0.0

        self.get_logger().info(f"Received Planned Path. Compilation/Planning Time: {format_metric(comp_time, '.4f')}s")
        
        # Find if current test matches a defined scenario
        scenario = next((s for s in self.scenarios if s["id"] == self.test_name), None)
        
        # Evaluate the path
        eval_res = self.evaluate_path(msg, scenario)
        
        # Store in run history
        run_record = {
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
            "test_name": self.test_name,
            "success": eval_res["success"],
            "outcome": 'FAIL' if eval_res['success'] is False else 'INCOMPLETE',
            "planning_time_s": comp_time,
            "path_length_m": eval_res["length"],
            "blocked_cells": eval_res["blocked_cells"],
            "avg_cost": eval_res["avg_cost"],
            "safety_margin_m": eval_res["safety_margin"],
            
            # Rich geometric and physical metrics
            "length_ratio": eval_res["ratio"],
            "min_angle": eval_res["min_angle"],
            "max_angle": eval_res["max_angle"],
            "avg_angle": eval_res["avg_angle"],
            "min_radius": eval_res["min_radius"],
            "max_radius": eval_res["max_radius"],
            "avg_radius": eval_res["avg_radius"],
            "path_cost": eval_res["total_cost"]
        }
        self.history.append(run_record)
        
        if self.test_name == "general" or scenario is None:
            # Just print the raw metrics in general mode
            self.print_general_metrics(eval_res, comp_time)
            return

        # Scenario mode evaluation
        self.path_count += 1
        dyn_obs = scenario.get("dynamic_obstacles", [])
        
        if len(dyn_obs) > 0:
            if self.path_count == 1:
                self.static_eval = eval_res
                self.static_time = comp_time
                self.get_logger().info("Static path evaluated. Waiting for dynamic replan...")
                return
            else:
                self.replan_eval = eval_res
                self.replan_time = comp_time
                self.get_logger().info("Replanned path evaluated. Computing final scores...")
        else:
            self.static_eval = eval_res
            self.static_time = comp_time

        # Calculate scores and write outputs
        self.process_scenario_results(scenario)

    def evaluate_path(self, path, scenario):
        coords = [(p.pose.position.x, p.pose.position.y) for p in path.poses] if path else []
        res = calculate_path_metrics(coords, self.start_pose, self.goal_pose, self.costmap)
        res['blocked_cells'] = None
        res['safety_margin'] = res['nearest_obstacle']
        if not res['success']:
            return res
        if (self.costmap is None or self.cost_arr is None
                or path.header.frame_id != self.costmap.header.frame_id
                or any(pose is not None and pose.header.frame_id != path.header.frame_id
                       for pose in (self.start_pose, self.goal_pose))):
            res.update(success=None, nearest_obstacle=None, safety_margin=None,
                       avg_cost=None, total_cost=None)
            return res

        # Calculate blocked cells using scenario robot radius
        robot_radius = scenario.get("robot_radius", 0.3) if scenario else 0.3
        blocked_cells = 0
        if self.cost_arr is not None and self.costmap is not None:
            resolution = self.costmap.info.resolution
            origin_x = self.costmap.info.origin.position.x
            origin_y = self.costmap.info.origin.position.y
            
            # Find obstacles (cost >= 100)
            obs_y, obs_x = np.where(self.cost_arr >= 100)
            obs_points = np.array([(origin_x + c * resolution, origin_y + r * resolution) for r, c in zip(obs_y, obs_x)])
            
            for x, y in coords:
                if len(obs_points) > 0:
                    dists = np.sqrt((x - obs_points[:, 0])**2 + (y - obs_points[:, 1])**2)
                    min_dist = np.min(dists)
                    if min_dist < robot_radius:
                        blocked_cells += 1
                        
        res["blocked_cells"] = blocked_cells
        res["safety_margin"] = res["nearest_obstacle"]
        res["success"] = res["success"] and blocked_cells == 0
        return res

    def print_general_metrics(self, metrics, compilation_time):
        print("\n================ MANUAL RUN PATH METRICS ================")
        print(f"Test Name:       {self.test_name}")
        print(f"Status:          {record_outcome(metrics)}")
        print(f"Planning Time:   {format_metric(compilation_time, '.4f')} s")
        print(f"Path Length:     {format_metric(metrics['length'], '.2f')} m")
        print(f"Euclidean Ratio: {format_metric(metrics['ratio'], '.3f')}")
        print(f"Turn Angles:     Min: {format_metric(metrics['min_angle'], '.1f')}° | Max: {format_metric(metrics['max_angle'], '.1f')}° | Avg: {format_metric(metrics['avg_angle'], '.1f')}°")
        print(f"Turn Radii:      Min: {format_metric(metrics['min_radius'], '.2f')} m | Max: {format_metric(metrics['max_radius'], '.2f')} m | Avg: {format_metric(metrics['avg_radius'], '.2f')} m")
        print(f"Blocked Cells:   {metrics['blocked_cells']}")
        print(f"Average Cost:    {format_metric(metrics['avg_cost'], '.1f')}")
        print(f"Total Cost:      {format_metric(metrics['total_cost'], '.1f')}")
        print(f"Safety Margin:   {format_metric(metrics['safety_margin'], '.2f')} m")
        print("=========================================================\n")

    def process_scenario_results(self, scenario):
        eval_res = self.static_eval
        plan_time = self.static_time
        
        ratio = eval_res["ratio"]
        
        s_time = s_cost = s_length = s_obstacle = s_replan = planner_score = None
        planner_evidence = (eval_res['success'] is True and is_number(plan_time)
                            and plan_time >= 0 and is_number(ratio) and is_number(eval_res['avg_cost']))
        if planner_evidence:
            # Replanning Success calculation
            replan_success_val = 0.0
            replan_path_coords = []
        
            dyn_obs = scenario.get("dynamic_obstacles", [])
            if len(dyn_obs) > 0:
                if self.replan_eval and self.replan_eval["success"] and self.replan_eval["blocked_cells"] == 0 and is_number(self.replan_time) and self.replan_time <= self.limits["timeout_planning_time"]:
                    replan_success_val = 100.0
                    replan_path_coords = [(p[0], p[1]) for p in self.replan_eval.get("coords", [])] if "coords" in self.replan_eval else []
                else:
                    replan_success_val = 0.0
            else:
                if eval_res["success"] and eval_res["blocked_cells"] == 0:
                    replan_success_val = 100.0
                
            # Scores formulas based on config limits
            s_success = 100.0 if eval_res["success"] else 0.0
        
            # S_time
            t_target = self.limits["target_planning_time"]
            t_timeout = self.limits["timeout_planning_time"]
            if plan_time <= t_target:
                s_time = 100.0
            elif plan_time >= t_timeout:
                s_time = 0.0
            else:
                s_time = 100.0 * (1.0 - (plan_time - t_target) / (t_timeout - t_target))
            
            # S_obstacle
            s_obstacle = 100.0 if (eval_res["blocked_cells"] == 0 and s_success == 100.0) else 0.0
        
            # S_cost
            s_cost = max(0.0, 100.0 - eval_res["avg_cost"])
        
            # S_length
            len_limit = self.limits["target_length_ratio"]
            if ratio <= 1.0:
                s_length = 100.0
            elif ratio >= len_limit:
                s_length = 0.0
            else:
                s_length = 100.0 * (1.0 - (ratio - 1.0) / (len_limit - 1.0))
            
            # S_replan
            s_replan = replan_success_val
        
            # Compute final weighted Path Planning Score
            w = self.weights
            planner_score = (w["success"] * s_success +
                             w["time"] * s_time +
                             w["obstacle"] * s_obstacle +
                             w["cost"] * s_cost +
                             w["length"] * s_length +
                             w["replan"] * s_replan)

        # Plan coordinates
        plan_coords = eval_res.get("coords", [])
        replan_path_coords = self.replan_eval.get("coords", []) if self.replan_eval else []

        # Calculate MPPI Controller metrics
        ctrl_res = calculate_control_metrics(self.cmd_history, self.odom_history, plan_coords, self.goal_pose)
        controller_score = ctrl_res["controller_score"]
        
        # Combined Overall System Score (50% Planning + 50% Control)
        overall_score = None
        if is_number(planner_score) and is_number(controller_score):
            overall_score = 0.50 * planner_score + 0.50 * controller_score
        invalid_path = (eval_res['success'] is False or
                        (self.replan_eval is not None and self.replan_eval['success'] is False))
        outcome = ('FAIL' if invalid_path else 'INCOMPLETE' if overall_score is None
                   else 'PASS' if overall_score >= self.pass_threshold else 'FAIL')
        
        scenario_res = {
            "id": scenario["id"],
            "success": eval_res["success"],
            "planning_time_s": plan_time,
            "planning_time_score": s_time,
            "path_length_m": eval_res["length"],
            "length_ratio": ratio,
            "length_score": s_length,
            "blocked_cells": eval_res["blocked_cells"],
            "obstacle_avoidance_score": s_obstacle,
            "avg_cost": eval_res["avg_cost"],
            "cost_score": s_cost,
            "safety_margin_m": eval_res["safety_margin"],
            "replanning_score": s_replan,
            "replanning_time_s": self.replan_time,
            
            # Scores
            "planner_score": planner_score,
            "controller_score": controller_score,
            "controller_execution_complete": False,
            "final_score": overall_score,
            "outcome": outcome,
            
            # Geometric Planning Metrics
            "min_angle": eval_res["min_angle"],
            "max_angle": eval_res["max_angle"],
            "avg_angle": eval_res["avg_angle"],
            "min_radius": eval_res["min_radius"],
            "max_radius": eval_res["max_radius"],
            "avg_radius": eval_res["avg_radius"],
            "path_cost": eval_res["total_cost"],

            # Control Metrics
            "mean_cte": ctrl_res["mean_cte"],
            "max_cte": ctrl_res["max_cte"],
            "rms_cte": ctrl_res["rms_cte"],
            "mean_linear_vel": ctrl_res["mean_linear_vel"],
            "max_linear_vel": ctrl_res["max_linear_vel"],
            "mean_angular_vel": ctrl_res["mean_angular_vel"],
            "max_angular_vel": ctrl_res["max_angular_vel"],
            "linear_jerk_std": ctrl_res["linear_jerk_std"],
            "angular_jerk_std": ctrl_res["angular_jerk_std"],
            "cmd_freq_hz": ctrl_res["cmd_freq_hz"],
            "goal_accuracy_m": ctrl_res["goal_accuracy_m"],
            "control_effort": ctrl_res.get("control_effort"),
            "steering_reversals": ctrl_res.get("steering_reversals"),
            "heading_rmse_deg": ctrl_res.get("heading_rmse_deg")
        }
        
        # Save temp JSON
        os.makedirs("reports", exist_ok=True)
        temp_file = f"reports/temp_{scenario['id']}.json"
        try:
            with open(temp_file, "w") as f:
                json.dump(scenario_res, f, indent=4, allow_nan=False)
            self.get_logger().info(f"Saved temp scenario results to {temp_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save temp results: {e}")
            
        # Print Scenario Summary
        print(f"\n================ SCENARIO RESULTS: {scenario['id']} ================")
        print(f"Status:             {outcome}")
        print(f"\n--- 🗺️  GLOBAL PLANNER (Smac Hybrid A*) ---")
        print(f"Planning Time:      {format_metric(plan_time, '.3f')} s  (Score: {format_metric(s_time, '.1f')}/100)")
        print(f"Path Length:        {format_metric(eval_res['length'], '.2f')} m  (Ratio: {format_metric(ratio, '.2f')}, Score: {format_metric(s_length, '.1f')}/100)")
        print(f"Turn Angles:        Min: {format_metric(eval_res['min_angle'], '.1f')}° | Max: {format_metric(eval_res['max_angle'], '.1f')}° | Avg: {format_metric(eval_res['avg_angle'], '.1f')}°")
        print(f"Turn Radii:         Min: {format_metric(eval_res['min_radius'], '.2f')} m | Max: {format_metric(eval_res['max_radius'], '.2f')} m | Avg: {format_metric(eval_res['avg_radius'], '.2f')} m")
        print(f"Blocked Cells:      {eval_res['blocked_cells']}  (Score: {format_metric(s_obstacle, '.1f')}/100)")
        print(f"Average Path Cost:  {format_metric(eval_res['avg_cost'], '.1f')}  (Score: {format_metric(s_cost, '.1f')}/100)")
        print(f"Total Path Cost:    {format_metric(eval_res['total_cost'], '.1f')}")
        print(f"Safety Margin:      {format_metric(eval_res['safety_margin'], '.2f')} m")
        print(f"Replanning:         Score: {format_metric(s_replan, '.1f')}/100 (Time: {format_metric(self.replan_time, '.3f')} s)")
        print(f"PLANNER SCORE:      {format_metric(planner_score, '.2f')} / 100")
        print(f"\n--- 🎮 LOCAL CONTROLLER (MPPI) ---")
        print(f"Cross-Track Error:  Mean: {format_metric(ctrl_res['mean_cte'], '.3f')} m | Max: {format_metric(ctrl_res['max_cte'], '.3f')} m | RMS: {format_metric(ctrl_res['rms_cte'], '.3f')} m")
        print(f"Velocity Profile:   Linear: Mean {format_metric(ctrl_res['mean_linear_vel'], '.2f')} m/s, Max {format_metric(ctrl_res['max_linear_vel'], '.2f')} m/s | Angular: Mean {format_metric(ctrl_res['mean_angular_vel'], '.2f')} rad/s, Max {format_metric(ctrl_res['max_angular_vel'], '.2f')} rad/s")
        print(f"Control Stability:  Lin Accel Std: {format_metric(ctrl_res['linear_jerk_std'], '.2f')} m/s² | Ang Accel Std: {format_metric(ctrl_res['angular_jerk_std'], '.2f')} rad/s²")
        print(f"Command Rate:       {format_metric(ctrl_res['cmd_freq_hz'], '.1f')} Hz")
        print(f"Last Goal Distance:      {format_metric(ctrl_res['goal_accuracy_m'], '.3f')} m error")
        print(f"Control Effort:     {format_metric(ctrl_res.get('control_effort'), '.2f')} (J_u integral)")
        print(f"Steering Chatter:   {ctrl_res.get('steering_reversals')} reversals")
        print(f"Heading RMSE:       {format_metric(ctrl_res.get('heading_rmse_deg'), '.2f')}°")
        print(f"CONTROLLER SCORE:   {format_metric(controller_score, '.2f')} / 100")
        print(f"-----------------------------------------------------")
        print(f"COMBINED SYSTEM SCORE: {format_metric(overall_score, '.2f')} / 100")
        print(f"FINAL OUTCOME:         {outcome} (Required: >= {self.pass_threshold}/100)")
        print(f"=====================================================\n")
        
        # Generate plot (coords are converted to normal tuples)
        plan_coords = []
        if "coords" in eval_res:
            plan_coords = eval_res["coords"]
        self.generate_comparison_plot(scenario, plan_coords, replan_path_coords, overall_score, scenario_res)

    def generate_comparison_plot(self, scenario, plan_coords, replan_coords, score, metrics):
        map_image = scenario["map_image"]
        ref_path = scenario["reference_path"]
        resolution = scenario["resolution"]
        origin = scenario["origin"]
        robot_radius = scenario.get("robot_radius", 0.3)
        
        if not os.path.exists(map_image):
            self.get_logger().warn(f"Map image {map_image} does not exist. Skipping plot.")
            return
            
        try:
            img = Image.open(map_image).convert("L")
            img_arr = np.array(img)
            
            fig, ax = plt.subplots(figsize=(10, 10))
            extent = [origin[0], origin[0] + img_arr.shape[1]*resolution, 
                      origin[1], origin[1] + img_arr.shape[0]*resolution]
            ax.imshow(img_arr, cmap='gray', origin='lower', extent=extent)
            
            # Reference Path (Green)
            ref_x = [p[0] for p in ref_path]
            ref_y = [p[1] for p in ref_path]
            ax.plot(ref_x, ref_y, 'g--', label='Reference Path (Perfect)', linewidth=2.5)
            
            # Planned Path (Red)
            if len(plan_coords) > 0:
                plan_x = [p[0] for p in plan_coords]
                plan_y = [p[1] for p in plan_coords]
                ax.plot(plan_x, plan_y, 'r-o', label='Planned Path', linewidth=2, markersize=4)
                
                # Draw footprint circles
                for idx, (px, py) in enumerate(plan_coords):
                    if idx % 5 == 0 or idx == len(plan_coords) - 1:
                        circle = plt.Circle((px, py), robot_radius, color='r', fill=False, linestyle=':', alpha=0.5)
                        ax.add_patch(circle)
            
            # Replan Path (Cyan)
            if len(replan_coords) > 0:
                rep_x = [p[0] for p in replan_coords]
                rep_y = [p[1] for p in replan_coords]
                ax.plot(rep_x, rep_y, 'c-^', label='Replanned Path (Detour)', linewidth=1.5, markersize=4)
                
            # Start/Goal
            ax.plot(scenario["start"][0], scenario["start"][1], 'bs', markersize=10, label='Start')
            ax.plot(scenario["goal"][0], scenario["goal"][1], 'r*', markersize=12, label='Goal')
            
            # Dynamic Obstacles
            for obs in scenario.get("dynamic_obstacles", []):
                obs_circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='orange', fill=True, alpha=0.6, label='Dynamic Obstacle')
                ax.add_patch(obs_circle)
                
            # Display scores box
            outcome = metrics.get("outcome", "FAIL")
            text_box = (
                f"Score: {format_metric(score, '.1f')} / 100\n"
                f"Outcome: {outcome}\n"
                f"Success: {'YES' if metrics['success'] is True else 'NO' if metrics['success'] is False else 'N/A'}\n"
                f"Time: {format_metric(metrics['planning_time_s'], '.3f')} s\n"
                f"Length Ratio: {format_metric(metrics['length_ratio'], '.2f')}\n"
                f"Safety Margin: {format_metric(metrics['safety_margin_m'], '.2f')} m\n"
                f"Blocked Cells: {metrics['blocked_cells']}"
            )
            bg_color = '#d4edda' if outcome == 'PASS' else '#f8d7da'
            props = dict(boxstyle='round', facecolor=bg_color, alpha=0.9)
            ax.text(0.05, 0.95, text_box, transform=ax.transAxes, fontsize=12,
                    verticalalignment='top', bbox=props)
            
            ax.set_title(f"Benchmark Results: {scenario['id']}")
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.legend(loc='lower right')
            ax.grid(True, alpha=0.3)
            
            out_file = f"reports/result_scenario_{scenario['id']}.png"
            plt.savefig(out_file, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Saved result comparison plot to: {out_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate comparison plot: {e}")

    def save_final_reports(self):
        if not self.history:
            print("[INFO] [benchmarking_node]: No runs recorded. Skipping report generation.")
            return
            
        os.makedirs("reports", exist_ok=True)
        
        # Group runs by test_name
        grouped_runs = {}
        for r in self.history:
            name = r.get("test_name", "general")
            # Sanitize name for clean filenames
            safe_name = name.lower().replace(" ", "_").replace("/", "_").strip()
            if not safe_name:
                safe_name = "general"
            if safe_name not in grouped_runs:
                grouped_runs[safe_name] = []
            grouped_runs[safe_name].append(r)
            
        for safe_name, runs in grouped_runs.items():
            csv_path = f"reports/{safe_name}.csv"
            html_path = f"reports/{safe_name}.html"
            pdf_path = f"reports/{safe_name}.pdf"
            
            # 1. Generate CSV
            try:
                if generate_csv_report(runs, csv_path):
                    print(f"[INFO] [benchmarking_node]: Successfully generated CSV report at: {csv_path}")
                else:
                    print("[WARN] [benchmarking_node]: CSV report unavailable or export failed")
            except Exception as e:
                print(f"[ERROR] [benchmarking_node]: Failed to generate CSV report for {safe_name}: {e}")
                
            # 2. Generate HTML
            try:
                if generate_html_report(runs, html_path):
                    print(f"[INFO] [benchmarking_node]: Successfully generated HTML report at: {html_path}")
                else:
                    print("[WARN] [benchmarking_node]: HTML report unavailable or export failed")
            except Exception as e:
                print(f"[ERROR] [benchmarking_node]: Failed to generate HTML report for {safe_name}: {e}")

            # 3. Generate PDF
            try:
                if generate_pdf_report(runs, pdf_path):
                    print(f"[INFO] [benchmarking_node]: Successfully generated PDF report at: {pdf_path}")
                else:
                    print("[WARN] [benchmarking_node]: PDF report unavailable or export failed")
            except Exception as e:
                print(f"[ERROR] [benchmarking_node]: Failed to generate PDF report for {safe_name}: {e}")

def main(args=None):
    import signal
    rclpy.init(args=args)
    node = BenchmarkingNode()
    
    # Register SIGTERM to cleanly save reports and shutdown
    def sigterm_handler(signum, frame):
        node.save_final_reports()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGTERM, sigterm_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_final_reports()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
