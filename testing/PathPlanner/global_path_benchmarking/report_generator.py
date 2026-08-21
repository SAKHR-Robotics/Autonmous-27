#!/usr/bin/env python3

import os
import math
import time
import json
import csv
import numpy as np

# Try importing ReportLab components
try:
    from reportlab.lib.pagesizes import letter
    from reportlab.lib import colors
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, Image as RLImage
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.pdfgen import canvas
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False

def calculate_path_metrics(coords, start_pose=None, goal_pose=None, costmap=None):
    """
    Computes all path metrics requested by the user:
    - Distance covered
    - Ratio of distance covered vs Euclidean distance
    - Min, max, and average turn angles (degrees)
    - Min, max, and average turn radii (meters)
    - Nearest distance to a wall or obstacle
    - Total and average path cost
    """
    default_metrics = {
        "success": False,
        "length": 0.0,
        "euclidean_dist": 0.0,
        "ratio": 1.0,
        "min_angle": 0.0,
        "max_angle": 0.0,
        "avg_angle": 0.0,
        "min_radius": 0.0,
        "max_radius": 0.0,
        "avg_radius": 0.0,
        "nearest_obstacle": 10.0,
        "total_cost": 0.0,
        "avg_cost": 0.0
    }

    if not coords or len(coords) < 2:
        return default_metrics

    # 2. Distance covered (Path length)
    length = 0.0
    for p1, p2 in zip(coords[:-1], coords[1:]):
        length += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    # 3. Euclidean distance
    if start_pose is not None:
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
    else:
        start_x, start_y = coords[0]

    if goal_pose is not None:
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
    else:
        goal_x, goal_y = coords[-1]

    euclidean_dist = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
    ratio = length / euclidean_dist if euclidean_dist > 0.0 else 1.0

    # 4. Turn angles and turn radii
    angles = []
    radii = []

    if len(coords) >= 3:
        for i in range(1, len(coords) - 1):
            A = coords[i-1]
            B = coords[i]
            C = coords[i+1]

            # Vector AB and BC
            v1 = (B[0] - A[0], B[1] - A[1])
            v2 = (C[0] - B[0], C[1] - B[1])

            m1 = math.sqrt(v1[0]**2 + v1[1]**2)
            m2 = math.sqrt(v2[0]**2 + v2[1]**2)

            if m1 < 1e-6 or m2 < 1e-6:
                continue

            # Turn angle using dot product
            dot = v1[0]*v2[0] + v1[1]*v2[1]
            cos_theta = max(-1.0, min(1.0, dot / (m1 * m2)))
            theta = math.acos(cos_theta)
            theta_deg = math.degrees(theta)
            angles.append(theta_deg)

            # Turn radius using circumradius of triangle ABC
            area = 0.5 * abs(A[0]*(B[1] - C[1]) + B[0]*(C[1] - A[1]) + C[0]*(A[1] - B[1]))
            if area > 1e-5:
                ac_dist = math.sqrt((C[0] - A[0])**2 + (C[1] - A[1])**2)
                R = (m1 * m2 * ac_dist) / (4.0 * area)
                
                # Only register turn radius for actual turns (turn angle > 1.0 degree)
                # and cap extremely large values (almost straight line) at 100.0m
                if theta_deg > 1.0 and R < 100.0:
                    radii.append(R)

    # Summarize angles
    if angles:
        min_angle = float(np.min(angles))
        max_angle = float(np.max(angles))
        avg_angle = float(np.mean(angles))
    else:
        min_angle, max_angle, avg_angle = 0.0, 0.0, 0.0

    # Summarize radii
    if radii:
        min_radius = float(np.min(radii))
        max_radius = float(np.max(radii))
        avg_radius = float(np.mean(radii))
    else:
        min_radius, max_radius, avg_radius = 0.0, 0.0, 0.0

    # 5. Nearest distance to obstacle & Path cost
    nearest_obstacle = 10.0  # Default safe clearance fallback
    total_cost = 0.0

    if costmap is not None:
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height
        cost_arr = np.array(costmap.data, dtype=np.int8).reshape((height, width))

        # Find all obstacle coordinates (cost >= 100)
        obs_y_indices, obs_x_indices = np.where(cost_arr >= 100)
        obs_points = []
        for r, c in zip(obs_y_indices, obs_x_indices):
            x_m = origin_x + c * resolution
            y_m = origin_y + r * resolution
            obs_points.append((x_m, y_m))
        obs_points = np.array(obs_points)

        min_obs_dist = float('inf')
        for px, py in coords:
            # Map cost
            col = int((px - origin_x) / resolution)
            row = int((py - origin_y) / resolution)
            if 0 <= row < height and 0 <= col < width:
                total_cost += float(cost_arr[row, col])
            else:
                total_cost += 100.0

            # Clearance distance
            if len(obs_points) > 0:
                dists = np.sqrt((px - obs_points[:, 0])**2 + (py - obs_points[:, 1])**2)
                d = np.min(dists)
                if d < min_obs_dist:
                    min_obs_dist = d

        if min_obs_dist != float('inf'):
            nearest_obstacle = min_obs_dist

    avg_cost = total_cost / len(coords) if len(coords) > 0 else 0.0

    return {
        "success": True,
        "length": length,
        "euclidean_dist": euclidean_dist,
        "ratio": ratio,
        "min_angle": min_angle,
        "max_angle": max_angle,
        "avg_angle": avg_angle,
        "min_radius": min_radius,
        "max_radius": max_radius,
        "avg_radius": avg_radius,
        "nearest_obstacle": nearest_obstacle,
        "total_cost": total_cost,
        "avg_cost": avg_cost
    }

def aggregate_statistics(history):
    stats = {
        "success_rate": 0.0,
        "comp_time": {"min": 0.0, "avg": 0.0, "max": 0.0},
        "turn_angle": {"min": 0.0, "avg": 0.0, "max": 0.0},
        "turn_radius": {"min": 0.0, "avg": 0.0, "max": 0.0},
        "length_ratio": {"min": 0.0, "avg": 0.0, "max": 0.0},
        "clearance": {"min": 0.0, "avg": 0.0, "max": 0.0},
        "path_cost": {"min": 0.0, "avg": 0.0, "max": 0.0}
    }

    if not history:
        return stats

    total_runs = len(history)
    successful_runs = [r for r in history if r.get("success", False)]
    success_count = len(successful_runs)
    stats["success_rate"] = (success_count / total_runs) * 100.0

    comp_times = [r.get("planning_time_s", 0.0) for r in history]
    stats["comp_time"] = {
        "min": float(np.min(comp_times)),
        "avg": float(np.mean(comp_times)),
        "max": float(np.max(comp_times))
    }

    if success_count > 0:
        ratios = [r.get("length_ratio", 1.0) for r in successful_runs]
        clearances = [r.get("safety_margin_m", 10.0) for r in successful_runs]
        costs = [r.get("path_cost", 0.0) for r in successful_runs]

        min_angles = [r.get("min_angle", 0.0) for r in successful_runs]
        max_angles = [r.get("max_angle", 0.0) for r in successful_runs]
        avg_angles = [r.get("avg_angle", 0.0) for r in successful_runs]

        min_radii = [r.get("min_radius", 0.0) for r in successful_runs]
        max_radii = [r.get("max_radius", 0.0) for r in successful_runs]
        avg_radii = [r.get("avg_radius", 0.0) for r in successful_runs]

        stats["length_ratio"] = {
            "min": float(np.min(ratios)),
            "avg": float(np.mean(ratios)),
            "max": float(np.max(ratios))
        }
        stats["clearance"] = {
            "min": float(np.min(clearances)),
            "avg": float(np.mean(clearances)),
            "max": float(np.max(clearances))
        }
        stats["path_cost"] = {
            "min": float(np.min(costs)),
            "avg": float(np.mean(costs)),
            "max": float(np.max(costs))
        }
        stats["turn_angle"] = {
            "min": float(np.min(min_angles)),
            "avg": float(np.mean(avg_angles)),
            "max": float(np.max(max_angles))
        }
        stats["turn_radius"] = {
            "min": float(np.min(min_radii)),
            "avg": float(np.mean(avg_radii)),
            "max": float(np.max(max_radii))
        }

    return stats

def generate_html_report(history, output_path):
    """
    Generates a premium HTML report matching 'mostafa report .pdf' aesthetics perfectly.
    Supports both individual scenario reports and aggregate benchmark reports.
    """
    if not history:
        return False

    stats = aggregate_statistics(history)
    overall_pass = stats["success_rate"] >= 85.0
    status_text = "OVERALL SYSTEM EVALUATION: SYSTEM STABLE / PERFORMANCE ENVELOPE MAINTAINED" if overall_pass else "OVERALL SYSTEM EVALUATION: CRITICAL SIGNALS DETECTED / ENVELOPE BREACHED"
    status_border = "#10B981" if overall_pass else "#EF4444"

    # Define warning threshold boundaries for summary cards
    success_crit = stats["success_rate"] < 100.0
    time_crit = stats["comp_time"]["avg"] > 2.0
    angle_crit = stats["turn_angle"]["max"] > 45.0
    clearance_crit = stats["clearance"]["min"] < 0.3

    card_borders = [
        "#EF4444" if success_crit else "#94A3B8",
        "#EF4444" if time_crit else "#94A3B8",
        "#EF4444" if angle_crit else "#94A3B8",
        "#EF4444" if clearance_crit else "#94A3B8"
    ]
    card_text_colors = [
        "#EF4444" if success_crit else "#1D2939",
        "#EF4444" if time_crit else "#1D2939",
        "#EF4444" if angle_crit else "#1D2939",
        "#EF4444" if clearance_crit else "#1D2939"
    ]

    is_aggregate = len(set(r.get("test_name", "general") for r in history)) > 1

    # CSS styles
    css_styles = f"""
        body {{
            font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif;
            color: #1E293B;
            margin: 0;
            padding: 40px;
            background-color: #F8FAFC;
        }}
        .container {{
            max-width: 1000px;
            margin: 0 auto;
            background: #FFFFFF;
            padding: 40px;
            box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1), 0 2px 4px -1px rgba(0,0,0,0.06);
            border-radius: 4px;
        }}
        .header {{
            background-color: #0F172A;
            padding: 30px 40px;
            margin: -40px -40px 25px -40px;
            border-bottom: 5px solid #EF4444;
        }}
        .header-top {{
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 1.5px;
            color: #94A3B8;
            margin-bottom: 5px;
        }}
        .header-title {{
            font-size: 32px;
            font-weight: 800;
            color: #FFFFFF;
            margin: 0;
            letter-spacing: -0.5px;
        }}
        .header-title span {{
            color: #EF4444;
        }}
        .header-subtitle {{
            font-size: 13px;
            color: #CBD5E1;
            margin-top: 5px;
            margin-bottom: 0;
            font-weight: 500;
        }}
        .status-box {{
            background-color: #0F172A;
            border: 4px solid {status_border};
            color: #FFFFFF;
            padding: 15px;
            border-radius: 4px;
            font-weight: bold;
            font-size: 14px;
            text-align: center;
            letter-spacing: 0.5px;
            margin-bottom: 30px;
        }}
        .section-title {{
            font-size: 16px;
            font-weight: bold;
            color: #0F172A;
            margin-top: 30px;
            margin-bottom: 15px;
            border-left: 4px solid #EF4444;
            padding-left: 10px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }}
        .cards-container {{
            display: flex;
            justify-content: space-between;
            margin-bottom: 30px;
            gap: 15px;
        }}
        .card {{
            flex: 1;
            background: #FFFFFF;
            border: 1px solid #E2E8F0;
            border-radius: 4px;
            padding: 15px;
            text-align: center;
            box-shadow: 0 1px 3px rgba(0,0,0,0.05);
        }}
        .card-label {{
            font-size: 10px;
            font-weight: bold;
            color: #64748B;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 8px;
            height: 24px;
            display: flex;
            align-items: center;
            justify-content: center;
        }}
        .card-value {{
            font-size: 20px;
            font-weight: bold;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-bottom: 30px;
            font-size: 13px;
            border-bottom: 4px solid #EF4444;
        }}
        th {{
            background-color: #0F172A;
            color: white;
            text-align: left;
            padding: 10px 12px;
            font-weight: bold;
            text-transform: uppercase;
            font-size: 11px;
            letter-spacing: 0.5px;
            border: 1px solid #334155;
        }}
        td {{
            padding: 10px 12px;
            border: 1px solid #E2E8F0;
        }}
        tr:nth-child(even) td {{
            background-color: #F8FAFC;
        }}
        .text-right {{
            text-align: right;
        }}
        .badge {{
            padding: 3px 8px;
            font-weight: bold;
            font-size: 11px;
            border-radius: 3px;
            text-transform: uppercase;
        }}
        .badge-pass {{
            background-color: #C6F6D5;
            color: #22543D;
        }}
        .badge-fail {{
            background-color: #FED7D7;
            color: #9B2C2C;
        }}
        .text-warning {{
            color: #EF4444;
            font-weight: bold;
        }}
        .footer {{
            margin-top: 40px;
            border-top: 1px solid #E2E8F0;
            padding-top: 15px;
            font-size: 11px;
            color: #A0AEC0;
            display: flex;
            justify-content: space-between;
        }}
        .plot-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }}
        .plot-card {{
            background: #FFFFFF;
            border: 1px solid #CBD5E0;
            border-radius: 4px;
            padding: 10px;
            text-align: center;
            box-shadow: 0 2px 4px rgba(0,0,0,0.05);
        }}
        .plot-card img {{
            max-width: 100%;
            height: auto;
            border-radius: 2px;
        }}
        .plot-title {{
            font-size: 12px;
            font-weight: bold;
            margin-top: 8px;
            color: #0F172A;
        }}
    """

    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ASU ROAR — Path Planner Diagnostics Report</title>
    <style>
        {css_styles}
    </style>
</head>
<body>
    <div class="container">
        <!-- HEADER -->
        <div class="header">
            <div class="header-top">ASU ROAR — Controller Diagnostics Report</div>
            <h1 class="header-title">ASU <span>ROAR</span></h1>
            <p class="header-subtitle">Continuous Global Planner Performance & Path Geometry Evaluation</p>
        </div>

        <!-- STATUS -->
        <div class="status-box">
            {status_text}
        </div>

        <!-- SUMMARY CARDS -->
        <div class="section-title">System Summary Diagnostics</div>
        <div class="cards-container">
            <div class="card" style="border-left: 4px solid {card_borders[0]};">
                <div class="card-label">Path Success Rate</div>
                <div class="card-value" style="color: {card_text_colors[0]};">{stats["success_rate"]:.1f}%</div>
            </div>
            <div class="card" style="border-left: 4px solid {card_borders[1]};">
                <div class="card-label">Signal Smoothness (Avg Time)</div>
                <div class="card-value" style="color: {card_text_colors[1]};">{stats["comp_time"]["avg"]:.4f} s</div>
            </div>
            <div class="card" style="border-left: 4px solid {card_borders[2]};">
                <div class="card-label">Max Turn Angle</div>
                <div class="card-value" style="color: {card_text_colors[2]};">{stats["turn_angle"]["max"]:.1f}°</div>
            </div>
            <div class="card" style="border-left: 4px solid {card_borders[3]};">
                <div class="card-label">Min Obstacle Clearance</div>
                <div class="card-value" style="color: {card_text_colors[3]};">{stats["clearance"]["min"]:.2f} m</div>
            </div>
        </div>
"""

    if is_aggregate:
        # AGGREGATE SUMMARY LAYOUT
        html_content += """
        <!-- STATISTICAL ANALYSIS TABLE -->
        <div class="section-title">Scenario Performance Breakdown</div>
        <table>
            <thead>
                <tr>
                    <th>Scenario ID</th>
                    <th>Status</th>
                    <th class="text-right">Success</th>
                    <th class="text-right">Planning Time</th>
                    <th class="text-right">Distance</th>
                    <th class="text-right">Ratio</th>
                    <th class="text-right">Max Turn Angle</th>
                    <th class="text-right">Min Clearance</th>
                    <th class="text-right">Path Cost</th>
                </tr>
            </thead>
            <tbody>
        """
        for r in history:
            s_name = r.get("test_name", "general")
            outcome = r.get("outcome", "PASS" if r.get("success", False) else "FAIL")
            badge_class = "badge-pass" if outcome == "PASS" else "badge-fail"
            success_str = "YES" if r.get("success", False) else "NO"
            
            html_content += f"""
                <tr>
                    <td><b>{s_name}</b></td>
                    <td><span class="badge {badge_class}">{outcome}</span></td>
                    <td class="text-right">{success_str}</td>
                    <td class="text-right">{r.get("planning_time_s", 0.0):.4f} s</td>
                    <td class="text-right">{r.get("path_length_m", 0.0):.2f} m</td>
                    <td class="text-right">{r.get("length_ratio", 1.0):.3f}</td>
                    <td class="text-right">{r.get("max_angle", 0.0):.1f}°</td>
                    <td class="text-right">{r.get("safety_margin_m", 0.0):.2f} m</td>
                    <td class="text-right">{r.get("path_cost", 0.0):.1f}</td>
                </tr>
            """
        html_content += """
            </tbody>
        </table>
        
        <!-- PATH VISUALIZATIONS GRID -->
        <div class="section-title">Path Visualizations</div>
        <div class="plot-grid">
        """
        for r in history:
            s_name = r.get("test_name", "general")
            safe_s_name = s_name.lower().replace(" ", "_").replace("/", "_").strip()
            plot_file = f"result_scenario_{safe_s_name}.png"
            html_content += f"""
            <div class="plot-card">
                <img src="{plot_file}" alt="{s_name}" onerror="this.style.display='none'">
                <div class="plot-title">{s_name}</div>
            </div>
            """
        html_content += """
        </div>
        """
    else:
        # DETAILED SINGLE-SCENARIO LAYOUT
        has_replan = len(history) >= 2
        r_static = history[0]
        r_replan = history[1] if has_replan else None

        test_name = r_static.get("test_name", "general")
        safe_name = test_name.lower().replace(" ", "_").replace("/", "_").strip()
        if not safe_name:
            safe_name = "general"

        html_content += f"""
        <!-- STATISTICAL ANALYSIS TABLE -->
        <div class="section-title">Measured Path Metrics</div>
        <table>
            <thead>
                <tr>
                    <th>Metric</th>
                    <th class="text-right">Static Path</th>
                    {"<th class='text-right'>Replanned Path (Detour)</th>" if has_replan else ""}
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><b>Path Compilation Time (s)</b></td>
                    <td class="text-right">{r_static.get("planning_time_s", 0.0):.4f}</td>
                    {f"<td class='text-right'>{r_replan.get('planning_time_s', 0.0):.4f}</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Distance Covered (m)</b></td>
                    <td class="text-right">{r_static.get("path_length_m", 0.0):.2f}</td>
                    {f"<td class='text-right'>{r_replan.get('path_length_m', 0.0):.2f}</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Path Length / Euclidean Ratio</b></td>
                    <td class="text-right">{r_static.get("length_ratio", 1.0):.3f}</td>
                    {f"<td class='text-right'>{r_replan.get('length_ratio', 1.0):.3f}</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Turn Angles</b></td>
                    <td class="text-right">Min: {r_static.get("min_angle", 0.0):.1f}° | Max: {r_static.get("max_angle", 0.0):.1f}° | Avg: {r_static.get("avg_angle", 0.0):.1f}°</td>
                    {f"<td class='text-right'>Min: {r_replan.get('min_angle', 0.0):.1f}° | Max: {r_replan.get('max_angle', 0.0):.1f}° | Avg: {r_replan.get('avg_angle', 0.0):.1f}°</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Turn Radii</b></td>
                    <td class="text-right">Min: {r_static.get("min_radius", 0.0):.2f} m | Max: {r_static.get("max_radius", 0.0):.2f} m | Avg: {r_static.get("avg_radius", 0.0):.2f} m</td>
                    {f"<td class='text-right'>Min: {r_replan.get('min_radius', 0.0):.2f} m | Max: {r_replan.get('max_radius', 0.0):.2f} m | Avg: {r_replan.get('avg_radius', 0.0):.2f} m</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Nearest Distance to Obstacle (m)</b></td>
                    <td class="text-right">{r_static.get("safety_margin_m", 10.0):.3f}</td>
                    {f"<td class='text-right'>{r_replan.get('safety_margin_m', 10.0):.3f}</td>" if has_replan else ""}
                </tr>
                <tr>
                    <td><b>Path Cost</b></td>
                    <td class="text-right">{r_static.get("path_cost", 0.0):.1f}</td>
                    {f"<td class='text-right'>{r_replan.get('path_cost', 0.0):.1f}</td>" if has_replan else ""}
                </tr>
            </tbody>
        </table>

        <!-- PATH VISUALIZATION -->
        <div class="section-title">Path Visualization</div>
        <div style="text-align: center; margin-bottom: 30px;">
            <img src="result_scenario_{safe_name}.png" style="max-width: 500px; border: 1px solid #CBD5E0; border-radius: 4px; box-shadow: 0 2px 4px rgba(0,0,0,0.05);" alt="Path Plot">
        </div>
        """

    html_content += f"""
        <!-- FOOTER -->
        <div class="footer">
            <span>Report generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}</span>
            <span>ASU ROAR Benchmarking Utility</span>
        </div>
    </div>
</body>
</html>
"""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html_content)
    return True

def generate_pdf_report(history, output_path):
    """
    Generates a ReportLab PDF matching 'mostafa report .pdf' aesthetics perfectly.
    """
    if not REPORTLAB_AVAILABLE:
        print("[WARNING] ReportLab is not available. Skipping PDF generation.")
        return False

    stats = aggregate_statistics(history)
    overall_pass = stats["success_rate"] >= 85.0
    status_text = "OVERALL SYSTEM EVALUATION: SYSTEM STABLE / PERFORMANCE ENVELOPE MAINTAINED" if overall_pass else "OVERALL SYSTEM EVALUATION: CRITICAL SIGNALS DETECTED / ENVELOPE BREACHED"
    status_color_hex = "#EF4444" if not overall_pass else "#10B981"

    # Define warning threshold boundaries for summary cards
    success_crit = stats["success_rate"] < 100.0
    time_crit = stats["comp_time"]["avg"] > 2.0
    angle_crit = stats["turn_angle"]["max"] > 45.0
    clearance_crit = stats["clearance"]["min"] < 0.3

    card_borders = [
        "#EF4444" if success_crit else "#CBD5E0",
        "#EF4444" if time_crit else "#CBD5E0",
        "#EF4444" if angle_crit else "#CBD5E0",
        "#EF4444" if clearance_crit else "#CBD5E0"
    ]
    card_text_colors = [
        "#EF4444" if success_crit else "#1D2939",
        "#EF4444" if time_crit else "#1D2939",
        "#EF4444" if angle_crit else "#1D2939",
        "#EF4444" if clearance_crit else "#1D2939"
    ]

    class NumberedCanvas(canvas.Canvas):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._saved_page_states = []

        def showPage(self):
            self._saved_page_states.append(dict(self.__dict__))
            self._startPage()

        def save(self):
            num_pages = len(self._saved_page_states)
            for state in self._saved_page_states:
                self.__dict__.update(state)
                self.draw_page_decorations(num_pages)
                super().showPage()
            super().save()

        def draw_page_decorations(self, page_count):
            self.saveState()
            
            # Simple top separator bar
            self.setStrokeColor(colors.HexColor("#CBD5E0"))
            self.setLineWidth(0.5)
            self.line(54, 755, 558, 755)
            
            # Footer Line
            self.line(54, 50, 558, 50)
            
            # Footer Text
            self.setFont("Helvetica", 8)
            self.setFillColor(colors.HexColor("#718096"))
            self.drawString(54, 38, f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}  |  ASU ROAR Benchmarking Node")
            self.drawRightString(558, 38, f"Page {self._pageNumber} of {page_count}")
            
            self.restoreState()

    doc = SimpleDocTemplate(
        output_path,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=72
    )

    styles = getSampleStyleSheet()

    # Define custom ParagraphStyles
    header_top_style = ParagraphStyle(
        'HeaderTop',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=9,
        textColor=colors.HexColor("#94A3B8")
    )
    header_title_style = ParagraphStyle(
        'HeaderTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=24,
        leading=28,
        textColor=colors.white
    )
    header_subtitle_style = ParagraphStyle(
        'HeaderSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9.5,
        leading=13,
        textColor=colors.HexColor("#CBD5E1")
    )

    h1_style = ParagraphStyle(
        'SectionHeader',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=10,
        leading=13,
        textColor=colors.HexColor("#0F172A"),
        spaceBefore=14,
        spaceAfter=8,
        keepWithNext=True
    )

    status_p_style = ParagraphStyle(
        'StatusText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9,
        leading=12,
        textColor=colors.white,
        alignment=1 # Center
    )

    card_label_style = ParagraphStyle(
        'CardLabelText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=9,
        textColor=colors.HexColor("#64748B"),
        alignment=1
    )

    table_header_style = ParagraphStyle(
        'TableHeaderText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=9,
        textColor=colors.white
    )

    table_header_right_style = ParagraphStyle(
        'TableHeaderRightText',
        parent=table_header_style,
        alignment=2 # Right
    )

    table_cell_style = ParagraphStyle(
        'TableCellText',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8,
        leading=10,
        textColor=colors.HexColor("#1E293B")
    )

    table_cell_bold_style = ParagraphStyle(
        'TableCellBoldText',
        parent=table_cell_style,
        fontName='Helvetica-Bold'
    )

    table_cell_right_style = ParagraphStyle(
        'TableCellRightText',
        parent=table_cell_style,
        alignment=2 # Right
    )

    badge_pass_style = ParagraphStyle(
        'BadgePassText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7,
        leading=9,
        textColor=colors.HexColor("#22543D"),
        alignment=1
    )

    badge_fail_style = ParagraphStyle(
        'BadgeFailText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7,
        leading=9,
        textColor=colors.HexColor("#9B2C2C"),
        alignment=1
    )

    story = []

    # 1. DARK HEADER BLOCK WITH LOGO & TITLES (Margin-to-Margin Table)
    header_data = [[
        [
            Paragraph("ASU ROAR — PATH PLANNER PERFORMANCE & DIAGNOSTICS REPORT", header_top_style),
            Spacer(1, 4),
            Paragraph("ASU <font color='#EF4444'>ROAR</font>", header_title_style),
            Spacer(1, 4),
            Paragraph("Continuous Global Planner Performance & Path Geometry Evaluation", header_subtitle_style)
        ]
    ]]
    header_table = Table(header_data, colWidths=[504])
    header_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#0F172A")),
        ('TOPPADDING', (0,0), (-1,-1), 16),
        ('BOTTOMPADDING', (0,0), (-1,-1), 16),
        ('LEFTPADDING', (0,0), (-1,-1), 16),
        ('RIGHTPADDING', (0,0), (-1,-1), 16),
        ('LINEBELOW', (0,0), (-1,-1), 4.5, colors.HexColor("#EF4444")),
    ]))
    story.append(header_table)
    story.append(Spacer(1, 15))

    # 2. STATUS BANNER
    status_data = [[Paragraph(status_text, status_p_style)]]
    status_table = Table(status_data, colWidths=[504])
    status_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#0F172A")),
        ('BOX', (0,0), (-1,-1), 3.5, colors.HexColor(status_color_hex)),
        ('TOPPADDING', (0,0), (-1,-1), 10),
        ('BOTTOMPADDING', (0,0), (-1,-1), 10),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(status_table)
    story.append(Spacer(1, 10))

    # 3. SUMMARY DIAGNOSTICS CARDS
    story.append(Paragraph("System Summary Diagnostics", h1_style))
    
    # 4 distinct card boxes within a single grid
    cards_data = [
        [
            Paragraph("PATH SUCCESS RATE", card_label_style),
            Paragraph("SIGNAL SMOOTHNESS", card_label_style),
            Paragraph("MAX TURN ANGLE", card_label_style),
            Paragraph("MIN OBSTACLE CLEARANCE", card_label_style)
        ],
        [
            Paragraph(f"<font color='{card_text_colors[0]}'><b>{stats['success_rate']:.1f}%</b></font>", ParagraphStyle('C1', parent=styles['Normal'], fontSize=14, leading=16, alignment=1)),
            Paragraph(f"<font color='{card_text_colors[1]}'><b>{stats['comp_time']['avg']:.4f} s</b></font>", ParagraphStyle('C2', parent=styles['Normal'], fontSize=14, leading=16, alignment=1)),
            Paragraph(f"<font color='{card_text_colors[2]}'><b>{stats['turn_angle']['max']:.1f}°</b></font>", ParagraphStyle('C3', parent=styles['Normal'], fontSize=14, leading=16, alignment=1)),
            Paragraph(f"<font color='{card_text_colors[3]}'><b>{stats['clearance']['min']:.2f} m</b></font>", ParagraphStyle('C4', parent=styles['Normal'], fontSize=14, leading=16, alignment=1))
        ]
    ]
    cards_table = Table(cards_data, colWidths=[126, 126, 126, 126])
    
    # Base cards grid styles
    cards_style = [
        ('BACKGROUND', (0,0), (-1,-1), colors.white),
        ('BOX', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
        ('INNERGRID', (0,0), (-1,-1), 0.5, colors.HexColor("#E2E8F0")),
        ('TOPPADDING', (0,0), (-1,-1), 8),
        ('BOTTOMPADDING', (0,0), (-1,-1), 8),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]
    # Highlight left borders of individual cells
    for i in range(4):
        cards_style.append(('LINELEFT', (i, 0), (i, 1), 3.5, colors.HexColor(card_borders[i])))
        
    cards_table.setStyle(TableStyle(cards_style))
    story.append(cards_table)
    story.append(Spacer(1, 10))

    is_aggregate = len(set(r.get("test_name", "general") for r in history)) > 1

    if is_aggregate:
        # AGGREGATE SUMMARY LAYOUT
        story.append(Paragraph("Scenario Performance Breakdown", h1_style))
        
        headers = [
            Paragraph("Scenario ID", table_header_style),
            Paragraph("Status", table_header_style),
            Paragraph("Planning Time", table_header_right_style),
            Paragraph("Distance", table_header_right_style),
            Paragraph("Ratio", table_header_right_style),
            Paragraph("Max Turn", table_header_right_style),
            Paragraph("Min Clearance", table_header_right_style),
            Paragraph("Cost", table_header_right_style)
        ]
        col_widths = [104, 55, 65, 55, 45, 55, 65, 60]
        
        table_data = [headers]
        for r in history:
            s_name = r.get("test_name", "general")
            outcome = r.get("outcome", "PASS" if r.get("success", False) else "FAIL")
            
            badge_text = f"<b>{outcome}</b>"
            badge_para = Paragraph(badge_text, badge_pass_style if outcome == "PASS" else badge_fail_style)
            
            table_data.append([
                Paragraph(f"<b>{s_name}</b>", table_cell_bold_style),
                badge_para,
                Paragraph(f"{r.get('planning_time_s', 0.0):.3f} s", table_cell_right_style),
                Paragraph(f"{r.get('path_length_m', 0.0):.2f} m", table_cell_right_style),
                Paragraph(f"{r.get('length_ratio', 1.0):.2f}", table_cell_right_style),
                Paragraph(f"{r.get('max_angle', 0.0):.1f}°", table_cell_right_style),
                Paragraph(f"{r.get('safety_margin_m', 0.0):.2f} m", table_cell_right_style),
                Paragraph(f"{r.get('path_cost', 0.0):.1f}", table_cell_right_style)
            ])
            
        summary_table = Table(table_data, colWidths=col_widths)
        summary_table.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#0F172A")),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
            ('TOPPADDING', (0,0), (-1,-1), 5),
            ('BOTTOMPADDING', (0,0), (-1,-1), 5),
            ('LINEBELOW', (0, -1), (-1, -1), 4, colors.HexColor("#EF4444")),
        ]))
        story.append(summary_table)
        
        # Detailed pages for each scenario in the aggregate PDF
        for r in history:
            s_name = r.get("test_name", "general")
            safe_s_name = s_name.lower().replace(" ", "_").replace("/", "_").strip()
            
            story.append(PageBreak())
            story.append(Paragraph(f"Detailed Diagnostics: {s_name}", h1_style))
            story.append(Spacer(1, 10))
            
            det_headers = [
                Paragraph("Metric", table_header_style),
                Paragraph("Measured Value", table_header_right_style)
            ]
            det_data = [det_headers]
            
            def add_det_row(label, val_str):
                det_data.append([
                    Paragraph(label, table_cell_bold_style),
                    Paragraph(val_str, table_cell_right_style)
                ])
                
            add_det_row("Path Compilation Time", f"{r.get('planning_time_s', 0.0):.4f} s")
            add_det_row("Distance Covered (Path Length)", f"{r.get('path_length_m', 0.0):.2f} m")
            add_det_row("Path Length / Euclidean Ratio", f"{r.get('length_ratio', 1.0):.3f}")
            add_det_row("Turn Angles", f"Min: {r.get('min_angle', 0.0):.1f}° | Max: {r.get('max_angle', 0.0):.1f}° | Avg: {r.get('avg_angle', 0.0):.1f}°")
            add_det_row("Turn Radii", f"Min: {r.get('min_radius', 0.0):.2f} m | Max: {r.get('max_radius', 0.0):.2f} m | Avg: {r.get('avg_radius', 0.0):.2f} m")
            add_det_row("Nearest Distance to Obstacle (Clearance)", f"{r.get('safety_margin_m', 0.0):.2f} m")
            add_det_row("Total Path Cost", f"{r.get('path_cost', 0.0):.1f}")
            
            det_table = Table(det_data, colWidths=[254, 250])
            det_table.setStyle(TableStyle([
                ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#0F172A")),
                ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
                ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
                ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
                ('TOPPADDING', (0,0), (-1,-1), 5),
                ('BOTTOMPADDING', (0,0), (-1,-1), 5),
                ('LINEBELOW', (0, -1), (-1, -1), 4, colors.HexColor("#EF4444")),
            ]))
            story.append(det_table)
            story.append(Spacer(1, 15))
            
            plot_file = f"reports/result_scenario_{safe_s_name}.png"
            if os.path.exists(plot_file):
                story.append(Paragraph("Path Visualization", h1_style))
                story.append(Spacer(1, 5))
                story.append(RLImage(plot_file, width=280, height=280))
    else:
        # SINGLE SCENARIO DETAILED LAYOUT
        story.append(Paragraph("Measured Path Metrics", h1_style))
        
        has_replan = len(history) >= 2
        r_static = history[0]
        r_replan = history[1] if has_replan else None

        test_name = r_static.get("test_name", "general")
        safe_name = test_name.lower().replace(" ", "_").replace("/", "_").strip()
        if not safe_name:
            safe_name = "general"

        if has_replan:
            headers = [
                Paragraph("Metric", table_header_style),
                Paragraph("Static Path", table_header_right_style),
                Paragraph("Replanned Path (Detour)", table_header_right_style)
            ]
            col_widths = [204, 150, 150]
        else:
            headers = [
                Paragraph("Metric", table_header_style),
                Paragraph("Measured Value", table_header_right_style)
            ]
            col_widths = [304, 200]

        def get_row(label, val_static, val_replan=None, is_time=False, is_angle=False, is_radius=False, is_ratio=False, is_cost=False):
            if is_time:
                s_s = f"{val_static:.4f} s"
                s_r = f"{val_replan:.4f} s" if val_replan is not None else ""
            elif is_angle:
                s_s = f"Min: {val_static[0]:.1f}° | Max: {val_static[1]:.1f}° | Avg: {val_static[2]:.1f}°"
                s_r = f"Min: {val_replan[0]:.1f}° | Max: {val_replan[1]:.1f}° | Avg: {val_replan[2]:.1f}°" if val_replan is not None else ""
            elif is_radius:
                s_s = f"Min: {val_static[0]:.2f} m | Max: {val_static[1]:.2f} m | Avg: {val_static[2]:.2f} m"
                s_r = f"Min: {val_replan[0]:.2f} m | Max: {val_replan[1]:.2f} m | Avg: {val_replan[2]:.2f} m" if val_replan is not None else ""
            elif is_ratio:
                s_s = f"{val_static:.3f}"
                s_r = f"{val_replan:.3f}" if val_replan is not None else ""
            elif is_cost:
                s_s = f"{val_static:.1f}"
                s_r = f"{val_replan:.1f}" if val_replan is not None else ""
            else:
                s_s = f"{val_static:.2f} m"
                s_r = f"{val_replan:.2f} m" if val_replan is not None else ""
                
            if has_replan:
                return [
                    Paragraph(label, table_cell_bold_style),
                    Paragraph(s_s, table_cell_right_style),
                    Paragraph(s_r, table_cell_right_style)
                ]
            else:
                return [
                    Paragraph(label, table_cell_bold_style),
                    Paragraph(s_s, table_cell_right_style)
                ]

        metric_table_data = [headers]
        
        # 1. Path Compilation Time
        metric_table_data.append(get_row("Path Compilation Time", r_static.get("planning_time_s", 0.0), r_replan.get("planning_time_s", 0.0) if has_replan else None, is_time=True))
        # 2. Distance Covered
        metric_table_data.append(get_row("Distance Covered (Path Length)", r_static.get("path_length_m", 0.0), r_replan.get("path_length_m", 0.0) if has_replan else None))
        # 3. Ratio of Distance / Euclidean
        metric_table_data.append(get_row("Path Length / Euclidean Ratio", r_static.get("length_ratio", 1.0), r_replan.get("length_ratio", 1.0) if has_replan else None, is_ratio=True))
        # 4. Turn Angle
        metric_table_data.append(get_row("Turn Angles", 
                                         (r_static.get("min_angle", 0.0), r_static.get("max_angle", 0.0), r_static.get("avg_angle", 0.0)),
                                         (r_replan.get("min_angle", 0.0), r_replan.get("max_angle", 0.0), r_replan.get("avg_angle", 0.0)) if has_replan else None,
                                         is_angle=True))
        # 5. Turn Radius
        metric_table_data.append(get_row("Turn Radii", 
                                         (r_static.get("min_radius", 0.0), r_static.get("max_radius", 0.0), r_static.get("avg_radius", 0.0)),
                                         (r_replan.get("min_radius", 0.0), r_replan.get("max_radius", 0.0), r_replan.get("avg_radius", 0.0)) if has_replan else None,
                                         is_radius=True))
        # 6. Nearest Distance to Obstacle
        metric_table_data.append(get_row("Nearest Distance to Obstacle (Clearance)", r_static.get("safety_margin_m", 10.0), r_replan.get("safety_margin_m", 10.0) if has_replan else None))
        # 7. Path Cost
        metric_table_data.append(get_row("Total Path Cost", r_static.get("path_cost", 0.0), r_replan.get("path_cost", 0.0) if has_replan else None, is_cost=True))

        metric_table = Table(metric_table_data, colWidths=col_widths)
        metric_table.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#0F172A")),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
            ('TOPPADDING', (0,0), (-1,-1), 6),
            ('BOTTOMPADDING', (0,0), (-1,-1), 6),
            ('LINEBELOW', (0, -1), (-1, -1), 4, colors.HexColor("#EF4444")),
        ]))
        story.append(metric_table)
        
        # Path Visualization Plot Image (on page 2)
        plot_file = f"reports/result_scenario_{safe_name}.png"
        if os.path.exists(plot_file):
            story.append(PageBreak())
            story.append(Paragraph("Path Visualization", h1_style))
            story.append(Spacer(1, 5))
            story.append(RLImage(plot_file, width=320, height=320))

    # Build the document
    doc.build(story, canvasmaker=NumberedCanvas)
    return True

def generate_csv_report(history, output_path):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp", "Test Name", "Success", "Planning Time (s)", "Path Length (m)", 
                "Length Ratio", "Avg Turn Angle (deg)", "Max Turn Angle (deg)", "Avg Turn Radius (m)", 
                "Min Turn Radius (m)", "Nearest Obstacle Distance (m)", "Path Cost"
            ])
            for r in history:
                writer.writerow([
                    r.get("timestamp", time.strftime('%Y-%m-%d %H:%M:%S')),
                    r.get("test_name", "general"),
                    "YES" if r.get("success", False) else "NO",
                    f"{r.get('planning_time_s', 0.0):.4f}",
                    f"{r.get('path_length_m', 0.0):.2f}",
                    f"{r.get('length_ratio', 1.0):.3f}",
                    f"{r.get('avg_angle', 0.0):.1f}",
                    f"{r.get('max_angle', 0.0):.1f}",
                    f"{r.get('avg_radius', 0.0):.2f}",
                    f"{r.get('min_radius', 0.0):.2f}",
                    f"{r.get('safety_margin_m', 0.0):.2f}",
                    f"{r.get('path_cost', 0.0):.1f}"
                ])
        return True
    except Exception as e:
        print(f"[ERROR] Failed to generate CSV report: {e}")
        return False
