#!/usr/bin/env python3
"""Action-correlated, Two-Tier PathPlanner and Controller scenario benchmark runner."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any

import numpy as np
from PIL import Image, ImageDraw
import yaml

from global_path_benchmarking.navigation_probe import (
    evaluate_plan_geometry,
    swept_collision,
    swept_min_clearance,
    yaw,
)
from global_path_benchmarking.report_generator import (
    calculate_control_metrics,
    generate_pdf_report,
    generate_v2_csv_report,
    generate_v2_html_report,
    generate_v2_markdown_report,
    is_number,
    load_benchmark_config,
)
from global_path_benchmarking.schema import (
    build_provenance,
    build_trial_record,
    gate,
    write_suite_summary,
    write_trial_record,
)


def resolve_asset_path(
    rel_or_abs: str | Path,
    *,
    config_dir: str | Path | None = None,
    package_names: tuple[str, ...] = ("global_path_benchmarking", "pathplanner_mock_tools"),
) -> Path:
    """Resolve scenario/map assets independent of caller CWD (T07)."""
    candidate = Path(rel_or_abs)
    if candidate.is_absolute() and candidate.exists():
        return candidate

    search_roots: list[Path] = []
    if config_dir is not None:
        cpath = Path(config_dir)
        search_roots.extend([cpath, cpath.parent])

    pkg_root = Path(__file__).resolve().parents[1]
    search_roots.append(pkg_root)
    mock_pkg_root = Path(__file__).resolve().parents[3] / "mocks" / "pathplanning" / "pathplanner_mock_tools"
    if mock_pkg_root.exists():
        search_roots.append(mock_pkg_root)

    try:
        from ament_index_python.packages import get_package_share_directory

        for pkg in package_names:
            try:
                search_roots.append(Path(get_package_share_directory(pkg)))
            except Exception:
                pass
    except ImportError:
        pass

    for root in search_roots:
        resolved = root / candidate
        if resolved.exists():
            return resolved

    raise FileNotFoundError(f"Asset {str(rel_or_abs)!r} not found in package share or source roots")


def _generate_synthetic_barrier_pgm(spec: dict[str, Any], out_pgm: Path) -> tuple[float, list[float]]:
    size_px = int(spec.get("size_px", 240))
    res = float(spec.get("resolution", 0.05))
    origin = [float(v) for v in spec.get("origin", [-6.0, -6.0])]
    gap_m = float(spec.get("gap_m", 0.0))

    image = np.full((size_px, size_px), 255, dtype=np.uint8)
    mid = size_px // 2
    image[:, mid - 1 : mid + 2] = 0
    if gap_m > 0.0:
        center = size_px // 4
        half = max(1, int(gap_m / res / 2.0))
        image[size_px - center - half : size_px - center + half, mid - 1 : mid + 2] = 255

    Image.fromarray(image).save(out_pgm)
    return res, origin


def materialize_truth_map(
    scenario: dict[str, Any],
    maps_out_dir: str | Path,
    *,
    config_dir: str | Path | None = None,
) -> Path:
    """Create a standalone Nav2 YAML+PGM truth map in maps_out_dir (never inside source)."""
    out_dir = Path(maps_out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sid = str(scenario["id"])
    out_pgm = out_dir / f"{sid}.pgm"
    out_yaml = out_dir / f"{sid}.yaml"

    if "synthetic_barrier" in scenario:
        resolution, origin_xy = _generate_synthetic_barrier_pgm(scenario["synthetic_barrier"], out_pgm)
    else:
        src_img = resolve_asset_path(scenario["map_image"], config_dir=config_dir)
        Image.open(src_img).convert("L").save(out_pgm)
        resolution = float(scenario["resolution"])
        origin_xy = [float(v) for v in scenario["origin"]]

    yaw_origin = float(origin_xy[2]) if len(origin_xy) >= 3 else 0.0
    map_spec = {
        "image": out_pgm.name,
        "resolution": resolution,
        "origin": [origin_xy[0], origin_xy[1], yaw_origin],
        "negate": 0,
        "free_thresh": 0.196,
        "occupied_thresh": 0.65,
    }
    out_yaml.write_text(yaml.safe_dump(map_spec))
    return out_yaml


def materialize_dynamic_truth_map(
    base_yaml: str | Path,
    obstacles: list[dict[str, Any]],
    out_yaml: str | Path,
) -> Path:
    """Clone base_yaml and paint injected dynamic circular obstacles into truth PGM."""
    base_path = Path(base_yaml)
    spec = yaml.safe_load(base_path.read_text())
    src_img = Path(spec["image"])
    if not src_img.is_absolute():
        src_img = base_path.parent / src_img

    img = Image.open(src_img).convert("L").copy()
    width, height = img.size
    res = float(spec["resolution"])
    ox, oy = float(spec["origin"][0]), float(spec["origin"][1])
    draw = ImageDraw.Draw(img)

    for obs in obstacles:
        cx = (float(obs["x"]) - ox) / res
        cy_bottom = (float(obs["y"]) - oy) / res
        cy_img = (height - 1) - cy_bottom
        rad_px = float(obs.get("radius", 0.4)) / res
        draw.ellipse([cx - rad_px, cy_img - rad_px, cx + rad_px, cy_img + rad_px], fill=0)

    out_path = Path(out_yaml)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_pgm = out_path.with_suffix(".pgm")
    img.save(out_pgm)

    new_spec = dict(spec)
    new_spec["image"] = out_pgm.name
    out_path.write_text(yaml.safe_dump(new_spec))
    return out_path


def _check_cmd_bounds(cmd_rows: list[tuple[float, float, float, float]]) -> bool | None:
    if not cmd_rows:
        return None
    return all(
        -0.50 - 1e-4 <= vx <= 1.00 + 1e-4
        and abs(vy) <= 0.01 + 1e-4
        and abs(wz) <= 1.00 + 1e-4
        for _, vx, vy, wz in cmd_rows
    )


def evaluate_trial_from_observations(
    scenario: dict[str, Any],
    mode: str,
    cfg: dict[str, Any],
    obs: dict[str, Any],
    provenance: dict[str, Any],
) -> dict[str, Any]:
    """Apply Two-Tier gates and physical SI metrics from recorded trial observations."""
    sid = str(scenario["id"])
    category = str(scenario.get("category", "positive_static"))
    expected_outcome = str(scenario.get("expected_outcome", "PASS"))
    terminal = obs.get("terminal_state")
    blocked_reason = obs.get("blocked_reason")
    invalid_reason = obs.get("invalid_reason")

    fp = scenario.get("footprint") or [
        float(cfg.get("footprint", {}).get("length", 0.90)),
        float(cfg.get("footprint", {}).get("width", 0.70)),
    ]
    fp_len, fp_wid = float(fp[0]), float(fp[1])
    allow_unknown = bool(scenario.get("allow_unknown", cfg.get("allow_unknown_default", False)))
    tols = cfg.get("tolerances", {}) or {}
    start_tol = float(tols.get("start_xy_m", 0.35))
    goal_tol = float(tols.get("goal_xy_m", 0.35))
    planner_goal_tol = float(tols.get("planner_goal_xy_m", 0.50))

    t_req = obs.get("t_request_sent")
    t_resp = obs.get("t_initial_response")
    t_inj = obs.get("t_obstacle_injected")
    t_replan = obs.get("t_post_event_response")

    plan_time = obs.get("internal_planning_time_s")
    if not is_number(plan_time) and is_number(t_req) and is_number(t_resp) and t_resp >= t_req:
        plan_time = float(t_resp - t_req)

    replan_reaction = (
        float(t_replan - t_inj)
        if is_number(t_inj) and is_number(t_replan) and t_replan > t_inj
        else None
    )

    plan_se2 = obs.get("plan_se2") or []
    replan_se2 = obs.get("replan_se2") or []
    truth_map = obs.get("truth_map")
    dyn_truth_map = obs.get("dynamic_truth_map")
    goal_xy = scenario.get("goal", [0.0, 0.0])[:2]
    start_xy = scenario.get("start", [0.0, 0.0])[:2]
    ref_path = scenario.get("reference_path")

    gates: dict[str, dict[str, Any]] = {}
    metrics: dict[str, Any] = {
        "planning_time_s": plan_time,
        "replan_reaction_time_s": replan_reaction,
        "execution_time_s": (
            float(obs["t_terminal_result"] - t_req)
            if is_number(t_req) and is_number(obs.get("t_terminal_result"))
            else None
        ),
        "error_code": obs.get("error_code"),
    }

    diag_score: float | None = None

    if blocked_reason or terminal == "SERVER_UNAVAILABLE":
        blocked_reason = blocked_reason or "Required Nav2 server or service unavailable"
        gates["prerequisite_available"] = gate(None, blocked_reason)
        return build_trial_record(
            provenance=provenance,
            scenario_id=sid,
            category=category,
            expected_outcome=expected_outcome,
            gates=gates,
            metrics=metrics,
            diagnostic_score=None,
            phases=obs.get("phases", []),
            artifacts=obs.get("artifacts", {}),
            blocked_reason=blocked_reason,
        )

    if category == "negative_infeasible":
        rejected = terminal in ("ABORTED", "REJECTED") and len(plan_se2) == 0
        gates["expected_infeasible_rejected"] = gate(
            rejected if terminal is not None else None,
            f"terminal={terminal}, poses={len(plan_se2)}, error_code={obs.get('error_code')}",
        )
    elif category == "safety_cancel":
        canceled = terminal == "CANCELED" if terminal is not None else None
        gates["action_canceled"] = gate(canceled, f"terminal={terminal}")
        post_cmds = obs.get("post_fault_commands")
        if post_cmds is not None:
            policy = cfg.get("stale_input_policy", {}) or {}
            max_v = float(policy.get("max_allowed_linear_mps", 0.02))
            max_w = float(policy.get("max_allowed_angular_rps", 0.05))
            stopped = all(abs(r[1]) <= max_v and abs(r[3]) <= max_w for r in post_cmds)
            gates["stopped_after_cancel"] = gate(stopped, f"post_cancel_samples={len(post_cmds)}")
    elif category == "safety_stale_input":
        post_cmds = obs.get("post_fault_commands")
        policy = cfg.get("stale_input_policy", {}) or {}
        max_v = float(policy.get("max_allowed_linear_mps", 0.02))
        max_w = float(policy.get("max_allowed_angular_rps", 0.05))
        if post_cmds is None:
            gates["stale_input_fail_closed"] = gate(None, "No post-fault command telemetry captured")
        else:
            violations = [r for r in post_cmds if abs(r[1]) > max_v or abs(r[3]) > max_w]
            peak_w = max((abs(r[3]) for r in post_cmds), default=0.0)
            peak_v = max((abs(r[1]) for r in post_cmds), default=0.0)
            gates["stale_input_fail_closed"] = gate(
                len(violations) == 0,
                f"violations={len(violations)}/{len(post_cmds)}, peak_v={peak_v:.3f}, peak_w={peak_w:.3f}",
            )
            metrics["post_fault_peak_linear_mps"] = peak_v
            metrics["post_fault_peak_angular_rps"] = peak_w
    else:
        # positive_static or dynamic_replan
        expected_post = str(scenario.get("post_obstacle_expected", "SUCCEEDED"))
        if category == "dynamic_replan" and expected_post == "ABORTED":
            # Initial plan must succeed; post-obstacle either cleanly aborts or finds valid detour
            init_ok = obs.get("initial_plan_succeeded")
            if init_ok is None:
                init_ok = len(plan_se2) >= 2
            gates["initial_plan_succeeded"] = gate(bool(init_ok), f"initial_poses={len(plan_se2)}")
        else:
            gates["action_succeeded"] = gate(
                True if terminal == "SUCCEEDED" else (None if terminal is None else False),
                f"terminal={terminal}, error_code={obs.get('error_code')}",
            )

        plan_geom = evaluate_plan_geometry(
            plan_se2,
            start_xy=start_xy,
            goal_xy=goal_xy,
            truth_map=truth_map,
            reference_path=ref_path,
            length=fp_len,
            width=fp_wid,
            allow_unknown=allow_unknown,
            start_tol=start_tol,
            goal_tol=planner_goal_tol if mode == "planner_only" else goal_tol,
        )
        metrics.update({
            k: v for k, v in plan_geom.items()
            if k not in ("valid_geometry", "endpoints_valid", "swept_collision")
        })
        gates["valid_plan_geometry"] = gate(
            plan_geom["valid_geometry"],
            f"poses={len(plan_se2)}",
        )
        gates["endpoints_within_tolerance"] = gate(
            plan_geom["endpoints_valid"],
            f"start_err={plan_geom['start_error_m']}, goal_err={plan_geom['goal_error_m']}",
        )
        gates["plan_swept_collision_free"] = gate(
            (plan_geom["swept_collision"] is False) if plan_geom["swept_collision"] is not None else None,
            f"swept_collision={plan_geom['swept_collision']}, clearance={plan_geom['swept_min_clearance_m']}",
        )

        if category == "dynamic_replan":
            # Strict phase correlation: replan must arrive AFTER obstacle injection (P05)
            phase_correlated = is_number(t_inj) and is_number(t_replan) and (t_replan > t_inj)
            if expected_post == "ABORTED":
                replan_clean = (terminal == "ABORTED") or (
                    phase_correlated
                    and len(replan_se2) >= 2
                    and swept_collision(replan_se2, dyn_truth_map, length=fp_len, width=fp_wid, allow_unknown=allow_unknown) is False
                )
                gates["dynamic_replan_handled"] = gate(
                    replan_clean if is_number(t_inj) else None,
                    f"post_obstacle_expected=ABORTED, terminal={terminal}, phase_correlated={phase_correlated}",
                )
            else:
                replan_geom = evaluate_plan_geometry(
                    replan_se2,
                    goal_xy=goal_xy,
                    truth_map=dyn_truth_map,
                    length=fp_len,
                    width=fp_wid,
                    allow_unknown=allow_unknown,
                    goal_tol=planner_goal_tol if mode == "planner_only" else goal_tol,
                )
                gates["dynamic_replan_handled"] = gate(
                    bool(
                        phase_correlated
                        and replan_geom["valid_geometry"]
                        and replan_geom["swept_collision"] is False
                    )
                    if is_number(t_inj)
                    else None,
                    f"phase_correlated={phase_correlated}, replan_swept_collision={replan_geom['swept_collision']}",
                )
                metrics["replan_swept_min_clearance_m"] = replan_geom["swept_min_clearance_m"]

        if mode != "planner_only":
            cmd_rows = obs.get("cmd_history") or []
            odom_rows = obs.get("odom_history") or []
            truth_rows = obs.get("truth_odom_history") or odom_rows
            active_xy = [(p[0], p[1]) for p in (replan_se2 or plan_se2)]
            goal_msg = SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=float(goal_xy[0]), y=float(goal_xy[1])))
            )
            ctrl_weights = (cfg.get("control_benchmarking", {}) or {}).get("weights")
            ctrl = calculate_control_metrics(
                cmd_rows,
                truth_rows,
                active_xy,
                goal_msg,
                execution_complete=(terminal == "SUCCEEDED"),
                weights=ctrl_weights,
            )
            metrics.update({
                "mean_cte_m": ctrl["mean_cte"],
                "max_cte_m": ctrl["max_cte"],
                "rms_cte_m": ctrl["rms_cte"],
                "mean_linear_vel_mps": ctrl["mean_linear_vel"],
                "max_linear_vel_mps": ctrl["max_linear_vel"],
                "mean_angular_vel_rps": ctrl["mean_angular_vel"],
                "max_angular_vel_rps": ctrl["max_angular_vel"],
                "linear_jerk_std": ctrl["linear_jerk_std"],
                "angular_jerk_std": ctrl["angular_jerk_std"],
                "cmd_freq_hz": ctrl["cmd_freq_hz"],
                "goal_error_truth_m": ctrl["goal_accuracy_m"],
                "control_effort": ctrl["control_effort"],
                "steering_reversals": ctrl["steering_reversals"],
                "heading_rmse_deg": ctrl["heading_rmse_deg"],
            })

            exec_map = dyn_truth_map or truth_map
            if truth_rows and exec_map:
                exec_se2 = [(r[1], r[2], r[3]) for r in truth_rows]
                exec_coll = swept_collision(
                    exec_se2, exec_map, length=fp_len, width=fp_wid, allow_unknown=allow_unknown
                )
                exec_clear = swept_min_clearance(
                    exec_se2, exec_map, length=fp_len, width=fp_wid, allow_unknown=allow_unknown
                )
                metrics["executed_swept_min_clearance_m"] = exec_clear
                gates["executed_swept_collision_free"] = gate(
                    exec_coll is False if exec_coll is not None else None,
                    f"executed_swept_collision={exec_coll}, min_clearance={exec_clear}",
                )
            else:
                gates["executed_swept_collision_free"] = gate(None, "Missing truth trajectory or truth map")

            if expected_post != "ABORTED":
                g_err = ctrl["goal_accuracy_m"]
                gates["goal_arrived_within_tolerance"] = gate(
                    (g_err <= goal_tol) if is_number(g_err) else None,
                    f"goal_error_truth_m={g_err}, tol={goal_tol}",
                )

            cmd_bounds_ok = _check_cmd_bounds(cmd_rows)
            gates["cmd_vel_within_limits"] = gate(cmd_bounds_ok, f"samples={len(cmd_rows)}")

            pub_count = obs.get("cmd_vel_publishers")
            if pub_count is not None:
                gates["single_cmd_vel_publisher"] = gate(
                    pub_count == 1, f"cmd_vel_publishers={pub_count}"
                )

            # Secondary single-mode diagnostic score only when everything completes cleanly
            if ctrl.get("controller_score") is not None:
                diag_score = float(ctrl["controller_score"])

    return build_trial_record(
        provenance=provenance,
        scenario_id=sid,
        category=category,
        expected_outcome=expected_outcome,
        gates=gates,
        metrics=metrics,
        diagnostic_score=diag_score,
        phases=obs.get("phases", []),
        artifacts=obs.get("artifacts", {}),
        terminal_state="TIMEOUT" if terminal == "TIMEOUT" else None,
        invalid_reason=invalid_reason,
    )


def plot_scenario_overlay(
    scenario: dict[str, Any],
    truth_yaml: str | Path,
    plan_coords: list[tuple[float, float]],
    replan_coords: list[tuple[float, float]],
    exec_coords: list[tuple[float, float]],
    record: dict[str, Any],
    out_png: str | Path,
) -> bool:
    """Render trajectory vs independent truth map PNG into the run output directory."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        yaml_path = Path(truth_yaml)
        spec = yaml.safe_load(yaml_path.read_text())
        img_file = Path(spec["image"])
        if not img_file.is_absolute():
            img_file = yaml_path.parent / img_file
        img_arr = np.flipud(np.asarray(Image.open(img_file).convert("L")))
        res = float(spec["resolution"])
        ox, oy = float(spec["origin"][0]), float(spec["origin"][1])
        extent = [ox, ox + img_arr.shape[1] * res, oy, oy + img_arr.shape[0] * res]

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(img_arr, cmap="gray", origin="lower", extent=extent)

        ref = scenario.get("reference_path") or []
        if len(ref) >= 2:
            ax.plot([p[0] for p in ref], [p[1] for p in ref], "g--", linewidth=2, label="Reference")
        if len(plan_coords) >= 2:
            ax.plot([p[0] for p in plan_coords], [p[1] for p in plan_coords], "r-", linewidth=2, label="Initial Plan")
        if len(replan_coords) >= 2:
            ax.plot([p[0] for p in replan_coords], [p[1] for p in replan_coords], "c-.", linewidth=2, label="Replanned")
        if len(exec_coords) >= 2:
            ax.plot([p[0] for p in exec_coords], [p[1] for p in exec_coords], "b-", linewidth=1.8, label="Executed")

        ax.plot(scenario["start"][0], scenario["start"][1], "bs", markersize=9, label="Start")
        ax.plot(scenario["goal"][0], scenario["goal"][1], "r*", markersize=12, label="Goal")
        for obs in scenario.get("dynamic_obstacles", []):
            ax.add_patch(plt.Circle((obs["x"], obs["y"]), obs["radius"], color="orange", alpha=0.6))

        m = record.get("metrics", {})
        box = (
            f"Outcome: {record.get('outcome')}\n"
            f"Plan Time: {m.get('planning_time_s')}\n"
            f"Swept Clear: {m.get('swept_min_clearance_m')}\n"
            f"Mean CTE: {m.get('mean_cte_m')}"
        )
        ax.text(0.03, 0.97, box, transform=ax.transAxes, va="top",
                bbox=dict(boxstyle="round", facecolor="#e2e8f0", alpha=0.9))
        ax.set_title(f"{scenario['id']} ({record.get('outcome')})")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(loc="lower right")
        ax.grid(True, alpha=0.3)

        target = Path(out_png)
        target.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(target, bbox_inches="tight")
        plt.close(fig)
        return True
    except Exception:
        return False


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry, Path as NavPath
    from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap, LoadMap
    from std_msgs.msg import Empty, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, help="Directory outside source tree for benchmark outputs")
    parser.add_argument("--mode", choices=["planner_only", "nav_closed_loop", "mixed"], default=None)
    parser.add_argument("--scenarios", default="config/scenarios.yaml")
    parser.add_argument("--benchmark-config", default="config/benchmark_config.yaml")
    parser.add_argument("--scenario-id", default="", help="Run a single scenario ID")
    parser.add_argument("--category", default="", help="Filter by scenario category")
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--mocked-rover", action="store_true", default=True)
    parser.add_argument("--no-mocked-rover", dest="mocked_rover", action="store_false")
    options = parser.parse_args(args)

    out_root = Path(options.output_dir).resolve()
    pkg_src_root = Path(__file__).resolve().parents[1]
    if out_root == pkg_src_root or pkg_src_root in out_root.parents:
        parser.error("--output-dir must be outside the source package tree (T07)")

    scenarios_path = resolve_asset_path(options.scenarios)
    bench_cfg_path = resolve_asset_path(options.benchmark_config)
    cfg = load_benchmark_config(bench_cfg_path)
    mode = options.mode or cfg.get("evaluation_mode", "nav_closed_loop")

    scenarios_data = yaml.safe_load(scenarios_path.read_text()) or {}
    scenarios = scenarios_data.get("scenarios", [])
    if options.scenario_id:
        scenarios = [s for s in scenarios if s.get("id") == options.scenario_id]
    if options.category:
        scenarios = [s for s in scenarios if s.get("category") == options.category]
    if not scenarios:
        raise RuntimeError("No scenarios selected matching filters")

    interfaces = cfg.get("interfaces", {}) or {}
    odom_topic = interfaces.get("odom_topic", "/odometry/filtered")
    cmd_topic = interfaces.get("cmd_vel_topic", "/cmd_vel")
    plan_topic = interfaces.get("plan_topic", "/plan")

    prov = build_provenance(
        evaluation_mode=mode,
        dependency_manifest={
            "map": "MOCK",
            "localization": "MOCK" if options.mocked_rover else "REAL",
            "perception": "MOCK",
            "rover_plant": "MOCK" if options.mocked_rover else "GAZEBO",
            "clock": "SIM" if options.use_sim_time else "WALL",
        },
        truth_source="independent_scenario_yaml",
        claim_scope=(
            "Automated multi-scenario Nav2 probe against independent scenario truth. "
            "Mocked runs do not establish physical localization, perception, or hardware readiness."
        ),
        seed=options.seed,
        use_sim_time=options.use_sim_time,
    )

    rclpy.init()
    node = Node(
        "scenario_benchmark_runner",
        parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=options.use_sim_time)],
    )

    load_client = node.create_client(LoadMap, "/map_server/load_map")
    clear_global = node.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
    clear_local = node.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
    init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    clear_obs_pub = node.create_publisher(Empty, "/mock_perception/clear_obstacles", 10)
    spawn_obs_pub = node.create_publisher(String, "/mock_perception/spawn_obstacle_full", 10)

    plan_client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    nav_through_client = ActionClient(node, NavigateThroughPoses, "navigate_through_poses")

    plans: list[tuple[float, Any]] = []
    cmds: list[tuple[float, float, float, float]] = []
    odoms: list[tuple[float, float, float, float, float, float]] = []

    def _on_plan(msg):
        plans.append((time.monotonic(), msg))

    def _on_cmd(msg):
        cmds.append((time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_odom(msg):
        odoms.append((
            time.monotonic(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ))

    node.create_subscription(NavPath, plan_topic, _on_plan, 50)
    node.create_subscription(Twist, cmd_topic, _on_cmd, 100)
    node.create_subscription(Odometry, odom_topic, _on_odom, 100)

    def wait_future(future, seconds):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def make_pose(x, y, heading=0.0):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.orientation.w = math.cos(heading / 2.0)
        return msg

    def set_mock_fault_params(freeze_odom=False, freeze_perception=False):
        try:
            from rclpy.parameter import Parameter
            from rclpy.parameter_client import AsyncParameterClient

            for target, pairs in (
                ("/mock_rover_sim", [("freeze_odom", bool(freeze_odom))]),
                ("/mock_perception", [("freeze_perception", bool(freeze_perception))]),
            ):
                pclient = AsyncParameterClient(node, target)
                if pclient.wait_for_services(timeout_sec=1.5):
                    fut = pclient.set_parameters([Parameter(k, value=v) for k, v in pairs])
                    wait_future(fut, 2.0)
        except Exception:
            pass

    maps_dir = out_root / "truth_maps"
    plots_dir = out_root / "plots"
    trials_dir = out_root / "trials"
    records: list[dict[str, Any]] = []

    try:
        map_ready = load_client.wait_for_service(timeout_sec=10.0) and clear_global.wait_for_service(timeout_sec=10.0)
        server_ready = plan_client.wait_for_server(timeout_sec=15.0)
        if mode != "planner_only":
            server_ready = server_ready and nav_client.wait_for_server(timeout_sec=15.0)

        for scenario in scenarios:
            sid = str(scenario["id"])
            category = str(scenario.get("category", "positive_static"))
            plans.clear()
            cmds.clear()
            odoms.clear()

            truth_yaml = materialize_truth_map(scenario, maps_dir, config_dir=scenarios_path.parent)
            dyn_truth_yaml = None

            if not map_ready or not server_ready:
                rec = evaluate_trial_from_observations(
                    scenario,
                    mode,
                    cfg,
                    {"terminal_state": "SERVER_UNAVAILABLE", "truth_map": str(truth_yaml)},
                    prov,
                )
                records.append(rec)
                write_trial_record(rec, trials_dir / f"{sid}.json")
                continue

            # Reset scenario state
            set_mock_fault_params(freeze_odom=False, freeze_perception=False)
            clear_obs_pub.publish(Empty())

            req = LoadMap.Request()
            req.map_url = str(truth_yaml)
            load_res = wait_future(load_client.call_async(req), 10.0)
            if load_res is None or load_res.result != 0:
                rec = evaluate_trial_from_observations(
                    scenario,
                    mode,
                    cfg,
                    {"blocked_reason": f"Failed to load truth map {truth_yaml}", "truth_map": str(truth_yaml)},
                    prov,
                )
                records.append(rec)
                write_trial_record(rec, trials_dir / f"{sid}.json")
                continue

            wait_future(clear_global.call_async(ClearEntireCostmap.Request()), 5.0)
            if clear_local.service_is_ready():
                wait_future(clear_local.call_async(ClearEntireCostmap.Request()), 5.0)

            if options.mocked_rover:
                init = PoseWithCovarianceStamped()
                init.header.frame_id = "map"
                init.header.stamp = node.get_clock().now().to_msg()
                init.pose.pose.position.x = float(scenario["start"][0])
                init.pose.pose.position.y = float(scenario["start"][1])
                init.pose.pose.orientation.w = 1.0
                init_pub.publish(init)

            # Allow costmap/mock state to settle
            settle_end = time.monotonic() + 1.2
            while time.monotonic() < settle_end:
                rclpy.spin_once(node, timeout_sec=0.05)

            plans.clear()
            cmds.clear()
            odoms.clear()

            use_plan_only = (mode == "planner_only") or (category == "negative_infeasible")
            waypoints = scenario.get("waypoints") or []
            use_through = (not use_plan_only) and (mode != "planner_only") and (len(waypoints) >= 2) and nav_through_client.wait_for_server(timeout_sec=0.5)

            t_req = time.monotonic()
            t_resp = None
            t_inj = None
            t_replan = None
            t_term = None
            terminal_state = "TIMEOUT"
            error_code = None
            goal_id = None
            plan_se2: list[tuple[float, float, float]] = []
            replan_se2: list[tuple[float, float, float]] = []
            post_fault_cmds = None

            if use_plan_only:
                goal_msg = ComputePathToPose.Goal()
                goal_msg.start = make_pose(scenario["start"][0], scenario["start"][1])
                goal_msg.goal = make_pose(scenario["goal"][0], scenario["goal"][1])
                goal_msg.use_start = True
                goal_msg.planner_id = "GridBased"
                handle = wait_future(plan_client.send_goal_async(goal_msg), 5.0)
                if handle is None or not handle.accepted:
                    terminal_state = "NO_ACCEPTANCE" if handle is None else "REJECTED"
                else:
                    goal_id = bytes(handle.goal_id.uuid).hex()
                    res = wait_future(handle.get_result_async(), min(options.timeout, 15.0))
                    t_term = time.monotonic()
                    if res is None:
                        terminal_state = "TIMEOUT"
                        wait_future(handle.cancel_goal_async(), 3.0)
                    else:
                        t_resp = t_term
                        error_code = getattr(res.result, "error_code", None)
                        terminal_state = {
                            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                            GoalStatus.STATUS_ABORTED: "ABORTED",
                            GoalStatus.STATUS_CANCELED: "CANCELED",
                        }.get(res.status, "OTHER")
                        plan_se2 = [
                            (p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
                            for p in res.result.path.poses
                        ]
            else:
                if use_through:
                    goal_msg = NavigateThroughPoses.Goal()
                    goal_msg.poses = [make_pose(w[0], w[1], w[2] if len(w) > 2 else 0.0) for w in waypoints]
                    active_client = nav_through_client
                else:
                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose = make_pose(scenario["goal"][0], scenario["goal"][1])
                    active_client = nav_client

                handle = wait_future(active_client.send_goal_async(goal_msg), 5.0)
                if handle is None or not handle.accepted:
                    terminal_state = "NO_ACCEPTANCE" if handle is None else "REJECTED"
                else:
                    goal_id = bytes(handle.goal_id.uuid).hex()
                    res_future = handle.get_result_async()
                    deadline = time.monotonic() + options.timeout
                    injected = False
                    canceled_sent = False
                    fault_active = False
                    fault_start_t = None

                    while not res_future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        now_m = time.monotonic()
                        if t_resp is None and plans:
                            t_resp = plans[0][0]

                        if category == "safety_cancel" and not canceled_sent:
                            if now_m - t_req >= float(scenario.get("cancel_after_s", 0.5)):
                                canceled_sent = True
                                wait_future(handle.cancel_goal_async(), 3.0)
                                post_fault_cmds = []
                                settle_cancel = time.monotonic() + 0.8
                                while time.monotonic() < settle_cancel:
                                    rclpy.spin_once(node, timeout_sec=0.05)
                                post_fault_cmds = [c for c in cmds if c[0] >= now_m]

                        elif category == "dynamic_replan" and not injected and t_resp is not None:
                            dyn_obs = scenario.get("dynamic_obstacles") or []
                            if dyn_obs and (now_m - t_resp >= float(dyn_obs[0].get("trigger_time", 0.5))):
                                injected = True
                                t_inj = now_m
                                dyn_truth_yaml = materialize_dynamic_truth_map(
                                    truth_yaml, dyn_obs, maps_dir / f"{sid}_dynamic.yaml"
                                )
                                for d in dyn_obs:
                                    spawn_obs_pub.publish(String(data=json.dumps({
                                        "x": float(d["x"]), "y": float(d["y"]), "radius": float(d.get("radius", 0.4))
                                    })))

                        elif category == "safety_stale_input" and not fault_active:
                            if now_m - t_req >= float(scenario.get("fault_trigger_s", 0.5)):
                                fault_active = True
                                fault_start_t = now_m
                                set_mock_fault_params(freeze_odom=True, freeze_perception=True)

                        if fault_active and fault_start_t is not None:
                            max_age = float((cfg.get("stale_input_policy", {}) or {}).get("max_input_age_s", 1.0))
                            dur = float(scenario.get("fault_duration_s", 2.5))
                            if now_m >= fault_start_t + dur:
                                post_fault_cmds = [c for c in cmds if c[0] >= fault_start_t + max_age]
                                wait_future(handle.cancel_goal_async(), 3.0)
                                break

                    res = res_future.result() if res_future.done() else None
                    t_term = time.monotonic()
                    if res is None:
                        terminal_state = "TIMEOUT"
                        wait_future(handle.cancel_goal_async(), 3.0)
                    else:
                        terminal_state = {
                            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                            GoalStatus.STATUS_ABORTED: "ABORTED",
                            GoalStatus.STATUS_CANCELED: "CANCELED",
                        }.get(res.status, "OTHER")
                        error_code = getattr(res.result, "error_code", None)

                    set_mock_fault_params(freeze_odom=False, freeze_perception=False)

                    if plans:
                        initial_plans = [p for p in plans if t_inj is None or p[0] <= t_inj]
                        post_plans = [p for p in plans if t_inj is not None and p[0] > t_inj]
                        if initial_plans:
                            plan_se2 = [
                                (pt.pose.position.x, pt.pose.position.y, yaw(pt.pose.orientation))
                                for pt in initial_plans[0][1].poses
                            ]
                        if post_plans:
                            t_replan = post_plans[-1][0]
                            replan_se2 = [
                                (pt.pose.position.x, pt.pose.position.y, yaw(pt.pose.orientation))
                                for pt in post_plans[-1][1].poses
                            ]

            plot_rel = f"plots/{sid}.png"
            obs_payload = {
                "terminal_state": terminal_state,
                "error_code": error_code,
                "truth_map": str(truth_yaml),
                "dynamic_truth_map": str(dyn_truth_yaml) if dyn_truth_yaml else None,
                "plan_se2": plan_se2,
                "replan_se2": replan_se2,
                "t_request_sent": t_req,
                "t_initial_response": t_resp,
                "t_obstacle_injected": t_inj,
                "t_post_event_response": t_replan,
                "t_terminal_result": t_term,
                "goal_id": goal_id,
                "cmd_history": list(cmds),
                "odom_history": list(odoms),
                "cmd_vel_publishers": node.count_publishers(cmd_topic),
                "post_fault_commands": post_fault_cmds,
                "phases": [{"name": "primary", "goal_id": goal_id, "t_start": t_req, "t_end": t_term}],
                "artifacts": {"plot_png": plot_rel},
            }
            rec = evaluate_trial_from_observations(scenario, mode, cfg, obs_payload, prov)
            plot_scenario_overlay(
                scenario,
                dyn_truth_yaml or truth_yaml,
                [(p[0], p[1]) for p in plan_se2],
                [(p[0], p[1]) for p in replan_se2],
                [(o[1], o[2]) for o in odoms],
                rec,
                out_root / plot_rel,
            )
            records.append(rec)
            write_trial_record(rec, trials_dir / f"{sid}.json")
            node.get_logger().info(f"[{sid}] outcome={rec['outcome']}")

        write_suite_summary(records, prov, out_root / "results_v2.json")
        generate_v2_csv_report(records, str(out_root / "summary.csv"))
        generate_v2_markdown_report(records, prov, str(out_root / "summary.md"))
        generate_v2_html_report(records, prov, str(out_root / "summary.html"))
        generate_pdf_report(
            [
                {
                    "test_name": r["scenario"]["id"],
                    "outcome": r["outcome"],
                    "success": r["outcome"] == "PASS",
                    "planning_time_s": r["metrics"].get("planning_time_s"),
                    "path_length_m": r["metrics"].get("path_length_m"),
                    "length_ratio": r["metrics"].get("reference_length_ratio") or r["metrics"].get("euclidean_length_ratio"),
                    "min_angle": r["metrics"].get("min_angle_deg"),
                    "max_angle": r["metrics"].get("max_angle_deg"),
                    "avg_angle": r["metrics"].get("avg_angle_deg"),
                    "min_radius": r["metrics"].get("min_turn_radius_m"),
                    "avg_radius": r["metrics"].get("avg_turn_radius_m"),
                    "safety_margin_m": r["metrics"].get("swept_min_clearance_m"),
                    "path_cost": r["metrics"].get("total_cost"),
                }
                for r in records
            ],
            str(out_root / "summary.pdf"),
        )
    finally:
        set_mock_fault_params(freeze_odom=False, freeze_perception=False)
        node.destroy_node()
        rclpy.try_shutdown()

    return 0 if records and all(r["outcome"] == "PASS" for r in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
