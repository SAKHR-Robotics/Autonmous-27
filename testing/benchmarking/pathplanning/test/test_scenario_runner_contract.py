"""Offline contracts for scenario_benchmark_runner, geometry evaluation, and config wiring."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from global_path_benchmarking.navigation_probe import (
    evaluate_plan_geometry,
    sample_truth_map_cost,
    swept_collision,
    swept_min_clearance,
)
from global_path_benchmarking.report_generator import load_benchmark_config
from global_path_benchmarking.scenario_benchmark_runner import (
    evaluate_trial_from_observations,
    materialize_dynamic_truth_map,
    materialize_truth_map,
    plot_scenario_overlay,
    resolve_asset_path,
)
from global_path_benchmarking.schema import build_provenance, validate_trial_record


def _truth_yaml(tmp_path, wall_col=None, unknown_col=None):
    img = np.full((40, 40), 255, dtype=np.uint8)
    if wall_col is not None:
        img[:, wall_col] = 0
    if unknown_col is not None:
        img[:, unknown_col] = 128
    Image.fromarray(img).save(tmp_path / "map.pgm")
    spec = {
        "image": "map.pgm",
        "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "free_thresh": 0.196,
        "occupied_thresh": 0.65,
    }
    p = tmp_path / "map.yaml"
    p.write_text(yaml.safe_dump(spec))
    return p


def _prov():
    return build_provenance(
        evaluation_mode="nav_closed_loop",
        dependency_manifest={
            "map": "MOCK",
            "localization": "MOCK",
            "perception": "MOCK",
            "rover_plant": "MOCK",
            "clock": "WALL",
        },
        truth_source="independent_scenario_yaml",
        claim_scope="Unit contract test only.",
        seed=0,
    )


def _cfg():
    cfg_path = Path(__file__).parents[1] / "config" / "benchmark_config.yaml"
    return load_benchmark_config(cfg_path)


def test_resolve_asset_path_independent_of_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    resolved = resolve_asset_path("config/scenarios.yaml")
    assert resolved.is_file()
    with pytest.raises(FileNotFoundError):
        resolve_asset_path("nonexistent_asset_xyz.yaml")


def test_load_benchmark_config_rejects_unsupported_modes(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump({"evaluation_mode": "telepathy"}))
    with pytest.raises(ValueError):
        load_benchmark_config(bad)

    bad.write_text(yaml.safe_dump({"interfaces": {"goal_type": "magic"}}))
    with pytest.raises(ValueError):
        load_benchmark_config(bad)


def test_unknown_space_policy_and_clearance(tmp_path):
    tmap = _truth_yaml(tmp_path, unknown_col=20)
    path = [(1.0, 2.0, 0.0), (3.0, 2.0, 0.0)]
    assert swept_collision(path, str(tmap), allow_unknown=False) is True
    assert swept_min_clearance(path, str(tmap), allow_unknown=False) == 0.0
    assert swept_collision(path, str(tmap), allow_unknown=True) is False
    # Add real wall at col 35 (x=3.55 m); path ends at x=3.0 (half-length 0.45 -> front at 3.45 -> ~0.05 m)
    tmap_wall = _truth_yaml(tmp_path, wall_col=35)
    clearance = swept_min_clearance([(1.0, 2.0, 0.0), (2.5, 2.0, 0.0)], str(tmap_wall))
    assert clearance is not None and clearance > 0.5
    avg_c, tot_c = sample_truth_map_cost(path, str(tmap_wall))
    assert avg_c == pytest.approx(0.0) and tot_c == pytest.approx(0.0)


def test_evaluate_plan_geometry_separates_cusps_and_checks_endpoints(tmp_path):
    tmap = _truth_yaml(tmp_path)
    # Forward then reverse cusp at (2.0, 2.0)
    path = [(1.0, 2.0, 0.0), (2.0, 2.0, 0.0), (1.0, 2.0, 0.0), (2.5, 2.0, 0.0)]
    res = evaluate_plan_geometry(
        path,
        start_xy=(1.0, 2.0),
        goal_xy=(2.5, 2.0),
        truth_map=str(tmap),
        reference_path=[(1.0, 2.0), (2.5, 2.0)],
    )
    assert res["valid_geometry"] is True
    assert res["cusp_count"] == 2
    assert res["endpoints_valid"] is True
    assert res["reference_length_ratio"] > 1.5

    bad_end = evaluate_plan_geometry(
        [(1.0, 2.0, 0.0), (1.5, 2.0, 0.0)],
        start_xy=(1.0, 2.0),
        goal_xy=(3.5, 2.0),
        goal_tol=0.50,
    )
    assert bad_end["endpoints_valid"] is False


def test_materialize_truth_and_dynamic_obstacle_maps(tmp_path):
    scen = {
        "id": "dyn_test",
        "synthetic_barrier": {"size_px": 100, "resolution": 0.1, "origin": [-5.0, -5.0], "gap_m": 2.0},
    }
    base_yaml = materialize_truth_map(scen, tmp_path / "maps")
    assert base_yaml.is_file()
    dyn_yaml = materialize_dynamic_truth_map(
        base_yaml,
        [{"x": -2.0, "y": -2.0, "radius": 0.5}],
        tmp_path / "maps" / "dyn_test_dynamic.yaml",
    )
    assert swept_collision([(-3.0, -2.0, 0.0), (-1.0, -2.0, 0.0)], str(dyn_yaml)) is True


def test_trial_evaluation_p05_p08_pp11(tmp_path):
    cfg = _cfg()
    prov = _prov()
    tmap = str(_truth_yaml(tmp_path))

    # 1. Negative infeasible fails if path is returned (PP10)
    neg = evaluate_trial_from_observations(
        {"id": "blocked_wall", "category": "negative_infeasible", "start": [1, 2], "goal": [3, 2]},
        "planner_only",
        cfg,
        {"terminal_state": "SUCCEEDED", "plan_se2": [(1, 2, 0), (3, 2, 0)], "truth_map": tmap},
        prov,
    )
    assert neg["outcome"] == "FAIL"
    assert neg["diagnostic_score"] is None
    assert validate_trial_record(neg) == []

    # 2. Dynamic replan ignores pre-injection plan updates (P05)
    dyn = evaluate_trial_from_observations(
        {
            "id": "scattered_rocks_detour",
            "category": "dynamic_replan",
            "start": [1.0, 2.0],
            "goal": [3.0, 2.0],
            "dynamic_obstacles": [{"trigger_time": 0.5, "x": 2.0, "y": 2.0, "radius": 0.4}],
        },
        "planner_only",
        cfg,
        {
            "terminal_state": "SUCCEEDED",
            "truth_map": tmap,
            "dynamic_truth_map": tmap,
            "plan_se2": [(1.0, 2.0, 0.0), (3.0, 2.0, 0.0)],
            "replan_se2": [(1.0, 2.0, 0.0), (3.0, 2.0, 0.0)],
            "t_request_sent": 1.0,
            "t_initial_response": 1.2,
            "t_obstacle_injected": 5.0,
            "t_post_event_response": 3.0,  # BEFORE injection -> must fail P05!
            "t_terminal_result": 6.0,
        },
        prov,
    )
    assert dyn["outcome"] == "FAIL"
    assert dyn["gates"]["dynamic_replan_handled"]["passed"] is False

    # 3. Stale-input stopping gate fails when angular rotation exceeds policy (PP11)
    stale = evaluate_trial_from_observations(
        {"id": "stale_input_stop", "category": "safety_stale_input", "start": [1, 2], "goal": [3, 2]},
        "nav_closed_loop",
        cfg,
        {
            "terminal_state": "CANCELED",
            "post_fault_commands": [(10.0, 0.0, 0.0, 1.0)],
        },
        prov,
    )
    assert stale["outcome"] == "FAIL"
    assert stale["gates"]["stale_input_fail_closed"]["passed"] is False

    # 4. Positive closed-loop navigation PASS + overlay plot export
    cmds = [(t, 0.5, 0.0, 0.0) for t in (0.0, 1.0, 2.0, 3.0, 4.0)]
    odom = [(t, 1.0 + 0.5 * t, 2.0, 0.0, 0.5, 0.0) for t in (0.0, 1.0, 2.0, 3.0, 4.0)]
    pos = evaluate_trial_from_observations(
        {
            "id": "empty_straight",
            "category": "positive_static",
            "start": [1.0, 2.0],
            "goal": [3.0, 2.0],
            "reference_path": [[1.0, 2.0], [3.0, 2.0]],
        },
        "nav_closed_loop",
        cfg,
        {
            "terminal_state": "SUCCEEDED",
            "truth_map": tmap,
            "plan_se2": [(1.0, 2.0, 0.0), (2.0, 2.0, 0.0), (3.0, 2.0, 0.0)],
            "t_request_sent": 0.0,
            "t_initial_response": 0.2,
            "t_terminal_result": 4.0,
            "cmd_history": cmds,
            "odom_history": odom,
            "cmd_vel_publishers": 1,
        },
        prov,
    )
    assert pos["outcome"] == "PASS"
    assert pos["diagnostic_score"] is not None
    assert validate_trial_record(pos) == []
    out_png = tmp_path / "overlay.png"
    assert plot_scenario_overlay(
        {"id": "empty_straight", "start": [1.0, 2.0], "goal": [3.0, 2.0], "reference_path": [[1.0, 2.0], [3.0, 2.0]]},
        tmap,
        [(1.0, 2.0), (3.0, 2.0)],
        [],
        [(o[1], o[2]) for o in odom],
        pos,
        out_png,
    )
    assert out_png.is_file()
