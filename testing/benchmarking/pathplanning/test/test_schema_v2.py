"""Unit tests for the unified v2.0.0 schema, provenance, and Two-Tier gating."""
import json
import math

import pytest

from global_path_benchmarking.schema import (
    SCHEMA_VERSION,
    build_provenance,
    build_trial_record,
    evaluate_outcome,
    gate,
    sanitize_metrics,
    validate_trial_record,
    write_suite_summary,
    write_trial_record,
)


def _sample_provenance():
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
        claim_scope="Mocked closed-loop test only.",
        seed=7,
    )


def test_provenance_rejects_invalid_dependency_source():
    with pytest.raises(ValueError):
        build_provenance(
            evaluation_mode="planner_only",
            dependency_manifest={"map": "IMAGINARY"},
            truth_source="independent_scenario_yaml",
            claim_scope="test",
        )


def test_two_tier_gating_precedence_and_score_suppression():
    prov = _sample_provenance()

    # Passing trial preserves finite diagnostic score
    passed = build_trial_record(
        provenance=prov,
        scenario_id="empty_straight",
        category="positive_static",
        expected_outcome="PASS",
        gates={"collision_free": gate(True, "ok"), "arrived": gate(True, "ok")},
        metrics={"planning_time_s": 0.4, "nan_val": float("nan")},
        diagnostic_score=91.5,
    )
    assert passed["schema_version"] == SCHEMA_VERSION
    assert passed["outcome"] == "PASS"
    assert passed["diagnostic_score"] == pytest.approx(91.5)
    assert passed["metrics"]["nan_val"] is None
    assert validate_trial_record(passed) == []

    # Failed gate suppresses diagnostic_score even if supplied
    failed = build_trial_record(
        provenance=prov,
        scenario_id="blocked_wall",
        category="negative_infeasible",
        expected_outcome="PASS",
        gates={"expected_rejected": gate(False, "returned path"), "other": gate(True, "ok")},
        metrics={"planning_time_s": 0.2},
        diagnostic_score=99.0,
    )
    assert failed["outcome"] == "FAIL"
    assert failed["diagnostic_score"] is None
    assert validate_trial_record(failed) == []


def test_all_six_mutually_exclusive_outcomes():
    assert evaluate_outcome({"a": gate(True)}, blocked_reason="no gazebo") == "BLOCKED"
    assert evaluate_outcome({"a": gate(False)}, invalid_reason="nan frame") == "INVALID"
    assert evaluate_outcome({"a": gate(True)}, terminal_state="TIMEOUT") == "TIMEOUT"
    assert evaluate_outcome({"a": gate(False), "b": gate(None)}) == "FAIL"
    assert evaluate_outcome({"a": gate(True), "b": gate(None)}) == "INCOMPLETE"
    assert evaluate_outcome({}) == "INCOMPLETE"
    assert evaluate_outcome({"a": gate(True)}) == "PASS"


def test_sanitize_metrics_and_disk_write(tmp_path):
    cleaned = sanitize_metrics(
        {"ok": 1.25, "nan": float("nan"), "inf": math.inf, "flag": True}
    )
    assert cleaned == {"ok": 1.25, "nan": None, "inf": None, "flag": True}

    prov = _sample_provenance()
    rec = build_trial_record(
        provenance=prov,
        scenario_id="s1",
        category="positive_static",
        expected_outcome="PASS",
        gates={"g": gate(True, "ok")},
        metrics=cleaned,
    )
    trial_path = write_trial_record(rec, tmp_path / "trials" / "s1.json")
    summary_path = write_suite_summary([rec], prov, tmp_path / "results_v2.json")

    loaded_trial = json.loads(trial_path.read_text())
    loaded_summary = json.loads(summary_path.read_text())
    assert loaded_trial["outcome"] == "PASS"
    assert loaded_summary["summary"]["counts"]["PASS"] == 1
    assert loaded_summary["summary"]["all_passed"] is True
