"""Unified v2.0.0 benchmark provenance, Two-Tier gating, and JSON schema helpers."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
from typing import Any
import uuid

SCHEMA_VERSION = "2.0.0"

VALID_OUTCOMES = (
    "PASS",
    "FAIL",
    "TIMEOUT",
    "INVALID",
    "INCOMPLETE",
    "BLOCKED",
)

VALID_DEP_SOURCES = (
    "REAL",
    "MOCK",
    "GAZEBO",
    "ROSBAG",
    "DISABLED",
    "WALL",
    "SIM",
)

REQUIRED_PROVENANCE_KEYS = (
    "run_id",
    "timestamp_utc",
    "git_commit",
    "git_dirty",
    "ros_distro",
    "docker_image_id",
    "use_sim_time",
    "seed",
    "evaluation_mode",
    "dependency_manifest",
    "truth_source",
    "claim_scope",
)

REQUIRED_RECORD_KEYS = (
    "schema_version",
    "provenance",
    "scenario",
    "outcome",
    "gates",
    "metrics",
    "diagnostic_score",
    "phases",
    "artifacts",
)


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _git_metadata(repo_root: str | Path | None = None) -> tuple[str | None, bool | None]:
    cwd = str(repo_root) if repo_root is not None else str(Path(__file__).resolve().parent)
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        ).strip()
        porcelain = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
        return (sha or None), bool(porcelain.strip())
    except Exception:
        return None, None


def build_provenance(
    *,
    evaluation_mode: str,
    dependency_manifest: dict[str, str],
    truth_source: str,
    claim_scope: str,
    seed: int = 0,
    run_id: str | None = None,
    ros_distro: str | None = None,
    docker_image_id: str | None = None,
    use_sim_time: bool = False,
    repo_root: str | Path | None = None,
) -> dict[str, Any]:
    """Build a mandatory provenance envelope distinguishing mocks from real runs."""
    if not isinstance(dependency_manifest, dict) or not dependency_manifest:
        raise ValueError("dependency_manifest must be a non-empty dictionary")
    normalized_manifest: dict[str, str] = {}
    for key, source in dependency_manifest.items():
        source_upper = str(source).upper()
        if source_upper not in VALID_DEP_SOURCES:
            raise ValueError(
                f"Unsupported dependency source {source!r} for {key!r}; "
                f"expected one of {VALID_DEP_SOURCES}"
            )
        normalized_manifest[str(key)] = source_upper

    now_utc = datetime.now(timezone.utc)
    stamp = now_utc.strftime("%Y-%m-%dT%H:%M:%SZ")
    compact = now_utc.strftime("%Y%m%dT%H%M%SZ")
    git_commit, git_dirty = _git_metadata(repo_root)

    return {
        "run_id": run_id or f"{compact}_{evaluation_mode}_{uuid.uuid4().hex[:8]}",
        "timestamp_utc": stamp,
        "git_commit": git_commit,
        "git_dirty": git_dirty,
        "ros_distro": ros_distro if ros_distro is not None else os.environ.get("ROS_DISTRO"),
        "docker_image_id": (
            docker_image_id
            if docker_image_id is not None
            else os.environ.get("AUTONOMOUS27_IMAGE_ID")
        ),
        "use_sim_time": bool(use_sim_time),
        "seed": int(seed),
        "evaluation_mode": str(evaluation_mode),
        "dependency_manifest": normalized_manifest,
        "truth_source": str(truth_source),
        "claim_scope": str(claim_scope),
    }


def gate(passed: bool | None, detail: str = "") -> dict[str, Any]:
    """Create a Tier A binary/unavailable gate record."""
    if passed is not None and not isinstance(passed, bool):
        raise TypeError("Gate 'passed' must be bool or None")
    return {"passed": passed, "detail": str(detail)}


def evaluate_outcome(
    gates: dict[str, dict[str, Any]],
    *,
    terminal_state: str | None = None,
    blocked_reason: str | None = None,
    invalid_reason: str | None = None,
) -> str:
    """Resolve mutually exclusive trial outcome under Two-Tier hard gating."""
    if blocked_reason:
        return "BLOCKED"
    if invalid_reason:
        return "INVALID"
    if terminal_state == "TIMEOUT":
        return "TIMEOUT"
    if terminal_state in ("INVALID", "BLOCKED"):
        return terminal_state
    if any(entry.get("passed") is False for entry in gates.values()):
        return "FAIL"
    if not gates or any(entry.get("passed") is None for entry in gates.values()):
        return "INCOMPLETE"
    if terminal_state == "INCOMPLETE":
        return "INCOMPLETE"
    if terminal_state == "FAIL":
        return "FAIL"
    return "PASS"


def _sanitize_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value) if math.isfinite(value) else None
    if hasattr(value, "item") and callable(value.item):
        return _sanitize_value(value.item())
    if isinstance(value, dict):
        return {str(k): _sanitize_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize_value(item) for item in value]
    return str(value)


def sanitize_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    """Convert raw SI measurements to JSON-safe values; nonfinite floats become None."""
    return {str(k): _sanitize_value(v) for k, v in (metrics or {}).items()}


def build_trial_record(
    *,
    provenance: dict[str, Any],
    scenario_id: str,
    category: str,
    expected_outcome: str,
    gates: dict[str, dict[str, Any]],
    metrics: dict[str, Any],
    diagnostic_score: float | None = None,
    phases: list[dict[str, Any]] | None = None,
    artifacts: dict[str, Any] | None = None,
    terminal_state: str | None = None,
    blocked_reason: str | None = None,
    invalid_reason: str | None = None,
) -> dict[str, Any]:
    """Construct a complete v2.0.0 trial record enforcing Two-Tier scoring."""
    outcome = evaluate_outcome(
        gates,
        terminal_state=terminal_state,
        blocked_reason=blocked_reason,
        invalid_reason=invalid_reason,
    )
    sanitized_score = (
        float(diagnostic_score)
        if outcome == "PASS" and _is_finite_number(diagnostic_score)
        else None
    )
    sanitized_phases = _sanitize_value(phases or [])
    sanitized_artifacts = _sanitize_value(artifacts or {})
    scenario_block: dict[str, Any] = {
        "id": str(scenario_id),
        "category": str(category),
        "expected_outcome": str(expected_outcome),
    }
    if blocked_reason:
        scenario_block["blocked_reason"] = str(blocked_reason)
    if invalid_reason:
        scenario_block["invalid_reason"] = str(invalid_reason)

    return {
        "schema_version": SCHEMA_VERSION,
        "provenance": dict(provenance),
        "scenario": scenario_block,
        "outcome": outcome,
        "gates": {str(k): dict(v) for k, v in gates.items()},
        "metrics": sanitize_metrics(metrics),
        "diagnostic_score": sanitized_score,
        "phases": sanitized_phases,
        "artifacts": sanitized_artifacts,
    }


def _check_finite_json(value: Any, path: str, errors: list[str]) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        errors.append(f"Non-finite float at {path}")
    elif isinstance(value, dict):
        for key, item in value.items():
            _check_finite_json(item, f"{path}.{key}", errors)
    elif isinstance(value, list):
        for idx, item in enumerate(value):
            _check_finite_json(item, f"{path}[{idx}]", errors)


def validate_trial_record(record: dict[str, Any]) -> list[str]:
    """Return a list of schema violations for a v2.0.0 trial record (empty means valid)."""
    errors: list[str] = []
    if not isinstance(record, dict):
        return ["Record must be a dictionary"]

    for key in REQUIRED_RECORD_KEYS:
        if key not in record:
            errors.append(f"Missing required key: {key}")

    if record.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"Unsupported schema_version: {record.get('schema_version')!r}")

    outcome = record.get("outcome")
    if outcome not in VALID_OUTCOMES:
        errors.append(f"Invalid outcome: {outcome!r}")

    prov = record.get("provenance")
    if not isinstance(prov, dict):
        errors.append("provenance must be a dict")
    else:
        for pkey in REQUIRED_PROVENANCE_KEYS:
            if pkey not in prov:
                errors.append(f"Missing provenance key: {pkey}")

    scen = record.get("scenario")
    if not isinstance(scen, dict):
        errors.append("scenario must be a dict")
    else:
        for skey in ("id", "category", "expected_outcome"):
            if not scen.get(skey):
                errors.append(f"Missing scenario.{skey}")

    gates = record.get("gates")
    if not isinstance(gates, dict):
        errors.append("gates must be a dict")
    else:
        for gname, gval in gates.items():
            if not isinstance(gval, dict) or "passed" not in gval or "detail" not in gval:
                errors.append(f"Gate {gname!r} must contain 'passed' and 'detail'")
            elif gval["passed"] is not None and not isinstance(gval["passed"], bool):
                errors.append(f"Gate {gname!r} 'passed' must be bool or None")

    if outcome != "PASS" and record.get("diagnostic_score") is not None:
        errors.append("diagnostic_score must be None when outcome != 'PASS'")

    _check_finite_json(record.get("metrics", {}), "metrics", errors)
    return errors


def write_trial_record(record: dict[str, Any], output_path: str | Path) -> Path:
    """Validate and write a single trial record as strict JSON."""
    errors = validate_trial_record(record)
    if errors:
        raise ValueError("Invalid trial record: " + "; ".join(errors))
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    return target


def write_suite_summary(
    records: list[dict[str, Any]],
    provenance: dict[str, Any],
    output_path: str | Path,
) -> Path:
    """Write a suite summary JSON combining all scheduled trials."""
    counts = {outcome: 0 for outcome in VALID_OUTCOMES}
    for rec in records:
        errors = validate_trial_record(rec)
        if errors:
            raise ValueError("Invalid trial record in suite: " + "; ".join(errors))
        counts[rec["outcome"]] += 1

    total = len(records)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "provenance": dict(provenance),
        "summary": {
            "total_trials": total,
            "counts": counts,
            "all_passed": bool(total > 0 and counts["PASS"] == total),
        },
        "trials": records,
    }
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    return target
