"""Builds the single high-level marker interface for manipulation/planning
nodes (Parts 3, 5, 6, 7 of the ERC downstream-interface hardening pass), from
the existing Stage 4 tracked-pose output plus an optional TF2-resolved
base_link pose (frame_transform.py).

Kept ROS-free and pure so it is fully unit-testable without rclpy (see
test/test_action_target_builder.py); marker_action_interface_node.py is the
thin ROS wrapper that supplies the MarkerTrackedPose fields and the
TF-resolved base_link pose.

`stable` and `usable_for_action` are derived entirely from the *existing*
tracking state machine (tracker_manager.TrackState) and quality-flag bitmask
(quality_flags.QualityFlag) -- this module introduces no second tracking or
quality system, per the "reuse, do not duplicate" requirement.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .quality_flags import QualityFlag
from .tracker_manager import TrackState

# Existing flags (quality_flags.py) that disqualify a track from
# usable_for_action regardless of confidence/covariance -- reusing the
# package's existing flag vocabulary rather than a parallel disqualification
# list. TF_UNAVAILABLE is folded in automatically whenever no base_link pose
# could be resolved (see build_action_target below), even if the caller did
# not already set it on quality_flags.
_DISQUALIFYING_FLAGS = (
    QualityFlag.TRACK_LOST | QualityFlag.TRACK_DEGRADED | QualityFlag.TEMPORAL_OUTLIER |
    QualityFlag.POSE_AMBIGUOUS | QualityFlag.TF_UNAVAILABLE | QualityFlag.STALE_DATA)


@dataclass(frozen=True)
class ActionTarget:
    marker_id: int
    detected: bool
    stable: bool
    usable_for_action: bool
    tracking_state: str
    pose_valid: bool
    position: np.ndarray | None           # base_link (m); None if pose_valid is False.
    orientation: np.ndarray | None        # base_link (x, y, z, w); None if pose_valid is False.
    distance_to_marker: float             # sqrt(x^2+y^2+z^2) in base_link; NaN if pose_valid is False.
    forward_distance: float               # base_link +X; NaN if pose_valid is False.
    lateral_distance: float                # base_link +Y (left-positive, REP-103); NaN if pose_valid is False.
    vertical_distance: float               # base_link +Z (up-positive, REP-103); NaN if pose_valid is False.
    confidence: float
    quality_flags: int
    position_covariance: np.ndarray | None  # base_link-frame 3x3 (m^2); None if unavailable.
    age_seconds: float


def build_action_target(*, marker_id: int, detection_valid: bool, tracking_valid: bool,
                        tracking_state: str, confidence: float, quality_flags: int, age_seconds: float,
                        base_link_position: np.ndarray | None, base_link_orientation: np.ndarray | None,
                        base_link_position_covariance: np.ndarray | None,
                        min_action_confidence: float, max_action_covariance_trace_m2: float) -> ActionTarget:
    """Pure function: one Stage-4 track's fields + an optional resolved
    base_link pose -> the clean, high-level ActionTarget.

    Never fabricates a base_link pose: if `base_link_position`/
    `base_link_orientation` is None (TF lookup failed/unavailable), position,
    orientation, and all three distance fields are invalid/NaN and
    usable_for_action is forced False, with TF_UNAVAILABLE folded into the
    reported flags (Part 4/Part 19).
    """
    flags = QualityFlag(quality_flags)
    stable = bool(detection_valid and tracking_valid and tracking_state == TrackState.CONFIRMED.value)
    pose_valid = base_link_position is not None and base_link_orientation is not None

    if not pose_valid:
        flags |= QualityFlag.TF_UNAVAILABLE
        distance = forward = lateral = vertical = float("nan")
    else:
        forward = float(base_link_position[0])
        lateral = float(base_link_position[1])
        vertical = float(base_link_position[2])
        distance = float(np.linalg.norm(base_link_position))

    covariance_trace = (float(np.trace(base_link_position_covariance))
                        if base_link_position_covariance is not None else float("inf"))
    usable_for_action = bool(
        pose_valid and stable and detection_valid and tracking_valid and
        np.isfinite(confidence) and confidence >= min_action_confidence and
        covariance_trace <= max_action_covariance_trace_m2 and
        not (flags & _DISQUALIFYING_FLAGS))

    return ActionTarget(marker_id, detection_valid, stable, usable_for_action, tracking_state, pose_valid,
                        base_link_position, base_link_orientation, distance, forward, lateral, vertical,
                        confidence, int(flags), base_link_position_covariance, age_seconds)
