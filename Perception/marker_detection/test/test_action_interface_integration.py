"""Integration-level test (Part 15) covering:

    tracked pose (Stage 4, tracker_manager)
        -> TF2-resolved camera->base_link composition (frame_transform)
        -> clean downstream marker interface (action_target_builder)
        -> distance calculation
        -> usable_for_action

This does not stand up rclpy/TF2/a real camera; it exercises the same pure
functions marker_action_interface_node.py wires together, with a synthetic
(translation, quaternion) standing in for what a real TF2 lookup would
return. Kept lightweight and consistent with the rest of the test suite
(see test_tracking.py, test_frame_transform.py, test_action_target_builder.py
for the per-module tests this composes).
"""
import numpy as np
from marker_detection.action_target_builder import build_action_target
from marker_detection.frame_transform import transform_to_base_link
from marker_detection.quality_flags import QualityFlag
from marker_detection.tracker_manager import RawPose, TrackState, TrackerManager


def _tracker() -> TrackerManager:
    return TrackerManager(min_confirmations=3, confirmation_window_seconds=1.0,
                          degraded_after_seconds=0.5, lost_timeout_seconds=1.5, track_timeout_seconds=5.0,
                          process_noise=0.5, measurement_noise=0.01, initial_covariance=0.1,
                          orientation_alpha=0.25, innovation_gate_threshold=11.34,
                          max_orientation_innovation_deg=45.0, max_reprojection_error_px=3.0,
                          position_jump_tolerance_m=0.05, max_linear_velocity_mps=2.0)


def _detection(marker_id: int, position: np.ndarray) -> RawPose:
    return RawPose(marker_id, position, np.array([0.0, 0.0, 0.0, 1.0]), True, 0.5,
                   position_noise=np.diag([1e-4, 1e-4, 1e-4]),
                   detection_confidence=0.95, pose_quality=0.95)


# A rover with the camera mounted 0.30m forward / 0.20m up from base_link,
# with no rotation between the two frames (the common, simplest mounting).
_CAMERA_IN_BASE_LINK_TRANSLATION = np.array([0.30, 0.0, 0.20])
_CAMERA_IN_BASE_LINK_ROTATION = np.array([0.0, 0.0, 0.0, 1.0])


def _resolve(tracked_position: np.ndarray, tracked_orientation: np.ndarray, covariance: np.ndarray | None):
    return transform_to_base_link(
        transform_translation=_CAMERA_IN_BASE_LINK_TRANSLATION, transform_rotation=_CAMERA_IN_BASE_LINK_ROTATION,
        camera_position=tracked_position, camera_orientation=tracked_orientation,
        camera_position_covariance=covariance)


def test_full_pipeline_confirms_and_becomes_usable_for_action():
    tracker = _tracker()
    position = np.array([1.55, -0.42, 0.43])  # camera-frame; base_link adds (0.30, 0, 0.20).
    tracker.process([_detection(17, position)], 0.00)
    tracker.process([_detection(17, position)], 0.02)
    tracked = tracker.process([_detection(17, position)], 0.04)[0]
    assert tracked.state == TrackState.CONFIRMED  # Stage 4: confirmed after 3 detections in-window.

    resolved = _resolve(tracked.position, tracked.orientation, tracked.position_covariance)
    assert resolved is not None  # TF2 lookup succeeded (synthetic, but never fabricated on failure).

    target = build_action_target(
        marker_id=tracked.marker_id, detection_valid=tracked.detection_valid, tracking_valid=tracked.tracking_valid,
        tracking_state=tracked.state.value, confidence=tracked.confidence, quality_flags=tracked.quality_flags,
        age_seconds=tracked.age_seconds, base_link_position=resolved.position,
        base_link_orientation=resolved.orientation, base_link_position_covariance=resolved.position_covariance,
        min_action_confidence=0.5, max_action_covariance_trace_m2=1.0)

    assert target.stable is True
    assert target.pose_valid is True
    # Camera-frame X=1.55 + 0.30 offset -> base_link forward_distance == 1.85.
    assert target.forward_distance == 1.55 + 0.30
    assert target.lateral_distance == -0.42
    assert target.vertical_distance == 0.43 + 0.20
    assert target.usable_for_action is True


def test_full_pipeline_reports_unusable_while_track_is_still_new():
    tracker = _tracker()
    position = np.array([1.0, 0.0, 0.5])
    tracked = tracker.process([_detection(3, position)], 0.0)[0]
    assert tracked.state == TrackState.NEW

    resolved = _resolve(tracked.position, tracked.orientation, tracked.position_covariance)
    target = build_action_target(
        marker_id=tracked.marker_id, detection_valid=tracked.detection_valid, tracking_valid=tracked.tracking_valid,
        tracking_state=tracked.state.value, confidence=tracked.confidence, quality_flags=tracked.quality_flags,
        age_seconds=tracked.age_seconds, base_link_position=resolved.position if resolved else None,
        base_link_orientation=resolved.orientation if resolved else None,
        base_link_position_covariance=resolved.position_covariance if resolved else None,
        min_action_confidence=0.5, max_action_covariance_trace_m2=1.0)

    assert target.pose_valid is True  # TF/geometry succeeded...
    assert target.stable is False     # ...but tracking has not confirmed yet...
    assert target.usable_for_action is False  # ...so it must not be actionable.


def test_full_pipeline_reports_unavailable_pose_when_tf_lookup_fails():
    """No synthetic transform this time -- simulates a real TF2 lookup failure
    (TransformException) that marker_action_interface_node.py would catch and
    turn into `resolved=None`; the base_link pose must not be fabricated."""
    tracker = _tracker()
    position = np.array([1.0, 0.0, 0.5])
    tracker.process([_detection(9, position)], 0.00)
    tracker.process([_detection(9, position)], 0.02)
    tracked = tracker.process([_detection(9, position)], 0.04)[0]
    assert tracked.state == TrackState.CONFIRMED

    target = build_action_target(
        marker_id=tracked.marker_id, detection_valid=tracked.detection_valid, tracking_valid=tracked.tracking_valid,
        tracking_state=tracked.state.value, confidence=tracked.confidence, quality_flags=tracked.quality_flags,
        age_seconds=tracked.age_seconds, base_link_position=None, base_link_orientation=None,
        base_link_position_covariance=None, min_action_confidence=0.5, max_action_covariance_trace_m2=1.0)

    assert target.stable is True             # tracking itself is fine...
    assert target.pose_valid is False        # ...but base_link pose could not be resolved...
    assert target.usable_for_action is False  # ...so it is correctly reported as not actionable.
    assert QualityFlag.TF_UNAVAILABLE & target.quality_flags


def test_depth_loss_still_produces_a_pnp_only_pose_that_becomes_usable():
    """Mirrors Part 2's requirement at the tracking layer: a track with no
    depth-Z fusion (depth_z_m/variance never supplied) still reaches
    CONFIRMED and can become usable_for_action -- RGB-only PnP tracking is
    not degraded by the absence of depth."""
    tracker = _tracker()
    position = np.array([2.0, 0.1, 0.3])
    detection = RawPose(5, position, np.array([0.0, 0.0, 0.0, 1.0]), True, 0.5,
                        position_noise=np.diag([1e-4, 1e-4, 1e-4]), detection_confidence=0.9, pose_quality=0.9,
                        depth_z_m=None, depth_z_variance=None)  # depth unavailable this whole test
    tracker.process([detection], 0.00)
    tracker.process([detection], 0.02)
    tracked = tracker.process([detection], 0.04)[0]
    assert tracked.state == TrackState.CONFIRMED
    assert tracked.depth_fused is False

    resolved = _resolve(tracked.position, tracked.orientation, tracked.position_covariance)
    target = build_action_target(
        marker_id=tracked.marker_id, detection_valid=tracked.detection_valid, tracking_valid=tracked.tracking_valid,
        tracking_state=tracked.state.value, confidence=tracked.confidence, quality_flags=tracked.quality_flags,
        age_seconds=tracked.age_seconds, base_link_position=resolved.position,
        base_link_orientation=resolved.orientation, base_link_position_covariance=resolved.position_covariance,
        min_action_confidence=0.5, max_action_covariance_trace_m2=1.0)
    assert target.usable_for_action is True
