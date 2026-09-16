import math
import numpy as np
import pytest
from marker_detection.action_target_builder import build_action_target
from marker_detection.quality_flags import QualityFlag
from marker_detection.tracker_manager import TrackState


def _confirmed_target(**overrides):
    defaults = dict(
        marker_id=17, detection_valid=True, tracking_valid=True,
        tracking_state=TrackState.CONFIRMED.value, confidence=0.9, quality_flags=int(QualityFlag.VALID),
        age_seconds=0.0, base_link_position=np.array([1.85, -0.42, 0.63]),
        base_link_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        base_link_position_covariance=np.diag([0.001, 0.001, 0.001]),
        min_action_confidence=0.6, max_action_covariance_trace_m2=0.01)
    defaults.update(overrides)
    return build_action_target(**defaults)


def test_stable_and_usable_when_all_conditions_are_satisfied():
    target = _confirmed_target()
    assert target.stable is True
    assert target.pose_valid is True
    assert target.usable_for_action is True


def test_distances_match_the_worked_example_in_the_spec():
    target = _confirmed_target()
    assert target.forward_distance == pytest.approx(1.85)
    assert target.lateral_distance == pytest.approx(-0.42)
    assert target.vertical_distance == pytest.approx(0.63)
    expected_euclidean = math.sqrt(1.85 ** 2 + 0.42 ** 2 + 0.63 ** 2)
    assert target.distance_to_marker == pytest.approx(expected_euclidean)
    assert target.distance_to_marker == pytest.approx(1.99, abs=0.01)


def test_missing_base_link_pose_is_never_fabricated():
    """Part 4/19: TF unavailable -> no invented pose, no invented distances."""
    target = _confirmed_target(base_link_position=None, base_link_orientation=None)
    assert target.pose_valid is False
    assert target.position is None and target.orientation is None
    assert math.isnan(target.distance_to_marker)
    assert math.isnan(target.forward_distance)
    assert math.isnan(target.lateral_distance)
    assert math.isnan(target.vertical_distance)
    assert target.usable_for_action is False
    assert QualityFlag.TF_UNAVAILABLE & target.quality_flags


def test_not_stable_when_track_is_only_new():
    target = _confirmed_target(tracking_state=TrackState.NEW.value)
    assert target.stable is False
    assert target.usable_for_action is False  # not stable -> never usable, even with a valid pose


def test_not_usable_when_degraded_even_with_a_valid_pose():
    target = _confirmed_target(tracking_state=TrackState.DEGRADED.value,
                               quality_flags=int(QualityFlag.TRACK_DEGRADED))
    assert target.stable is False
    assert target.usable_for_action is False


def test_stale_or_lost_track_can_never_be_usable_for_action():
    lost = _confirmed_target(tracking_state=TrackState.LOST.value, detection_valid=False,
                             quality_flags=int(QualityFlag.TRACK_LOST))
    assert lost.usable_for_action is False
    stale = _confirmed_target(quality_flags=int(QualityFlag.STALE_DATA))
    assert stale.usable_for_action is False


def test_low_confidence_is_not_usable_even_when_stable():
    target = _confirmed_target(confidence=0.3, min_action_confidence=0.6)
    assert target.stable is True
    assert target.usable_for_action is False


def test_high_covariance_is_not_usable_even_when_stable_and_confident():
    high_covariance = np.diag([0.05, 0.05, 0.05])  # trace 0.15 >> 0.01 threshold
    target = _confirmed_target(base_link_position_covariance=high_covariance)
    assert target.usable_for_action is False


def test_pose_ambiguous_flag_blocks_usable_for_action():
    target = _confirmed_target(quality_flags=int(QualityFlag.POSE_AMBIGUOUS))
    assert target.usable_for_action is False


def test_benign_flags_do_not_block_usable_for_action():
    """LOW_MARKER_RESOLUTION/HIGH_VIEWING_ANGLE are informational, not
    disqualifying -- only the flags in _DISQUALIFYING_FLAGS should gate."""
    target = _confirmed_target(quality_flags=int(QualityFlag.LOW_MARKER_RESOLUTION | QualityFlag.HIGH_VIEWING_ANGLE))
    assert target.usable_for_action is True


def test_detected_flag_reflects_input_directly():
    target = _confirmed_target(detection_valid=False)
    assert target.detected is False
    assert target.usable_for_action is False
