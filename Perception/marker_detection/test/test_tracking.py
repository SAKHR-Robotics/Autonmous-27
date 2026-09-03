import numpy as np
import pytest
from marker_detection.quality_flags import QualityFlag
from marker_detection.quaternion_filter import angular_difference_deg, slerp
from marker_detection.tracker_manager import RawPose, TrackState, TrackerManager


def manager(*, degraded_after: float = 0.10, lost_timeout: float = 0.20, track_timeout: float = 0.40,
           window: float = 0.20, min_confirmations: int = 3) -> TrackerManager:
    return TrackerManager(min_confirmations=min_confirmations, confirmation_window_seconds=window,
                          degraded_after_seconds=degraded_after, lost_timeout_seconds=lost_timeout,
                          track_timeout_seconds=track_timeout, process_noise=0.5, measurement_noise=0.01,
                          initial_covariance=0.1, orientation_alpha=0.25, innovation_gate_threshold=11.34,
                          max_orientation_innovation_deg=45.0, max_reprojection_error_px=3.0,
                          position_jump_tolerance_m=0.05, max_linear_velocity_mps=2.0)


def pose(marker_id: int, z: float, valid: bool = True, *, detection_confidence: float = 0.9,
        pose_quality: float = 0.9, depth_z_m: float | None = None, depth_z_variance: float | None = None) -> RawPose:
    return RawPose(marker_id, np.array([0.2, 0.1, z]), np.array([0.0, 0.0, 0.0, 1.0]), valid, 0.5,
                   detection_confidence=detection_confidence, pose_quality=pose_quality,
                   depth_z_m=depth_z_m, depth_z_variance=depth_z_variance)


def test_confirmation_is_time_aware_not_frame_counted():
    tracks = manager()
    assert tracks.process([pose(7, 2.0)], 0.0)[0].state == TrackState.NEW
    tracks.process([pose(7, 2.01)], 0.02)
    confirmed = tracks.process([pose(7, 1.99)], 0.04)[0]
    assert confirmed.state == TrackState.CONFIRMED
    assert confirmed.confidence > 0.0


def test_confirmed_track_survives_a_single_missed_frame_within_grace_period():
    tracks = manager()
    tracks.process([pose(7, 2.0)], 0.0)
    tracks.process([pose(7, 2.01)], 0.02)
    tracks.process([pose(7, 1.99)], 0.04)
    grace = tracks.process([], 0.09)[0]  # gap 0.05s, degraded_after_seconds=0.10
    assert grace.state == TrackState.CONFIRMED
    assert not grace.detection_valid


def test_track_degrades_then_is_lost_purely_by_elapsed_time():
    tracks = manager()
    tracks.process([pose(7, 2.0)], 0.0)
    tracks.process([pose(7, 2.01)], 0.02)
    tracks.process([pose(7, 1.99)], 0.04)
    degraded = tracks.process([], 0.16)[0]  # gap 0.12s: past degraded_after, within lost_timeout
    assert degraded.state == TrackState.DEGRADED
    assert QualityFlag.TRACK_DEGRADED & degraded.quality_flags
    lost = tracks.process([], 0.30)[0]      # gap 0.26s: past lost_timeout, within track_timeout
    assert lost.state == TrackState.LOST
    assert QualityFlag.TRACK_LOST & lost.quality_flags


def test_gross_outlier_is_rejected_without_disturbing_confirmed_state():
    tracks = manager()
    tracks.process([pose(7, 2.0)], 0.0)
    tracks.process([pose(7, 2.01)], 0.02)
    tracks.process([pose(7, 1.99)], 0.04)
    rejected = tracks.process([pose(7, 8.5)], 0.07)[0]
    assert not rejected.detection_valid
    assert rejected.state == TrackState.CONFIRMED  # still within the degraded grace period


def test_reacquisition_after_loss_requires_a_fresh_confirmation_window():
    tracks = manager()
    tracks.process([pose(7, 2.0)], 0.0)
    tracks.process([pose(7, 2.01)], 0.02)
    tracks.process([pose(7, 1.99)], 0.04)
    tracks.process([], 0.30)  # -> LOST
    reacquiring = tracks.process([pose(7, 2.0)], 0.40)[0]
    assert reacquiring.state == TrackState.NEW  # same track object, not a duplicate
    tracks.process([pose(7, 2.0)], 0.42)
    recovered = tracks.process([pose(7, 2.0)], 0.44)[0]
    assert recovered.state == TrackState.CONFIRMED
    assert len(tracks.tracks) == 1  # reappearance updated the same track, no duplicate created


def test_multiple_marker_filters_are_independent_and_stale_track_is_deleted():
    tracks = manager(degraded_after=0.05, lost_timeout=0.08, track_timeout=0.10)
    results = tracks.process([pose(1, 1.0), pose(5, 2.0), pose(17, 3.0)], 0.0)
    assert {result.marker_id for result in results} == {1, 5, 17}
    tracks.process([], 0.20)
    assert not tracks.tracks


def test_depth_z_fusion_pulls_the_filtered_z_toward_the_depth_measurement():
    tracks = manager()
    # PnP says Z=2.10m; an independent, tight-variance depth measurement says Z=2.00m.
    first = tracks.process([pose(7, 2.10, depth_z_m=2.00, depth_z_variance=1e-6)], 0.0)[0]
    assert first.depth_fused
    assert first.position[2] < 2.10  # pulled toward the depth measurement, not left at the raw PnP value
    assert abs(first.position[2] - 2.00) < 0.05


def test_position_covariance_is_exposed_and_shrinks_with_confirmations():
    tracks = manager()
    first = tracks.process([pose(7, 2.0)], 0.0)[0]
    assert first.position_covariance is not None and first.position_covariance.shape == (3, 3)
    initial_trace = float(np.trace(first.position_covariance))
    tracks.process([pose(7, 2.01)], 0.02)
    later = tracks.process([pose(7, 1.99)], 0.04)[0]
    assert float(np.trace(later.position_covariance)) < initial_trace


def test_confidence_is_the_conservative_minimum_of_the_three_components():
    tracks = manager()
    weak_detection = pose(7, 2.0, detection_confidence=0.2, pose_quality=0.95)
    result = tracks.process([weak_detection], 0.0)[0]
    assert result.confidence <= 0.2 + 1e-9


def test_age_seconds_is_zero_on_detection_and_grows_while_missing():
    tracks = manager()
    detected = tracks.process([pose(7, 2.0)], 0.0)[0]
    assert detected.age_seconds == 0.0
    still_missing = tracks.process([], 0.07)[0]  # gap 0.07s, no detection this update
    assert still_missing.age_seconds == pytest.approx(0.07, abs=1e-9)
    redetected = tracks.process([pose(7, 2.0)], 0.09)[0]
    assert redetected.age_seconds == 0.0  # back to zero the moment it is seen again


def test_quaternion_sign_flip_and_wrap_are_continuous():
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    assert angular_difference_deg(identity, -identity) == 0.0
    assert np.allclose(slerp(identity, -identity, 0.25), identity)
    plus_179 = np.array([0.0, 0.0, np.sin(np.deg2rad(179 / 2)), np.cos(np.deg2rad(179 / 2))])
    minus_179 = np.array([0.0, 0.0, np.sin(np.deg2rad(-179 / 2)), np.cos(np.deg2rad(-179 / 2))])
    assert abs(angular_difference_deg(plus_179, minus_179) - 2.0) < 1e-6
