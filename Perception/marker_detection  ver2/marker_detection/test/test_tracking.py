import numpy as np
import pytest
from marker_detection.quality_flags import QualityFlag
from marker_detection.quaternion_filter import angular_difference_deg, slerp
from marker_detection.tracker_manager import (
    RawPose, TrackState, TrackerManager, _candidate_rank_key, select_canonical_with_source)


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


def dup_pose(marker_id: int, x: float, y: float, z: float, *, valid: bool = True,
            reprojection_error_px: float = 0.5, viewing_angle_deg: float = 10.0,
            pose_quality: float = 0.9, detection_confidence: float = 0.9,
            depth_z_m: float | None = None, depth_z_variance: float | None = None) -> RawPose:
    """Like pose(), but with an independent (x, y, z) so two same-ID candidates
    can represent two distinct physical faces for the Section-8 duplicate-ID tests."""
    return RawPose(marker_id, np.array([x, y, z]), np.array([0.0, 0.0, 0.0, 1.0]), valid,
                   reprojection_error_px, detection_confidence=detection_confidence, pose_quality=pose_quality,
                   viewing_angle_deg=viewing_angle_deg, depth_z_m=depth_z_m, depth_z_variance=depth_z_variance)


def test_two_same_id_detections_do_not_silently_overwrite_each_other():
    """Section 8: the naive {item.marker_id: item for item in measurements}
    silently keeps whichever candidate is last in the input list. Assert the
    tracker instead makes an explicit, quality-based choice and flags it."""
    tracks = manager()
    better = dup_pose(9, 0.0, 0.0, 2.0, pose_quality=0.95, reprojection_error_px=0.4)
    worse = dup_pose(9, 5.0, 5.0, 2.0, pose_quality=0.30, reprojection_error_px=2.9)
    result = tracks.process([worse, better], 0.0)  # worse listed first/last-wins would pick it
    assert len(result) == 1
    assert result[0].position[0] == pytest.approx(0.0, abs=1e-6)  # picked `better`, not whichever came last
    assert QualityFlag.MULTIPLE_INSTANCES_SAME_ID & result[0].quality_flags
    assert len(tracks.tracks) == 1  # never two tracks for one ID


def test_duplicate_selection_is_independent_of_input_order():
    tracks_a = manager()
    tracks_b = manager()
    better = dup_pose(9, 0.0, 0.0, 2.0, pose_quality=0.95, reprojection_error_px=0.4)
    worse = dup_pose(9, 5.0, 5.0, 2.0, pose_quality=0.30, reprojection_error_px=2.9)
    result_a = tracks_a.process([better, worse], 0.0)[0]
    result_b = tracks_b.process([worse, better], 0.0)[0]
    assert result_a.position[0] == pytest.approx(result_b.position[0], abs=1e-9)
    assert result_a.position[0] == pytest.approx(0.0, abs=1e-6)


def test_three_same_id_detections_pick_one_deterministic_winner():
    tracks = manager()
    candidates = [
        dup_pose(9, 1.0, 0.0, 2.0, pose_quality=0.5, reprojection_error_px=1.5),
        dup_pose(9, 2.0, 0.0, 2.0, pose_quality=0.9, reprojection_error_px=0.3),  # best
        dup_pose(9, 3.0, 0.0, 2.0, pose_quality=0.7, reprojection_error_px=0.8),
    ]
    result = tracks.process(candidates, 0.0)
    assert len(result) == 1
    assert result[0].position[0] == pytest.approx(2.0, abs=1e-6)
    assert QualityFlag.MULTIPLE_INSTANCES_SAME_ID & result[0].quality_flags


def test_reversed_detection_order_still_picks_the_same_winner():
    tracks_forward = manager()
    tracks_reversed = manager()
    candidates = [
        dup_pose(9, 1.0, 0.0, 2.0, pose_quality=0.5, reprojection_error_px=1.5),
        dup_pose(9, 2.0, 0.0, 2.0, pose_quality=0.9, reprojection_error_px=0.3),
        dup_pose(9, 3.0, 0.0, 2.0, pose_quality=0.7, reprojection_error_px=0.8),
    ]
    forward = tracks_forward.process(candidates, 0.0)[0]
    reversed_result = tracks_reversed.process(list(reversed(candidates)), 0.0)[0]
    assert forward.position[0] == pytest.approx(reversed_result.position[0], abs=1e-9)


def test_one_valid_pose_beats_one_invalid_pose_regardless_of_quality_score():
    tracks = manager()
    invalid_but_high_score = dup_pose(9, 9.0, 9.0, 2.0, valid=False, pose_quality=0.99)
    valid_but_lower_score = dup_pose(9, 1.0, 1.0, 2.0, valid=True, pose_quality=0.40)
    result = tracks.process([invalid_but_high_score, valid_but_lower_score], 0.0)[0]
    assert result.position[0] == pytest.approx(1.0, abs=1e-6)


def test_duplicate_can_appear_and_disappear_across_frames_without_creating_extra_tracks():
    tracks = manager()
    tracks.process([dup_pose(9, 0.0, 0.0, 2.0)], 0.0)
    assert len(tracks.tracks) == 1
    duplicated = tracks.process(
        [dup_pose(9, 0.0, 0.0, 2.01, pose_quality=0.9), dup_pose(9, 5.0, 5.0, 2.0, pose_quality=0.2)], 0.02)
    assert len(tracks.tracks) == 1
    assert QualityFlag.MULTIPLE_INSTANCES_SAME_ID & duplicated[0].quality_flags
    single_again = tracks.process([dup_pose(9, 0.0, 0.0, 1.99, pose_quality=0.9)], 0.04)[0]
    assert len(tracks.tracks) == 1
    assert not (QualityFlag.MULTIPLE_INSTANCES_SAME_ID & single_again.quality_flags)


def test_duplicate_candidates_with_different_depth_quality_prefer_the_one_with_valid_depth():
    tracks = manager()
    no_depth = dup_pose(9, 8.0, 8.0, 2.0, pose_quality=0.9, reprojection_error_px=0.3)
    with_depth = dup_pose(9, 1.0, 1.0, 2.0, pose_quality=0.9, reprojection_error_px=0.3,
                          depth_z_m=2.0, depth_z_variance=1e-4)
    result = tracks.process([no_depth, with_depth], 0.0)[0]
    assert result.position[0] == pytest.approx(1.0, abs=1e-6)


def test_candidate_rank_key_ties_break_on_value_not_order():
    identical_a = dup_pose(9, 1.0, 0.0, 2.0)
    identical_b = dup_pose(9, 1.0, 0.0, 2.0)
    assert _candidate_rank_key(identical_a) == _candidate_rank_key(identical_b)


def test_select_canonical_with_source_gives_the_debug_image_the_same_pick_as_tracking():
    """Priority 1 (debug-image consistency): the primary debug image must draw the
    same canonical pick TrackerManager.process() tracks, recovered via the matching
    'source' object (standing in for AssociatedPose, which carries 2D corners that
    RawPose does not) -- never a second, independently-computed selection."""
    better = dup_pose(9, 0.0, 0.0, 2.0, pose_quality=0.95, reprojection_error_px=0.4)
    worse = dup_pose(9, 5.0, 5.0, 2.0, pose_quality=0.30, reprojection_error_px=2.9)
    single = dup_pose(3, 1.0, 1.0, 1.0)
    measurements = [worse, single, better]  # "better" listed last on purpose
    sources = ["worse-source", "single-source", "better-source"]
    canonical_measurements, canonical_sources, duplicate_ids = select_canonical_with_source(measurements, sources)
    assert duplicate_ids == {9}
    by_id = {m.marker_id: s for m, s in zip(canonical_measurements, canonical_sources)}
    assert by_id[9] == "better-source"  # matches the winner TrackerManager.process() would track
    assert by_id[3] == "single-source"

    tracks = manager()
    tracked = tracks.process(measurements, 0.0)
    tracked_by_id = {t.marker_id: t for t in tracked}
    assert tracked_by_id[9].position[0] == pytest.approx(0.0, abs=1e-6)  # same winner as above


def test_select_canonical_with_source_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        select_canonical_with_source([dup_pose(1, 0.0, 0.0, 1.0)], [])


def test_quaternion_sign_flip_and_wrap_are_continuous():
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    assert angular_difference_deg(identity, -identity) == 0.0
    assert np.allclose(slerp(identity, -identity, 0.25), identity)
    plus_179 = np.array([0.0, 0.0, np.sin(np.deg2rad(179 / 2)), np.cos(np.deg2rad(179 / 2))])
    minus_179 = np.array([0.0, 0.0, np.sin(np.deg2rad(-179 / 2)), np.cos(np.deg2rad(-179 / 2))])
    assert abs(angular_difference_deg(plus_179, minus_179) - 2.0) < 1e-6
