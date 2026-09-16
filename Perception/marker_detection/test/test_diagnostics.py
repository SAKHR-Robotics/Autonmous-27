import math
from marker_detection.diagnostics import (
    HealthLevel, camera_info_is_stale, latency_level, latency_stats, overall_level, sync_delta_stats,
    synchronization_level)


def test_latency_stats_empty_window_is_safe():
    stats = latency_stats([], 5.0)
    assert stats.count == 0 and stats.fps == 0.0 and stats.avg_ms == 0.0


def test_latency_percentiles_order_correctly_for_a_known_distribution():
    samples = [0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.100]  # one outlier of 10
    stats = latency_stats(samples, window_seconds=1.0)
    assert stats.count == 10
    assert stats.p50_ms == 10.0
    assert stats.max_ms == 100.0
    assert stats.p95_ms <= stats.max_ms
    assert stats.p50_ms <= stats.p95_ms <= stats.p99_ms
    assert math.isclose(stats.fps, 10.0)


def test_synchronization_level_thresholds():
    assert synchronization_level(0.1, stale_threshold_s=1.0) == HealthLevel.OK
    assert synchronization_level(1.5, stale_threshold_s=1.0) == HealthLevel.WARN
    assert synchronization_level(5.0, stale_threshold_s=1.0) == HealthLevel.ERROR


def test_latency_level_thresholds_and_no_data_is_a_warning():
    good = latency_stats([0.01] * 5, 1.0)
    slow = latency_stats([0.05] * 5, 1.0)
    very_slow = latency_stats([0.20] * 5, 1.0)
    assert latency_level(good, warn_p95_ms=30.0, error_p95_ms=100.0) == HealthLevel.OK
    assert latency_level(slow, warn_p95_ms=30.0, error_p95_ms=100.0) == HealthLevel.WARN
    assert latency_level(very_slow, warn_p95_ms=30.0, error_p95_ms=100.0) == HealthLevel.ERROR
    assert latency_level(latency_stats([], 1.0), warn_p95_ms=30.0, error_p95_ms=100.0) == HealthLevel.WARN


def test_overall_level_is_the_worst_of_its_inputs():
    assert overall_level(HealthLevel.OK, HealthLevel.WARN, HealthLevel.OK) == HealthLevel.WARN
    assert overall_level(HealthLevel.ERROR, HealthLevel.OK) == HealthLevel.ERROR
    assert overall_level() == HealthLevel.OK


def test_sync_delta_stats_empty_is_safe():
    stats = sync_delta_stats([])
    assert stats.count == 0 and stats.avg_ms == 0.0 and stats.max_ms == 0.0


def test_sync_delta_stats_percentiles_order_correctly_for_a_known_distribution():
    # Nine well-synchronized pairs (5ms delta) and one badly-lagged one (80ms).
    deltas = [0.005] * 9 + [0.080]
    stats = sync_delta_stats(deltas)
    assert stats.count == 10
    assert stats.p50_ms == 5.0
    assert stats.max_ms == 80.0
    assert stats.p50_ms <= stats.p95_ms <= stats.max_ms


def test_camera_info_fresh_is_not_stale():
    """Priority 3, case 1: fresh CameraInfo -> pose estimation allowed (not stale)."""
    assert camera_info_is_stale(age_s=0.1, max_age_s=5.0) is False


def test_camera_info_older_than_max_age_is_stale():
    """Priority 3, case 2: stale CameraInfo -> pose estimation rejected/skipped."""
    assert camera_info_is_stale(age_s=10.0, max_age_s=5.0) is True


def test_camera_info_missing_is_not_reported_as_stale():
    """Priority 3, case 3: missing CameraInfo (age_s=None, i.e. never received) is a
    distinct failure mode from staleness -- pose estimation already fails correctly
    for it independent of this check (PoseEstimator.estimate(camera_info=None))."""
    assert camera_info_is_stale(age_s=None, max_age_s=5.0) is False


def test_camera_info_staleness_boundary_condition():
    """Priority 3, case 4: exactly at the threshold is still fresh (strict >, not >=);
    the smallest amount over it is stale."""
    assert camera_info_is_stale(age_s=5.0, max_age_s=5.0) is False
    assert camera_info_is_stale(age_s=5.0 + 1e-9, max_age_s=5.0) is True
