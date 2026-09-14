"""
test_perception_health.py

Part 3 coverage: VALID / DEGRADED / INVALID classification and the
"never assert free on a bad frame" safety property.
"""

import pytest

from terrain_geometry.perception_health import (
    PerceptionHealthMonitor,
    PerceptionState,
)


@pytest.fixture
def monitor():
    return PerceptionHealthMonitor(
        max_cloud_age_sec=0.5,
        min_valid_points=20,
        min_ground_points=50,
        max_processing_latency_sec=0.5,
    )


def test_healthy_frame_is_valid_and_may_report_free(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.95,
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
        last_frame_latency_sec=0.02,
    )
    assert health.state == PerceptionState.VALID
    assert health.should_report_free is True
    assert health.reasons == []


def test_empty_cloud_is_invalid_and_never_reports_free(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=None,
        num_valid_points=None,
        tf_available=True,
        ground_segmentation_ok=False,
    )
    assert health.state == PerceptionState.INVALID
    assert health.should_report_free is False


def test_missing_tf_is_invalid(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.99,
        num_valid_points=5000,
        tf_available=False,
        ground_segmentation_ok=True,
        num_ground_points=2000,
    )
    assert health.state == PerceptionState.INVALID
    assert health.should_report_free is False
    assert any("TF" in r for r in health.reasons)


def test_ground_segmentation_failure_is_invalid(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.99,
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=False,
    )
    assert health.state == PerceptionState.INVALID
    assert health.should_report_free is False


def test_too_few_points_is_invalid(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.99,
        num_valid_points=3,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
    )
    assert health.state == PerceptionState.INVALID


def test_severely_stale_cloud_is_invalid(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=5.0,  # 5s old, >> 4*0.5s
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
    )
    assert health.state == PerceptionState.INVALID


def test_mildly_stale_cloud_is_degraded_not_invalid(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.4,  # 0.6s old, > 0.5s but << 2.0s
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
    )
    assert health.state == PerceptionState.DEGRADED
    assert health.should_report_free is True  # default: DEGRADED may still report free


def test_few_ground_points_is_degraded(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.99,
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=10,  # < min_ground_points=50
    )
    assert health.state == PerceptionState.DEGRADED


def test_high_latency_is_degraded(monitor):
    health = monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.99,
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
        last_frame_latency_sec=1.0,  # > 0.5s
    )
    assert health.state == PerceptionState.DEGRADED


def test_degraded_may_report_free_can_be_disabled():
    strict_monitor = PerceptionHealthMonitor(degraded_may_report_free=False)
    health = strict_monitor.check(
        now_sec=10.0,
        cloud_stamp_sec=9.0,  # stale -> degraded
        num_valid_points=5000,
        tf_available=True,
        ground_segmentation_ok=True,
        num_ground_points=2000,
    )
    assert health.state == PerceptionState.DEGRADED
    assert health.should_report_free is False


def test_invalid_construction_rejected():
    with pytest.raises(ValueError):
        PerceptionHealthMonitor(max_cloud_age_sec=0.0)
