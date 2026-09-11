import numpy as np
from marker_detection.marker_map_manager import GlobalObservation, MapState, MarkerMapManager


def manager() -> MarkerMapManager:
    return MarkerMapManager(min_confirmations=3, position_measurement_noise=0.04,
                            initial_covariance=1.0, position_gate_threshold=11.34,
                            orientation_gate_threshold_deg=30.0, orientation_alpha=0.2,
                            stale_timeout_seconds=10.0, min_tracking_confidence=0.3,
                            max_reprojection_error_px=3.0)


def observation(marker_id: int, x: float, timestamp: float, orientation=None) -> GlobalObservation:
    return GlobalObservation(marker_id, np.array([x, -0.4, 0.1]),
                             np.array([0.0, 0.0, 0.0, 1.0]) if orientation is None else orientation,
                             0.9, 0.5, timestamp)


def test_first_observation_initializes_and_consistent_observations_confirm():
    map_ = manager()
    assert map_.update(observation(17, 2.0, 1.0))
    assert map_.entries[17].state == MapState.INITIALIZING
    assert map_.update(observation(17, 2.03, 2.0))
    assert map_.update(observation(17, 1.98, 3.0))
    assert map_.entries[17].state == MapState.CONFIRMED
    assert abs(map_.entries[17].position[0] - 2.0) < 0.05


def test_outlier_is_rejected_and_does_not_move_static_map():
    map_ = manager()
    map_.update(observation(7, 2.0, 1.0))
    before = map_.entries[7].position.copy()
    assert not map_.update(observation(7, 8.5, 2.0))
    assert np.allclose(map_.entries[7].position, before)
    assert map_.entries[7].rejected_observations == 1


def test_invalid_quaternion_observation_is_rejected_without_creating_entry():
    map_ = manager()
    bad = observation(9, 2.0, 1.0, orientation=np.array([0.0, 0.0, 0.0, 0.0]))
    assert not map_.update(bad)
    assert 9 not in map_.entries


def test_low_confidence_observation_below_threshold_is_rejected():
    map_ = manager()
    low_confidence = GlobalObservation(3, np.array([1.0, 0.0, 0.5]),
                                       np.array([0.0, 0.0, 0.0, 1.0]), 0.1, 0.5, 1.0)
    assert not map_.update(low_confidence)
    assert 3 not in map_.entries


def test_multiple_ids_stale_state_and_reappearance_reuse_same_entry():
    map_ = manager()
    map_.update(observation(1, 1.0, 0.0))
    map_.update(observation(5, 2.0, 0.0))
    map_.update_staleness(11.0)
    assert map_.entries[1].state == MapState.STALE
    map_.update(observation(1, 1.02, 12.0))
    assert map_.entries[1].state == MapState.INITIALIZING
    assert set(map_.entries) == {1, 5}
