import numpy as np
import pytest
from marker_detection.depth_processor import DepthProcessor


def processor() -> DepthProcessor:
    return DepthProcessor(0.001, 0.15, 10.0, 0.10, 5, "mad", 3.0)


def test_16uc1_depth_is_explicitly_scaled_to_metres():
    values = processor().to_meters(np.array([[2300]], dtype=np.uint16), "16UC1")
    # float32 arithmetic: compare with tolerance rather than bit-exact equality.
    assert values[0, 0] == pytest.approx(2.3, abs=1e-5)


def test_invalid_values_and_outliers_do_not_move_median_depth():
    depth = np.full((100, 100), 2.30, dtype=np.float32)
    depth[45:50, 45:50] = 8.5
    depth[50:55, 50:55] = 9.2
    depth[55:60, 55:60] = 0.0
    estimate = processor().estimate(depth, np.array([[20, 20], [80, 20], [80, 80], [20, 80]], dtype=np.float32))
    assert estimate.valid
    assert abs(estimate.depth_m - 2.30) < 0.01
    assert estimate.valid_samples >= 5


def test_insufficient_valid_depth_is_invalid():
    depth = np.zeros((60, 60), dtype=np.float32)
    estimate = processor().estimate(depth, np.array([[5, 5], [55, 5], [55, 55], [5, 55]], dtype=np.float32))
    assert not estimate.valid
