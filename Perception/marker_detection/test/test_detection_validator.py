import numpy as np
from marker_detection.aruco_detector import RawMarker
from marker_detection.detection_validator import DetectionValidator


def test_valid_marker_center_and_area():
    validator = DetectionValidator([], 10, 50, 5)
    marker = RawMarker(7, np.array([[10, 10], [30, 10], [30, 30], [10, 30]], dtype=np.float32))
    result = validator.validate(marker, 100, 100)
    assert result is not None
    assert result.center == (20.0, 20.0)
    assert result.area_px == 400.0


def test_whitelist_and_degenerate_marker_are_rejected():
    marker = RawMarker(7, np.array([[10, 10], [30, 10], [30, 30], [10, 30]], dtype=np.float32))
    assert DetectionValidator([4], 10, 50, 5).validate(marker, 100, 100) is None
    degenerate = RawMarker(4, np.array([[10, 10], [20, 10], [30, 10], [40, 10]], dtype=np.float32))
    assert DetectionValidator([], 1, 1, 0).validate(degenerate, 100, 100) is None
