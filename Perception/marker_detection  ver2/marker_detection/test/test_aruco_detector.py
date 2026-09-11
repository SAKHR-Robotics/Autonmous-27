import cv2
import numpy as np
import pytest
from marker_detection.aruco_detector import ArucoDetector, resolve_corner_refinement_method


PARAMETERS = {
    "adaptiveThreshWinSizeMin": 3, "adaptiveThreshWinSizeMax": 23,
    "adaptiveThreshWinSizeStep": 10, "adaptiveThreshConstant": 7.0,
    "minMarkerPerimeterRate": 0.03, "maxMarkerPerimeterRate": 4.0,
    "polygonalApproxAccuracyRate": 0.03, "minCornerDistanceRate": 0.05,
    "minDistanceToBorder": 3, "perspectiveRemovePixelPerCell": 4,
    "perspectiveRemoveIgnoredMarginPerCell": 0.13,
    "maxErroneousBitsInBorderRate": 0.35, "errorCorrectionRate": 0.6,
}


def test_generated_marker_is_detected():
    detector = ArucoDetector("DICT_6X6_250", PARAMETERS)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker = (cv2.aruco.generateImageMarker(dictionary, 17, 160)
              if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker(dictionary, 17, 160))
    canvas = np.full((240, 240), 255, dtype=np.uint8)
    canvas[40:200, 40:200] = marker
    detected = detector.detect(canvas)
    assert [item.marker_id for item in detected] == [17]
    assert detected[0].corners.shape == (4, 2)


def test_generated_marker_is_detected_under_the_competition_dictionary():
    """Section 6: the physical/simulation marker set (aruco.zip) decodes uniquely
    under DICT_4X4_50 -- this is the single source of truth for the whole project."""
    detector = ArucoDetector("DICT_4X4_50", PARAMETERS)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = (cv2.aruco.generateImageMarker(dictionary, 3, 160)
              if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker(dictionary, 3, 160))
    canvas = np.full((240, 240), 255, dtype=np.uint8)
    canvas[40:200, 40:200] = marker
    detected = detector.detect(canvas)
    assert [item.marker_id for item in detected] == [3]


def test_resolve_corner_refinement_method_translates_known_names():
    assert resolve_corner_refinement_method("CORNER_REFINE_SUBPIX") == cv2.aruco.CORNER_REFINE_SUBPIX
    assert resolve_corner_refinement_method("CORNER_REFINE_NONE") == cv2.aruco.CORNER_REFINE_NONE


def test_resolve_corner_refinement_method_rejects_unknown_names():
    with pytest.raises(ValueError):
        resolve_corner_refinement_method("NOT_A_REAL_METHOD")
    with pytest.raises(ValueError):
        resolve_corner_refinement_method("DICT_4X4_50")  # a real cv2.aruco attribute, wrong category


def test_corner_refinement_parameters_are_actually_applied_to_opencv():
    """Guards against declaring a parameter that is never passed to OpenCV (Section 7)."""
    values = dict(PARAMETERS)
    values["cornerRefinementMethod"] = resolve_corner_refinement_method("CORNER_REFINE_SUBPIX")
    values["cornerRefinementWinSize"] = 7
    values["cornerRefinementMaxIterations"] = 42
    values["cornerRefinementMinAccuracy"] = 0.05
    detector = ArucoDetector("DICT_4X4_50", values)
    assert detector.parameters.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX
    assert detector.parameters.cornerRefinementWinSize == 7
    assert detector.parameters.cornerRefinementMaxIterations == 42
    assert detector.parameters.cornerRefinementMinAccuracy == pytest.approx(0.05)
