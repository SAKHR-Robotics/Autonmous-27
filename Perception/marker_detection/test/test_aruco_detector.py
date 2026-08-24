import cv2
import numpy as np
from marker_detection.aruco_detector import ArucoDetector


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
