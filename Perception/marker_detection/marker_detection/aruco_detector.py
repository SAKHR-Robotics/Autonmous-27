"""OpenCV-version-compatible ArUco detector wrapper."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import cv2
import numpy as np


@dataclass(frozen=True)
class RawMarker:
    marker_id: int
    corners: np.ndarray  # Shape (4, 2), OpenCV's unmodified corner ordering.


class ArucoDetector:
    """Initializes OpenCV ArUco state once and detects markers per image."""

    def __init__(self, dictionary_name: str, parameter_values: dict[str, float | int]) -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was built without cv2.aruco; install opencv-contrib-python.")
        try:
            dictionary_id = getattr(cv2.aruco, dictionary_name)
        except AttributeError as error:
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}") from error
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = (cv2.aruco.DetectorParameters_create()
                           if hasattr(cv2.aruco, "DetectorParameters_create")
                           else cv2.aruco.DetectorParameters())
        valid_attributes = set(dir(self.parameters))
        for name, value in parameter_values.items():
            if name not in valid_attributes:
                raise ValueError(f"Installed OpenCV does not support detector parameter '{name}'.")
            setattr(self.parameters, name, value)
        self._modern_detector = getattr(cv2.aruco, "ArucoDetector", None)
        self.detector = self._modern_detector(self.dictionary, self.parameters) if self._modern_detector else None

    @property
    def api_name(self) -> str:
        return "cv2.aruco.ArucoDetector" if self.detector else "cv2.aruco.detectMarkers (legacy)"

    def detect(self, gray: np.ndarray) -> list[RawMarker]:
        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is None:
            return []
        return [RawMarker(int(marker_id), np.asarray(corner, dtype=np.float32).reshape(4, 2))
                for corner, marker_id in zip(corners, ids.flatten())]


def supported_dictionaries() -> Iterable[str]:
    return (name for name in dir(cv2.aruco) if name.startswith("DICT_"))
