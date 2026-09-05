"""Lightweight 2D detection checks, deliberately independent of pose estimation."""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np
from .aruco_detector import RawMarker


@dataclass(frozen=True)
class ValidatedMarker:
    marker_id: int
    corners: np.ndarray
    center: tuple[float, float]
    area_px: float


class DetectionValidator:
    def __init__(self, allowed_ids: list[int], min_size_px: float, min_area_px: float,
                 min_distance_to_border: int) -> None:
        if min_size_px <= 0 or min_area_px <= 0 or min_distance_to_border < 0:
            raise ValueError("Marker validation thresholds are out of range.")
        self.allowed_ids = set(allowed_ids)
        self.min_size_px = min_size_px
        self.min_area_px = min_area_px
        self.min_distance_to_border = min_distance_to_border

    def validate(self, marker: RawMarker, width: int, height: int) -> ValidatedMarker | None:
        corners = marker.corners
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            return None
        if self.allowed_ids and marker.marker_id not in self.allowed_ids:
            return None
        min_x, min_y = corners.min(axis=0)
        max_x, max_y = corners.max(axis=0)
        border = self.min_distance_to_border
        if min_x < border or min_y < border or max_x > width - 1 - border or max_y > height - 1 - border:
            return None
        area = abs(float(cv2.contourArea(corners.astype(np.float32))))
        if area < self.min_area_px or min(max_x - min_x, max_y - min_y) < self.min_size_px:
            return None
        center = tuple(float(value) for value in corners.mean(axis=0))
        return ValidatedMarker(marker.marker_id, corners, center, area)
