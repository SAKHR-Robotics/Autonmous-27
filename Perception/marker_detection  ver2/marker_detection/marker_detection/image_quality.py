"""Purely 2D image-quality metrics for marker ROIs (Improvement #7).

Evaluated on the 2D detection, before any 3D/pose computation, so a poor
observation can be flagged or rejected before solvePnP runs on it.
Viewing-angle and pose-ambiguity checks live in pose_estimator.py instead,
because they require the estimated 3D rotation, not just pixels.

The caller passes a pre-computed full-frame Laplacian so the (relatively
expensive) convolution runs once per frame rather than once per marker.
"""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class ImageQuality:
    blur_score: float       # Laplacian variance inside the marker interior; higher = sharper.
    contrast_score: float   # Grey-level standard deviation inside the marker interior.
    convex: bool             # Whether the detected quadrilateral is convex.
    side_length_cv: float   # Coefficient of variation of the four side lengths; 0 = perfect square.


def full_frame_laplacian(gray: np.ndarray) -> np.ndarray:
    """Compute once per frame and share across every marker's assess() call."""
    return cv2.Laplacian(gray, cv2.CV_64F)


def _interior_mask(shape: tuple[int, int], corners: np.ndarray, shrink_ratio: float) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    polygon = np.rint(corners).astype(np.int32).reshape((-1, 1, 2))
    cv2.fillPoly(mask, [polygon], 255)
    bounds = corners.max(axis=0) - corners.min(axis=0)
    margin = max(1, int(round(min(bounds) * shrink_ratio)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
    return cv2.erode(mask, kernel)


def corner_geometry(corners: np.ndarray) -> tuple[bool, float]:
    """Quadrilateral shape sanity, independent of pixel content: (is_convex, side_length_cv).

    Shared by image_quality.assess() and pose_estimator's candidate geometric-quality
    score, since both need the same purely-2D corner-shape check.
    """
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    convex = bool(cv2.isContourConvex(np.rint(corners).astype(np.int32).reshape(-1, 1, 2)))
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
    side_length_cv = float(sides.std() / sides.mean()) if sides.mean() > 0 else float("inf")
    return convex, side_length_cv


def assess(gray: np.ndarray, corners: np.ndarray, laplacian: np.ndarray,
           interior_shrink_ratio: float = 0.15) -> ImageQuality:
    """Assess blur, contrast, and quadrilateral geometry inside a marker's interior."""
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    mask = _interior_mask(gray.shape[:2], corners, interior_shrink_ratio)
    interior = mask > 0
    if not interior.any():
        return ImageQuality(0.0, 0.0, False, float("inf"))
    blur_score = float(laplacian[interior].var())
    contrast_score = float(gray[interior].astype(np.float64).std())
    convex, side_length_cv = corner_geometry(corners)
    return ImageQuality(blur_score, contrast_score, convex, side_length_cv)
