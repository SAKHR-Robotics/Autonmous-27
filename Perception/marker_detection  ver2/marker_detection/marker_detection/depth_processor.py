"""Robust interior-polygon depth sampling for aligned RGB-D marker association."""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    valid: bool
    depth_m: float
    valid_samples: int
    quality: float
    mad_m: float
    roi_mask: np.ndarray


class DepthProcessor:
    """Converts depth explicitly, samples marker interiors, and estimates median Z."""
    def __init__(self, depth_scale: float, min_depth_m: float, max_depth_m: float,
                 shrink_ratio: float, min_valid_samples: int, outlier_method: str,
                 outlier_threshold: float) -> None:
        if depth_scale <= 0 or min_depth_m <= 0 or max_depth_m <= min_depth_m:
            raise ValueError("Invalid depth-scale or depth-range parameters.")
        if not 0.0 <= shrink_ratio < 0.5 or min_valid_samples < 1 or outlier_threshold <= 0:
            raise ValueError("Invalid depth ROI or outlier parameters.")
        if outlier_method != "mad":
            raise ValueError("Only the documented robust 'mad' depth outlier method is supported.")
        self.depth_scale = depth_scale
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.shrink_ratio = shrink_ratio
        self.min_valid_samples = min_valid_samples
        self.outlier_threshold = outlier_threshold

    def to_meters(self, depth: np.ndarray, encoding: str) -> np.ndarray:
        """Convert 16UC1 using configured scale; 32FC1 is already metres."""
        if depth.ndim != 2:
            raise ValueError(f"Expected one-channel depth image, got shape {depth.shape}.")
        normalized = encoding.lower()
        if normalized in ("16uc1", "mono16"):
            return depth.astype(np.float32) * self.depth_scale
        if normalized == "32fc1":
            return depth.astype(np.float32, copy=False)
        raise ValueError(f"Unsupported aligned depth encoding '{encoding}'; expected 16UC1 or 32FC1.")

    def estimate(self, depth_m: np.ndarray, corners: np.ndarray) -> DepthEstimate:
        mask = self._interior_mask(depth_m.shape[:2], corners)
        sampled = depth_m[mask > 0]
        valid = sampled[np.isfinite(sampled) & (sampled > 0.0) &
                        (sampled >= self.min_depth_m) & (sampled <= self.max_depth_m)]
        quality = float(valid.size / sampled.size) if sampled.size else 0.0
        if valid.size < self.min_valid_samples:
            return DepthEstimate(False, float("nan"), int(valid.size), quality, float("nan"), mask)
        median = float(np.median(valid))
        mad = float(np.median(np.abs(valid - median)))
        # 1.4826 makes MAD comparable to a standard deviation for Gaussian noise.
        tolerance = self.outlier_threshold * 1.4826 * mad
        inliers = valid[np.abs(valid - median) <= tolerance] if tolerance > 0.0 else valid[valid == median]
        if inliers.size < self.min_valid_samples:
            return DepthEstimate(False, float("nan"), int(inliers.size), quality, mad, mask)
        return DepthEstimate(True, float(np.median(inliers)), int(inliers.size), quality, mad, mask)

    def _interior_mask(self, shape: tuple[int, int], corners: np.ndarray) -> np.ndarray:
        mask = np.zeros(shape, dtype=np.uint8)
        polygon = np.rint(corners).astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [polygon], 255)
        bounds = corners.max(axis=0) - corners.min(axis=0)
        margin = max(1, int(round(min(bounds) * self.shrink_ratio)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
        return cv2.erode(mask, kernel)
