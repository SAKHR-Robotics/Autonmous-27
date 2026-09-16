"""Normalized SLERP orientation filtering and quaternion innovation utilities."""
from __future__ import annotations
import math
import numpy as np


def normalize(quaternion: np.ndarray) -> np.ndarray | None:
    value = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(value))
    return value / norm if norm > 0.0 and np.isfinite(value).all() else None


def angular_difference_deg(first: np.ndarray, second: np.ndarray) -> float:
    a, b = normalize(first), normalize(second)
    if a is None or b is None:
        return float("inf")
    return math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(a, b))))))


def slerp(previous: np.ndarray, measurement: np.ndarray, alpha: float) -> np.ndarray:
    a, b = normalize(previous), normalize(measurement)
    if a is None or b is None:
        raise ValueError("Invalid quaternion")
    if np.dot(a, b) < 0.0:
        b = -b  # q and -q are identical rotations; ensure sign continuity.
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot > 0.9995:
        return normalize(a + alpha * (b - a))
    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    return normalize((math.sin((1.0 - alpha) * angle) / sin_angle) * a +
                     (math.sin(alpha * angle) / sin_angle) * b)


def to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray | None:
    """3x3 rotation matrix for a normalized ROS-order (x, y, z, w) quaternion.

    Added for the base_link downstream-interface transform (frame_transform.py):
    a position-only covariance rotates by this matrix, R @ cov @ R.T, under a
    rigid change of frame (translation does not affect covariance).
    """
    q = normalize(quaternion)
    if q is None:
        return None
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray | None:
    """Hamilton product of two ROS-order (x, y, z, w) quaternions.

    R(multiply(a, b)) == R(a) @ R(b) -- used to compose a TF2 transform's
    rotation with a camera-frame marker orientation (frame_transform.py).
    """
    a, b = normalize(first), normalize(second)
    if a is None or b is None:
        return None
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return normalize(np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]))
