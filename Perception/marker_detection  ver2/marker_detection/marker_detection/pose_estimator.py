"""Planar square-marker solvePnP estimation and robust candidate selection.

Improvement #1 (robust planar pose estimation): IPPE_SQUARE can return two
mathematically valid solutions for a planar marker. Instead of always taking
the lowest-reprojection-error candidate, every candidate is scored on
reprojection quality, 2D corner geometry, agreement with an independently
measured depth position (if available), and agreement with the previous
tracked pose scaled by elapsed time (if available). The highest-scoring
candidate that also passes the hard validity gates is selected. With no
depth/previous-pose context (e.g. the very first detection of a marker) the
depth/temporal terms are neutral and selection falls back to
reprojection + geometry, matching the guidance that temporal consistency
should resolve ambiguity from the *second* detection onward.

Improvement #2 (depth+PnP fusion): this module does not silently overwrite
the PnP position with a depth-derived one. It only *scores* candidates using
depth agreement. The actual fusion of the PnP position and the independent
depth-derived Z happens one stage later, in tracker_manager.py, as a
second, dedicated Kalman measurement update -- see MarkerTrack.update() and
PositionKalman.update_axis(). This keeps this stage's message contract
(MarkerPose, the Step 3 output) unchanged: it is still "the PnP pose,
validated" as before, while the Step 4 *tracked* pose is now a true
multi-sensor fusion product.

Improvement #3 (pose covariance): PoseEstimate.position_noise is a diagonal
3x3 measurement-noise covariance (m^2) for the returned PnP position, derived
from the reprojection error and viewing geometry rather than being a fixed
constant -- see _position_noise().
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from .image_quality import corner_geometry
from .quality_flags import QualityFlag
from .quaternion_filter import angular_difference_deg


@dataclass(frozen=True)
class PoseEstimate:
    valid: bool
    rvec: np.ndarray | None
    tvec: np.ndarray | None
    rotation: np.ndarray | None
    quaternion: tuple[float, float, float, float] | None
    euler: tuple[float, float, float] | None
    mean_reprojection_error_px: float
    max_reprojection_error_px: float
    viewing_angle_deg: float = float("nan")
    geometric_quality: float = 0.0
    pose_quality: float = 0.0
    position_noise: np.ndarray | None = None
    ambiguous: bool = False
    quality_flags: int = int(QualityFlag.VALID)


class PoseEstimator:
    """Estimate marker-object-to-RGB-optical-camera poses from ArUco corners.

    OpenCV ArUco returns corners ordered top-left, top-right, bottom-right,
    bottom-left.  IPPE_SQUARE requires object points in exactly that order:
    (-H,+H), (+H,+H), (+H,-H), (-H,-H), where H is half marker side length.
    The marker origin is its centre; +X is right, +Y is up, and +Z is its normal.
    """
    def __init__(self, marker_size_m: float, method: str, max_reprojection_error_px: float,
                 min_depth_m: float, max_depth_m: float, *,
                 max_viewing_angle_deg: float = 75.0, high_viewing_angle_warn_deg: float = 60.0,
                 high_reprojection_warn_px: float | None = None,
                 depth_mismatch_quality_threshold: float = 0.5,
                 max_depth_position_difference_m: float = 0.20,
                 position_jump_tolerance_m: float = 0.05, max_linear_velocity_mps: float = 2.0,
                 orientation_jump_tolerance_deg: float = 10.0, max_angular_velocity_dps: float = 180.0,
                 pnp_lateral_uncertainty_px: float = 1.0, pnp_z_uncertainty_ratio: float = 0.03,
                 min_position_variance_m2: float = 1e-6, ambiguity_score_margin: float = 0.05,
                 candidate_weights: dict[str, float] | None = None) -> None:
        if marker_size_m <= 0 or max_reprojection_error_px <= 0 or min_depth_m <= 0 or max_depth_m <= min_depth_m:
            raise ValueError("Invalid pose-estimation parameter.")
        if not 0.0 < max_viewing_angle_deg <= 90.0 or high_viewing_angle_warn_deg > max_viewing_angle_deg:
            raise ValueError("Invalid viewing-angle thresholds.")
        if pnp_lateral_uncertainty_px <= 0 or pnp_z_uncertainty_ratio <= 0 or min_position_variance_m2 <= 0:
            raise ValueError("Invalid pose-covariance parameters.")
        flags = {"IPPE_SQUARE": getattr(cv2, "SOLVEPNP_IPPE_SQUARE", None),
                 "ITERATIVE": cv2.SOLVEPNP_ITERATIVE}
        if method not in flags or flags[method] is None:
            raise ValueError(f"Unsupported solvePnP method '{method}' for this OpenCV build.")
        self.marker_size_m = marker_size_m
        self.method = method
        self.flag = flags[method]
        self.max_reprojection_error_px = max_reprojection_error_px
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.max_viewing_angle_deg = max_viewing_angle_deg
        self.high_viewing_angle_warn_deg = high_viewing_angle_warn_deg
        self.high_reprojection_warn_px = (high_reprojection_warn_px if high_reprojection_warn_px is not None
                                          else 0.66 * max_reprojection_error_px)
        self.depth_mismatch_quality_threshold = depth_mismatch_quality_threshold
        self.max_depth_position_difference_m = max_depth_position_difference_m
        self.position_jump_tolerance_m = position_jump_tolerance_m
        self.max_linear_velocity_mps = max_linear_velocity_mps
        self.orientation_jump_tolerance_deg = orientation_jump_tolerance_deg
        self.max_angular_velocity_dps = max_angular_velocity_dps
        self.pnp_lateral_uncertainty_px = pnp_lateral_uncertainty_px
        self.pnp_z_uncertainty_ratio = pnp_z_uncertainty_ratio
        self.min_position_variance_m2 = min_position_variance_m2
        self.ambiguity_score_margin = ambiguity_score_margin
        weights = dict(reprojection=0.35, geometry=0.15, depth=0.15,
                       temporal_position=0.20, temporal_orientation=0.10, viewing_angle=0.05)
        if candidate_weights:
            weights.update(candidate_weights)
        total = sum(weights.values())
        if total <= 0:
            raise ValueError("Candidate scoring weights must sum to a positive value.")
        self.weights = {name: value / total for name, value in weights.items()}
        half = marker_size_m / 2.0
        self.object_points = np.array([[-half, half, 0.0], [half, half, 0.0],
                                       [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)

    @staticmethod
    def camera_parameters(info: CameraInfo) -> tuple[np.ndarray, np.ndarray] | None:
        if len(info.k) != 9 or not np.isfinite(np.asarray(info.k, dtype=np.float64)).all():
            return None
        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64).reshape(-1, 1)
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.isfinite(distortion).all():
            return None
        return matrix, distortion

    def estimate(self, corners: np.ndarray, camera_info: CameraInfo | None, *,
                 depth_position: np.ndarray | None = None, depth_z_variance: float | None = None,
                 previous_position: np.ndarray | None = None, previous_orientation: np.ndarray | None = None,
                 dt_since_previous_s: float | None = None) -> PoseEstimate:
        """Select the best-scoring valid IPPE candidate (see module docstring)."""
        parameters = self.camera_parameters(camera_info) if camera_info else None
        image_points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        if parameters is None or image_points.shape != (4, 2) or not np.isfinite(image_points).all():
            return self._invalid()
        matrix, distortion = parameters
        candidates = self._solve_candidates(image_points, matrix, distortion)
        scored: list[tuple[float, PoseEstimate]] = []
        for rvec, tvec in candidates:
            estimate = self._build_estimate(rvec, tvec, image_points, matrix, distortion,
                                            depth_position, depth_z_variance,
                                            previous_position, previous_orientation, dt_since_previous_s)
            if estimate.valid:
                scored.append((estimate.pose_quality, estimate))
        if not scored:
            return self._invalid()
        scored.sort(key=lambda item: item[0], reverse=True)
        best_score, best = scored[0]
        ambiguous = len(scored) > 1 and (best_score - scored[1][0]) < self.ambiguity_score_margin
        if ambiguous and not best.ambiguous:
            best = self._with_flag(best, QualityFlag.POSE_AMBIGUOUS, ambiguous=True)
        return best

    def _solve_candidates(self, image_points: np.ndarray, matrix: np.ndarray,
                          distortion: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
        try:
            if self.method == "IPPE_SQUARE" and hasattr(cv2, "solvePnPGeneric"):
                result = cv2.solvePnPGeneric(self.object_points, image_points, matrix, distortion, flags=self.flag)
                if not result[0]:
                    return []
                return list(zip(result[1], result[2]))
            success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, matrix, distortion, flags=self.flag)
            return [(rvec, tvec)] if success else []
        except cv2.error:
            return []

    def _build_estimate(self, rvec: np.ndarray, tvec: np.ndarray, image_points: np.ndarray,
                        matrix: np.ndarray, distortion: np.ndarray,
                        depth_position: np.ndarray | None, depth_z_variance: float | None,
                        previous_position: np.ndarray | None, previous_orientation: np.ndarray | None,
                        dt_since_previous_s: float | None) -> PoseEstimate:
        rvec, tvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1), np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if not np.isfinite(rvec).all() or not np.isfinite(tvec).all() or not self.min_depth_m <= tvec[2, 0] <= self.max_depth_m:
            return self._invalid()
        rotation, _ = cv2.Rodrigues(rvec)
        if not self._valid_rotation(rotation):
            return self._invalid()
        projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, matrix, distortion)
        residuals = np.linalg.norm(projected.reshape(4, 2) - image_points, axis=1)
        mean_error, max_error = float(residuals.mean()), float(residuals.max())
        quaternion = self.rotation_to_quaternion(rotation)
        if quaternion is None or max_error > self.max_reprojection_error_px:
            return self._invalid(mean_error, max_error)
        viewing_angle = self.viewing_angle_deg(rotation)
        if viewing_angle > self.max_viewing_angle_deg:
            return self._invalid(mean_error, max_error)
        convex, side_length_cv = corner_geometry(image_points)
        geometric_quality = (1.0 if convex else 0.3) * max(0.0, 1.0 - min(side_length_cv, 1.0))
        position = tvec.reshape(3)
        score, _, flags = self._score(mean_error, viewing_angle, geometric_quality, position,
                                      depth_position, previous_position, previous_orientation,
                                      dt_since_previous_s, quaternion)
        position_noise = self._position_noise(position[2], mean_error, matrix)
        return PoseEstimate(True, rvec, tvec, rotation, quaternion, self.rotation_to_euler(rotation),
                            mean_error, max_error, viewing_angle, geometric_quality, score,
                            position_noise, False, int(flags))

    def _score(self, mean_error: float, viewing_angle: float, geometric_quality: float, position: np.ndarray,
              depth_position: np.ndarray | None, previous_position: np.ndarray | None,
              previous_orientation: np.ndarray | None, dt: float | None,
              quaternion: tuple[float, float, float, float]) -> tuple[float, bool, "QualityFlag"]:
        flags = QualityFlag.VALID
        reprojection_quality = max(0.0, 1.0 - mean_error / self.max_reprojection_error_px)
        if mean_error > self.high_reprojection_warn_px:
            flags |= QualityFlag.HIGH_REPROJECTION_ERROR
        viewing_angle_quality = max(0.0, 1.0 - viewing_angle / self.max_viewing_angle_deg)
        if viewing_angle > self.high_viewing_angle_warn_deg:
            flags |= QualityFlag.HIGH_VIEWING_ANGLE
        depth_ok = True
        if depth_position is not None and np.isfinite(depth_position).all():
            diff = float(np.linalg.norm(position - np.asarray(depth_position)))
            depth_quality = max(0.0, 1.0 - diff / self.max_depth_position_difference_m)
            if depth_quality < self.depth_mismatch_quality_threshold:
                flags |= QualityFlag.DEPTH_MISMATCH
                depth_ok = False
        else:
            depth_quality = 0.5  # Neutral: no independent depth measurement to compare against.
        if previous_position is not None and dt is not None and np.isfinite(previous_position).all():
            budget = self.position_jump_tolerance_m + self.max_linear_velocity_mps * max(0.0, dt)
            diff = float(np.linalg.norm(position - np.asarray(previous_position)))
            temporal_position_quality = max(0.0, 1.0 - diff / budget) if budget > 0 else 0.0
        else:
            temporal_position_quality = 0.5  # Neutral: no previous track to compare against yet.
        if previous_orientation is not None and dt is not None and np.isfinite(previous_orientation).all():
            budget_deg = self.orientation_jump_tolerance_deg + self.max_angular_velocity_dps * max(0.0, dt)
            angle_diff = angular_difference_deg(np.asarray(previous_orientation), np.asarray(quaternion))
            temporal_orientation_quality = max(0.0, 1.0 - angle_diff / budget_deg) if budget_deg > 0 else 0.0
        else:
            temporal_orientation_quality = 0.5
        w = self.weights
        score = (w["reprojection"] * reprojection_quality + w["geometry"] * geometric_quality +
                w["depth"] * depth_quality + w["temporal_position"] * temporal_position_quality +
                w["temporal_orientation"] * temporal_orientation_quality + w["viewing_angle"] * viewing_angle_quality)
        return score, depth_ok, flags

    def _position_noise(self, distance_m: float, mean_reprojection_error_px: float, matrix: np.ndarray) -> np.ndarray:
        """Diagonal position-measurement covariance (m^2), see module docstring #3.

        Lateral (X, Y) uncertainty comes from back-projecting an assumed corner
        pixel-localization error through the pinhole model at the measured
        range. Z (depth-along-optical-axis) uncertainty for a monocular planar
        solve grows with the square of distance for a fixed pixel error
        (apparent marker size ~ f*s/Z, so a small error in apparent size maps
        to a Z error that scales like Z^2); this is approximated with a
        configurable fractional-of-range term rather than a fixed constant.
        """
        pixel_error = max(self.pnp_lateral_uncertainty_px, mean_reprojection_error_px)
        fx, fy = max(matrix[0, 0], 1e-6), max(matrix[1, 1], 1e-6)
        var_x = max(self.min_position_variance_m2, (pixel_error * distance_m / fx) ** 2)
        var_y = max(self.min_position_variance_m2, (pixel_error * distance_m / fy) ** 2)
        var_z = max(self.min_position_variance_m2, (self.pnp_z_uncertainty_ratio * distance_m) ** 2)
        return np.diag([var_x, var_y, var_z])

    @staticmethod
    def _with_flag(estimate: PoseEstimate, flag: QualityFlag, *, ambiguous: bool) -> PoseEstimate:
        return PoseEstimate(estimate.valid, estimate.rvec, estimate.tvec, estimate.rotation, estimate.quaternion,
                            estimate.euler, estimate.mean_reprojection_error_px, estimate.max_reprojection_error_px,
                            estimate.viewing_angle_deg, estimate.geometric_quality, estimate.pose_quality,
                            estimate.position_noise, ambiguous, int(QualityFlag(estimate.quality_flags) | flag))

    @staticmethod
    def viewing_angle_deg(rotation: np.ndarray) -> float:
        """Angle between the marker's surface normal and the camera's optical axis.

        0 deg = fronto-parallel (best case); 90 deg = edge-on (worst case,
        pose becomes numerically ill-conditioned). The object's local +Z axis
        transforms into the camera frame as the third column of `rotation`;
        its alignment with the camera Z axis is that column's Z component.
        """
        return math.degrees(math.acos(float(np.clip(abs(rotation[2, 2]), 0.0, 1.0))))

    @staticmethod
    def _valid_rotation(rotation: np.ndarray) -> bool:
        return (rotation.shape == (3, 3) and np.isfinite(rotation).all() and
                np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) and
                abs(float(np.linalg.det(rotation)) - 1.0) <= 1e-5)

    @staticmethod
    def rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float] | None:
        """Return normalized ROS-order (x, y, z, w) quaternion."""
        if not PoseEstimator._valid_rotation(rotation):
            return None
        trace = float(np.trace(rotation))
        if trace > 0:
            scale = math.sqrt(trace + 1.0) * 2.0
            q = ((rotation[2, 1] - rotation[1, 2]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale,
                 (rotation[1, 0] - rotation[0, 1]) / scale, 0.25 * scale)
        else:
            index = int(np.argmax(np.diag(rotation)))
            if index == 0:
                scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
                q = (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale)
            elif index == 1:
                scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
                q = ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale)
            else:
                scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
                q = ((rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale)
        norm = math.sqrt(sum(value * value for value in q))
        return tuple(float(value / norm) for value in q) if norm > 0 else None

    @staticmethod
    def rotation_to_euler(rotation: np.ndarray) -> tuple[float, float, float]:
        """Intrinsic XYZ / conventional roll-pitch-yaw angles in radians."""
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        return roll, pitch, yaw

    @staticmethod
    def _invalid(mean_error: float = float("nan"), max_error: float = float("nan")) -> PoseEstimate:
        return PoseEstimate(False, None, None, None, None, None, mean_error, max_error)
