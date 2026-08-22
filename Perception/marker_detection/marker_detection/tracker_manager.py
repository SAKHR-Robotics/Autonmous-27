"""ID-keyed marker-track lifecycle management; contains no ROS dependencies.

Improvement #5 (time-aware confirmation): track confirmation and loss are
driven by elapsed wall-clock time against the *measurement* timestamp, not by
counting consecutive callback invocations, so behavior is correct regardless
of the camera's actual FPS. See MarkerTrack._update_state().

Improvement #4 (separated confidence types): each track carries
detection_confidence (2D detection quality, from the node/detector),
pose_quality (3D pose quality, from PoseEstimator), and tracking_confidence
(temporal-track health, computed here from detection density, freshness, and
filter innovation stability). The overall `confidence` is the minimum of the
three -- a conservative choice: one weak link is enough to distrust the
observation, per the "report uncertainty rather than a confident bad pose"
principle.

Improvement #2 (depth+PnP fusion): see MarkerTrack.update(), which performs
the PnP position update followed by an optional depth-Z-only update through
PositionKalman.update_axis().

Improvement #10 (dt-aware outlier rejection): the innovation gate combines
the existing Mahalanobis distance test (whose covariance already grows with
dt via the Kalman predict step) with an explicit dt-scaled maximum-jump
budget, so a given absolute jump is judged relative to how much time could
plausibly explain it.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import numpy as np
from .position_kalman import PositionKalman
from .quality_flags import QualityFlag
from .quaternion_filter import angular_difference_deg, normalize, slerp


class TrackState(str, Enum):
    NEW = "NEW"
    CONFIRMED = "CONFIRMED"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


@dataclass(frozen=True)
class RawPose:
    marker_id: int
    position: np.ndarray
    orientation: np.ndarray
    pose_valid: bool
    reprojection_error_px: float
    position_noise: np.ndarray | None = None       # 3x3 measurement covariance (m^2); see PoseEstimator.
    depth_z_m: float | None = None                  # Independent RGB-D-derived Z, if available this frame.
    depth_z_variance: float | None = None           # Variance of depth_z_m (m^2).
    detection_confidence: float = 0.0                # 2D-detection reliability, [0, 1].
    pose_quality: float = 0.0                        # 3D-pose reliability, [0, 1].
    viewing_angle_deg: float = float("nan")
    quality_flags: int = 0


@dataclass(frozen=True)
class TrackedPose:
    marker_id: int
    position: np.ndarray
    orientation: np.ndarray
    detection_valid: bool
    tracking_valid: bool
    state: TrackState
    confidence: float
    reprojection_error_px: float
    position_innovation_m: float
    orientation_innovation_deg: float
    detection_count: int
    missed_frames: int
    detection_confidence: float = 0.0
    pose_quality: float = 0.0
    tracking_confidence: float = 0.0
    viewing_angle_deg: float = float("nan")
    distance_m: float = float("nan")
    quality_flags: int = 0
    position_covariance: np.ndarray | None = None    # 3x3 (m^2), from the fused Kalman state.
    depth_fused: bool = False
    # Downstream-interface Part 3: seconds since the last *valid* detection (0.0 if
    # detected this update); purely derived from the existing timestamps this class
    # already tracks, not a new staleness mechanism.
    age_seconds: float = 0.0


class MarkerTrack:
    def __init__(self, measurement: RawPose, timestamp: float, manager: "TrackerManager") -> None:
        self.marker_id, self.manager = measurement.marker_id, manager
        self.filter = PositionKalman(measurement.position, manager.initial_covariance,
                                     manager.process_noise, manager.measurement_noise)
        self.orientation = normalize(measurement.orientation)
        self.last_timestamp = timestamp
        self.last_detection_timestamp = timestamp
        self.detection_timestamps: list[float] = [timestamp]
        self.detection_count = 1
        self.missed_frames = 0
        self.detection_confidence = measurement.detection_confidence
        self.pose_quality = measurement.pose_quality
        self.viewing_angle_deg = measurement.viewing_angle_deg
        self.quality_flags = measurement.quality_flags
        self.depth_fused = False
        if (measurement.depth_z_m is not None and measurement.depth_z_variance is not None and
                np.isfinite(measurement.depth_z_m) and measurement.depth_z_variance > 0.0):
            self.filter.update_axis(2, measurement.depth_z_m, measurement.depth_z_variance)
            self.depth_fused = True
        self.reprojection_error_px = measurement.reprojection_error_px
        self.position_innovation_m = 0.0
        self.orientation_innovation_deg = 0.0
        self.state = TrackState.NEW
        self._update_state(timestamp)

    def update(self, measurement: RawPose, timestamp: float) -> TrackedPose:
        dt = max(0.0, timestamp - self.last_timestamp)
        self.filter.predict(dt)
        self.last_timestamp = timestamp
        if not self._measurement_valid(measurement):
            return self._miss(timestamp)
        residual, _, mahalanobis = self.filter.innovation(measurement.position, measurement.position_noise)
        self.position_innovation_m = float(np.linalg.norm(residual))
        self.orientation_innovation_deg = angular_difference_deg(self.orientation, measurement.orientation)
        jump_budget = self.manager.position_jump_tolerance_m + self.manager.max_linear_velocity_mps * dt
        if (mahalanobis > self.manager.innovation_gate_threshold or
                self.orientation_innovation_deg > self.manager.max_orientation_innovation_deg or
                self.position_innovation_m > jump_budget):
            self.quality_flags = int(QualityFlag(measurement.quality_flags) | QualityFlag.TEMPORAL_OUTLIER)
            return self._miss(timestamp)
        self.filter.update(measurement.position, measurement.position_noise)
        self.depth_fused = False
        if (measurement.depth_z_m is not None and measurement.depth_z_variance is not None and
                np.isfinite(measurement.depth_z_m) and measurement.depth_z_variance > 0.0):
            self.filter.update_axis(2, measurement.depth_z_m, measurement.depth_z_variance)
            self.depth_fused = True
        self.orientation = slerp(self.orientation, measurement.orientation, self.manager.orientation_alpha)
        self.last_detection_timestamp = timestamp
        self.detection_count += 1
        self.missed_frames = 0
        self.detection_timestamps.append(timestamp)
        self._prune_window(timestamp)
        self.reprojection_error_px = measurement.reprojection_error_px
        self.detection_confidence = measurement.detection_confidence
        self.pose_quality = measurement.pose_quality
        self.viewing_angle_deg = measurement.viewing_angle_deg
        self.quality_flags = measurement.quality_flags
        self._update_state(timestamp)
        return self._output(True)

    def mark_missing(self, timestamp: float) -> TrackedPose:
        self.filter.predict(max(0.0, timestamp - self.last_timestamp))
        self.last_timestamp = timestamp
        return self._miss(timestamp)

    def expired(self, timestamp: float) -> bool:
        return timestamp - self.last_detection_timestamp > self.manager.track_timeout_seconds

    def _miss(self, timestamp: float) -> TrackedPose:
        self.missed_frames += 1
        self.depth_fused = False
        self._update_state(timestamp)
        return self._output(False)

    def _prune_window(self, timestamp: float) -> None:
        window = self.manager.confirmation_window_seconds
        self.detection_timestamps = [t for t in self.detection_timestamps if timestamp - t <= window]

    def _update_state(self, timestamp: float) -> None:
        """Time-aware NEW -> CONFIRMED -> DEGRADED -> LOST state machine (Improvement #5)."""
        gap = timestamp - self.last_detection_timestamp
        if gap > self.manager.lost_timeout_seconds:
            self.state = TrackState.LOST
        elif gap > self.manager.degraded_after_seconds:
            self.state = TrackState.DEGRADED
        elif self.state == TrackState.CONFIRMED:
            pass  # Grace period: one bad frame (or a few) does not drop a confirmed track.
        elif len(self.detection_timestamps) >= self.manager.min_confirmations:
            self.state = TrackState.CONFIRMED
        else:
            self.state = TrackState.NEW

    def _measurement_valid(self, measurement: RawPose) -> bool:
        return (measurement.pose_valid and np.isfinite(measurement.position).all() and
                normalize(measurement.orientation) is not None and measurement.position[2] > 0.0 and
                np.isfinite(measurement.reprojection_error_px) and
                measurement.reprojection_error_px <= self.manager.max_reprojection_error_px)

    def _tracking_confidence(self, timestamp: float) -> float:
        """How reliable is the temporal track itself (Improvement #4)."""
        density = min(1.0, len(self.detection_timestamps) / max(1, self.manager.min_confirmations))
        window = max(self.manager.lost_timeout_seconds, 1e-6)
        freshness = max(0.0, 1.0 - (timestamp - self.last_detection_timestamp) / window)
        tolerance = max(self.manager.position_jump_tolerance_m, 1e-6)
        stability = max(0.0, 1.0 - min(1.0, self.position_innovation_m / tolerance))
        return max(0.0, min(1.0, 0.4 * density + 0.4 * freshness + 0.2 * stability))

    def _output(self, detected: bool) -> TrackedPose:
        tracking_confidence = self._tracking_confidence(self.last_timestamp)
        overall = min(self.detection_confidence, self.pose_quality, tracking_confidence)
        flags = QualityFlag(self.quality_flags)
        if self.state == TrackState.DEGRADED:
            flags |= QualityFlag.TRACK_DEGRADED
        if self.state == TrackState.LOST:
            flags |= QualityFlag.TRACK_LOST
        age_seconds = max(0.0, self.last_timestamp - self.last_detection_timestamp)
        return TrackedPose(self.marker_id, self.filter.state[:3].copy(), self.orientation.copy(), detected, True,
                           self.state, overall, self.reprojection_error_px, self.position_innovation_m,
                           self.orientation_innovation_deg, self.detection_count, self.missed_frames,
                           self.detection_confidence, self.pose_quality, tracking_confidence,
                           self.viewing_angle_deg, float(np.linalg.norm(self.filter.state[:3])), int(flags),
                           self.filter.covariance[:3, :3].copy(), self.depth_fused, age_seconds)


class TrackerManager:
    """Owns independent ID-keyed tracks and deletes them only after track_timeout_seconds."""
    def __init__(self, *, min_confirmations: int, confirmation_window_seconds: float,
                 degraded_after_seconds: float, lost_timeout_seconds: float, track_timeout_seconds: float,
                 process_noise: float, measurement_noise: float, initial_covariance: float,
                 orientation_alpha: float, innovation_gate_threshold: float,
                 max_orientation_innovation_deg: float, max_reprojection_error_px: float,
                 position_jump_tolerance_m: float, max_linear_velocity_mps: float) -> None:
        if min_confirmations < 1 or confirmation_window_seconds <= 0:
            raise ValueError("min_confirmations must be >= 1 and confirmation_window_seconds must be > 0.")
        if not (0.0 < degraded_after_seconds <= lost_timeout_seconds <= track_timeout_seconds):
            raise ValueError("Require 0 < degraded_after_seconds <= lost_timeout_seconds <= track_timeout_seconds.")
        if position_jump_tolerance_m <= 0 or max_linear_velocity_mps <= 0:
            raise ValueError("position_jump_tolerance_m and max_linear_velocity_mps must be positive.")
        self.min_confirmations, self.confirmation_window_seconds = min_confirmations, confirmation_window_seconds
        self.degraded_after_seconds, self.lost_timeout_seconds = degraded_after_seconds, lost_timeout_seconds
        self.track_timeout_seconds = track_timeout_seconds
        self.process_noise, self.measurement_noise, self.initial_covariance = process_noise, measurement_noise, initial_covariance
        self.orientation_alpha, self.innovation_gate_threshold = orientation_alpha, innovation_gate_threshold
        self.max_orientation_innovation_deg, self.max_reprojection_error_px = max_orientation_innovation_deg, max_reprojection_error_px
        self.position_jump_tolerance_m, self.max_linear_velocity_mps = position_jump_tolerance_m, max_linear_velocity_mps
        self.tracks: dict[int, MarkerTrack] = {}

    def process(self, measurements: list[RawPose], timestamp: float) -> list[TrackedPose]:
        observations = {item.marker_id: item for item in measurements}
        output: list[TrackedPose] = []
        for marker_id, measurement in observations.items():
            if marker_id not in self.tracks:
                if self._is_valid_new_measurement(measurement):
                    self.tracks[marker_id] = MarkerTrack(measurement, timestamp, self)
                    output.append(self.tracks[marker_id]._output(True))
                continue
            output.append(self.tracks[marker_id].update(measurement, timestamp))
        for marker_id, track in list(self.tracks.items()):
            if marker_id in observations:
                continue
            if track.expired(timestamp):
                del self.tracks[marker_id]
                continue
            output.append(track.mark_missing(timestamp))
        return output

    def _is_valid_new_measurement(self, measurement: RawPose) -> bool:
        return (measurement.pose_valid and np.isfinite(measurement.position).all() and measurement.position[2] > 0 and
                normalize(measurement.orientation) is not None and np.isfinite(measurement.reprojection_error_px) and
                measurement.reprojection_error_px <= self.max_reprojection_error_px)
