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
from dataclasses import dataclass, replace
from enum import Enum
import numpy as np
from .position_kalman import PositionKalman
from .quality_flags import QualityFlag
from .quaternion_filter import angular_difference_deg, normalize, slerp


def _candidate_rank_key(item: "RawPose") -> tuple:
    """Sort key for same-frame, same-ID candidates; the minimum is selected.

    Priority order (Section 8): valid pose, valid depth, lower viewing
    angle, lower reprojection error, then the existing composite
    `pose_quality` (which already folds in depth agreement, geometry, and
    temporal consistency against the previous track -- see
    pose_estimator.PoseEstimator._score -- so it is reused here rather than
    re-deriving those same terms a second time) and `detection_confidence`.
    Apparent marker pixel size / normalized reprojection error are not
    tracked on RawPose (only the pose-stage output is), so they are not part
    of this key; `pose_quality`'s geometry term already penalizes small or
    skewed quadrilaterals as a proxy. Ties (which should be exceedingly
    rare) fall back to the candidate's own 3D position, which is a property
    of the observation's *value*, never of list/iteration order -- so the
    result is always fully deterministic regardless of detection order.
    """
    has_depth = (item.depth_z_m is not None and item.depth_z_variance is not None and
                np.isfinite(item.depth_z_m) and item.depth_z_variance > 0.0)
    angle = item.viewing_angle_deg if np.isfinite(item.viewing_angle_deg) else float("inf")
    reprojection = item.reprojection_error_px if np.isfinite(item.reprojection_error_px) else float("inf")
    quality_rank = -item.pose_quality if np.isfinite(item.pose_quality) else 0.0
    confidence_rank = -item.detection_confidence if np.isfinite(item.detection_confidence) else 0.0
    position = (tuple(float(v) for v in item.position)
                if item.position.shape == (3,) and np.isfinite(item.position).all() else (float("inf"),) * 3)
    return (0 if item.pose_valid else 1, 0 if has_depth else 1, angle, reprojection,
            quality_rank, confidence_rank, position)


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
        """Fix for the Section-8 duplicate-same-ID bug: a naive
        `{item.marker_id: item for item in measurements}` silently keeps
        whichever detection happens to be *last* in OpenCV's detection order
        for a given frame -- not the best one, and not a stable choice across
        frames. Two same-ID detections in one frame are legitimate (e.g. two
        faces of a cube both carrying the same physical ID are visible at
        once); this deterministically picks ONE logical observation per ID
        via `select_canonical` (order-independent, so detection order can
        never change the output) and flags every ID that had more than one
        candidate with MULTIPLE_INSTANCES_SAME_ID instead of silently
        discarding or averaging the rest -- averaging two different physical
        faces would fabricate a physically invalid pose between them.
        """
        observations, duplicate_ids = self.select_canonical(measurements)
        output: list[TrackedPose] = []
        for marker_id, measurement in observations.items():
            if marker_id in duplicate_ids:
                measurement = replace(
                    measurement,
                    quality_flags=int(QualityFlag(measurement.quality_flags) | QualityFlag.MULTIPLE_INSTANCES_SAME_ID))
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

    @staticmethod
    def select_canonical(measurements: list[RawPose]) -> tuple[dict[int, RawPose], set[int]]:
        """Group by marker_id and deterministically pick one candidate per ID.

        Selection never depends on input order (see `_candidate_rank_key`):
        the same set of candidates, in any order, always yields the same
        pick. Returns (selected_by_id, ids_that_had_more_than_one_candidate).

        Public (not `_`-prefixed) and a `@staticmethod` so it is the single
        source of truth for "which candidate is canonical" wherever that
        question comes up -- both here in `process()` and via
        `select_canonical_with_source` (module-level, below), which
        `MarkerDetectionNode._track_poses` uses to pick the matching
        canonical `AssociatedPose` for the primary debug image (Priority 1:
        the debug image must show the same selection decision as tracking,
        not a second, separately-implemented one).
        """
        grouped: dict[int, list[RawPose]] = {}
        for item in measurements:
            grouped.setdefault(item.marker_id, []).append(item)
        selected: dict[int, RawPose] = {}
        duplicate_ids: set[int] = set()
        for marker_id, candidates in grouped.items():
            if len(candidates) == 1:
                selected[marker_id] = candidates[0]
                continue
            duplicate_ids.add(marker_id)
            selected[marker_id] = min(candidates, key=_candidate_rank_key)
        return selected, duplicate_ids

    def _is_valid_new_measurement(self, measurement: RawPose) -> bool:
        return (measurement.pose_valid and np.isfinite(measurement.position).all() and measurement.position[2] > 0 and
                normalize(measurement.orientation) is not None and np.isfinite(measurement.reprojection_error_px) and
                measurement.reprojection_error_px <= self.max_reprojection_error_px)


def select_canonical_with_source(measurements: list[RawPose], sources: list) -> tuple[list[RawPose], list, set[int]]:
    """Priority 1: pure-Python helper so callers with a richer per-measurement
    "source" object (e.g. `AssociatedPose`, which carries the 2D corners
    needed for drawing but is not itself trackable) can recover which source
    object the canonical pick for each ID came from -- without duplicating
    `TrackerManager.select_canonical`'s ranking algorithm.

    `sources[i]` must correspond to `measurements[i]` (index-aligned, e.g.
    built by the same `for item in poses: ...` loop that builds `measurements`).
    Matching is by RawPose object identity (`id()`), which is safe here because
    each RawPose is freshly constructed per frame and never shared/reused.

    Used by `MarkerDetectionNode._track_poses` to feed the primary debug image
    the exact same same-ID selection decision as `TrackerManager.process()`,
    per Priority 1 (see test_tracking.py for coverage).
    """
    if len(measurements) != len(sources):
        raise ValueError("measurements and sources must be the same length (index-aligned).")
    selected, duplicate_ids = TrackerManager.select_canonical(measurements)
    index_by_identity = {id(item): index for index, item in enumerate(measurements)}
    canonical_measurements = list(selected.values())
    canonical_sources = [sources[index_by_identity[id(chosen)]] for chosen in canonical_measurements]
    return canonical_measurements, canonical_sources, duplicate_ids
