"""In-memory static global marker map with covariance-aware position fusion."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
import numpy as np
from .quaternion_filter import angular_difference_deg, normalize, slerp


class MapState(str, Enum):
    INITIALIZING = "INITIALIZING"
    CONFIRMED = "CONFIRMED"
    STALE = "STALE"


@dataclass(frozen=True)
class GlobalObservation:
    marker_id: int
    position: np.ndarray
    orientation: np.ndarray
    tracking_confidence: float
    reprojection_error_px: float
    timestamp: float


@dataclass
class MarkerMapEntryData:
    marker_id: int
    position: np.ndarray
    orientation: np.ndarray
    covariance: np.ndarray
    confidence: float
    observation_count: int
    accepted_observations: int
    rejected_observations: int
    first_seen: float
    last_seen: float
    state: MapState


class MarkerMapManager:
    """One static constant-position estimator per ArUco ID; no disk persistence."""
    def __init__(self, *, min_confirmations: int, position_measurement_noise: float,
                 initial_covariance: float, position_gate_threshold: float,
                 orientation_gate_threshold_deg: float, orientation_alpha: float,
                 stale_timeout_seconds: float, min_tracking_confidence: float,
                 max_reprojection_error_px: float) -> None:
        self.min_confirmations = min_confirmations
        self.position_measurement_noise = position_measurement_noise
        self.initial_covariance = initial_covariance
        self.position_gate_threshold = position_gate_threshold
        self.orientation_gate_threshold_deg = orientation_gate_threshold_deg
        self.orientation_alpha = orientation_alpha
        self.stale_timeout_seconds = stale_timeout_seconds
        self.min_tracking_confidence = min_tracking_confidence
        self.max_reprojection_error_px = max_reprojection_error_px
        self.entries: dict[int, MarkerMapEntryData] = {}

    def update(self, observation: GlobalObservation) -> bool:
        """Fuse a valid map-frame observation; return whether it was accepted."""
        entry = self.entries.get(observation.marker_id)
        if not self._valid_observation(observation):
            if entry:
                entry.observation_count += 1
                entry.rejected_observations += 1
                entry.confidence = max(0.0, entry.confidence - 0.10)
            return False
        if entry is None:
            self.entries[observation.marker_id] = MarkerMapEntryData(
                observation.marker_id, observation.position.copy(), normalize(observation.orientation),
                np.eye(3) * self.initial_covariance, self._quality(observation), 1, 1, 0,
                observation.timestamp, observation.timestamp, MapState.INITIALIZING)
            return True
        entry.observation_count += 1
        innovation = observation.position - entry.position
        innovation_covariance = entry.covariance + np.eye(3) * self.position_measurement_noise
        distance = float(innovation.T @ np.linalg.solve(innovation_covariance, innovation))
        angle = angular_difference_deg(entry.orientation, observation.orientation)
        if distance > self.position_gate_threshold or angle > self.orientation_gate_threshold_deg:
            entry.rejected_observations += 1
            entry.confidence = max(0.0, entry.confidence - 0.10)
            return False
        # Static-state Kalman update: x is constant, P shrinks with agreeing observations.
        gain = entry.covariance @ np.linalg.inv(innovation_covariance)
        entry.position += gain @ innovation
        entry.covariance = (np.eye(3) - gain) @ entry.covariance
        quality = self._quality(observation)
        entry.orientation = slerp(entry.orientation, observation.orientation, self.orientation_alpha * quality)
        entry.accepted_observations += 1
        entry.last_seen = observation.timestamp
        entry.confidence = min(1.0, entry.confidence + 0.15 * quality)
        entry.state = MapState.CONFIRMED if entry.accepted_observations >= self.min_confirmations else MapState.INITIALIZING
        return True

    def update_staleness(self, timestamp: float) -> None:
        for entry in self.entries.values():
            if timestamp - entry.last_seen > self.stale_timeout_seconds:
                entry.state = MapState.STALE

    def clear(self) -> None:
        self.entries.clear()

    def _valid_observation(self, observation: GlobalObservation) -> bool:
        return (np.isfinite(observation.position).all() and normalize(observation.orientation) is not None and
                math.isfinite(observation.tracking_confidence) and observation.tracking_confidence >= self.min_tracking_confidence and
                math.isfinite(observation.reprojection_error_px) and observation.reprojection_error_px <= self.max_reprojection_error_px)

    def _quality(self, observation: GlobalObservation) -> float:
        reprojection = max(0.0, 1.0 - observation.reprojection_error_px / self.max_reprojection_error_px)
        return float(np.clip(observation.tracking_confidence * reprojection, 0.05, 1.0))
