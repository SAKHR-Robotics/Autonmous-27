#!/usr/bin/env python3
"""
rock_landmarks.py

Persistent rock/obstacle landmark database (Parts 10-19, Part 24
confidence, Part 19 optional persistence).

WHY THIS EXISTS
    `obstacle_tracking.py`'s `ObstacleTracker` already gives detections
    a stable ID frame-to-frame -- but only while the object stays
    inside the sensor's field of view and keeps getting matched every
    single frame (`max_missed_frames` is small, by design, so a
    genuinely gone obstacle doesn't linger as a phantom in the
    costmap). That is the correct behavior for the *local, safety-
    critical* obstacle list, but it is NOT long-term memory: if the
    rover drives past a rock and it leaves the camera's FOV for more
    than a few frames, `ObstacleTracker` forgets it -- and when the
    rover later re-approaches the same rock, it gets a brand-new
    frame-local track ID.

    `RockLandmarkDatabase` is the separate PERSISTENT WORLD MEMORY
    layer described in Part 17: it is fed the same per-frame
    detections (after classification has confirmed they are ROCK/
    OBSTACLE/STEP, not TERRAIN or UNKNOWN), transformed into a stable
    map/world frame, and maintains a `rock_N` identity for each
    physical object for the lifetime of the mission -- independent of
    whether `ObstacleTracker`'s frame-local `track_id` for that same
    object is currently alive, lost, or has changed.

FRAME
    All landmark positions are stored in whatever stable frame the
    caller transforms observations into before calling `update()`
    (typically "map", falling back to "odom" if no map frame exists --
    terrain_node.py decides which and passes it in as `frame_id` for
    bookkeeping only; this module does no TF work itself).

ASSOCIATION (Part 14)
    A new observation is matched to an existing landmark using BOTH
    map-frame distance AND dimension similarity (not distance alone,
    per Part 14's explicit requirement) via greedy nearest-neighbor
    under a combined gate -- the same simple, explainable style as
    `obstacle_tracking.ObstacleTracker._greedy_match`.

POSITION ESTIMATION (Part 15)
    A matched landmark's position (and dimensions) are updated via
    exponential moving average, never overwritten outright -- so a
    single noisy D435 depth reading can't yank a well-established
    landmark's position around.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np


class RockLandmarkConfigError(ValueError):
    """Raised when RockLandmarkDatabase is constructed with invalid parameters."""


@dataclass
class RockObservation:
    """One frame's single detection, already classified as a persistent-
    tracking candidate (ROCK/OBSTACLE/STEP) and already transformed
    into the stable map/world frame.

    Attributes:
        position_map: (3,) array, centroid in the map/world frame.
        width, depth, height: AABB extents (meters), same convention
            as `ObstacleFeature` (width=Y, depth=X, height=Z) -- note
            these are computed in base_link *before* the map-frame
            rotation, so they describe the object's own footprint, not
            axes re-expressed in map frame; that is fine for the loose
            similarity gate used here.
        classification: One of `TerrainClassification`'s labels.
        confidence: 0..1 detection confidence (Part 24).
        track_id: The current frame's `ObstacleTracker` track_id, kept
            only for debugging/telemetry -- never used as the
            persistent identity (Part 17).
        num_points, distance: Passed through for confidence/telemetry.
    """

    position_map: np.ndarray
    width: float
    depth: float
    height: float
    classification: str
    confidence: float
    track_id: int
    num_points: int
    distance: float


@dataclass
class RockLandmark:
    """A single persistent world-frame object (Part 13's `RockLandmark`)."""

    persistent_id: str
    classification: str
    position_map: np.ndarray
    dimensions: "tuple[float, float, float]"  # (width, depth, height)
    confidence: float
    first_seen: float
    last_seen: float
    currently_visible: bool
    observation_count: int
    last_track_id: Optional[int] = None
    # Simple scalar position uncertainty (meters), shrinks with each
    # re-observation -- a lightweight stand-in for a full covariance
    # matrix, sufficient for the association gate and for reporting.
    position_uncertainty: float = 0.5

    def to_dict(self) -> dict:
        d = asdict(self)
        d["position_map"] = [float(v) for v in self.position_map]
        return d

    @staticmethod
    def from_dict(d: dict) -> "RockLandmark":
        d = dict(d)
        d["position_map"] = np.asarray(d["position_map"], dtype=np.float64)
        d["dimensions"] = tuple(d["dimensions"])
        return RockLandmark(**d)


class RockLandmarkDatabase:
    """In-memory persistent landmark map with map-frame re-identification.

    Holds no ROS state and does no TF work -- `terrain_node.py` is
    responsible for transforming detections into the map/world frame
    before calling `update()`. This keeps the association/memory logic
    independently unit-testable with synthetic map-frame positions.
    """

    def __init__(
        self,
        association_distance_gate: float = 0.5,
        dimension_similarity_tolerance: float = 0.6,
        position_smoothing: float = 0.3,
        landmark_timeout_sec: Optional[float] = None,
        min_confidence_to_create: float = 0.3,
        id_prefix: str = "rock",
    ) -> None:
        """
        Args:
            association_distance_gate: Max map-frame centroid distance
                (meters) for a new observation to be considered a
                match for an existing landmark (`rock_association_distance`
                / `rock_position_gate`). Must be > 0.
            dimension_similarity_tolerance: Max fractional difference
                (0..1+) allowed between an observation's and a
                candidate landmark's (width, depth, height) for the
                match to be accepted -- e.g. 0.6 allows up to 60%
                relative difference. Prevents accepting a
                same-position-but-very-different-size object as a
                re-identification. Must be > 0.
            position_smoothing: EMA weight given to a new observation
                each time a landmark is re-matched, in (0, 1] (Part
                15). 1.0 = overwrite outright (not recommended); small
                values respond slowly but are very stable.
            landmark_timeout_sec: If set, a landmark not re-observed
                for longer than this is pruned from the database
                entirely. If None (default, recommended for a
                competition run per Part 18), landmarks persist for
                the whole mission/process lifetime.
            min_confidence_to_create: An observation below this
                confidence never spawns a brand-new landmark (but MAY
                still update an existing one it matches) -- keeps a
                few noisy frames from flooding the database with
                spurious `rock_N` IDs.
            id_prefix: Prefix used for generated persistent IDs
                (`rock_1`, `rock_2`, ...).
        """
        if association_distance_gate <= 0.0:
            raise RockLandmarkConfigError(
                f"association_distance_gate must be > 0, got {association_distance_gate}"
            )
        if dimension_similarity_tolerance <= 0.0:
            raise RockLandmarkConfigError(
                "dimension_similarity_tolerance must be > 0, got "
                f"{dimension_similarity_tolerance}"
            )
        if not (0.0 < position_smoothing <= 1.0):
            raise RockLandmarkConfigError(
                f"position_smoothing must be in (0, 1], got {position_smoothing}"
            )
        if landmark_timeout_sec is not None and landmark_timeout_sec <= 0.0:
            raise RockLandmarkConfigError(
                f"landmark_timeout_sec must be > 0 if set, got {landmark_timeout_sec}"
            )
        if not (0.0 <= min_confidence_to_create <= 1.0):
            raise RockLandmarkConfigError(
                f"min_confidence_to_create must be in [0, 1], got {min_confidence_to_create}"
            )

        self.association_distance_gate = float(association_distance_gate)
        self.dimension_similarity_tolerance = float(dimension_similarity_tolerance)
        self.position_smoothing = float(position_smoothing)
        self.landmark_timeout_sec = landmark_timeout_sec
        self.min_confidence_to_create = float(min_confidence_to_create)
        self.id_prefix = str(id_prefix)

        self._landmarks: Dict[str, RockLandmark] = {}
        self._next_numeric_id = 1

    # ------------------------------------------------------------------ #
    # Main entry point
    # ------------------------------------------------------------------ #

    def update(
        self, observations: Sequence[RockObservation], now_sec: Optional[float] = None
    ) -> List[RockLandmark]:
        """Associates this frame's observations with the persistent map.

        Args:
            observations: This frame's ROCK/OBSTACLE/STEP detections,
                already in the map/world frame. May be empty.
            now_sec: Current time (seconds); defaults to `time.time()`.

        Returns:
            The list of landmarks matched or created THIS frame
            (`currently_visible=True`). Landmarks not seen this frame
            are NOT included here but remain in the database -- use
            `all_landmarks()` for the full persistent map (Part 18).
        """
        if now_sec is None:
            now_sec = time.time()

        self._prune_expired(now_sec)

        # Mark everything not-yet-seen-this-frame as invisible; flipped
        # back to True below for whatever actually matches/spawns.
        for lm in self._landmarks.values():
            lm.currently_visible = False

        if not observations:
            return []

        visible_this_frame: List[RockLandmark] = []

        if not self._landmarks:
            for obs in observations:
                lm = self._maybe_spawn(obs, now_sec)
                if lm is not None:
                    visible_this_frame.append(lm)
            return visible_this_frame

        landmark_ids = list(self._landmarks.keys())
        landmark_positions = np.array(
            [self._landmarks[lid].position_map for lid in landmark_ids]
        )
        obs_positions = np.array([obs.position_map for obs in observations])

        diff = landmark_positions[:, np.newaxis, :] - obs_positions[np.newaxis, :, :]
        dist_matrix = np.linalg.norm(diff, axis=2)

        obs_to_landmark = self._greedy_match(dist_matrix, landmark_ids, observations)

        matched_obs_idx = set()
        for obs_idx, lid in obs_to_landmark.items():
            matched_obs_idx.add(obs_idx)
            lm = self._update_existing(lid, observations[obs_idx], now_sec)
            visible_this_frame.append(lm)

        for obs_idx, obs in enumerate(observations):
            if obs_idx in matched_obs_idx:
                continue
            lm = self._maybe_spawn(obs, now_sec)
            if lm is not None:
                visible_this_frame.append(lm)

        return visible_this_frame

    def all_landmarks(self) -> List[RockLandmark]:
        """Returns every landmark currently in the database, visible or not."""
        return list(self._landmarks.values())

    def get(self, persistent_id: str) -> Optional[RockLandmark]:
        return self._landmarks.get(persistent_id)

    def __len__(self) -> int:
        return len(self._landmarks)

    # ------------------------------------------------------------------ #
    # Association
    # ------------------------------------------------------------------ #

    def _dimension_similarity_ok(
        self, obs: RockObservation, lm: RockLandmark
    ) -> bool:
        """Part 14: association must consider shape, not just position."""
        obs_dims = np.array([obs.width, obs.depth, obs.height])
        lm_dims = np.array(lm.dimensions)
        # Relative difference against the larger of each pair, so small
        # absolute noise on small objects doesn't fail the gate.
        denom = np.maximum(np.maximum(obs_dims, lm_dims), 1e-3)
        rel_diff = np.abs(obs_dims - lm_dims) / denom
        return bool(np.all(rel_diff <= self.dimension_similarity_tolerance))

    def _greedy_match(
        self,
        dist_matrix: np.ndarray,
        landmark_ids: List[str],
        observations: Sequence[RockObservation],
    ) -> Dict[int, str]:
        """Greedy nearest-neighbor association gated on BOTH distance and
        dimension similarity (Part 14) -- same simple/explainable style
        as `ObstacleTracker._greedy_match`, deliberately not the
        Hungarian algorithm (the number of concurrently-visible rocks
        is always small on an ERC course)."""
        num_landmarks, num_obs = dist_matrix.shape
        candidates = []
        for i in range(num_landmarks):
            lm = self._landmarks[landmark_ids[i]]
            for j in range(num_obs):
                if dist_matrix[i, j] > self.association_distance_gate:
                    continue
                if not self._dimension_similarity_ok(observations[j], lm):
                    continue
                candidates.append((dist_matrix[i, j], i, j))
        candidates.sort(key=lambda c: c[0])

        used_landmarks = set()
        used_obs = set()
        obs_to_landmark: Dict[int, str] = {}
        for _dist, i, j in candidates:
            if i in used_landmarks or j in used_obs:
                continue
            used_landmarks.add(i)
            used_obs.add(j)
            obs_to_landmark[j] = landmark_ids[i]
        return obs_to_landmark

    # ------------------------------------------------------------------ #
    # Landmark lifecycle
    # ------------------------------------------------------------------ #

    def _maybe_spawn(
        self, obs: RockObservation, now_sec: float
    ) -> Optional[RockLandmark]:
        if obs.confidence < self.min_confidence_to_create:
            return None
        persistent_id = f"{self.id_prefix}_{self._next_numeric_id}"
        self._next_numeric_id += 1
        lm = RockLandmark(
            persistent_id=persistent_id,
            classification=obs.classification,
            position_map=np.array(obs.position_map, dtype=np.float64),
            dimensions=(float(obs.width), float(obs.depth), float(obs.height)),
            confidence=float(obs.confidence),
            first_seen=now_sec,
            last_seen=now_sec,
            currently_visible=True,
            observation_count=1,
            last_track_id=obs.track_id,
            position_uncertainty=0.5,
        )
        self._landmarks[persistent_id] = lm
        return lm

    def _update_existing(
        self, persistent_id: str, obs: RockObservation, now_sec: float
    ) -> RockLandmark:
        lm = self._landmarks[persistent_id]
        alpha = self.position_smoothing

        lm.position_map = alpha * obs.position_map + (1.0 - alpha) * lm.position_map
        old_dims = np.array(lm.dimensions)
        new_dims = alpha * np.array([obs.width, obs.depth, obs.height]) + (1.0 - alpha) * old_dims
        lm.dimensions = (float(new_dims[0]), float(new_dims[1]), float(new_dims[2]))

        lm.confidence = float(max(lm.confidence, obs.confidence) * 0.5 + 0.5 * obs.confidence)
        lm.last_seen = now_sec
        lm.currently_visible = True
        lm.observation_count += 1
        lm.last_track_id = obs.track_id
        # Uncertainty shrinks (bounded below) as a landmark accumulates
        # observations -- simple stand-in for a real covariance update.
        lm.position_uncertainty = max(0.05, lm.position_uncertainty * 0.85)

        # A landmark's classification can be confirmed/upgraded (e.g.
        # first seen as a lower-confidence OBSTACLE, later confirmed as
        # ROCK) but a single frame's disagreement shouldn't thrash it
        # back and forth -- only accept a change with >= confidence.
        if obs.classification != lm.classification and obs.confidence >= lm.confidence:
            lm.classification = obs.classification

        return lm

    def _prune_expired(self, now_sec: float) -> None:
        if self.landmark_timeout_sec is None:
            return
        expired = [
            lid
            for lid, lm in self._landmarks.items()
            if (now_sec - lm.last_seen) > self.landmark_timeout_sec
        ]
        for lid in expired:
            del self._landmarks[lid]

    # ------------------------------------------------------------------ #
    # Optional persistence (Part 19) -- NEVER called from the per-frame
    # callback; terrain_node.py only calls this on shutdown or an
    # explicit trigger, so disk I/O never blocks perception.
    # ------------------------------------------------------------------ #

    def save_to_file(self, path: str) -> None:
        """Writes the full landmark database to a JSON file.

        Simple, human-readable format (Part 19 explicitly allows
        YAML *or* JSON); JSON is used here to avoid adding a new
        (PyYAML) dependency to a package that doesn't otherwise need
        one (Part 27/36 -- no unnecessary dependencies).
        """
        payload = {
            "rocks": [lm.to_dict() for lm in self._landmarks.values()],
            "next_numeric_id": self._next_numeric_id,
        }
        out_path = Path(path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w") as f:
            json.dump(payload, f, indent=2)

    def load_from_file(self, path: str) -> None:
        """Loads a previously saved landmark database, replacing the
        current in-memory state entirely. Intended for mission
        resume/debugging, called explicitly -- never automatically."""
        with Path(path).open("r") as f:
            payload = json.load(f)
        self._landmarks = {
            entry["persistent_id"]: RockLandmark.from_dict(entry)
            for entry in payload.get("rocks", [])
        }
        self._next_numeric_id = int(payload.get("next_numeric_id", 1))
