#!/usr/bin/env python3
"""
obstacle_tracking.py

Lightweight, distance-based temporal tracking layer for detected obstacles.

Deliberately kept modular and separate from clustering.py: this module has
no knowledge of point clouds, DBSCAN, or voxels -- it only ever sees the
per-frame List[ObstacleFeature] that obstacle_features.py already produces,
and hands back a List[ObstacleFeature] with stable IDs and smoothed
centroids. This keeps tracking logic decoupled from clustering, per the
project's "keep tracking modular / don't mix into the clustering algorithm"
requirement.

WHAT THIS INTENTIONALLY IS NOT:
    Not a Kalman filter, not a learned association model, not a multi-
    hypothesis tracker. Nearest-neighbor greedy association + exponential
    position smoothing is the simplest thing that actually reduces
    obstacle-ID flicker and jitter frame-to-frame for a rover's local
    obstacle list, and it's easy to reason about and safety-review.

SAFETY-RELEVANT DESIGN CHOICE:
    `update()` only ever returns tracks that were matched to a detection
    THIS frame. A track that goes unmatched is aged internally (so it can
    still be re-associated with a detection in a future frame, reducing
    ID flicker across a brief occlusion/miss), but it is never emitted
    on its own -- so a missed detection can never turn into a phantom
    obstacle that lingers in the occupancy grid / costmap after the real
    object is no longer being detected.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from terrain_geometry.obstacle_features import ObstacleFeature


class ObstacleTrackerConfigError(ValueError):
    """Raised when ObstacleTracker is constructed with invalid parameters."""


@dataclass
class _Track:
    """Internal bookkeeping for one tracked obstacle. Not exposed publicly."""

    track_id: int
    smoothed_centroid: np.ndarray
    missed_frames: int = 0


@dataclass
class ObstacleTracker:
    """Nearest-neighbor, distance-gated temporal obstacle tracker.

    Call `update(detections)` once per frame, after obstacle feature
    extraction and before publishing / rasterizing. Returns a new list of
    `ObstacleFeature` (same length as the number of *matched* detections
    this frame -- i.e. every input detection is represented exactly once,
    either bound to an existing track or seeded as a new one) with:
      - `id` replaced by a stable track ID (persists across frames as
        long as the association keeps succeeding).
      - `centroid` (and `min_point`/`max_point`/`obb_center`, shifted by
        the same delta) replaced by an exponentially-smoothed position,
        to reduce frame-to-frame jitter.
      - `width`/`height`/`depth`/`num_points` left untouched (always
        from the current detection -- the footprint size used for the
        occupancy grid/costmap is never stale).

    Attributes:
        max_association_distance: Maximum centroid-to-centroid distance
            (meters) for a detection to be associated with an existing
            track. Must be > 0.
        max_missed_frames: Number of consecutive frames a track may go
            unmatched before it is dropped. Must be >= 0 (0 means a
            track is dropped the very first frame it isn't matched --
            i.e. tracking degrades to same-frame IDs only).
        position_smoothing: Exponential smoothing weight given to the
            *new* measurement each time a track is matched, in (0, 1].
            1.0 means no smoothing (track jumps straight to the new
            detection's centroid every time). Smaller values smooth
            more aggressively but respond more slowly to real motion.
    """

    max_association_distance: float
    max_missed_frames: int
    position_smoothing: float

    _tracks: Dict[int, _Track] = field(default_factory=dict, repr=False)
    _next_id: int = field(default=0, repr=False)

    def __post_init__(self) -> None:
        if self.max_association_distance <= 0.0:
            raise ObstacleTrackerConfigError(
                "tracking_max_association_distance must be > 0, got "
                f"{self.max_association_distance}"
            )
        if self.max_missed_frames < 0:
            raise ObstacleTrackerConfigError(
                f"tracking_max_missed_frames must be >= 0, got {self.max_missed_frames}"
            )
        if not (0.0 < self.position_smoothing <= 1.0):
            raise ObstacleTrackerConfigError(
                "tracking_position_smoothing must be in (0, 1], got "
                f"{self.position_smoothing}"
            )

    def reset(self) -> None:
        """Drops all tracks. Useful for tests or after a large TF jump."""
        self._tracks.clear()
        self._next_id = 0

    def update(self, detections: List[ObstacleFeature]) -> List[ObstacleFeature]:
        """Associates this frame's detections with existing tracks.

        Args:
            detections: This frame's obstacle detections, as produced by
                `ObstacleFeatureExtractor.extract_all`. May be empty.

        Returns:
            A list of `ObstacleFeature`, one per input detection (same
            order is NOT guaranteed -- callers that need to preserve
            input order should not rely on it), each carrying a stable
            track ID and a smoothed centroid. Never contains an entry
            for a track that wasn't matched this frame -- see module
            docstring.
        """
        # Always age+prune first so a track that's been missing too long
        # doesn't win an association it shouldn't.
        self._prune_stale_tracks()

        if not detections:
            self._age_all_tracks()
            return []

        if not self._tracks:
            return [self._spawn_track(det) for det in detections]

        track_ids = list(self._tracks.keys())
        track_positions = np.array(
            [self._tracks[tid].smoothed_centroid for tid in track_ids]
        )
        det_positions = np.array([det.centroid for det in detections])

        # Vectorized pairwise distance matrix (num_tracks, num_detections).
        diff = track_positions[:, np.newaxis, :] - det_positions[np.newaxis, :, :]
        dist_matrix = np.linalg.norm(diff, axis=2)

        det_to_track = self._greedy_match(dist_matrix, track_ids)

        outputs: List[ObstacleFeature] = []
        matched_track_ids = set()
        for det_idx, det in enumerate(detections):
            if det_idx in det_to_track:
                tid = det_to_track[det_idx]
                track = self._tracks[tid]
                alpha = self.position_smoothing
                new_centroid = (
                    alpha * det.centroid + (1.0 - alpha) * track.smoothed_centroid
                )
                track.smoothed_centroid = new_centroid
                track.missed_frames = 0
                matched_track_ids.add(tid)
                outputs.append(self._recentered(det, tid, new_centroid))
            else:
                new_track = self._spawn_track(det)
                matched_track_ids.add(new_track.id)
                outputs.append(new_track)

        # Age every track that existed this frame but wasn't matched.
        for tid in track_ids:
            if tid not in matched_track_ids:
                self._tracks[tid].missed_frames += 1

        return outputs

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _greedy_match(
        self, dist_matrix: np.ndarray, track_ids: List[int]
    ) -> Dict[int, int]:
        """Greedy nearest-neighbor bipartite matching under a distance gate.

        Not globally optimal (that would be the Hungarian algorithm), but
        for the small obstacle counts a rover's local perception window
        produces, greedy nearest-neighbor is simple, fast, and easy to
        reason about -- consistent with the "simple and robust" tracker
        requirement.

        Returns:
            Mapping of detection index -> track_id for every accepted
            association (distance <= max_association_distance).
        """
        num_tracks, num_dets = dist_matrix.shape
        candidates = [
            (dist_matrix[i, j], i, j)
            for i in range(num_tracks)
            for j in range(num_dets)
            if dist_matrix[i, j] <= self.max_association_distance
        ]
        candidates.sort(key=lambda c: c[0])

        used_tracks = set()
        used_dets = set()
        det_to_track: Dict[int, int] = {}
        for _dist, i, j in candidates:
            if i in used_tracks or j in used_dets:
                continue
            used_tracks.add(i)
            used_dets.add(j)
            det_to_track[j] = track_ids[i]
        return det_to_track

    def _spawn_track(self, det: ObstacleFeature) -> ObstacleFeature:
        tid = self._next_id
        self._next_id += 1
        self._tracks[tid] = _Track(track_id=tid, smoothed_centroid=det.centroid.copy())
        return self._recentered(det, tid, det.centroid.copy())

    def _age_all_tracks(self) -> None:
        for track in self._tracks.values():
            track.missed_frames += 1

    def _prune_stale_tracks(self) -> None:
        stale_ids = [
            tid
            for tid, track in self._tracks.items()
            if track.missed_frames > self.max_missed_frames
        ]
        for tid in stale_ids:
            del self._tracks[tid]

    @staticmethod
    def _recentered(
        det: ObstacleFeature, track_id: int, smoothed_centroid: np.ndarray
    ) -> ObstacleFeature:
        """Returns a copy of `det` re-labeled with `track_id` and recentered
        on `smoothed_centroid`. The AABB (and OBB center, if present) are
        shifted by the same delta so box *size* is always the current
        detection's real, current-frame footprint -- only *position* is
        smoothed. `distance` (range from the robot origin) is recomputed
        to stay consistent with the smoothed centroid.
        """
        delta = smoothed_centroid - det.centroid
        return dataclasses.replace(
            det,
            id=track_id,
            centroid=smoothed_centroid,
            min_point=det.min_point + delta,
            max_point=det.max_point + delta,
            distance=float(np.linalg.norm(smoothed_centroid)),
            obb_center=(det.obb_center + delta) if det.obb_center is not None else None,
        )
