#!/usr/bin/env python3
"""
obstacle_features.py

Obstacle Feature Extraction module for the terrain_geometry package.

Per the project spec, this module implements ONLY feature extraction on
already-clustered obstacles. Ground removal, voxel filtering, statistical
outlier removal, DBSCAN clustering itself, occupancy mapping, SLAM,
navigation, and path planning are explicitly out of scope and are
assumed to already be implemented/tested upstream.

Design note -- bridging from the clustering stage:
    The upstream DBSCAN clustering stage now publishes `/cluster_labels`
    (sensor_msgs/PointCloud2 with an explicit INT32 `cluster_id` field
    per point, -1 = noise) alongside the RViz-only colored
    `/clustered_cloud`. This module consumes that direct integer label
    channel via `group_points_by_label()`, which is both cheaper and
    strictly more correct than reconstructing clusters by grouping
    identical (quantized) point colors: color reconstruction risked
    merging two distinct clusters whose golden-ratio-hue colors
    happened to quantize to the same RGB triplet once enough clusters
    were present in a frame, and required decoding/quantizing every
    point's packed color. Grouping by label is a single stable sort
    over integer IDs already assigned upstream -- no decoding, no
    collision risk.

Contents:
    1. `group_points_by_label()`: reconstructs per-cluster point groups
       from an (x, y, z) point array and a parallel integer label array.
    2. `ObstacleFeature`: a plain dataclass holding the extracted
       geometric features for a single obstacle (AABB fields match
       `ObstacleFeature.msg` 1:1; optional OBB fields are an additive,
       message-schema-safe extension used only by visualization.py).
    3. `ObstacleFeatureExtractor`: computes centroid, AABB, dimensions,
       and distance-from-origin for each obstacle's point group, using
       vectorized NumPy (`np.mean`, `np.min`, `np.max`); optionally also
       computes a PCA-based oriented bounding box (`np.cov`,
       `np.linalg.eigh`) for tighter, rotation-aware visualization.

No ROS node logic (subscriptions, publishers, message conversion) lives
here -- see `terrain_node.py` for that.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Cluster reconstruction from a labeled point cloud
# ---------------------------------------------------------------------------

def group_points_by_label(
    points_xyz: np.ndarray, labels: np.ndarray
) -> dict[int, np.ndarray]:
    """Group points into clusters based on their integer cluster label.

    Points with label -1 (DBSCAN noise, or a cluster rejected by the
    upstream min/max size filter) are excluded entirely -- they are not
    returned as a "cluster".

    Implementation is a single stable argsort over the label array
    followed by one `np.unique` call to find per-cluster segment
    boundaries -- O(N log N) total and allocation-light, versus a
    Python loop over unique keys with a fresh boolean mask each
    iteration.

    Args:
        points_xyz: (N, 3) float array of point coordinates.
        labels: (N,) int array of cluster IDs, aligned index-for-index
            with `points_xyz` (-1 = noise / filtered out).

    Returns:
        A dict mapping cluster ID (int, matching the upstream DBSCAN
        cluster ID) to an (M, 3) array of that cluster's points.

    Raises:
        ValueError: If `points_xyz` and `labels` have mismatched
            lengths.
    """
    if points_xyz.shape[0] != labels.shape[0]:
        raise ValueError(
            f"points_xyz has {points_xyz.shape[0]} rows but labels has "
            f"{labels.shape[0]} entries; they must be aligned 1:1."
        )

    if points_xyz.shape[0] == 0:
        return {}

    valid_mask = labels >= 0
    valid_points = points_xyz[valid_mask]
    valid_labels = labels[valid_mask]

    if valid_points.shape[0] == 0:
        return {}

    # Single stable sort groups same-label points contiguously; then
    # np.unique's return_index/return_counts gives each cluster's
    # segment boundaries in one vectorized pass.
    order = np.argsort(valid_labels, kind="stable")
    sorted_points = valid_points[order]
    sorted_labels = valid_labels[order]

    unique_labels, start_indices, counts = np.unique(
        sorted_labels, return_index=True, return_counts=True
    )

    clusters: dict[int, np.ndarray] = {}
    for label_value, start, count in zip(unique_labels, start_indices, counts):
        clusters[int(label_value)] = sorted_points[start : start + count]

    return clusters


# ---------------------------------------------------------------------------
# PCA-based oriented bounding box (optional, visualization-only)
# ---------------------------------------------------------------------------

def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 right-handed rotation matrix to an (x, y, z, w) quaternion.

    Standard numerically-stable conversion (branch on the largest
    diagonal term to avoid dividing by a near-zero value).
    """
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    return (float(x), float(y), float(z), float(w))


def compute_obb(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, tuple]:
    """Compute a PCA-based oriented bounding box for a set of points.

    Uses the covariance matrix of the (centered) points to find the
    obstacle's principal axes (`np.cov` + `np.linalg.eigh`), projects
    the points into that basis to size the box tightly, then converts
    the resulting rotation back into a quaternion for use as a Marker
    pose orientation.

    Args:
        points: (N, 3) float array, N >= 2 (need at least 2 points for
            a non-degenerate covariance matrix).

    Returns:
        A tuple `(center, extents, quaternion_xyzw)`:
            - center: (3,) world-frame center of the OBB.
            - extents: (3,) box size along each principal axis, in the
              same order as the rotation's columns.
            - quaternion_xyzw: (x, y, z, w) orientation of the box.
    """
    centroid = points.mean(axis=0)
    centered = points - centroid

    cov = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)  # ascending eigenvalue order

    # Reorder descending (largest-variance axis first) purely for a
    # deterministic, human-intuitive axis ordering; sign doesn't matter
    # for a bounding box but a right-handed basis is needed for a valid
    # rotation matrix -> quaternion conversion.
    order = np.argsort(eigenvalues)[::-1]
    rotation = eigenvectors[:, order]
    if np.linalg.det(rotation) < 0.0:
        rotation[:, -1] *= -1.0

    projected = centered @ rotation
    proj_min = projected.min(axis=0)
    proj_max = projected.max(axis=0)
    extents = proj_max - proj_min
    local_center = (proj_min + proj_max) / 2.0
    world_center = centroid + rotation @ local_center

    quaternion = _rotation_matrix_to_quaternion(rotation)
    return world_center, extents, quaternion


# ---------------------------------------------------------------------------
# Feature data structure
# ---------------------------------------------------------------------------

@dataclass
class ObstacleFeature:
    """Geometric features extracted for a single obstacle cluster.

    The first nine attributes map 1:1 onto `ObstacleFeature.msg` and
    must not change shape/meaning. `obb_*` fields are an additive,
    message-schema-safe extension: they are populated only when the
    extractor's OBB computation is enabled, are consumed only by
    `visualization.py` for tighter/rotation-aware markers, and are
    never serialized onto the ROS message.

    Attributes:
        id: Stable integer identifier for this obstacle within the
            current frame (matches the upstream DBSCAN cluster ID).
        num_points: Number of points comprising this obstacle.
        centroid: (3,) array, arithmetic mean of the obstacle's points.
        min_point: (3,) array, AABB minimum corner (elementwise min).
        max_point: (3,) array, AABB maximum corner (elementwise max).
        width: AABB extent along Y (lateral, left-right), meters.
        height: AABB extent along Z (vertical, up-down), meters.
        depth: AABB extent along X (longitudinal, front-back), meters.
        distance: Euclidean distance from the robot origin (0, 0, 0 in
            base_link) to the obstacle's centroid, meters.
        obb_center: (3,) OBB center, or None if OBB was not computed.
        obb_extents: (3,) OBB size along its principal axes, or None.
        obb_quaternion: (x, y, z, w) OBB orientation, or None.
    """

    id: int
    num_points: int
    centroid: np.ndarray
    min_point: np.ndarray
    max_point: np.ndarray
    width: float
    height: float
    depth: float
    distance: float
    obb_center: Optional[np.ndarray] = None
    obb_extents: Optional[np.ndarray] = None
    obb_quaternion: Optional[tuple] = None


# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------

class ObstacleFeatureExtractor:
    """Computes geometric features for a set of clustered obstacle points.

    Single responsibility: given a mapping of cluster ID -> (N, 3) point
    array, compute the centroid, axis-aligned bounding box (AABB),
    dimensions, and distance-from-origin for each cluster -- and,
    optionally, a PCA-based oriented bounding box (OBB) for tighter
    visualization of rotated/elongated obstacles.

    Holds no ROS state and performs no I/O -- safe to unit test alone.
    """

    # Robot origin in base_link, against which obstacle distance is
    # measured. base_link is defined with its origin at the robot's
    # reference point per REP-103, so this is always (0, 0, 0).
    ROBOT_ORIGIN = np.zeros(3, dtype=np.float64)

    # OBB computation needs at least this many points to form a
    # non-degenerate covariance matrix; below this, OBB fields are left
    # unset (a single point / a line has no well-defined orientation).
    MIN_POINTS_FOR_OBB = 3

    def __init__(self, compute_obb: bool = False) -> None:
        """Args:
            compute_obb: If True, also compute a PCA-based OBB
                (center, extents, quaternion) for each obstacle.
                Disabled by default to keep the default extraction path
                at its previous (AABB-only) cost.
        """
        self.compute_obb = bool(compute_obb)

    def extract_all(
        self, clusters: dict[int, np.ndarray]
    ) -> list[ObstacleFeature]:
        """Extract features for every cluster in the input mapping.

        Args:
            clusters: Mapping of cluster ID -> (N, 3) float array of
                that cluster's point coordinates (base_link frame).

        Returns:
            A list of `ObstacleFeature`, one per input cluster, sorted
            by ascending cluster ID for deterministic output ordering.
            Empty clusters (zero points) are skipped rather than
            producing a degenerate feature (undefined centroid/AABB).
        """
        features: list[ObstacleFeature] = []

        for cluster_id in sorted(clusters.keys()):
            cluster_points = clusters[cluster_id]
            if cluster_points.shape[0] == 0:
                continue
            features.append(self._extract_single(cluster_id, cluster_points))

        return features

    def _extract_single(
        self, cluster_id: int, points: np.ndarray
    ) -> ObstacleFeature:
        """Compute all geometric features for a single obstacle cluster.

        Args:
            cluster_id: Stable integer ID for this obstacle.
            points: (N, 3) float array of the obstacle's points, N >= 1.

        Returns:
            A populated `ObstacleFeature`.
        """
        num_points = points.shape[0]

        # --- Centroid: arithmetic mean of all member points -----------
        centroid = points.mean(axis=0)

        # --- Axis-Aligned Bounding Box: elementwise min/max -----------
        min_point = points.min(axis=0)
        max_point = points.max(axis=0)

        extent = max_point - min_point  # (dx, dy, dz)

        # Convention (documented on ObstacleFeature / the .msg file):
        #   width  <- Y extent (lateral, left-right)
        #   height <- Z extent (vertical, up-down)
        #   depth  <- X extent (longitudinal, front-back)
        width = float(extent[1])
        height = float(extent[2])
        depth = float(extent[0])

        # --- Distance from robot origin to this obstacle's centroid ---
        distance = float(np.linalg.norm(centroid - self.ROBOT_ORIGIN))

        obb_center = None
        obb_extents = None
        obb_quaternion = None
        if self.compute_obb and num_points >= self.MIN_POINTS_FOR_OBB:
            obb_center, obb_extents, obb_quaternion = compute_obb(points)

        return ObstacleFeature(
            id=cluster_id,
            num_points=num_points,
            centroid=centroid,
            min_point=min_point,
            max_point=max_point,
            width=width,
            height=height,
            depth=depth,
            distance=distance,
            obb_center=obb_center,
            obb_extents=obb_extents,
            obb_quaternion=obb_quaternion,
        )
