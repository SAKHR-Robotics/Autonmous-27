#!/usr/bin/env python3
"""
terrain_classification.py

Explicit rock-vs-terrain classification stage (Part 4, Part 8, Part 9).

THE CORE BUG THIS FIXES
    Previously, "non-ground" (from ground_removal.py) was treated as
    synonymous with "obstacle" -- everything that survived voxel
    filtering / outlier removal / DBSCAN clustering was rasterized
    straight into the occupancy grid. On a continuous incline, a
    range-ring-local (but azimuth-blind) ground estimate misclassifies
    much of the uphill surface as non-ground, which then became one
    giant "obstacle" cluster covering the whole slope.

WHAT THIS STAGE DOES
    Given a cluster of "obstacle-candidate" points (already
    ground-removed, voxel-filtered, outlier-removed, and DBSCAN'd) and
    a fitted `LocalTerrainModel` for this frame, decides whether the
    cluster is actually:
        TERRAIN  - explained by the local terrain surface; this is a
                   ground_removal false split, not a real object. Not
                   reported as an obstacle at all.
        ROCK     - a bounded, positive, roughly compact protrusion
                   above the local terrain -- the thing Part 10-19's
                   persistent landmark tracking cares about.
        OBSTACLE - a positive protrusion that doesn't fit ROCK's size/
                   shape profile (e.g. large or irregular) but is
                   still clearly above the local surface.
        STEP     - a wide, low-roughness, sharp discontinuity -- more
                   likely a ledge/step than a compact rock.
        UNKNOWN  - not enough reliable terrain-model coverage under
                   the cluster to classify it confidently either way.
                   Per Part 21/Part 3, UNKNOWN must never silently
                   collapse into "safe to drive over".

    Classification uses several features together (per Part 8), never
    a single threshold:
        - fraction of the cluster's points "explained" by the local
          terrain surface (within `max_terrain_residual` of it)
        - the cluster's height above the local terrain surface
        - horizontal footprint (width, depth)
        - local slope and roughness under the cluster
        - terrain-model confidence/coverage under the cluster
          (continuity)

Pure computation, zero ROS dependency -- unit-testable standalone.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from terrain_geometry.terrain_model import LocalTerrainModel


class TerrainClassification:
    """String constants for the classification label (kept as plain
    strings rather than enum.Enum so they serialize trivially onto
    RViz marker text / JSON landmark output without extra plumbing)."""

    TERRAIN = "TERRAIN"
    ROCK = "ROCK"
    OBSTACLE = "OBSTACLE"
    STEP = "STEP"
    UNKNOWN = "UNKNOWN"


@dataclass
class ClassificationConfig:
    """All thresholds are configurable per Part 25 -- these are just
    defaults, overridden by terrain_node.py's declared ROS parameters.
    """

    # A point within this many meters of the fitted local terrain
    # surface counts as "explained by terrain" (`max_terrain_residual`).
    max_terrain_residual: float = 0.08

    # A cluster is TERRAIN if at least this fraction of its points are
    # explained by the local terrain surface.
    terrain_explained_fraction: float = 0.6

    # A cluster needs at least this fraction of its points to fall in
    # patches with a *valid* terrain-model fit to be classified at all
    # (below this, there simply isn't enough local ground context --
    # classify UNKNOWN rather than guess).
    min_terrain_coverage_fraction: float = 0.3

    # Minimum height above the local terrain surface (meters) for a
    # cluster to be considered a genuine positive protrusion at all.
    min_rock_height: float = 0.06

    # Rocks/obstacles need at least this many points and this much
    # horizontal extent to be trusted (avoids classifying a handful of
    # noisy outlier points as a rock).
    min_rock_points: int = 10
    min_rock_width: float = 0.04
    min_rock_depth: float = 0.04

    # A cluster wider than this (meters, in EITHER horizontal
    # dimension) is too large to be a single "rock" in the ERC sense
    # and is instead reported as a generic OBSTACLE (or STEP, if it
    # also looks like a planar ledge).
    max_rock_footprint: float = 1.2

    # STEP heuristic: footprint wider than this AND roughness below
    # this looks like a planar ledge/step rather than a rock.
    step_min_width: float = 0.6
    step_max_roughness: float = 0.04

    def validate(self) -> None:
        if self.max_terrain_residual <= 0.0:
            raise ValueError("max_terrain_residual must be > 0")
        if not (0.0 <= self.terrain_explained_fraction <= 1.0):
            raise ValueError("terrain_explained_fraction must be in [0, 1]")
        if not (0.0 <= self.min_terrain_coverage_fraction <= 1.0):
            raise ValueError("min_terrain_coverage_fraction must be in [0, 1]")
        if self.min_rock_height < 0.0:
            raise ValueError("min_rock_height must be >= 0")
        if self.min_rock_points < 1:
            raise ValueError("min_rock_points must be >= 1")
        if self.min_rock_width < 0.0 or self.min_rock_depth < 0.0:
            raise ValueError("min_rock_width/min_rock_depth must be >= 0")
        if self.max_rock_footprint <= 0.0:
            raise ValueError("max_rock_footprint must be > 0")
        if self.step_min_width <= 0.0:
            raise ValueError("step_min_width must be > 0")
        if self.step_max_roughness <= 0.0:
            raise ValueError("step_max_roughness must be > 0")


@dataclass
class ClassificationResult:
    """Per-cluster classification output, additive to `ObstacleFeature`."""

    label: str
    height_above_terrain: float  # representative (median of explained-above points), NaN if unavailable
    local_slope_deg: float       # mean over covered patches, NaN if unavailable
    roughness: float             # mean over covered patches, NaN if unavailable
    terrain_coverage_fraction: float  # fraction of points in a *valid* terrain patch
    explained_fraction: float    # fraction of points within max_terrain_residual of terrain
    confidence: float            # 0..1, see obstacle_features/rock_landmarks for full confidence


def classify_cluster(
    points: np.ndarray,
    terrain_model: LocalTerrainModel,
    width: float,
    depth: float,
    config: ClassificationConfig,
) -> ClassificationResult:
    """Classifies a single obstacle-candidate cluster.

    Args:
        points: (N, 3) float array, this cluster's member points
            (base_link frame, Z up).
        terrain_model: A `LocalTerrainModel` already `fit()` for this
            frame (on the ground-classified points).
        width: Cluster AABB Y-extent (see obstacle_features.py).
        depth: Cluster AABB X-extent.
        config: Thresholds, see `ClassificationConfig`.

    Returns:
        A `ClassificationResult`.
    """
    n = points.shape[0]
    if n == 0:
        return ClassificationResult(
            label=TerrainClassification.UNKNOWN,
            height_above_terrain=float("nan"),
            local_slope_deg=float("nan"),
            roughness=float("nan"),
            terrain_coverage_fraction=0.0,
            explained_fraction=0.0,
            confidence=0.0,
        )

    result = terrain_model.query(points)
    coverage_fraction = float(np.count_nonzero(result.valid)) / n

    if coverage_fraction < config.min_terrain_coverage_fraction:
        # Not enough local ground context under this cluster to say
        # anything reliable -- Part 21: prefer UNKNOWN over a guess.
        return ClassificationResult(
            label=TerrainClassification.UNKNOWN,
            height_above_terrain=float("nan"),
            local_slope_deg=float("nan"),
            roughness=float("nan"),
            terrain_coverage_fraction=coverage_fraction,
            explained_fraction=0.0,
            confidence=0.0,
        )

    valid_height_above = result.height_above_terrain[result.valid]
    valid_slope = result.slope_deg[result.valid]
    valid_roughness = result.roughness[result.valid]
    valid_confidence = result.confidence[result.valid]

    explained_mask = np.abs(valid_height_above) <= config.max_terrain_residual
    explained_fraction = float(np.count_nonzero(explained_mask)) / valid_height_above.shape[0]

    mean_slope = float(np.mean(valid_slope))
    mean_roughness = float(np.mean(valid_roughness))
    mean_terrain_confidence = float(np.mean(valid_confidence))

    # Representative height above terrain: median of the points that
    # are NOT already "explained" (i.e. the part that actually
    # protrudes), falling back to the overall median if every point is
    # explained (a flat/terrain cluster).
    protruding = valid_height_above[~explained_mask]
    if protruding.size > 0:
        height_above_terrain = float(np.median(protruding))
    else:
        height_above_terrain = float(np.median(valid_height_above))

    if explained_fraction >= config.terrain_explained_fraction:
        # The local terrain surface (which itself can be steeply
        # sloped -- that's fine) already accounts for most of this
        # cluster. This is the Part 4 fix: a continuous incline is
        # TERRAIN, however steep, not an obstacle.
        return ClassificationResult(
            label=TerrainClassification.TERRAIN,
            height_above_terrain=height_above_terrain,
            local_slope_deg=mean_slope,
            roughness=mean_roughness,
            terrain_coverage_fraction=coverage_fraction,
            explained_fraction=explained_fraction,
            confidence=mean_terrain_confidence,
        )

    # Not explained by terrain -> genuine protrusion candidate.
    max_footprint = max(width, depth)
    tall_enough = height_above_terrain >= config.min_rock_height
    big_enough = (
        n >= config.min_rock_points
        and width >= config.min_rock_width
        and depth >= config.min_rock_depth
    )

    if not (tall_enough and big_enough):
        # Protrudes a little, but too small/sparse to trust as a real
        # object -- likely residual sensor noise near the terrain
        # surface. Keep it out of the obstacle stream as UNKNOWN
        # rather than fabricating a low-confidence ROCK.
        return ClassificationResult(
            label=TerrainClassification.UNKNOWN,
            height_above_terrain=height_above_terrain,
            local_slope_deg=mean_slope,
            roughness=mean_roughness,
            terrain_coverage_fraction=coverage_fraction,
            explained_fraction=explained_fraction,
            confidence=mean_terrain_confidence * 0.5,
        )

    is_step_like = (
        max_footprint >= config.step_min_width and mean_roughness <= config.step_max_roughness
    )
    if is_step_like:
        label = TerrainClassification.STEP
    elif max_footprint <= config.max_rock_footprint:
        label = TerrainClassification.ROCK
    else:
        label = TerrainClassification.OBSTACLE

    return ClassificationResult(
        label=label,
        height_above_terrain=height_above_terrain,
        local_slope_deg=mean_slope,
        roughness=mean_roughness,
        terrain_coverage_fraction=coverage_fraction,
        explained_fraction=explained_fraction,
        confidence=mean_terrain_confidence,
    )
