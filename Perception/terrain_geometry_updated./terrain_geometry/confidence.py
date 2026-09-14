#!/usr/bin/env python3
"""
confidence.py

Per-obstacle confidence scoring (Part 23, Part 24).

Computes a 0..1 confidence score from MEASURABLE statistics of the
cluster's own points -- point count, spatial density, depth spread,
distance, terrain separation, and (optionally) how many frames the
object has been re-observed. Never returns a fixed 1.0 (Part 24
explicitly forbids a fake constant confidence) and never invents a
sensor noise model that isn't backed by the data actually in the
cloud (Part 23).

This is intentionally a simple weighted blend of bounded [0, 1]
sub-scores, not a learned model -- consistent with Part 36 ("do not
overengineer"). Every weight/threshold is a plain constructor
argument so terrain_node.py can expose it as a ROS parameter if
needed, though sensible defaults are provided.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _sigmoid_ramp(x: float, low: float, high: float) -> float:
    """Linear 0..1 ramp from `low` to `high`, clamped outside that range.

    Simpler and more predictable than an actual sigmoid for this use
    (explainability matters more than smoothness at the tails here).
    """
    if high <= low:
        return 1.0 if x >= high else 0.0
    return float(np.clip((x - low) / (high - low), 0.0, 1.0))


@dataclass
class ConfidenceWeights:
    """Relative weight of each confidence sub-score. Does not need to
    sum to 1.0 -- normalized internally."""

    point_count: float = 0.25
    density: float = 0.2
    depth_consistency: float = 0.2
    distance: float = 0.15
    terrain_separation: float = 0.2

    def normalized(self) -> "ConfidenceWeights":
        total = (
            self.point_count
            + self.density
            + self.depth_consistency
            + self.distance
            + self.terrain_separation
        )
        if total <= 0.0:
            raise ValueError("At least one confidence weight must be > 0")
        return ConfidenceWeights(
            point_count=self.point_count / total,
            density=self.density / total,
            depth_consistency=self.depth_consistency / total,
            distance=self.distance / total,
            terrain_separation=self.terrain_separation / total,
        )


def compute_confidence(
    points: np.ndarray,
    distance: float,
    height_above_terrain: float,
    terrain_confidence: float,
    weights: ConfidenceWeights | None = None,
    *,
    point_count_saturate: int = 60,
    max_trusted_distance: float = 6.0,
    depth_std_good: float = 0.01,
    depth_std_bad: float = 0.08,
    terrain_separation_good: float = 0.15,
) -> float:
    """Computes a 0..1 confidence score for one detected cluster.

    Args:
        points: (N, 3) float array, the cluster's member points.
        distance: Euclidean distance (m) from the rover to the
            cluster's centroid.
        height_above_terrain: The cluster's representative height
            (m) above the local terrain surface (from
            `terrain_classification.classify_cluster`).
        terrain_confidence: The local terrain model's own confidence
            under this cluster (0..1) -- an obstacle sitting on a
            well-fit, high-confidence patch of terrain is itself more
            trustworthy than one sitting where the terrain estimate is
            shaky.
        weights: Relative sub-score weights; defaults to
            `ConfidenceWeights()`.
        point_count_saturate: Point count at which the point-count
            sub-score reaches 1.0 (more points than this doesn't add
            further confidence).
        max_trusted_distance: Distance (m) beyond which the distance
            sub-score bottoms out at 0 -- matches the practical D435
            depth-quality falloff range, not an invented sensor model.
        depth_std_good, depth_std_bad: Range-direction (depth = X in
            base_link) point-spread standard deviation (m) mapped to
            the depth-consistency sub-score: <= depth_std_good -> 1.0,
            >= depth_std_bad -> 0.0.
        terrain_separation_good: height_above_terrain (m) at or above
            which the terrain-separation sub-score reaches 1.0 -- a
            cluster that clearly rises well above the local surface is
            unambiguous; one just barely above it is more likely
            residual ground-removal noise.

    Returns:
        Confidence in [0, 1]. Never a fixed constant -- always derived
        from the arguments above.
    """
    if weights is None:
        weights = ConfidenceWeights()
    weights = weights.normalized()

    n_points = points.shape[0]
    point_count_score = _sigmoid_ramp(n_points, 1, point_count_saturate)

    if n_points >= 2:
        aabb_extent = points.max(axis=0) - points.min(axis=0)
        volume = float(np.prod(np.maximum(aabb_extent, 1e-3)))
        density = n_points / volume
        # Density above ~500 pts/m^3 (a modestly dense D435 cluster at
        # a few meters) is treated as fully confident; below that it
        # ramps down. This is a coarse but measurable, sensor-agnostic
        # threshold, not a fabricated noise model.
        density_score = _sigmoid_ramp(density, 20.0, 500.0)
        depth_std = float(np.std(points[:, 0]))
    else:
        density_score = 0.0
        depth_std = depth_std_bad

    # Lower spread = more consistent depth return = higher confidence;
    # ramp is inverted (good value -> low std).
    depth_consistency_score = 1.0 - _sigmoid_ramp(depth_std, depth_std_good, depth_std_bad)

    distance_score = 1.0 - _sigmoid_ramp(distance, 0.0, max_trusted_distance)

    terrain_separation_score = _sigmoid_ramp(
        height_above_terrain, 0.0, terrain_separation_good
    )
    # Blend in the terrain model's own confidence under this cluster --
    # an obstacle judged relative to a poorly-fit terrain patch is
    # itself less trustworthy.
    terrain_separation_score = 0.5 * terrain_separation_score + 0.5 * float(
        np.clip(terrain_confidence, 0.0, 1.0)
    )

    score = (
        weights.point_count * point_count_score
        + weights.density * density_score
        + weights.depth_consistency * depth_consistency_score
        + weights.distance * distance_score
        + weights.terrain_separation * terrain_separation_score
    )
    return float(np.clip(score, 0.0, 1.0))
