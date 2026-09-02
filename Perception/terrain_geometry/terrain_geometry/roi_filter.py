#!/usr/bin/env python3
"""
roi_filter.py

Vectorized 3D Region-of-Interest (ROI) cropping for the terrain_geometry
package.

This module implements ONLY the ROI cropping math: given an (N, 3) NumPy
array of XYZ points already expressed in `target_frame` (e.g. base_link),
keep only the points that fall inside a configurable axis-aligned box and
discard the rest. It has no knowledge of ROS topics, messages, or TF --
that wiring lives entirely in `terrain_node.py` -- so this class can be
unit-tested or reused standalone, consistent with every other stage in
this package (ground_removal.py, voxel_filter.py, outlier_filter.py, ...).

WHY ROI FILTERING, AND WHY HERE IN THE PIPELINE:
    A D435 depth frame can contain on the order of 3-30k finite points
    after decoding, but only a fraction of that volume is ever relevant
    to a forward-facing rover -- points far to the sides, far overhead,
    or beyond the rover's operational range still have to pay the full
    cost of ground segmentation, KD-tree neighbor queries, and DBSCAN
    even though they can never affect navigation. Cropping to a
    configurable box immediately after the TF transform (and before any
    of those expensive spatial-search stages) shrinks N as early as
    possible, so every downstream stage benefits.

    Filtering by an axis-aligned box in `target_frame` also composes
    correctly with the existing ground-removal range gating
    (`ground_min_range` / `ground_max_range`, which is a radial gate in
    XY) -- the ROI box is a complementary, independently configurable
    cuboid gate, not a replacement for it.

VECTORIZATION:
    A single boolean mask (six vectorized comparisons + five `&`
    reductions) over the whole (N, 3) array, then one fancy-index
    selection. No Python loop over individual points anywhere in this
    module, matching the project-wide "no per-point Python loops" rule.
"""

from __future__ import annotations

import numpy as np


class ROIFilterError(RuntimeError):
    """Raised when ROI filter parameters are invalid (e.g. min >= max)."""


class ROIFilter:
    """Crops an (N, 3) XYZ array to a configurable axis-aligned 3D box.

    Applied after the TF transform into `target_frame` and before ground
    removal / voxel downsampling / outlier removal / clustering, so those
    more expensive stages only ever see points inside the region the
    rover actually cares about.

    Attributes:
        min_x, max_x: X-axis (forward/back in base_link, REP-103) bounds,
            meters.
        min_y, max_y: Y-axis (left/right) bounds, meters.
        min_z, max_z: Z-axis (down/up) bounds, meters.
    """

    def __init__(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        min_z: float,
        max_z: float,
    ) -> None:
        """Validates and stores the ROI box bounds.

        Args:
            min_x, max_x: X bounds (meters). Must satisfy min_x < max_x.
            min_y, max_y: Y bounds (meters). Must satisfy min_y < max_y.
            min_z, max_z: Z bounds (meters). Must satisfy min_z < max_z.

        Raises:
            ROIFilterError: If any axis has min >= max.
        """
        if min_x >= max_x:
            raise ROIFilterError(
                f"roi_min_x ({min_x}) must be strictly less than roi_max_x ({max_x})."
            )
        if min_y >= max_y:
            raise ROIFilterError(
                f"roi_min_y ({min_y}) must be strictly less than roi_max_y ({max_y})."
            )
        if min_z >= max_z:
            raise ROIFilterError(
                f"roi_min_z ({min_z}) must be strictly less than roi_max_z ({max_z})."
            )

        self.min_x = float(min_x)
        self.max_x = float(max_x)
        self.min_y = float(min_y)
        self.max_y = float(max_y)
        self.min_z = float(min_z)
        self.max_z = float(max_z)

    def filter(self, xyz: np.ndarray) -> np.ndarray:
        """Keeps only points inside the configured axis-aligned box.

        Args:
            xyz: An (N, 3) array of XYZ points, expressed in the same
                frame the ROI bounds were configured for (target_frame,
                e.g. base_link).

        Returns:
            An (M, 3) array containing only the points that fall inside
            the box (inclusive bounds). If `xyz` has zero rows, an empty
            (0, 3) array of the same dtype is returned rather than
            raising -- an empty cloud cropped is still trivially empty.
        """
        if xyz.shape[0] == 0:
            return xyz

        # Single vectorized boolean mask over the whole array at once --
        # no Python loop over points.
        mask = (
            (xyz[:, 0] >= self.min_x)
            & (xyz[:, 0] <= self.max_x)
            & (xyz[:, 1] >= self.min_y)
            & (xyz[:, 1] <= self.max_y)
            & (xyz[:, 2] >= self.min_z)
            & (xyz[:, 2] <= self.max_z)
        )
        return xyz[mask]
