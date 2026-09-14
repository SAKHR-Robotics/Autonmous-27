#!/usr/bin/env python3
"""
terrain_model.py

Local terrain surface model: PART 5-8 of the ERC terrain/obstacle
perception upgrade.

PROBLEM THIS SOLVES
    The existing ground_removal.py backends (Patchwork++ and the
    vectorized concentric-ring fallback) already improve on a single
    global horizontal plane by binning points into concentric RANGE
    rings and estimating a per-ring floor height. That is still not
    enough: within one range ring, a point's true ground height can
    vary a lot with AZIMUTH (e.g. a rover facing a diagonal ramp, or a
    side-slope). A per-ring-only floor systematically misclassifies
    the *uphill* half of a ring as an obstacle, which is exactly the
    documented failure mode ("inclined terrain interpreted as an
    obstacle").

WHAT THIS MODULE DOES INSTEAD
    Builds a 2D (X, Y) grid of local terrain patches (`terrain_patch_size`
    meters/cell) over the region seen this frame. For every patch with
    enough ground-classified points, fits a robust local plane
    (least-squares with one iterative residual-based outlier trim, so
    that a few obstacle points leaking into "ground" -- e.g. from an
    imperfect upstream split -- don't drag the plane estimate off the
    true surface). Each patch then has an explicit:
        - elevation (height of the fitted plane at the patch center)
        - surface normal / slope angle
        - roughness (residual spread of the inlier points)
        - point count / confidence

    Any other point (typically an "obstacle" candidate from
    ground_removal) can then be queried against this model to get its
    HEIGHT ABOVE THE LOCAL TERRAIN SURFACE, not height above one
    global/range-only floor. That is the quantity Part 7/8's
    rock-vs-terrain classification actually needs.

    Patches with too few ground points (occluded, or actually a hole/
    negative obstacle) are left INVALID rather than guessing -- see
    `LocalTerrainModel.missing_patch_centers` and Part 21.

NOT INCLUDED ON PURPOSE (Part 36 -- do not overengineer):
    No deep learning, no SLAM, no full elevation-mapping library. This
    is a small, fully vectorized (except for a Python loop bounded by
    the number of *patches*, typically a few hundred, never by point
    count) geometric model, in the same style as the per-zone loop
    already used in ground_removal.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class TerrainModelConfigError(ValueError):
    """Raised when LocalTerrainModel is constructed with invalid parameters."""


@dataclass
class TerrainQueryResult:
    """Vectorized query result, one entry per queried point.

    All arrays are the same length as the input point array.

    Attributes:
        valid: True where the queried point fell into a patch that had
            a successful plane fit (enough ground points, acceptable
            residual). False elsewhere -- callers must not trust
            `height_above_terrain`/`slope_deg`/`roughness` for those.
        local_terrain_height: Z of the fitted local plane at the
            point's (x, y), meters. Only meaningful where `valid`.
        height_above_terrain: point.z - local_terrain_height. This is
            the quantity Part 7 requires instead of a global Z
            threshold.
        slope_deg: Angle (degrees) between the patch's surface normal
            and the vertical (+Z) axis. 0 = perfectly flat.
        roughness: Standard deviation of inlier residuals within the
            patch, meters. Larger = bumpier local terrain.
        confidence: 0..1 confidence of the local terrain estimate at
            this point (based on inlier count and fit residual).
    """

    valid: np.ndarray
    local_terrain_height: np.ndarray
    height_above_terrain: np.ndarray
    slope_deg: np.ndarray
    roughness: np.ndarray
    confidence: np.ndarray


class LocalTerrainModel:
    """Grid-based piecewise-planar local terrain model.

    Usage:
        model = LocalTerrainModel(patch_size=0.5, min_points=15,
                                   max_residual=0.06)
        model.fit(ground_xyz)                  # once per frame
        result = model.query(candidate_xyz)    # for obstacle candidates

    Holds no ROS state; safe to unit test standalone.
    """

    def __init__(
        self,
        patch_size: float = 0.5,
        min_points: int = 15,
        max_residual: float = 0.06,
        outlier_trim_sigma: float = 2.5,
        grid_margin: float = 2.0,
    ) -> None:
        """
        Args:
            patch_size: Side length (meters) of each square local
                terrain patch (`terrain_patch_size` parameter). Must
                be > 0. Smaller = follows terrain more tightly but
                needs more points per patch to fit reliably; larger =
                smoother but can blur a rock into the terrain if the
                rock is a large fraction of the patch.
            min_points: Minimum ground points required inside a patch
                to attempt a plane fit (`terrain_fit_min_points`).
                Below this, the patch is left invalid rather than
                extrapolated from neighbors -- an under-observed patch
                should read as UNKNOWN, not guessed.
            max_residual: Maximum acceptable RMS residual (meters)
                after outlier trimming for a patch's fit to be trusted
                (`terrain_fit_max_residual`). A larger residual means
                the "ground" points in that patch don't actually lie
                on a plane (e.g. genuinely rough terrain, or a
                ground-removal error) -- the patch is still returned,
                but with reduced confidence, and callers using a
                confidence threshold will discount it.
            outlier_trim_sigma: Points more than this many standard
                deviations from the initial plane fit are dropped
                before refitting once. Makes the fit robust to a
                handful of contaminating non-ground points.
            grid_margin: Extra meters padded around the observed XY
                extent when sizing the internal grid, so a query point
                slightly outside the fitted point cloud's bounding box
                (common at patch edges) still maps to a real patch
                instead of falling outside the grid.
        """
        if patch_size <= 0.0:
            raise TerrainModelConfigError(f"patch_size must be > 0, got {patch_size}")
        if min_points < 3:
            raise TerrainModelConfigError(
                f"min_points must be >= 3 (a plane needs 3+ points), got {min_points}"
            )
        if max_residual <= 0.0:
            raise TerrainModelConfigError(
                f"max_residual must be > 0, got {max_residual}"
            )
        if outlier_trim_sigma <= 0.0:
            raise TerrainModelConfigError(
                f"outlier_trim_sigma must be > 0, got {outlier_trim_sigma}"
            )

        self.patch_size = float(patch_size)
        self.min_points = int(min_points)
        self.max_residual = float(max_residual)
        self.outlier_trim_sigma = float(outlier_trim_sigma)
        self.grid_margin = float(grid_margin)

        # Populated by fit().
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._n_cols = 0
        self._n_rows = 0
        self._fitted = False

        # Per-cell arrays, flattened row-major (row = Y bin, col = X bin).
        self._cell_valid: np.ndarray | None = None
        self._cell_a: np.ndarray | None = None  # plane: z = a*x + b*y + c
        self._cell_b: np.ndarray | None = None
        self._cell_c: np.ndarray | None = None
        self._cell_normal_z: np.ndarray | None = None  # for slope
        self._cell_roughness: np.ndarray | None = None
        self._cell_confidence: np.ndarray | None = None
        self._cell_point_count: np.ndarray | None = None
        self._cell_has_any_points: np.ndarray | None = None

    # ------------------------------------------------------------------ #
    # Fitting
    # ------------------------------------------------------------------ #

    def fit(self, ground_xyz: np.ndarray) -> None:
        """Builds the local terrain grid from this frame's ground points.

        Args:
            ground_xyz: (N, 3) float array of points already classified
                as ground by the upstream ground-removal stage,
                expressed in base_link (or any frame where Z is up).
                May be empty -- the model is then simply all-invalid.
        """
        ground_xyz = np.asarray(ground_xyz, dtype=np.float64)
        if ground_xyz.ndim != 2 or ground_xyz.shape[1] != 3:
            raise ValueError(f"ground_xyz must be (N, 3), got shape {ground_xyz.shape}")

        if ground_xyz.shape[0] == 0:
            self._fitted = True
            self._n_cols = 0
            self._n_rows = 0
            self._cell_valid = np.zeros(0, dtype=bool)
            self._cell_has_any_points = np.zeros(0, dtype=bool)
            return

        min_x = float(ground_xyz[:, 0].min()) - self.grid_margin
        max_x = float(ground_xyz[:, 0].max()) + self.grid_margin
        min_y = float(ground_xyz[:, 1].min()) - self.grid_margin
        max_y = float(ground_xyz[:, 1].max()) + self.grid_margin

        self._origin_x = min_x
        self._origin_y = min_y
        self._n_cols = max(1, int(np.ceil((max_x - min_x) / self.patch_size)))
        self._n_rows = max(1, int(np.ceil((max_y - min_y) / self.patch_size)))
        n_cells = self._n_cols * self._n_rows

        col_idx = np.clip(
            ((ground_xyz[:, 0] - self._origin_x) / self.patch_size).astype(np.int64),
            0,
            self._n_cols - 1,
        )
        row_idx = np.clip(
            ((ground_xyz[:, 1] - self._origin_y) / self.patch_size).astype(np.int64),
            0,
            self._n_rows - 1,
        )
        cell_id = row_idx * self._n_cols + col_idx

        self._cell_valid = np.zeros(n_cells, dtype=bool)
        self._cell_a = np.zeros(n_cells, dtype=np.float64)
        self._cell_b = np.zeros(n_cells, dtype=np.float64)
        self._cell_c = np.zeros(n_cells, dtype=np.float64)
        self._cell_normal_z = np.ones(n_cells, dtype=np.float64)
        self._cell_roughness = np.zeros(n_cells, dtype=np.float64)
        self._cell_confidence = np.zeros(n_cells, dtype=np.float64)
        self._cell_point_count = np.zeros(n_cells, dtype=np.int64)
        self._cell_has_any_points = np.zeros(n_cells, dtype=bool)

        # Bounded Python loop: over occupied cells only (typically tens
        # to a few hundred for an ROI-sized cloud), never over points --
        # consistent with the per-zone loop pattern already used in
        # ground_removal.py's fallback backend.
        order = np.argsort(cell_id, kind="stable")
        sorted_cell_id = cell_id[order]
        sorted_points = ground_xyz[order]
        unique_cells, start_idx, counts = np.unique(
            sorted_cell_id, return_index=True, return_counts=True
        )

        for cid, start, count in zip(unique_cells, start_idx, counts):
            self._cell_has_any_points[cid] = True
            self._cell_point_count[cid] = count
            if count < self.min_points:
                continue
            pts = sorted_points[start : start + count]
            fit = self._fit_plane_robust(pts)
            if fit is None:
                continue
            a, b, c, roughness, n_inliers = fit
            self._cell_a[cid] = a
            self._cell_b[cid] = b
            self._cell_c[cid] = c
            normal = np.array([-a, -b, 1.0])
            normal /= np.linalg.norm(normal)
            self._cell_normal_z[cid] = float(normal[2])
            self._cell_roughness[cid] = roughness
            self._cell_valid[cid] = True
            # Confidence: more inliers + lower roughness (relative to
            # max_residual) => higher confidence. Simple, bounded,
            # explainable -- see Part 24.
            residual_term = np.clip(1.0 - roughness / self.max_residual, 0.0, 1.0)
            count_term = np.clip(n_inliers / (self.min_points * 3.0), 0.0, 1.0)
            self._cell_confidence[cid] = float(0.5 * residual_term + 0.5 * count_term)

        self._fitted = True

    @staticmethod
    def _fit_plane_robust(
        pts: np.ndarray, trim_sigma: float = 2.5
    ) -> "tuple[float, float, float, float, int] | None":
        """Least-squares plane fit (z = a*x + b*y + c) with one robust
        outlier-trim-and-refit pass.

        Returns (a, b, c, rms_residual, n_inliers) or None if the
        system is singular (degenerate/collinear points).
        """
        A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(pts.shape[0])])
        z = pts[:, 2]
        try:
            coeffs, *_ = np.linalg.lstsq(A, z, rcond=None)
        except np.linalg.LinAlgError:
            return None
        residuals = z - A @ coeffs
        std = float(np.std(residuals))
        if std > 1e-9:
            inlier_mask = np.abs(residuals) <= trim_sigma * std
            if np.count_nonzero(inlier_mask) >= 3 and not inlier_mask.all():
                A_in = A[inlier_mask]
                z_in = z[inlier_mask]
                try:
                    coeffs, *_ = np.linalg.lstsq(A_in, z_in, rcond=None)
                except np.linalg.LinAlgError:
                    pass
                else:
                    residuals = z_in - A_in @ coeffs
                    return (
                        float(coeffs[0]),
                        float(coeffs[1]),
                        float(coeffs[2]),
                        float(np.sqrt(np.mean(residuals**2))),
                        int(inlier_mask.sum()),
                    )
        return (
            float(coeffs[0]),
            float(coeffs[1]),
            float(coeffs[2]),
            float(np.sqrt(np.mean(residuals**2))),
            int(pts.shape[0]),
        )

    # ------------------------------------------------------------------ #
    # Querying
    # ------------------------------------------------------------------ #

    def query(self, xyz: np.ndarray) -> TerrainQueryResult:
        """Evaluates the local terrain model at a set of points.

        Fully vectorized: cell lookup and plane evaluation are single
        NumPy fancy-index operations, not a per-point Python loop.

        Args:
            xyz: (N, 3) float array of candidate points (base_link or
                whatever frame `fit()` was called with).

        Returns:
            A `TerrainQueryResult` with one entry per input point.

        Raises:
            RuntimeError: If `fit()` has not been called yet.
        """
        if not self._fitted:
            raise RuntimeError("LocalTerrainModel.query() called before fit().")

        xyz = np.asarray(xyz, dtype=np.float64)
        n = xyz.shape[0]
        if n == 0 or self._n_cols == 0:
            return TerrainQueryResult(
                valid=np.zeros(n, dtype=bool),
                local_terrain_height=np.zeros(n, dtype=np.float64),
                height_above_terrain=np.zeros(n, dtype=np.float64),
                slope_deg=np.zeros(n, dtype=np.float64),
                roughness=np.zeros(n, dtype=np.float64),
                confidence=np.zeros(n, dtype=np.float64),
            )

        col_idx = ((xyz[:, 0] - self._origin_x) / self.patch_size).astype(np.int64)
        row_idx = ((xyz[:, 1] - self._origin_y) / self.patch_size).astype(np.int64)
        in_grid = (
            (col_idx >= 0)
            & (col_idx < self._n_cols)
            & (row_idx >= 0)
            & (row_idx < self._n_rows)
        )
        cell_id = np.clip(row_idx * self._n_cols + col_idx, 0, len(self._cell_valid) - 1)

        cell_valid = self._cell_valid[cell_id] & in_grid
        a = self._cell_a[cell_id]
        b = self._cell_b[cell_id]
        c = self._cell_c[cell_id]
        normal_z = self._cell_normal_z[cell_id]
        roughness = self._cell_roughness[cell_id]
        confidence = self._cell_confidence[cell_id]

        local_height = a * xyz[:, 0] + b * xyz[:, 1] + c
        height_above = xyz[:, 2] - local_height
        slope_deg = np.degrees(np.arccos(np.clip(normal_z, -1.0, 1.0)))

        # Zero out derived quantities for invalid cells so callers never
        # accidentally trust a fabricated value.
        local_height = np.where(cell_valid, local_height, np.nan)
        height_above = np.where(cell_valid, height_above, np.nan)
        slope_deg = np.where(cell_valid, slope_deg, np.nan)
        roughness = np.where(cell_valid, roughness, np.nan)
        confidence = np.where(cell_valid, confidence, 0.0)

        return TerrainQueryResult(
            valid=cell_valid,
            local_terrain_height=local_height,
            height_above_terrain=height_above,
            slope_deg=slope_deg,
            roughness=roughness,
            confidence=confidence,
        )

    # ------------------------------------------------------------------ #
    # Negative-obstacle / missing-data support (Part 21)
    # ------------------------------------------------------------------ #

    def missing_patch_centers(self) -> np.ndarray:
        """Returns (M, 2) XY centers of grid patches with NO ground or
        obstacle points at all this frame (`_cell_has_any_points` is
        False), within the fitted grid extent.

        A patch with genuinely zero returns inside a region the sensor
        should be able to see is either occlusion or a potential
        negative obstacle/drop-off -- NOT evidence of "safe and free".
        Callers (terrain_node.py) use this to mark the corresponding
        occupancy-grid cells UNKNOWN rather than FREE. This is a
        conservative geometric heuristic, not a depth-based hole
        detector -- see Part 36 (do not overengineer).
        """
        if not self._fitted or self._n_cols == 0:
            return np.empty((0, 2), dtype=np.float64)

        missing = ~self._cell_has_any_points
        if not np.any(missing):
            return np.empty((0, 2), dtype=np.float64)

        idx = np.nonzero(missing)[0]
        row = idx // self._n_cols
        col = idx % self._n_cols
        cx = self._origin_x + (col + 0.5) * self.patch_size
        cy = self._origin_y + (row + 0.5) * self.patch_size
        return np.column_stack([cx, cy])

    @property
    def is_fitted(self) -> bool:
        return self._fitted

    @property
    def num_valid_patches(self) -> int:
        if not self._fitted or self._cell_valid is None:
            return 0
        return int(np.count_nonzero(self._cell_valid))

    @property
    def num_patches(self) -> int:
        if not self._fitted:
            return 0
        return self._n_cols * self._n_rows
