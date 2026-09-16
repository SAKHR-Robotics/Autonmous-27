"""
occupancy_grid.py

Pure computational core for converting a list of obstacles (as produced by
the upstream Obstacle Feature Extraction stage) into a nav_msgs/OccupancyGrid.

Deliberately contains ZERO rclpy / ROS node logic. It only knows how to turn
obstacle geometry into a numpy grid and package that grid into an
OccupancyGrid message. This separation makes the mapping math independently
unit-testable without spinning up a ROS node.

PERFORMANCE NOTES (Step 7 refactor):
    - Per-obstacle rasterization uses a single NumPy slice assignment
      (grid[r0:r1, c0:c1] = OCCUPIED) instead of a Python double for-loop
      building a list of (row, col) tuples. For an obstacle covering an
      NxM cell footprint this is O(1) NumPy calls instead of O(N*M)
      Python-level iterations.
    - Inflation is fully vectorized: all occupied cells are broadcast
      against the full offset kernel in one shot (N_occupied x M_offsets
      matrix op) instead of looping over the kernel in Python. There is
      no Python loop over grid cells anywhere in this module anymore.
    - The grid buffer is pre-allocated once in __init__ and reused
      (in-place .fill(FREE) + in-place writes) on every generate() call,
      so no per-callback heap allocation of the (height, width) array.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import time

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Header


@dataclass
class GenerationStats:
    """Summary statistics for a single occupancy grid generation pass."""

    num_obstacles_in: int = 0
    num_obstacles_skipped: int = 0
    occupied_cells_raw: int = 0
    occupied_cells_after_inflation: int = 0
    processing_time_ms: float = 0.0


class OccupancyGridGenerator:
    """
    Rasterizes obstacles into a 2D nav_msgs/OccupancyGrid.

    Responsibilities:
        - Convert obstacle centroids + bounding boxes from world (base_link)
          coordinates into grid cell indices.
        - Mark all cells covered by each obstacle's XY footprint as occupied
          (not just the centroid cell), via vectorized slice assignment.
        - Apply configurable circular inflation around occupied cells,
          via vectorized broadcasting (no Python-level cell loop).
        - Package the result into a nav_msgs/OccupancyGrid message.

    This class holds no ROS publishers/subscribers and performs no I/O.
    """

    UNKNOWN: int = -1
    FREE: int = 0
    OCCUPIED: int = 100

    def __init__(
        self,
        resolution: float,
        width: int,
        height: int,
        origin_x: float,
        origin_y: float,
        origin_z: float,
        inflation_radius: float,
        frame_id: str = "base_link",
        use_unknown_space: bool = False,
        known_region_min_x: Optional[float] = None,
        known_region_max_x: Optional[float] = None,
        known_region_min_y: Optional[float] = None,
        known_region_max_y: Optional[float] = None,
        logger: Optional[object] = None,
    ) -> None:
        """
        Args:
            resolution: Grid cell size in meters/cell. Must be > 0.
            width: Grid width in cells. Must be > 0.
            height: Grid height in cells. Must be > 0.
            origin_x: X position (meters, in `frame_id`) of cell (0,0)'s
                lower-left corner.
            origin_y: Y position (meters, in `frame_id`) of cell (0,0)'s
                lower-left corner.
            origin_z: Z position of the grid plane (informational only;
                the grid itself is 2D).
            inflation_radius: Radius in meters to inflate occupied cells by.
                Must be >= 0.
            frame_id: TF frame the published grid is expressed in (e.g.
                "base_link" for a robot-centric rolling window, or "odom"
                for a frame that doesn't jump when the robot moves).
            use_unknown_space: If False (default, preserves prior
                behavior), the grid starts entirely FREE and only
                obstacle footprints are marked OCCUPIED -- i.e. "no
                detected obstacle" is treated as "known free". If True,
                the grid instead starts entirely UNKNOWN (-1); only the
                axis-aligned box given by `known_region_*` (the sensor's
                actual coverage footprint -- typically the same box as
                the ROI filter) is marked FREE before obstacles are
                rasterized on top. This correctly distinguishes "sensed
                and clear" from "never observed" -- important because
                missing depth data can mean occlusion, out-of-range, or
                an invalid return, not necessarily clear terrain.
            known_region_min_x, known_region_max_x, known_region_min_y,
                known_region_max_y: Bounds (meters, in `frame_id`) of the
                region considered "actively sensed" this frame. Required
                if `use_unknown_space` is True; ignored otherwise.
            logger: Optional object exposing .warn()/.error()/.debug()
                (e.g. an rclpy node logger). If None, issues are silently
                tolerated (useful for unit tests).

        Raises:
            ValueError: If `use_unknown_space` is True but any
                `known_region_*` bound is missing or min >= max on an
                axis.
        """
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_z = float(origin_z)
        self.inflation_radius = float(inflation_radius)
        self.frame_id = str(frame_id)
        self.logger = logger

        self.use_unknown_space = bool(use_unknown_space)
        self._known_region: Optional[Tuple[float, float, float, float]] = None
        if self.use_unknown_space:
            bounds = (
                known_region_min_x,
                known_region_max_x,
                known_region_min_y,
                known_region_max_y,
            )
            if any(b is None for b in bounds):
                raise ValueError(
                    "use_unknown_space=True requires known_region_min_x/"
                    "max_x/min_y/max_y to all be set."
                )
            min_x, max_x, min_y, max_y = bounds
            if min_x >= max_x or min_y >= max_y:
                raise ValueError(
                    f"known_region bounds must satisfy min < max on both "
                    f"axes, got x=({min_x}, {max_x}), y=({min_y}, {max_y})"
                )
            self._known_region = (float(min_x), float(max_x), float(min_y), float(max_y))

        self._validate_config()

        # Precompute the circular inflation kernel once. Reused on every
        # update instead of recomputing per-callback.
        self._inflation_offsets: np.ndarray = self._compute_inflation_offsets()

        # Pre-allocated grid buffer, reused across generate() calls to
        # avoid a fresh (height, width) heap allocation every callback.
        # Reset in-place at the top of generate() via .fill(FREE).
        self._grid: np.ndarray = np.full(
            (self.height, self.width), self.FREE, dtype=np.int8
        )

    # ------------------------------------------------------------------ #
    # Configuration / validation
    # ------------------------------------------------------------------ #

    def _validate_config(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError(f"resolution must be > 0, got {self.resolution}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(
                f"width/height must be > 0, got width={self.width}, height={self.height}"
            )
        if self.inflation_radius < 0.0:
            raise ValueError(
                f"inflation_radius must be >= 0, got {self.inflation_radius}"
            )

    def _compute_inflation_offsets(self) -> np.ndarray:
        """
        Precompute integer (dr, dc) cell offsets covering a circle of
        radius `inflation_radius`, in cell units. Computed once at
        construction time (or whenever parameters change) since it never
        depends on obstacle data. Built with vectorized meshgrid + mask
        rather than a nested Python loop.
        """
        if self.inflation_radius <= 0.0:
            return np.zeros((0, 2), dtype=np.int32)

        cell_radius = int(np.ceil(self.inflation_radius / self.resolution))
        r2 = self.inflation_radius ** 2

        dr_range = np.arange(-cell_radius, cell_radius + 1)
        dr_grid, dc_grid = np.meshgrid(dr_range, dr_range, indexing="ij")
        dist2 = (dr_grid * self.resolution) ** 2 + (dc_grid * self.resolution) ** 2
        mask = dist2 <= r2

        offsets = np.stack([dr_grid[mask], dc_grid[mask]], axis=1).astype(np.int32)
        return offsets

    # ------------------------------------------------------------------ #
    # Coordinate conversion
    # ------------------------------------------------------------------ #

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert a world-frame (x, y) coordinate into a (row, col) grid
        index. row corresponds to Y, col corresponds to X, matching the
        nav_msgs/OccupancyGrid data layout convention
        (index = row * width + col).

        Note: the returned indices are NOT clamped to [0, height) / [0, width).
        Callers must clip/validate before indexing into the grid array.
        """
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        return row, col

    # ------------------------------------------------------------------ #
    # Main entry point
    # ------------------------------------------------------------------ #

    def generate(
        self, obstacles: Sequence[object], header: Header
    ) -> Tuple[OccupancyGrid, GenerationStats]:
        """
        Generate a complete OccupancyGrid message from a list of obstacles.

        Args:
            obstacles: Sequence of obstacle objects, each expected to expose
                .centroid (geometry_msgs/Point-like, with .x/.y), .width,
                .height, .depth (all floats). An empty sequence is valid
                and produces an all-free grid.
            header: The std_msgs/Header from the incoming ObstacleArray.
                Its stamp is preserved; frame_id is forced to
                `self.frame_id`.

        Returns:
            (OccupancyGrid message, GenerationStats)
        """
        start_time = time.time()
        stats = GenerationStats(num_obstacles_in=len(obstacles))

        # Reset the pre-allocated buffer in place instead of allocating a
        # new array every callback.
        grid = self._grid
        if self.use_unknown_space:
            # Grid starts entirely UNKNOWN: "no detected obstacle" no
            # longer silently means "free" everywhere -- only the
            # actively-sensed region (known_region, typically the ROI
            # filter's footprint) is stamped FREE below, before obstacle
            # footprints are rasterized on top of it.
            grid.fill(self.UNKNOWN)
            self._stamp_known_region_free(grid)
        else:
            # Prior behavior, preserved: grid starts fully FREE. Treats
            # the sensed local footprint as "known clear unless proven
            # occupied" rather than UNKNOWN, since this grid represents
            # actively-sensed nearby space, not an unexplored global map.
            grid.fill(self.FREE)

        for obstacle in obstacles:
            try:
                row_min, row_max, col_min, col_max = self._rasterize_obstacle(obstacle)
            except (ValueError, AttributeError, TypeError) as exc:
                stats.num_obstacles_skipped += 1
                if self.logger is not None:
                    self.logger.warn(f"Skipping invalid obstacle: {exc}")
                continue

            if row_min is None:
                # Footprint entirely outside grid bounds -- nothing to do.
                continue

            # Vectorized: one slice assignment marks the whole bounding
            # box occupied, replacing the old per-cell Python loop.
            grid[row_min : row_max + 1, col_min : col_max + 1] = self.OCCUPIED

        stats.occupied_cells_raw = int(np.count_nonzero(grid == self.OCCUPIED))

        if stats.occupied_cells_raw > 0 and self._inflation_offsets.shape[0] > 0:
            self._apply_inflation(grid)

        stats.occupied_cells_after_inflation = int(
            np.count_nonzero(grid == self.OCCUPIED)
        )

        msg = self._to_msg(grid, header)

        stats.processing_time_ms = (time.time() - start_time) * 1000.0
        return msg, stats

    # ------------------------------------------------------------------ #
    # Rasterization
    # ------------------------------------------------------------------ #

    def _stamp_known_region_free(self, grid: np.ndarray) -> None:
        """Marks the actively-sensed region (known_region) as FREE.

        Single vectorized slice assignment, exactly the same pattern as
        `_rasterize_obstacle`'s footprint marking -- no per-cell Python
        loop. Cells outside `known_region` (and outside the grid bounds
        entirely) are left at their current value (UNKNOWN).
        """
        min_x, max_x, min_y, max_y = self._known_region

        row_min, col_min = self.world_to_grid(min_x, min_y)
        row_max, col_max = self.world_to_grid(max_x, max_y)

        row_min = max(row_min, 0)
        row_max = min(row_max, self.height - 1)
        col_min = max(col_min, 0)
        col_max = min(col_max, self.width - 1)

        if row_min > row_max or col_min > col_max:
            # known_region falls entirely outside the grid -- nothing to
            # stamp; the whole grid stays UNKNOWN this frame.
            if self.logger is not None:
                self.logger.warn(
                    "OccupancyGridGenerator: known_region falls entirely "
                    "outside the grid bounds; grid will be all-UNKNOWN "
                    "this frame."
                )
            return

        grid[row_min : row_max + 1, col_min : col_max + 1] = self.FREE

    def _rasterize_obstacle(
        self, obstacle: object
    ) -> Tuple[Optional[int], Optional[int], Optional[int], Optional[int]]:
        """
        Convert a single obstacle's XY bounding box footprint into a
        clipped (row_min, row_max, col_min, col_max) index range suitable
        for a single NumPy slice assignment.

        Height/Z is intentionally ignored (2D map only). Obstacles fully
        outside the grid bounds return (None, None, None, None) rather
        than raising. Malformed obstacles (non-finite centroid, missing
        attributes) raise ValueError/AttributeError, which the caller
        catches and logs.
        """
        centroid: Point = obstacle.centroid
        cx, cy = float(centroid.x), float(centroid.y)

        if not (np.isfinite(cx) and np.isfinite(cy)):
            raise ValueError(f"non-finite centroid ({cx}, {cy})")

        obs_width = float(getattr(obstacle, "width", 0.0))
        obs_depth = float(getattr(obstacle, "depth", 0.0))

        # Invalid/degenerate bounding box dimensions: fall back to a
        # single-cell (point) footprint rather than dropping the obstacle
        # entirely, so a real detected obstacle is never silently ignored
        # just because upstream gave it a zero/negative extent.
        if not np.isfinite(obs_width) or obs_width <= 0.0:
            if self.logger is not None:
                self.logger.warn(
                    f"Obstacle has invalid width ({obs_width}); "
                    f"treating as point obstacle."
                )
            obs_width = self.resolution
        if not np.isfinite(obs_depth) or obs_depth <= 0.0:
            if self.logger is not None:
                self.logger.warn(
                    f"Obstacle has invalid depth ({obs_depth}); "
                    f"treating as point obstacle."
                )
            obs_depth = self.resolution

        x_min, x_max = cx - obs_width / 2.0, cx + obs_width / 2.0
        y_min, y_max = cy - obs_depth / 2.0, cy + obs_depth / 2.0

        row_min, col_min = self.world_to_grid(x_min, y_min)
        row_max, col_max = self.world_to_grid(x_max, y_max)

        # world_to_grid uses floor(), so min/max are already ordered
        # correctly as long as width/depth > 0 (guaranteed above).
        row_min = max(row_min, 0)
        row_max = min(row_max, self.height - 1)
        col_min = max(col_min, 0)
        col_max = min(col_max, self.width - 1)

        if row_min > row_max or col_min > col_max:
            # Entirely outside the grid bounds -- not an error, just
            # nothing to rasterize.
            return None, None, None, None

        return row_min, row_max, col_min, col_max

    # ------------------------------------------------------------------ #
    # Inflation
    # ------------------------------------------------------------------ #

    def _apply_inflation(self, grid: np.ndarray) -> None:
        """
        Grow every OCCUPIED cell outward by the precomputed circular
        inflation kernel, in place. Operates on the raw occupied-cell set
        from this update only (not iteratively on already-inflated cells),
        so inflation radius stays exact regardless of obstacle density.

        Fully vectorized: every occupied cell is broadcast against every
        kernel offset in a single (N_occupied x M_offsets) NumPy op,
        rather than looping over the offset kernel in Python.
        """
        occupied_rows, occupied_cols = np.nonzero(grid == self.OCCUPIED)
        if occupied_rows.size == 0:
            return

        # (N, 1) + (1, M) -> (N, M) broadcast, then flatten. No Python
        # loop over occupied cells or offsets.
        new_rows = occupied_rows[:, None] + self._inflation_offsets[None, :, 0]
        new_cols = occupied_cols[:, None] + self._inflation_offsets[None, :, 1]
        new_rows = new_rows.ravel()
        new_cols = new_cols.ravel()

        valid = (
            (new_rows >= 0)
            & (new_rows < self.height)
            & (new_cols >= 0)
            & (new_cols < self.width)
        )
        grid[new_rows[valid], new_cols[valid]] = self.OCCUPIED

    # ------------------------------------------------------------------ #
    # Message packaging
    # ------------------------------------------------------------------ #

    def _to_msg(self, grid: np.ndarray, header: Header) -> OccupancyGrid:
        """Package a numpy int8 grid into a nav_msgs/OccupancyGrid message."""
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.frame_id

        info = MapMetaData()
        info.map_load_time = header.stamp
        info.resolution = self.resolution
        info.width = self.width
        info.height = self.height

        origin = Pose()
        origin.position = Point(x=self.origin_x, y=self.origin_y, z=self.origin_z)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = origin

        msg.info = info
        # Row-major flatten: grid[row, col] -> data[row * width + col],
        # matching the OccupancyGrid convention exactly. .copy() so the
        # returned message doesn't alias the reused internal buffer.
        msg.data = grid.flatten(order="C").astype(np.int8).tolist()

        return msg
