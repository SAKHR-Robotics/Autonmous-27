"""
costmap_inflation.py

Pure computational core for turning a binary occupancy grid into an
inflated, exponentially-decaying costmap (nav2 InflationLayer-style),
expressed as a nav_msgs/OccupancyGrid.

Contains ZERO rclpy publishers/subscribers/parameters -- it only knows how
to turn a grid array into a cost array. This mirrors the separation
already used for occupancy_grid.py in the Step 7 stage: pure NumPy/SciPy
math lives here and is independently unit-testable; all ROS I/O lives in
terrain_node.py.

Why this is a NEW module rather than an edit to planner_interface.py:
    planner_interface.py's stated responsibility (see its own docstring)
    is validation/synchronization only -- it never touches cell values.
    Costmap rasterization is a distinct concern with its own failure
    modes (bad radii, degenerate grids) and its own dependency (SciPy),
    so it gets its own pure-logic module, exactly the same way
    occupancy_grid.py was kept separate from terrain_node.py in Step 7.
    PlannerInterface's validated/synchronized grid is simply the *input*
    to this stage.

Algorithm (vectorized, no Python-level loops over cells):
    1. Build a boolean obstacle mask directly from the OCCUPIED (=100)
       cells of the validated occupancy grid.
    2. Run scipy.ndimage.distance_transform_edt on the inverse mask to
       get, for every cell, the Euclidean distance (in cells) to the
       nearest obstacle cell. This is the C-accelerated replacement for
       a hand-rolled BFS/loop-based flood fill.
    3. Scale by grid resolution to convert cell distance -> meters.
    4. Vectorized threshold + exponential decay over the whole array at
       once:
           distance <= robot_radius            -> LETHAL (100)
           robot_radius < distance <= inflation_radius
               -> (LETHAL - 1) * exp(-cost_scaling_factor * (distance - robot_radius))
           distance > inflation_radius          -> unchanged (free/unknown)
       This matches the shape of nav2's inflation cost formula, adapted
       to the OccupancyGrid convention of 0-100 (vs. costmap_2d's 0-255).
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.ndimage import distance_transform_edt

from nav_msgs.msg import OccupancyGrid


@dataclass
class InflationStats:
    """Summary statistics for a single inflation pass."""

    lethal_cells: int = 0
    decayed_cells: int = 0
    free_cells: int = 0
    unknown_cells: int = 0
    processing_time_ms: float = 0.0


class CostmapInflator:
    """
    Converts a validated binary OccupancyGrid (0=free, 100=occupied,
    optionally -1=unknown) into an inflated costmap using a Euclidean
    distance transform and an exponential decay function.
    """

    LETHAL: int = 100
    FREE: int = 0
    UNKNOWN: int = -1

    def __init__(
        self,
        robot_radius: float,
        inflation_radius: float,
        cost_scaling_factor: float,
        inflate_unknown_cells: bool = False,
        logger: Optional[object] = None,
    ) -> None:
        """
        Args:
            robot_radius: Distance (m) from an obstacle within which cost
                is forced to LETHAL (100) -- i.e. the robot's own physical
                footprint. Must be >= 0.
            inflation_radius: Distance (m) from an obstacle beyond which
                cost decays back to the cell's original value. Must be
                strictly greater than robot_radius, or the decay band is
                empty/inverted.
            cost_scaling_factor: Exponential decay rate applied across the
                band between robot_radius and inflation_radius. Higher
                values produce a sharper (shorter-range) cost falloff.
                Must be >= 0.
            inflate_unknown_cells: If False (default, matches nav2's
                default InflationLayer behavior), cells whose original
                value is UNKNOWN (-1) are left untouched by inflation --
                the safety buffer only grows over cells we've positively
                observed as free. If True, inflation is applied uniformly
                regardless of original cell value.
            logger: Optional object exposing .warn()/.error()/.debug().
                If None, issues are silently tolerated (useful for tests).
        """
        if robot_radius < 0.0:
            raise ValueError(f"robot_radius must be >= 0, got {robot_radius}")
        if inflation_radius <= robot_radius:
            raise ValueError(
                f"inflation_radius ({inflation_radius}) must be > "
                f"robot_radius ({robot_radius})"
            )
        if cost_scaling_factor < 0.0:
            raise ValueError(
                f"cost_scaling_factor must be >= 0, got {cost_scaling_factor}"
            )

        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.cost_scaling_factor = float(cost_scaling_factor)
        self.inflate_unknown_cells = bool(inflate_unknown_cells)
        self.logger = logger

    # ------------------------------------------------------------------ #
    # Main entry point
    # ------------------------------------------------------------------ #

    def inflate(self, grid_msg: OccupancyGrid) -> "tuple[OccupancyGrid, InflationStats]":
        """
        Produce an inflated costmap OccupancyGrid from a validated input
        grid. header/info (resolution, width, height, origin) are carried
        over unchanged; only `.data` is replaced.

        Assumes `grid_msg` has already passed PlannerInterface's
        structural validation (width*height matches len(data), etc.) --
        this method does not re-validate that, to avoid duplicating
        checks the caller already performed.
        """
        import time

        start = time.time()
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution

        data = np.asarray(grid_msg.data, dtype=np.int16).reshape(height, width)
        obstacle_mask = data == self.LETHAL

        stats = InflationStats()

        if not obstacle_mask.any():
            # Nothing to inflate around -- pass the grid through as-is.
            # Still counts as a valid, cheap no-op pass.
            stats.free_cells = int(np.count_nonzero(data == self.FREE))
            stats.unknown_cells = int(np.count_nonzero(data == self.UNKNOWN))
            stats.processing_time_ms = (time.time() - start) * 1000.0
            return self._to_msg(data, grid_msg), stats

        # --- Distance transform (C-accelerated, whole-array, no loops) ---
        # distance_transform_edt(condition) gives, for every True cell in
        # `condition`, the distance to the nearest False cell. Passing the
        # inverse of the obstacle mask means: for every non-obstacle cell,
        # the distance to the nearest obstacle cell. Obstacle cells
        # themselves get distance 0.
        free_mask = ~obstacle_mask
        distance_cells = distance_transform_edt(free_mask)
        distance_m = distance_cells * resolution

        # --- Vectorized threshold + exponential decay (no loops) ---
        cost = data.astype(np.float64).copy()

        eligible = (
            np.ones_like(obstacle_mask)
            if self.inflate_unknown_cells
            else (data != self.UNKNOWN)
        )

        lethal_zone = obstacle_mask | ((distance_m <= self.robot_radius) & eligible)
        decay_zone = (
            ~lethal_zone
            & (distance_m > self.robot_radius)
            & (distance_m <= self.inflation_radius)
            & eligible
        )

        decay_values = (self.LETHAL - 1) * np.exp(
            -self.cost_scaling_factor * (distance_m - self.robot_radius)
        )
        cost[decay_zone] = decay_values[decay_zone]
        cost[lethal_zone] = self.LETHAL
        cost = np.clip(cost, 0, self.LETHAL)

        if not self.inflate_unknown_cells:
            # Cells that started UNKNOWN and were never pulled into the
            # lethal/decay zones stay UNKNOWN rather than becoming FREE
            # via the float->int8 round-trip.
            untouched_unknown = (data == self.UNKNOWN) & ~lethal_zone & ~decay_zone
            cost[untouched_unknown] = self.UNKNOWN

        stats.lethal_cells = int(np.count_nonzero(cost == self.LETHAL))
        stats.decayed_cells = int(np.count_nonzero(decay_zone))
        stats.free_cells = int(np.count_nonzero(cost == self.FREE))
        stats.unknown_cells = int(np.count_nonzero(cost == self.UNKNOWN))
        stats.processing_time_ms = (time.time() - start) * 1000.0

        return self._to_msg(cost, grid_msg), stats

    # ------------------------------------------------------------------ #
    # Message packaging
    # ------------------------------------------------------------------ #

    @staticmethod
    def _to_msg(cost: np.ndarray, source_msg: OccupancyGrid) -> OccupancyGrid:
        """
        Package a numpy cost array into a new OccupancyGrid, reusing the
        source message's header/info (resolution, width, height, origin,
        frame_id, stamp) unchanged.
        """
        msg = OccupancyGrid()
        msg.header = source_msg.header
        msg.info = source_msg.info
        msg.data = cost.astype(np.int8).flatten(order="C").tolist()
        return msg
