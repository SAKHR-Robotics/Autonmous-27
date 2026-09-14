"""
test_occupancy_grid_footprint.py

Part 2 mandatory regression test: an obstacle's occupancy-grid
footprint must use `depth` (X/longitudinal extent) for the grid's X
axis and `width` (Y/lateral extent) for the grid's Y axis -- NOT the
reverse. Before the fix, every obstacle was rasterized rotated 90
degrees.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from terrain_geometry.occupancy_grid import OccupancyGridGenerator
from std_msgs.msg import Header


def _make_generator(resolution=0.1, cells=40):
    # Grid centered on the origin: [-2, 2) x [-2, 2) meters at 0.1 m/cell.
    origin = -(cells * resolution) / 2.0
    return OccupancyGridGenerator(
        resolution=resolution,
        width=cells,
        height=cells,
        origin_x=origin,
        origin_y=origin,
        origin_z=0.0,
        inflation_radius=0.0,
    )


def _obstacle(x, y, width, depth, height=0.3):
    return SimpleNamespace(
        centroid=SimpleNamespace(x=x, y=y), width=width, depth=depth, height=height
    )


def _occupied_extent_cells(grid_np, resolution):
    rows, cols = np.nonzero(grid_np == OccupancyGridGenerator.OCCUPIED)
    row_extent = (rows.max() - rows.min() + 1) if rows.size else 0
    col_extent = (cols.max() - cols.min() + 1) if cols.size else 0
    return row_extent, col_extent  # (Y-extent-in-cells, X-extent-in-cells)


def test_elongated_obstacle_orientation_matches_depth_x_width_y():
    """Part 2's mandatory case: depth=1.0 m (X), width=0.4 m (Y) must
    rasterize LONG along X (columns) and SHORT along Y (rows)."""
    gen = _make_generator()
    header = Header()

    obstacle = _obstacle(x=0.0, y=0.0, width=0.4, depth=1.0)
    msg, stats = gen.generate([obstacle], header)

    grid_np = np.asarray(msg.data, dtype=np.int8).reshape(gen.height, gen.width)
    row_extent, col_extent = _occupied_extent_cells(grid_np, gen.resolution)

    row_extent_m = row_extent * gen.resolution
    col_extent_m = col_extent * gen.resolution

    # Row axis = Y (width = 0.4 m); column axis = X (depth = 1.0 m).
    assert row_extent_m == pytest.approx(0.4, abs=2 * gen.resolution)
    assert col_extent_m == pytest.approx(1.0, abs=2 * gen.resolution)
    assert col_extent > row_extent, (
        "Obstacle with depth > width must be rasterized LONGER along the "
        "grid's X (column) axis than its Y (row) axis."
    )


def test_wide_shallow_obstacle_orientation():
    """The complementary case: width=1.0 m (Y) > depth=0.3 m (X) must
    rasterize LONG along Y (rows) and SHORT along X (columns) -- the
    orientation must track the obstacle's actual shape, not a fixed
    axis."""
    gen = _make_generator()
    header = Header()

    obstacle = _obstacle(x=0.0, y=0.0, width=1.0, depth=0.3)
    msg, _stats = gen.generate([obstacle], header)

    grid_np = np.asarray(msg.data, dtype=np.int8).reshape(gen.height, gen.width)
    row_extent, col_extent = _occupied_extent_cells(grid_np, gen.resolution)

    assert row_extent > col_extent, (
        "Obstacle with width > depth must be rasterized LONGER along the "
        "grid's Y (row) axis than its X (column) axis."
    )


def test_square_obstacle_is_symmetric_regression_guard():
    """A square (width == depth) obstacle is orientation-agnostic, but
    still pins down the *position* of the footprint against regression:
    it must be centered on the obstacle's centroid, not offset."""
    gen = _make_generator()
    header = Header()

    obstacle = _obstacle(x=0.5, y=-0.3, width=0.5, depth=0.5)
    msg, _stats = gen.generate([obstacle], header)

    grid_np = np.asarray(msg.data, dtype=np.int8).reshape(gen.height, gen.width)
    rows, cols = np.nonzero(grid_np == OccupancyGridGenerator.OCCUPIED)

    center_row, center_col = gen.world_to_grid(0.5, -0.3)
    assert rows.min() <= center_row <= rows.max()
    assert cols.min() <= center_col <= cols.max()


def test_non_symmetric_offset_obstacle_footprint_bounds():
    """A rotated/off-center placement: verify the footprint's world-space
    bounding box (reconstructed from occupied cells) matches the
    obstacle's actual AABB in both axes independently, catching any
    axis swap even when centroid isn't at the origin."""
    gen = _make_generator(resolution=0.05, cells=80)
    header = Header()

    cx, cy, width, depth = 0.8, 0.2, 0.2, 0.6
    obstacle = _obstacle(x=cx, y=cy, width=width, depth=depth)
    msg, _stats = gen.generate([obstacle], header)

    grid_np = np.asarray(msg.data, dtype=np.int8).reshape(gen.height, gen.width)
    rows, cols = np.nonzero(grid_np == OccupancyGridGenerator.OCCUPIED)

    x_extent_m = (cols.max() - cols.min() + 1) * gen.resolution
    y_extent_m = (rows.max() - rows.min() + 1) * gen.resolution

    assert x_extent_m == pytest.approx(depth, abs=2 * gen.resolution)
    assert y_extent_m == pytest.approx(width, abs=2 * gen.resolution)
