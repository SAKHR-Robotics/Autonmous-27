#!/usr/bin/env python3
"""
outlier_filter.py

Radius Outlier Removal (ROR) module for the terrain_geometry package,
implemented with `scipy.spatial.cKDTree`.

This module is intentionally decoupled from ROS communication. It contains:

    1. Pure conversion helpers between `sensor_msgs/PointCloud2` and
       (N, 3) NumPy XYZ arrays (used by `terrain_node.py` only for the
       optional `publish_debug_topics` intermediate clouds -- the main
       pipeline stays in raw NumPy end-to-end).
    2. The `RadiusOutlierFilter` class, which builds a `cKDTree` over
       the input points and, in a single vectorized batch query, counts
       how many other points fall within `search_radius` of each point.
       Any point with fewer than `min_neighbors` such neighbors is
       classified as an isolated outlier (a floating depth artifact /
       phantom noise point) and removed.

No ROS node logic (subscriptions, publishers, parameter declarations)
lives here on purpose -- see `terrain_node.py` for that. This keeps the
point-cloud math unit-testable without needing a running ROS graph.

WHY RADIUS OUTLIER REMOVAL (ROR) INSTEAD OF STATISTICAL (SOR):
    SOR flags a point as an outlier based on how its mean distance to
    its k nearest neighbors compares to the *whole cloud's* global
    mean/std of that statistic. That is well suited to removing sparse
    noise spread evenly across a cloud, but it still assigns every
    point a neighbor-distance score even in regions that are already
    sparse for a legitimate reason (e.g. a distant, thinly-sampled
    obstacle edge), and its threshold depends on the whole cloud's
    statistics for the current frame.

    ROR instead asks a purely local, per-point question -- "does this
    point have at least `min_neighbors` other points within
    `search_radius` meters of it?" -- which directly targets floating
    depth artifacts and phantom noise (isolated points with almost no
    local support) without being influenced by the global distribution,
    and maps onto exactly two ROS-configurable parameters
    (`search_radius`, `min_neighbors`) as required by this pipeline's
    spec.

WHY cKDTree INSTEAD OF OPEN3D:
    Open3D's `remove_radius_outlier()` builds its own internal KD-tree
    and hides the per-point neighbor query inside its C++ backend, with
    no way to reuse the tree or the input array directly.
    `scipy.spatial.cKDTree` gives the same C-accelerated nearest-
    neighbor search, but as a single batched
    `tree.query_ball_point(xyz, r=..., return_length=True, workers=-1)`
    call over the whole (N, 3) array at once, and the inlier/outlier
    split is then a single vectorized boolean mask -- no per-point
    Python loop and no intermediate Open3D PointCloud object.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.spatial import cKDTree

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


# ---------------------------------------------------------------------------
# Conversion helpers: PointCloud2 <-> NumPy XYZ array
# ---------------------------------------------------------------------------

# Byte layout for an XYZ-only, float32 PointCloud2 point.
_XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_XYZ_POINT_STEP = 12  # 3 * float32 (4 bytes each)


def pointcloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    """Convert a `sensor_msgs/PointCloud2` message into an (N, 3) XYZ array.

    Only the (x, y, z) fields are extracted. NaN/Inf points are skipped,
    since depth-camera point clouds routinely contain invalid returns
    (e.g. from low-reflectance surfaces or out-of-range depth).

    Args:
        msg: Incoming PointCloud2 message (expected frame: base_link).

    Returns:
        An (N, 3) float32 NumPy array of the valid XYZ points. May be
        an (0, 3) array if the input message contains no valid points
        -- callers must handle this.

    Raises:
        ValueError: If the message cannot be parsed into XYZ float data.
    """
    try:
        structured_points = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
    except Exception as exc:  # noqa: BLE001 - re-raise as ValueError
        raise ValueError(f"Failed to parse PointCloud2 fields: {exc}") from exc

    if structured_points.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    return np.column_stack(
        (structured_points["x"], structured_points["y"], structured_points["z"])
    ).astype(np.float32, copy=False)


def xyz_array_to_pointcloud2(xyz: np.ndarray, header: Header) -> PointCloud2:
    """Pack an (N, 3) XYZ array into a `sensor_msgs/PointCloud2` message.

    Builds the `data` byte buffer directly via `ndarray.tobytes()`
    instead of `point_cloud2.create_cloud_xyz32()`, which packs each
    point with a per-point Python `struct.pack` call internally --
    meaningfully slower for large clouds.

    Args:
        xyz: An (N, 3) array of XYZ points (any float dtype; cast to
            float32 to match the declared PointField layout).
        header: ROS header to attach to the output message. Reusing
            the header from the input message preserves `frame_id`
            and the original `stamp`, as required by the pipeline
            contract.

    Returns:
        A valid `sensor_msgs/PointCloud2` message, XYZ32 layout. If
        `xyz` has zero rows, an empty-but-valid PointCloud2 is
        returned (correct field/metadata layout, zero rows) so
        downstream nodes never receive a malformed message.
    """
    xyz32 = np.ascontiguousarray(xyz, dtype=np.float32)
    n_points = xyz32.shape[0]

    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = n_points
    cloud_msg.fields = _XYZ_FIELDS
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = _XYZ_POINT_STEP
    cloud_msg.row_step = _XYZ_POINT_STEP * n_points
    cloud_msg.is_dense = True
    cloud_msg.data = xyz32.tobytes()
    return cloud_msg


# ---------------------------------------------------------------------------
# Radius Outlier Removal (cKDTree-accelerated)
# ---------------------------------------------------------------------------

class RadiusOutlierFilter:
    """Applies cKDTree-accelerated Radius Outlier Removal (ROR).

    This class has a single responsibility: given an (N, 3) NumPy XYZ
    array, compute which points have at least `min_neighbors` other
    points within `search_radius` meters of them (inliers), and which
    are isolated floating depth artifacts / phantom noise (outliers).

    It holds no ROS state and performs no I/O -- it is a pure
    point-cloud processing component, safe to unit test in isolation.
    """

    def __init__(
        self, search_radius: float, min_neighbors: int, max_workers: int = -1
    ) -> None:
        """Initialize the filter with ROR parameters.

        Args:
            search_radius: Radius (meters) within which neighbors are
                counted for each point.
            min_neighbors: Minimum number of *other* points a point
                must have within `search_radius` to be kept as an
                inlier.
            max_workers: Number of worker threads `cKDTree` may use for
                the batched radius query. `-1` (default) means "use all
                available CPU cores", matching SciPy's own convention.
                On a shared rover compute stack, set this to a smaller
                positive number (e.g. 2-4) to avoid this single stage
                monopolizing CPU time away from other ROS 2 nodes
                (localization, planning, control) running on the same
                machine -- see terrain_node.py's
                `outlier_removal_max_workers` parameter.

        Raises:
            ValueError: If parameters are non-physical (e.g. <= 0).
        """
        if search_radius <= 0.0:
            raise ValueError(f"search_radius must be > 0.0, got {search_radius}")
        if min_neighbors <= 0:
            raise ValueError(f"min_neighbors must be > 0, got {min_neighbors}")
        if max_workers == 0 or max_workers < -1:
            raise ValueError(
                f"max_workers must be -1 (all cores) or >= 1, got {max_workers}"
            )

        self.search_radius = float(search_radius)
        self.min_neighbors = int(min_neighbors)
        self.max_workers = int(max_workers)

    def filter(self, xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run Radius Outlier Removal on the given XYZ array.

        Args:
            xyz: Input (N, 3) XYZ array (obstacle cloud, post ground
                removal and voxel downsampling).

        Returns:
            A tuple `(inlier_xyz, outlier_xyz)`:
                - inlier_xyz: (M, 3) points that passed the ROR test.
                - outlier_xyz: (K, 3) points removed as outliers,
                  M + K == N.

            If the input is empty, both outputs are empty (0, 3)
            arrays -- no exception raised.
        """
        num_input_points = xyz.shape[0]
        if num_input_points == 0:
            return (
                np.empty((0, 3), dtype=xyz.dtype),
                np.empty((0, 3), dtype=xyz.dtype),
            )

        # --- C-accelerated spatial indexing --------------------------
        # A single cKDTree build over all points, then a single batched
        # radius-neighbor-count query for every point at once
        # (parallelized across available threads via workers=-1) --
        # no per-point Python loop. return_length=True asks SciPy to
        # return only the neighbor *count* per point (including the
        # point itself), skipping the cost of materializing every
        # neighbor index list.
        tree = cKDTree(xyz)
        neighbor_counts = tree.query_ball_point(
            xyz, r=self.search_radius, return_length=True, workers=self.max_workers
        )

        # Each point always finds itself (distance 0), so subtract 1 to
        # get the count of *other* points within the radius. Vectorized
        # comparison over the whole array -- no per-point loop.
        other_neighbor_counts = np.asarray(neighbor_counts, dtype=np.int64) - 1
        inlier_mask = other_neighbor_counts >= self.min_neighbors

        inlier_xyz = xyz[inlier_mask]
        outlier_xyz = xyz[~inlier_mask]

        return inlier_xyz, outlier_xyz
