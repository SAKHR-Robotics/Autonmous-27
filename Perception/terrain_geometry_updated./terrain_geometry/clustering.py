#!/usr/bin/env python3
"""
clustering.py

Obstacle Clustering module for the terrain_geometry package.

Per the project spec, this module implements ONLY the DBSCAN clustering
stage. Ground removal, voxel filtering, statistical outlier removal,
bounding boxes-as-classification, occupancy mapping, SLAM, navigation,
and path planning are explicitly out of scope and are NOT implemented
here.

Contents:
    1. Pure NumPy conversion helpers between `sensor_msgs/PointCloud2`
       and plain (N, 3) float32 arrays -- including a colored (XYZRGB)
       encoder for RViz2 visualization and a labeled (XYZ + cluster_id)
       encoder used to forward per-point cluster assignments to Step 6
       (Feature Extraction).
    2. `DBSCANClusterer`: wraps `sklearn.cluster.DBSCAN` (KD-tree
       backend) for fast 3D Euclidean clustering, applies min/max
       cluster-size filtering, assigns a distinct color per surviving
       cluster (noise / rejected clusters = black), and returns the
       per-point labels, colors, and compact per-cluster summaries
       (id, size, centroid, bounding box) for downstream consumption.

No ROS node logic lives here -- see `terrain_node.py` for that. This
keeps the clustering math unit-testable without a running ROS graph.

Implementation notes on performance:
    - Open3D has been removed as a dependency. Point extraction from
      PointCloud2 uses `sensor_msgs_py.point_cloud2.read_points_numpy`
      (a single vectorized call) instead of any per-point Python loop.
    - Clustering uses `sklearn.cluster.DBSCAN(algorithm="kd_tree")`,
      which builds a KD-tree over the (N, 3) array for O(N log N)
      neighborhood queries -- this scales far better than a naive
      O(N^2) radius search for typical D435 obstacle clouds (a few
      thousand points per frame after upstream filtering).
    - Cluster-size filtering and relabeling are done with vectorized
      NumPy (np.unique + a small lookup table), not per-point loops.
    - Per-cluster summaries (centroid / bbox) loop over the *clusters*
      (typically tens, not thousands), which is negligible cost.
"""

from __future__ import annotations

import colorsys
from dataclasses import dataclass, field

import numpy as np
from sklearn.cluster import DBSCAN

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


# ---------------------------------------------------------------------------
# Conversion helpers: PointCloud2 <-> plain NumPy arrays
# ---------------------------------------------------------------------------

def pointcloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    """Convert a `sensor_msgs/PointCloud2` message into an (N, 3) float32 array.

    Only the (x, y, z) fields are read. NaN/Inf points are skipped, since
    upstream stages (even after SOR) may still pass through occasional
    invalid values at message boundaries. Uses a single vectorized
    `read_points_numpy` call -- no per-point Python loop -- to keep the
    callback cheap.

    Args:
        msg: Incoming PointCloud2 message (expected frame: base_link).

    Returns:
        An (N, 3) float32 NumPy array of valid XYZ points. May be empty
        (shape (0, 3)) if the message contains no valid points.

    Raises:
        ValueError: If the message cannot be parsed into XYZ float data.
    """
    try:
        structured_points = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"Failed to parse PointCloud2 fields: {exc}") from exc

    if structured_points.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    # read_points_numpy already returns a plain (N, 3) float array for a
    # simple field-name tuple; make sure dtype/contiguity are what
    # downstream code (sklearn) expects, without copying if avoidable.
    points_xyz = np.ascontiguousarray(structured_points, dtype=np.float32)
    return points_xyz


def xyz_and_colors_to_pointcloud2(
    points_xyz: np.ndarray, colors_rgb01: np.ndarray, header: Header
) -> PointCloud2:
    """Pack (N, 3) XYZ + (N, 3) RGB[0,1] arrays into a colored PointCloud2.

    Colors are packed into a single `rgb` FLOAT32 field using the
    standard PCL/RViz convention (24-bit RGB packed into the mantissa of
    a float32), so the result renders with per-point color directly in
    RViz2's PointCloud2 display (Color Transformer = "RGB8").

    Args:
        points_xyz: (N, 3) float array of point positions.
        colors_rgb01: (N, 3) float array of RGB values in [0, 1].
        header: ROS header to reuse (preserves frame_id and stamp).

    Returns:
        A `sensor_msgs/PointCloud2` with fields (x, y, z, rgb). If N==0,
        a valid empty PointCloud2 with the correct field layout is
        returned.
    """
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 16  # 4 floats * 4 bytes

    num_points = points_xyz.shape[0]

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.is_dense = True

    if num_points == 0:
        msg.width = 0
        msg.row_step = 0
        msg.data = b""
        return msg

    xyz = np.asarray(points_xyz, dtype=np.float32)
    rgb_uint8 = np.clip(np.round(np.asarray(colors_rgb01, dtype=np.float64) * 255.0), 0, 255).astype(np.uint8)

    # Pack R, G, B (each 0-255) into a single uint32: 0x00RRGGBB, then
    # reinterpret those same bytes as a float32 -- this is the exact
    # convention PCL/RViz expect for the packed "rgb" field.
    rgb_uint32 = (
        (rgb_uint8[:, 0].astype(np.uint32) << 16)
        | (rgb_uint8[:, 1].astype(np.uint32) << 8)
        | (rgb_uint8[:, 2].astype(np.uint32))
    )
    rgb_as_float32 = rgb_uint32.view(np.float32)

    structured_buffer = np.empty((num_points, 4), dtype=np.float32)
    structured_buffer[:, 0:3] = xyz
    structured_buffer[:, 3] = rgb_as_float32

    msg.width = num_points
    msg.row_step = point_step * num_points
    msg.data = structured_buffer.tobytes()
    return msg


def xyz_and_labels_to_pointcloud2(
    points_xyz: np.ndarray, labels: np.ndarray, header: Header
) -> PointCloud2:
    """Pack (N, 3) XYZ + (N,) integer cluster labels into a labeled PointCloud2.

    This is the machine-readable channel consumed by Step 6 (Feature
    Extraction): each point carries its final cluster ID (-1 for noise
    or a cluster rejected by the size filter) in an INT32 `cluster_id`
    field, so downstream code can group points per object without
    having to reverse-engineer packed RGB colors.

    Args:
        points_xyz: (N, 3) float array of point positions.
        labels: (N,) int array of cluster IDs per point (-1 = noise /
            filtered out).
        header: ROS header to reuse (preserves frame_id and stamp).

    Returns:
        A `sensor_msgs/PointCloud2` with fields (x, y, z, cluster_id).
    """
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="cluster_id", offset=12, datatype=PointField.INT32, count=1),
    ]
    point_step = 16  # 3 floats + 1 int32, 4 bytes each

    num_points = points_xyz.shape[0]

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.is_dense = True

    if num_points == 0:
        msg.width = 0
        msg.row_step = 0
        msg.data = b""
        return msg

    # Build the packed buffer as raw bytes: 3x float32 + 1x int32 per
    # point. A structured dtype view lets us do this in one vectorized
    # pass without a per-point Python loop.
    packed_dtype = np.dtype(
        {"names": ["x", "y", "z", "cluster_id"],
         "formats": [np.float32, np.float32, np.float32, np.int32],
         "itemsize": point_step}
    )
    structured_buffer = np.empty(num_points, dtype=packed_dtype)
    structured_buffer["x"] = points_xyz[:, 0]
    structured_buffer["y"] = points_xyz[:, 1]
    structured_buffer["z"] = points_xyz[:, 2]
    structured_buffer["cluster_id"] = labels.astype(np.int32)

    msg.width = num_points
    msg.row_step = point_step * num_points
    msg.data = structured_buffer.tobytes()
    return msg


# ---------------------------------------------------------------------------
# Per-cluster summary (forwarded to Step 6 as compact metadata)
# ---------------------------------------------------------------------------

@dataclass
class ClusterSummary:
    """Compact per-cluster metadata forwarded to Step 6 (Feature Extraction)."""

    cluster_id: int
    size: int
    centroid: tuple = field(default_factory=tuple)
    bbox_min: tuple = field(default_factory=tuple)
    bbox_max: tuple = field(default_factory=tuple)

    def to_dict(self) -> dict:
        return {
            "id": self.cluster_id,
            "size": self.size,
            "centroid": list(self.centroid),
            "bbox_min": list(self.bbox_min),
            "bbox_max": list(self.bbox_max),
        }


# ---------------------------------------------------------------------------
# DBSCAN clustering
# ---------------------------------------------------------------------------

class DBSCANClusterer:
    """Applies fast KD-tree DBSCAN clustering to an obstacle point cloud.

    Single responsibility: given an (N, 3) obstacle point array, group
    points into density-connected clusters via
    `sklearn.cluster.DBSCAN(algorithm="kd_tree")`, drop clusters outside
    the configured size band, assign each surviving cluster a distinct
    visualization color (noise/rejected = black), and report the
    resulting per-point labels, colors, and per-cluster summaries.

    Holds no ROS state and performs no I/O -- safe to unit test alone.
    """

    # Color assigned to noise points and to clusters rejected by the
    # min/max size filter (label == -1 after filtering).
    NOISE_COLOR = (0.0, 0.0, 0.0)

    # Golden-ratio conjugate used to space hues evenly and reproducibly
    # across an unknown number of clusters, so adjacent cluster IDs get
    # visually distinct (not just incrementally shifted) colors.
    _GOLDEN_RATIO_CONJUGATE = 0.6180339887498949

    def __init__(
        self,
        eps: float,
        min_points: int,
        min_cluster_size: int = 1,
        max_cluster_size: int = 2_000_000,
    ) -> None:
        """Initialize the clusterer with DBSCAN + size-filter parameters.

        Args:
            eps: Maximum neighborhood radius (meters) used by DBSCAN to
                consider two points density-connected.
            min_points: Minimum number of neighbors within `eps`
                required for a point to be considered a DBSCAN core
                point (`min_samples`).
            min_cluster_size: Clusters with fewer points than this are
                discarded (relabeled as noise, -1).
            max_cluster_size: Clusters with more points than this are
                discarded (relabeled as noise, -1) -- guards against a
                single runaway "cluster" swallowing e.g. an entire wall
                that leaked past ground removal.

        Raises:
            ValueError: If parameters are non-physical (e.g. <= 0) or
                if `min_cluster_size > max_cluster_size`.
        """
        if eps <= 0.0:
            raise ValueError(f"eps must be > 0.0, got {eps}")
        if min_points <= 0:
            raise ValueError(f"min_points must be > 0, got {min_points}")
        if min_cluster_size <= 0:
            raise ValueError(f"min_cluster_size must be > 0, got {min_cluster_size}")
        if max_cluster_size <= 0:
            raise ValueError(f"max_cluster_size must be > 0, got {max_cluster_size}")
        if min_cluster_size > max_cluster_size:
            raise ValueError(
                f"min_cluster_size ({min_cluster_size}) must be <= "
                f"max_cluster_size ({max_cluster_size})"
            )

        self.eps = float(eps)
        self.min_points = int(min_points)
        self.min_cluster_size = int(min_cluster_size)
        self.max_cluster_size = int(max_cluster_size)

        # Reused across calls: constructing this is cheap, but keeping
        # it as a single attribute makes the "fast KD-tree DBSCAN"
        # configuration explicit and easy to swap (e.g. n_jobs tuning)
        # in one place.
        self._dbscan = DBSCAN(
            eps=self.eps,
            min_samples=self.min_points,
            algorithm="kd_tree",
            n_jobs=-1,
        )

    def cluster(
        self, points_xyz: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, list[ClusterSummary]]:
        """Run DBSCAN clustering, filter by size, and colorize the result.

        Args:
            points_xyz: (N, 3) float32 array of obstacle points (clean,
                post ground removal, voxel downsampling, and SOR).

        Returns:
            A tuple `(labels, colors, summaries)`:
                - labels: (N,) int32 array, final cluster ID per point
                  (-1 for noise or a cluster rejected by the size
                  filter), in the same point order as `points_xyz`.
                - colors: (N, 3) float64 array of RGB values in [0, 1],
                  one row per input point.
                - summaries: list of `ClusterSummary`, one per
                  surviving cluster, ordered by cluster_id (0..K-1).

            If the input array is empty, returns empty labels/colors
            arrays and an empty summary list -- no exception raised.
        """
        num_input_points = points_xyz.shape[0]

        if num_input_points == 0:
            return (
                np.empty((0,), dtype=np.int32),
                np.empty((0, 3), dtype=np.float64),
                [],
            )

        # --- Step 1: KD-tree DBSCAN -----------------------------------
        raw_labels = self._dbscan.fit_predict(points_xyz).astype(np.int32)

        if raw_labels.size == 0 or raw_labels.max() < 0:
            # Every point is noise -- nothing survives the size filter
            # either, so short-circuit straight to an all-noise result.
            colors = np.tile(self.NOISE_COLOR, (num_input_points, 1))
            return np.full(num_input_points, -1, dtype=np.int32), colors, []

        # --- Step 2: size-based filtering + relabeling (vectorized) ---
        raw_ids, raw_counts = np.unique(
            raw_labels[raw_labels >= 0], return_counts=True
        )
        size_ok = (raw_counts >= self.min_cluster_size) & (
            raw_counts <= self.max_cluster_size
        )
        valid_raw_ids = raw_ids[size_ok]
        valid_sizes = raw_counts[size_ok]
        num_clusters = int(valid_raw_ids.shape[0])

        # Lookup table: old (raw) cluster id -> new contiguous cluster
        # id, or -1 if the cluster was filtered out. Single fancy-index
        # pass over all N points -- no per-point Python loop.
        remap = np.full(int(raw_labels.max()) + 1, -1, dtype=np.int32)
        remap[valid_raw_ids] = np.arange(num_clusters, dtype=np.int32)

        final_labels = np.where(
            raw_labels >= 0, remap[np.clip(raw_labels, 0, None)], -1
        ).astype(np.int32)

        # --- Step 3: colorize (vectorized lookup) ---------------------
        colors = self._assign_colors(final_labels, num_clusters)

        # --- Step 4: per-cluster summaries (loop over clusters, not
        # points -- cheap even for hundreds of clusters) --------------
        summaries: list[ClusterSummary] = []
        for cid in range(num_clusters):
            member_points = points_xyz[final_labels == cid]
            centroid = member_points.mean(axis=0)
            bbox_min = member_points.min(axis=0)
            bbox_max = member_points.max(axis=0)
            summaries.append(
                ClusterSummary(
                    cluster_id=cid,
                    size=int(valid_sizes[cid]),
                    centroid=tuple(float(v) for v in centroid),
                    bbox_min=tuple(float(v) for v in bbox_min),
                    bbox_max=tuple(float(v) for v in bbox_max),
                )
            )

        return final_labels, colors, summaries

    def _assign_colors(self, labels: np.ndarray, num_clusters: int) -> np.ndarray:
        """Generate an (N, 3) RGB color array, one row per input point.

        Each distinct cluster ID gets a visually distinct color derived
        by walking the HSV hue wheel in golden-ratio-conjugate steps
        (a standard trick for generating maximally-spread-out colors for
        an a-priori unknown number of categories). Noise / filtered-out
        points (-1) are colored black.

        Args:
            labels: (N,) int array of final cluster IDs per point
                (-1 = noise or filtered out).
            num_clusters: number of surviving clusters.

        Returns:
            (N, 3) float64 array of RGB values in [0, 1].
        """
        # Build a lookup table of size (num_clusters + 1, 3): row 0 is
        # reserved for noise (black), rows 1..num_clusters hold one
        # distinct color per cluster ID. Using labels+1 as the index
        # lets us fill every point's color with a single vectorized
        # fancy-index lookup instead of a per-point Python loop, which
        # matters once N reaches several thousand points per frame.
        color_table = np.empty((num_clusters + 1, 3), dtype=np.float64)
        color_table[0] = self.NOISE_COLOR

        hue = 0.0
        for cluster_id in range(num_clusters):
            hue = (hue + self._GOLDEN_RATIO_CONJUGATE) % 1.0
            # High saturation/value keeps clusters visually vivid and
            # clearly distinguishable from the black noise color.
            r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
            color_table[cluster_id + 1] = (r, g, b)

        colors = color_table[labels + 1]
        return colors
