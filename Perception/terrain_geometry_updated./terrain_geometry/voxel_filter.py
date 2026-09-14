"""Vectorized NumPy voxel grid downsampling.

This module implements ONLY the downsampling math: given an (N, 3)
NumPy array of XYZ points, partition space into a regular voxel grid
and replace all points within each occupied voxel by their centroid.
It has no knowledge of ROS topics, messages, or TF -- that wiring
lives entirely in `terrain_node.py` -- so this class can be
unit-tested or reused standalone, and ROS communication stays cleanly
separated from point cloud processing.

WHY NOT OPEN3D:
    `open3d.geometry.PointCloud.voxel_down_sample` only supports a
    single uniform voxel size, requires converting to/from an Open3D
    cloud, and hides its per-voxel averaging inside its C++ backend
    with no control over axis-specific resolution. This module instead
    implements the same centroid-based downsampling directly on the
    (N, 3) NumPy array already produced by `pointcloud_utils`, with:
        - independent `leaf_size_x` / `leaf_size_y` / `leaf_size_z`
          voxel dimensions (non-cubic voxels), and
        - the grouping done via `np.unique(..., axis=0,
          return_inverse=True, return_counts=True)` plus `np.bincount`
          for the per-voxel sums -- both are vectorized, C-level
          "scatter" operations. There is no Python `for` loop over
          points or voxels anywhere in this module.
"""

from __future__ import annotations

import numpy as np


class VoxelFilterError(RuntimeError):
    """Raised when voxel downsampling cannot be performed on the input."""


class VoxelFilter:
    """Applies voxel grid downsampling to an (N, 3) NumPy XYZ array.

    Wraps a fully vectorized NumPy voxel-grid + centroid computation
    with input validation and clear parameterization so voxel
    dimensions can be tuned from ROS parameters without touching this
    class. This class has no dependency on rclpy, sensor_msgs, Open3D,
    or any ROS type -- it operates purely on NumPy arrays.

    Attributes:
        leaf_size_x: Voxel edge length (meters) along X.
        leaf_size_y: Voxel edge length (meters) along Y.
        leaf_size_z: Voxel edge length (meters) along Z.
    """

    def __init__(
        self,
        leaf_size_x: float = 0.03,
        leaf_size_y: float = 0.03,
        leaf_size_z: float = 0.03,
    ) -> None:
        """Validates and stores the per-axis voxel dimensions.

        Args:
            leaf_size_x: Voxel edge length along X, in meters. Must be
                strictly positive.
            leaf_size_y: Voxel edge length along Y, in meters. Must be
                strictly positive.
            leaf_size_z: Voxel edge length along Z, in meters. Must be
                strictly positive.

        Raises:
            ValueError: If any leaf size is not strictly positive.
        """
        if leaf_size_x <= 0.0 or leaf_size_y <= 0.0 or leaf_size_z <= 0.0:
            raise ValueError("leaf_size_x/y/z must all be positive.")

        self.leaf_size_x = leaf_size_x
        self.leaf_size_y = leaf_size_y
        self.leaf_size_z = leaf_size_z

        # Stored once as a NumPy array so the per-callback division
        # below doesn't rebuild it from three Python floats every time.
        self._leaf_size = np.array(
            (leaf_size_x, leaf_size_y, leaf_size_z), dtype=np.float64
        )

    def filter(self, xyz: np.ndarray) -> np.ndarray:
        """Downsamples an XYZ array using a per-axis voxel grid.

        Every point is assigned to the voxel it falls into (a regular
        box partition of space with edge lengths `leaf_size_x/y/z`);
        all points sharing a voxel are then replaced by their
        centroid. This both reduces point count and gives an
        (approximately) spatially-uniform point density, regardless of
        how dense the original sampling was in any particular region.

        Args:
            xyz: An (N, 3) array of XYZ points to downsample.

        Returns:
            An (M, 3) float32 array containing one centroid point per
            occupied voxel. If `xyz` has zero rows, an empty (0, 3)
            array is returned rather than raising an error, since an
            empty cloud downsampled is still trivially empty.

        Raises:
            VoxelFilterError: If the voxel-grid computation itself
                fails (e.g. due to non-finite input that slipped past
                upstream NaN filtering).
        """
        if xyz.shape[0] == 0:
            # Downsampling an empty cloud is a well-defined no-op; no
            # need to treat it as an error here. Callers that want to
            # skip empty input entirely can check before calling.
            return np.empty((0, 3), dtype=np.float32)

        try:
            # --- Vectorized voxel index computation ------------------
            # One floor-division over the whole (N, 3) array at once;
            # no per-point Python loop. Indices are relative to the
            # cloud's own bounding box minimum so voxel boundaries
            # don't depend on the sensor's absolute coordinate origin.
            mins = xyz.min(axis=0)
            voxel_indices = np.floor((xyz - mins) / self._leaf_size).astype(
                np.int64
            )

            # --- Vectorized grouping -----------------------------------
            # np.unique(..., axis=0, return_inverse=True) groups
            # identical voxel-index rows without any Python-level
            # loop; `inverse` maps each input point to its voxel's row
            # in the unique array, and `counts` gives each voxel's
            # point count in the same single call.
            _, inverse, counts = np.unique(
                voxel_indices, axis=0, return_inverse=True, return_counts=True
            )
            inverse = inverse.reshape(-1)  # defensive: keep 1-D across NumPy versions
            n_voxels = counts.shape[0]

            # --- Vectorized centroid computation -----------------------
            # np.bincount is a vectorized C-level scatter-add (grouped
            # sum keyed by `inverse`), run once per axis -- three calls
            # total, still no Python loop over points or voxels.
            sums = np.empty((n_voxels, 3), dtype=np.float64)
            sums[:, 0] = np.bincount(inverse, weights=xyz[:, 0], minlength=n_voxels)
            sums[:, 1] = np.bincount(inverse, weights=xyz[:, 1], minlength=n_voxels)
            sums[:, 2] = np.bincount(inverse, weights=xyz[:, 2], minlength=n_voxels)

            # In-place divide: reuses the `sums` buffer as the output
            # rather than allocating a separate centroids array before
            # the final float32 cast.
            counts_f = counts.astype(np.float64, copy=False)
            np.divide(sums, counts_f[:, None], out=sums)

            return sums.astype(np.float32, copy=False)
        except Exception as exc:  # noqa: BLE001 - re-raise as a domain error
            raise VoxelFilterError(
                f"Vectorized voxel downsampling failed: {exc}"
            ) from exc
