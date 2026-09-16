"""TF2 transformation helper for sensor_msgs/PointCloud2 messages.

This module isolates all TF2-related logic (buffer, listener, lookup,
and the actual cloud transformation) behind a single class,
`CloudFrameTransformer`, so that the ROS 2 node itself stays thin and
easy to read.

Only the coordinate-transformation responsibility lives here. No
filtering, clustering, ground removal, or navigation logic is performed.

PERFORMANCE NOTES (RealSense D435, rigidly mounted sensor):
    1. Static Transform Caching - the TF lookup is only ever performed
       until it succeeds *once*. After that, the resulting 4x4 NumPy
       matrix is cached on `self.transform_matrix` and reused for every
       subsequent point cloud, completely skipping
       `tf_buffer.lookup_transform()` in steady-state operation.
    2. NumPy Vectorization - point transformation is a single batched
       `P_out = P_in @ R.T + t` matrix multiply. There is no Python
       loop over individual points.
    3. Memory Efficiency - the incoming PointCloud2 byte buffer is
       viewed (not copied) via `np.frombuffer` for reading, and only
       one copy of the raw bytes is made to build the outgoing
       message (unavoidable, since a new message must own its data).
       No extra per-field temporary arrays or full-precision (float64)
       upcasts are created.
"""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField

import tf2_ros
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from geometry_msgs.msg import TransformStamped

# NOTE: tf2_sensor_msgs.do_transform_cloud() is no longer used. It looks
# up + applies a transform generically for *any* pair of frames on every
# call and internally re-derives the rotation matrix from the quaternion
# each time. Since the D435 -> base_link transform is static/rigid, we
# derive that matrix once ourselves (see `_transform_to_matrix`) and
# apply it with a single vectorized NumPy matmul instead.

# Maps ROS PointField datatypes to NumPy scalar types, used to build a
# structured dtype that lets us read/write PointCloud2.data as a NumPy
# array with zero manual byte-offset math per point.
_DATATYPE_TO_NUMPY = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def _fields_to_dtype(fields: list[PointField], point_step: int) -> np.dtype:
    """Builds a structured NumPy dtype matching a PointCloud2 layout.

    This lets `np.frombuffer(cloud.data, dtype=...)` treat the raw byte
    buffer directly as an array of structured points (x, y, z, and
    whatever else the sensor publishes, e.g. rgb), with zero copying
    and zero per-point Python-level unpacking.
    """
    names, formats, offsets = [], [], []
    for f in fields:
        if f.count != 1:
            # Uncommon for RealSense PointCloud2 output (x/y/z/rgb are
            # all count=1); skip multi-count fields rather than guess.
            continue
        names.append(f.name)
        formats.append(_DATATYPE_TO_NUMPY[f.datatype])
        offsets.append(f.offset)
    return np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": point_step,
    })


def pointcloud2_to_xyz_array(cloud: PointCloud2) -> np.ndarray:
    """Zero-copy-read a PointCloud2's (x, y, z) fields into an (N, 3) array.

    Used as the very first step of the unified pipeline in
    `terrain_node.py`: the raw byte buffer is only *viewed* (no copy) via
    `np.frombuffer`, and NaN/Inf points (routine on a D435 -- low
    reflectance, out-of-range depth) are dropped with a single vectorized
    boolean mask.

    Args:
        cloud: Incoming PointCloud2 message.

    Returns:
        An (N, 3) float32 array of valid, finite XYZ points. May be
        `(0, 3)` if the message is empty or has no valid points.
    """
    if cloud.width * cloud.height == 0 or not cloud.data:
        return np.empty((0, 3), dtype=np.float32)

    dtype = _fields_to_dtype(cloud.fields, cloud.point_step)
    raw = np.frombuffer(bytes(cloud.data), dtype=dtype)

    xyz = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
        np.float32, copy=False
    )
    finite_mask = np.isfinite(xyz).all(axis=1)
    return xyz[finite_mask]


class CloudFrameTransformer:
    """Wraps a TF2 Buffer/TransformListener to transform PointCloud2 data.

    This class owns the TF2 buffer and listener lifecycle and exposes a
    single high-level method, `transform_cloud`, that (on the very first
    successful call) looks up the required transform, caches it as a
    4x4 NumPy matrix, and from then on applies that cached matrix to
    incoming clouds using vectorized NumPy operations -- no manual
    per-point Python loop and no repeated `lookup_transform` calls.

    Attributes:
        node: The owning rclpy Node, used for logging and clock access.
        buffer: The TF2 buffer that stores the transform tree.
        listener: The TF2 transform listener that populates `buffer`.
        timeout: Maximum duration to wait for a transform to become
            available before giving up on a given frame.
        transform_matrix: Cached 4x4 NumPy transform (source -> target),
            populated the first time a TF lookup succeeds. `None` until
            then.
    """

    def __init__(self, node: Node, timeout_sec: float = 0.2) -> None:
        """Initializes the TF2 buffer, listener, and internal state.

        Args:
            node: The rclpy Node that owns this transformer. Used for
                logging, clock, and executor integration.
            timeout_sec: How long (seconds) to wait for a transform
                lookup before treating it as unavailable.
        """
        self._node = node
        self._logger = node.get_logger()

        # The Buffer stores the transform tree; a longer cache time
        # (default is 10s in tf2_ros) helps absorb small timing jitter
        # between the point cloud timestamp and the TF broadcast.
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))

        # The TransformListener subscribes to /tf and /tf_static and
        # fills the buffer in the background. It must be kept alive for
        # as long as lookups are needed, hence storing it as an attribute
        # rather than a local variable.
        self.listener = TransformListener(self.buffer, node)

        self.timeout = Duration(seconds=timeout_sec)

        # --- Static transform cache (OPTIMIZATION 1) -------------------
        # `transform_matrix` starts empty and is derived exactly once,
        # the first time a lookup between a given (source, target) pair
        # succeeds. Because the D435 is rigidly mounted to the chassis,
        # that single lookup is valid for the lifetime of the node, so
        # every callback after the first successful one skips
        # `tf_buffer.lookup_transform()` entirely and just reuses this
        # matrix. If TF isn't ready yet on early frames, each incoming
        # message naturally acts as a retry until the lookup succeeds.
        self.transform_matrix: np.ndarray | None = None
        self._cached_source_frame: str | None = None
        self._cached_target_frame: str | None = None

        # Cache of structured dtypes keyed by (point_step, field layout)
        # so repeated messages with the same PointCloud2 layout (the
        # normal case) don't rebuild the dtype every callback.
        self._dtype_cache: dict[tuple, np.dtype] = {}

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> TransformStamped | None:
        """Looks up a transform between two frames at a given time.

        Args:
            target_frame: The frame to transform into (e.g. "base_link").
            source_frame: The frame the data currently lives in
                (e.g. "camera_depth_optical_frame").
            stamp: The timestamp associated with the data being
                transformed, taken from the incoming message header.

        Returns:
            A TransformStamped if the lookup succeeded, otherwise None.
            On failure, a warning is logged describing the specific TF2
            exception that occurred; the caller is expected to skip the
            frame and continue running rather than crash.
        """
        try:
            return self.buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=self.timeout,
            )
        except LookupException as exc:
            self._logger.warning(
                f"TF lookup failed (frame not yet known): {exc}"
            )
        except ConnectivityException as exc:
            self._logger.warning(
                f"TF lookup failed (no connection between "
                f"'{source_frame}' and '{target_frame}'): {exc}"
            )
        except ExtrapolationException as exc:
            self._logger.warning(
                f"TF lookup failed (extrapolation into the "
                f"future/past): {exc}"
            )
        except Exception as exc:  # noqa: BLE001 - defensive catch-all
            self._logger.error(
                f"Unexpected error during TF lookup: {exc}"
            )
        return None

    @staticmethod
    def _quaternion_to_rotation_matrix(
        x: float, y: float, z: float, w: float
    ) -> np.ndarray:
        """Converts a unit quaternion to a 3x3 rotation matrix.

        Implemented with plain NumPy (no tf_transformations / transforms3d
        dependency) per the "no unnecessary dependencies" constraint.
        """
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    def _transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """Converts a TransformStamped into a cached 4x4 homogeneous matrix."""
        t = transform.transform.translation
        q = transform.transform.rotation
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self._quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        matrix[:3, 3] = (t.x, t.y, t.z)
        return matrix

    def _get_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> np.ndarray | None:
        """Returns the cached static transform, looking it up only once.

        Fast path (steady-state, every frame after the first): returns
        the already-cached 4x4 matrix with no TF2 call at all.

        Slow path (only until the first success): performs a normal
        `lookup_transform`, which itself already retries/waits up to
        `self.timeout`; if TF isn't published yet, this method simply
        returns None and the *next* incoming point cloud will try again,
        naturally implementing "retry until dynamic TF becomes
        available" without blocking the executor.
        """
        if (
            self.transform_matrix is not None
            and source_frame == self._cached_source_frame
            and target_frame == self._cached_target_frame
        ):
            return self.transform_matrix

        transform = self.lookup_transform(target_frame, source_frame, stamp)
        if transform is None:
            return None

        self.transform_matrix = self._transform_to_matrix(transform)
        self._cached_source_frame = source_frame
        self._cached_target_frame = target_frame
        self._logger.info(
            f"Cached static transform '{source_frame}' -> '{target_frame}'. "
            "Future point clouds will reuse this matrix directly, without "
            "further tf_buffer.lookup_transform() calls."
        )
        return self.transform_matrix

    def _get_dtype(self, cloud: PointCloud2) -> np.dtype:
        """Returns (and caches) the structured dtype for this cloud's layout."""
        key = (
            cloud.point_step,
            tuple((f.name, f.offset, f.datatype, f.count) for f in cloud.fields),
        )
        dtype = self._dtype_cache.get(key)
        if dtype is None:
            dtype = _fields_to_dtype(cloud.fields, cloud.point_step)
            self._dtype_cache[key] = dtype
        return dtype

    def get_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> np.ndarray | None:
        """Public accessor for the cached (or freshly looked-up) 4x4 matrix.

        Exposed separately from `transform_cloud` so callers that already
        hold a raw (N, 3) NumPy array (e.g. `terrain_node.py`'s unified,
        zero-IPC pipeline) can apply the transform directly with a single
        matmul, without needing to round-trip through a PointCloud2
        message. Returns None if the transform is not yet available
        (caller should skip the frame and retry on the next message).
        """
        return self._get_transform_matrix(target_frame, source_frame, stamp)

    def transform_points(
        self,
        xyz: np.ndarray,
        source_frame: str,
        target_frame: str,
        stamp: Time,
    ) -> np.ndarray | None:
        """Transforms a raw (N, 3) XYZ array using the cached static matrix.

        This is the array-in/array-out counterpart to `transform_cloud`,
        used by the unified pipeline in `terrain_node.py` to keep all
        inter-step data as plain float32 NumPy arrays -- no PointCloud2
        (de)serialization between pipeline stages.

        Args:
            xyz: (N, 3) array of points expressed in `source_frame`.
            source_frame: The frame `xyz` is currently expressed in.
            target_frame: The frame to transform into.
            stamp: Timestamp associated with `xyz`, from the source
                message header.

        Returns:
            An (N, 3) float32 array in `target_frame`, or None if the
            transform is not yet available.
        """
        matrix = self._get_transform_matrix(target_frame, source_frame, stamp)
        if matrix is None:
            return None

        if xyz.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        # OPTIMIZATION 2: single vectorized batch transform, identical
        # math to `_apply_matrix` but operating directly on the caller's
        # array -- no PointCloud2 byte-buffer round trip at all.
        r_t = matrix[:3, :3].T.astype(np.float32, copy=False)
        t_vec = matrix[:3, 3].astype(np.float32, copy=False)
        return (xyz.astype(np.float32, copy=False) @ r_t) + t_vec

    def transform_cloud(
        self,
        cloud: PointCloud2,
        target_frame: str,
    ) -> PointCloud2 | None:
        """Transforms a PointCloud2 message into `target_frame`.

        Uses the cached static transform matrix (looked up once, then
        reused) and applies it with a single vectorized NumPy matmul
        over all points at once -- no per-point Python loop, and no
        repeated TF2 lookups per message.

        Args:
            cloud: The incoming PointCloud2 message, expressed in
                `cloud.header.frame_id`.
            target_frame: The frame to transform the cloud into.

        Returns:
            A new PointCloud2 message expressed in `target_frame`, or
            None if the transform was unavailable or the transformation
            itself failed. Callers should treat None as "skip this
            frame" rather than an error to propagate.
        """
        source_frame = cloud.header.frame_id
        stamp = Time.from_msg(cloud.header.stamp)

        # OPTIMIZATION 1: cached lookup -- only performs a real TF2
        # `lookup_transform` call until the first success; every call
        # after that returns the cached matrix directly.
        matrix = self._get_transform_matrix(target_frame, source_frame, stamp)
        if matrix is None:
            # Reason for the failure was already logged in
            # lookup_transform(); nothing further to do here.
            return None

        if cloud.width * cloud.height == 0 or not cloud.data:
            # Empty cloud: nothing to transform, just relabel the frame.
            transformed = PointCloud2()
            transformed.header = cloud.header
            transformed.header.frame_id = target_frame
            transformed.height = cloud.height
            transformed.width = cloud.width
            transformed.fields = cloud.fields
            transformed.is_bigendian = cloud.is_bigendian
            transformed.point_step = cloud.point_step
            transformed.row_step = cloud.row_step
            transformed.is_dense = cloud.is_dense
            transformed.data = cloud.data
            return transformed

        try:
            transformed_cloud = self._apply_matrix(cloud, matrix, target_frame)
        except Exception as exc:  # noqa: BLE001 - defensive catch-all
            self._logger.error(
                f"Vectorized cloud transform failed for frame "
                f"'{source_frame}' -> '{target_frame}': {exc}"
            )
            return None

        return transformed_cloud

    def _apply_matrix(
        self,
        cloud: PointCloud2,
        matrix: np.ndarray,
        target_frame: str,
    ) -> PointCloud2:
        """Applies a cached 4x4 transform to every point via one NumPy matmul.

        MEMORY EFFICIENCY (OPTIMIZATION 3): the input buffer is only
        *viewed* (`np.frombuffer`, zero copy) to read x/y/z. A single
        `bytearray` copy of the raw bytes is made to back the outgoing
        message (this copy is unavoidable -- the new message must own
        its own buffer and the non-geometric fields, e.g. rgb, must be
        preserved byte-for-byte). That output buffer is then updated
        in place through a writable structured view, so x/y/z are
        overwritten directly with no extra per-field temporary arrays.
        """
        dtype = self._get_dtype(cloud)

        # Zero-copy read view over the incoming message's raw bytes.
        raw = np.frombuffer(bytes(cloud.data), dtype=dtype)

        # OPTIMIZATION 2: fully vectorized batch transform.
        # P_in: (N, 3) array of all points at once (single stacking
        # copy, unavoidable since x/y/z are interleaved with other
        # fields like rgb in the source buffer).
        # P_out = P_in @ R.T + t   <-- single matmul, no Python loop.
        r_t = matrix[:3, :3].T.astype(np.float32, copy=False)
        t_vec = matrix[:3, 3].astype(np.float32, copy=False)

        p_in = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
            np.float32, copy=False
        )
        p_out = p_in @ r_t + t_vec

        # Single copy of the raw bytes to build the new message; then
        # overwrite x/y/z in place via a writable structured view so no
        # extra full-cloud array is allocated on top of it.
        out_bytes = bytearray(cloud.data)
        out_view = np.frombuffer(out_bytes, dtype=dtype)
        out_view["x"] = p_out[:, 0]
        out_view["y"] = p_out[:, 1]
        out_view["z"] = p_out[:, 2]

        transformed = PointCloud2()
        transformed.header = cloud.header
        transformed.header.frame_id = target_frame
        transformed.header.stamp = cloud.header.stamp
        transformed.height = cloud.height
        transformed.width = cloud.width
        transformed.fields = cloud.fields
        transformed.is_bigendian = cloud.is_bigendian
        transformed.point_step = cloud.point_step
        transformed.row_step = cloud.row_step
        transformed.is_dense = cloud.is_dense
        transformed.data = bytes(out_bytes)
        return transformed
