"""Ground plane segmentation: Patchwork++ with a vectorized fallback.

This module implements ONLY the segmentation math: given an (N, 3)
NumPy array of XYZ points (already expressed in base_link), split it
into ground and non-ground (obstacle) subsets. It has no knowledge of
ROS topics, messages, or TF -- that wiring lives entirely in
`terrain_node.py` -- so this class can be unit-tested or reused
standalone, and ROS communication stays cleanly separated from point
cloud processing.

TWO BACKENDS:
    1. Patchwork++ (`pypatchworkpp`), used when the official bindings
       are importable. Patchwork++ uses a Concentric Zone Model
       (concentric rings/sectors around the sensor origin) with
       region-wise plane fitting, so the ground estimate follows local
       terrain instead of assuming one flat plane for the whole scene
       -- while still being fully vectorized C++ under the hood,
       exposed to Python as a single batched call per cloud.
    2. A pure-NumPy vectorized concentric slope/height threshold, used
       automatically when `pypatchworkpp` is not installed. Points are
       binned into the same style of concentric range rings; each
       ring's local ground height is estimated as a low percentile of
       that ring's Z values, and any point within `height_threshold`
       meters of its ring's local floor is classified as ground. The
       only Python-level loop is over the (small, typically <= 16)
       number of rings -- never over individual points -- so this
       stays consistent with the "no per-point Python loops" rule.

`GroundRemoval` picks whichever backend is available at construction
time and exposes a single `.segment(xyz) -> (ground_xyz, obstacle_xyz)`
method regardless of which one is active.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

try:
    import pypatchworkpp
    _PATCHWORK_AVAILABLE = True
except ImportError:  # pragma: no cover - environment-dependent
    pypatchworkpp = None
    _PATCHWORK_AVAILABLE = False


class GroundNotFoundError(RuntimeError):
    """Raised when there are too few points to attempt ground segmentation."""


class GroundRemovalConfigError(RuntimeError):
    """Raised when the requested ground-removal backend cannot be honored.

    Specifically: `ground_removal_backend="patchwork"` was requested but
    `pypatchworkpp` is not importable, or the Patchwork++ backend raised
    while constructing/initializing. This is intentionally a *hard*
    failure (not a silent fallback) -- see `GroundRemoval.__init__` for
    the reasoning: a caller that explicitly asked for the production
    backend should never be silently downgraded to the weaker fallback.
    """


class _PatchworkBackend:
    """Wraps the official `pypatchworkpp` Concentric Zone Model bindings."""

    def __init__(
        self,
        sensor_height: float,
        num_zones: int,
        min_range: float,
        max_range: float,
        verbose: bool,
    ) -> None:
        params = pypatchworkpp.Parameters()
        params.sensor_height = sensor_height
        params.num_zones = num_zones
        params.min_range = min_range
        params.max_range = max_range
        params.verbose = verbose

        # One Patchwork++ instance is constructed at startup and reused
        # for every incoming cloud (the C++ backend keeps its own
        # internal working buffers), rather than re-constructing it --
        # and re-paying its setup cost -- per message.
        self._patchwork = pypatchworkpp.patchworkpp(params)

    def segment(self, xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        n_points = xyz.shape[0]

        # Patchwork++ expects an (N, 4) array: x, y, z, intensity. The
        # D435 depth-only cloud has no intensity channel, so a zero
        # column is appended -- a single vectorized allocation, not a
        # per-point Python loop.
        cloud_xyzi = np.zeros((n_points, 4), dtype=np.float32)
        cloud_xyzi[:, :3] = xyz

        # Single batched call: Patchwork++'s Concentric Zone Model plane
        # fitting runs entirely inside the C++ backend across all
        # points at once.
        self._patchwork.estimateGround(cloud_xyzi)

        ground_xyz = np.asarray(self._patchwork.getGround(), dtype=np.float32)
        obstacle_xyz = np.asarray(self._patchwork.getNonground(), dtype=np.float32)
        return ground_xyz, obstacle_xyz


class _VectorizedConcentricGroundRemoval:
    """Vectorized concentric-ring slope/height ground threshold.

    Fallback used when `pypatchworkpp` is unavailable. Approximates the
    spirit of a Concentric Zone Model without a compiled dependency:

        1. Bin every point into one of `num_zones` concentric range
           rings based on its horizontal distance `sqrt(x^2 + y^2)`
           from the sensor origin (vectorized `np.digitize`).
        2. Estimate each ring's local ground height as a low percentile
           (`floor_percentile`) of that ring's Z values -- robust to a
           handful of low outlier points, unlike a bare minimum.
        3. Classify a point as ground if it sits within
           `height_threshold` meters of its own ring's local floor
           (vectorized fancy-indexing + boolean comparison over the
           whole array at once).

    Points outside [min_range, max_range] are conservatively classified
    as obstacles (no reliable local ground estimate is available for
    them), matching the same convention as Patchwork++'s
    min_range/max_range parameters.

    The only Python-level loop is over the `num_zones` rings
    themselves (small and constant, e.g. 4-16), never over points.
    """

    def __init__(
        self,
        sensor_height: float,
        num_zones: int,
        min_range: float,
        max_range: float,
        height_threshold: float = 0.15,
        floor_percentile: float = 5.0,
    ) -> None:
        self.sensor_height = sensor_height
        self.num_zones = num_zones
        self.min_range = min_range
        self.max_range = max_range
        self.height_threshold = height_threshold
        self.floor_percentile = floor_percentile
        self._zone_edges = np.linspace(min_range, max_range, num_zones + 1)

    def segment(self, xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ranges = np.hypot(xyz[:, 0], xyz[:, 1])
        in_range = (ranges >= self.min_range) & (ranges <= self.max_range)

        # Vectorized bin assignment for every point at once.
        zone_idx = np.clip(
            np.digitize(ranges, self._zone_edges) - 1, 0, self.num_zones - 1
        )

        # Loop bound is `num_zones` (a handful of concentric rings),
        # not the point count -- this is the same style of
        # per-cluster/per-zone loop already used elsewhere in this
        # package (e.g. clustering.py's per-cluster summaries).
        floors = np.full(self.num_zones, -self.sensor_height, dtype=np.float64)
        for zone in range(self.num_zones):
            zone_mask = in_range & (zone_idx == zone)
            if np.any(zone_mask):
                floors[zone] = np.percentile(
                    xyz[zone_mask, 2], self.floor_percentile
                )

        point_floor = floors[zone_idx]
        height_above_floor = xyz[:, 2] - point_floor

        ground_mask = in_range & (height_above_floor <= self.height_threshold)
        obstacle_mask = ~ground_mask

        return (
            xyz[ground_mask].astype(np.float32, copy=False),
            xyz[obstacle_mask].astype(np.float32, copy=False),
        )


class GroundRemoval:
    """Splits a point cloud into ground and obstacle subsets.

    Uses Patchwork++ (`pypatchworkpp`) when available; otherwise falls
    back to a pure-NumPy vectorized concentric slope/height threshold
    (`_VectorizedConcentricGroundRemoval`). Either way, this class has
    no dependency on rclpy, sensor_msgs, or any ROS type -- it operates
    purely on (N, 3) NumPy XYZ arrays.

    Attributes:
        sensor_height: Approximate height (meters) of the sensor/base
            origin above the ground plane.
        num_zones: Number of concentric zones (rings) used for
            region-wise ground estimation.
        min_range: Points closer than this (meters, in the XY plane)
            are excluded from ground estimation.
        max_range: Points farther than this (meters, in the XY plane)
            are excluded from ground estimation.
        backend_name: Either "patchwork++" or "vectorized_fallback",
            reporting which backend is actually active.
    """

    def __init__(
        self,
        sensor_height: float = 0.2,
        num_zones: int = 4,
        min_range: float = 0.2,
        max_range: float = 10.0,
        verbose: bool = False,
        height_threshold: float = 0.15,
        force_fallback: bool = False,
        backend: str = "auto",
        logger: object | None = None,
    ) -> None:
        """Validates parameters and constructs the selected backend.

        Args:
            sensor_height: Height of the sensor/base above the ground
                plane, in meters. Defaults are tuned for a small
                indoor mobile robot carrying a D435 rather than the
                library's automotive (KITTI) defaults.
            num_zones: Number of concentric zones for the Concentric
                Zone Model (must be >= 1).
            min_range: Minimum XY range (meters) considered for ground
                estimation (must be >= 0).
            max_range: Maximum XY range (meters) considered for ground
                estimation (must be > min_range).
            verbose: Enable Patchwork++'s own internal logging (ignored
                by the fallback backend).
            height_threshold: Fallback-only. Meters above a ring's
                local floor a point may sit and still count as ground.
            force_fallback: Deprecated alias, kept for backwards
                compatibility with any existing callers/tests. If True
                and `backend` is left at its default ("auto"), behaves
                as `backend="fallback"`. Has no effect if `backend` is
                explicitly set to "patchwork" or "fallback".
            backend: Which ground-removal backend to use. One of:
                - "patchwork": REQUIRE Patchwork++. If `pypatchworkpp`
                  is not importable, or it raises during construction,
                  this raises `GroundRemovalConfigError` immediately --
                  it never silently falls back to the weaker fallback
                  implementation, since a caller that explicitly asked
                  for the production backend should get a clear error
                  rather than a silent quality downgrade.
                - "fallback": Explicitly use the vectorized NumPy
                  fallback, regardless of whether Patchwork++ is
                  available. Useful for development/testing.
                - "auto" (default): Use Patchwork++ if it is importable
                  and initializes successfully; otherwise use the
                  fallback and emit a clear warning explaining why.
            logger: Optional object exposing `.warning()` / `.info()` /
                `.error()`, e.g. an rclpy node logger, used to report
                which backend was selected (and why, on fallback).

        Raises:
            ValueError: If any parameter is out of its valid range, or
                `backend` is not one of "patchwork"/"fallback"/"auto".
            GroundRemovalConfigError: If `backend="patchwork"` was
                requested but Patchwork++ is unavailable or fails to
                initialize.
        """
        if num_zones < 1:
            raise ValueError("num_zones must be >= 1.")
        if min_range < 0.0:
            raise ValueError("min_range must be >= 0.")
        if max_range <= min_range:
            raise ValueError("max_range must be greater than min_range.")
        if backend not in ("patchwork", "fallback", "auto"):
            raise ValueError(
                f"ground_removal_backend must be one of 'patchwork', "
                f"'fallback', 'auto' -- got {backend!r}."
            )

        self.sensor_height = sensor_height
        self.num_zones = num_zones
        self.min_range = min_range
        self.max_range = max_range

        # Legacy alias: force_fallback=True only takes effect when the
        # caller left `backend` at its default, so an explicit `backend`
        # argument always wins.
        effective_backend = backend
        if force_fallback and backend == "auto":
            effective_backend = "fallback"

        def _build_fallback(reason: str) -> None:
            self.backend_name = "vectorized_fallback"
            self._backend = _VectorizedConcentricGroundRemoval(
                sensor_height, num_zones, min_range, max_range, height_threshold
            )
            if logger is not None:
                logger.warning(
                    f"GroundRemoval: using vectorized concentric "
                    f"slope/height threshold fallback ({reason}). Install "
                    "'pypatchworkpp' for the full Patchwork++ backend: "
                    "https://github.com/url-kaist/patchwork-plusplus"
                )

        if effective_backend == "patchwork":
            # REQUIRE Patchwork++: never silently downgrade. A
            # configuration or initialization failure here is reported
            # clearly and propagated -- the node should fail to start
            # rather than run with weaker, unrequested ground removal.
            if not _PATCHWORK_AVAILABLE:
                raise GroundRemovalConfigError(
                    "ground_removal_backend='patchwork' was requested but "
                    "'pypatchworkpp' is not installed. Install it "
                    "(https://github.com/url-kaist/patchwork-plusplus) or "
                    "set ground_removal_backend to 'fallback' or 'auto'."
                )
            try:
                self._backend = _PatchworkBackend(
                    sensor_height, num_zones, min_range, max_range, verbose
                )
            except Exception as exc:  # noqa: BLE001 - re-raise as a domain error
                raise GroundRemovalConfigError(
                    "ground_removal_backend='patchwork' was requested but "
                    f"Patchwork++ failed to initialize: {exc}"
                ) from exc
            self.backend_name = "patchwork++"
            if logger is not None:
                logger.info(
                    "GroundRemoval: using Patchwork++ backend "
                    "(ground_removal_backend='patchwork')."
                )

        elif effective_backend == "fallback":
            _build_fallback("ground_removal_backend='fallback' (explicit)")

        else:  # "auto"
            if not _PATCHWORK_AVAILABLE:
                _build_fallback("'pypatchworkpp' is not installed")
            else:
                try:
                    self._backend = _PatchworkBackend(
                        sensor_height, num_zones, min_range, max_range, verbose
                    )
                    self.backend_name = "patchwork++"
                    if logger is not None:
                        logger.info(
                            "GroundRemoval: using Patchwork++ backend "
                            "(ground_removal_backend='auto')."
                        )
                except Exception as exc:  # noqa: BLE001
                    # Patchwork++ IS available but failed to init: this
                    # is reported as an explicit error (not hidden),
                    # then auto-mode falls back as documented.
                    if logger is not None:
                        logger.error(
                            f"GroundRemoval: Patchwork++ is installed but "
                            f"failed to initialize: {exc}"
                        )
                    _build_fallback(
                        "Patchwork++ failed to initialize (see error above)"
                    )

    def segment(self, xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Splits an XYZ point cloud into ground and obstacle subsets.

        Args:
            xyz: An (N, 3) array of XYZ points, expressed in a frame
                where "up" is well-defined relative to `sensor_height`
                (e.g. base_link).

        Returns:
            A 2-tuple of:
                - ground_xyz: (M, 3) float32 array of points classified
                  as ground.
                - obstacle_xyz: (K, 3) float32 array of points
                  classified as non-ground (obstacles).

        Raises:
            GroundNotFoundError: If the cloud has fewer than 3 points,
                too few for any meaningful ground estimate.
        """
        n_points = xyz.shape[0]
        if n_points < 3:
            raise GroundNotFoundError(
                f"Not enough points ({n_points}) to attempt ground "
                "segmentation; need at least 3."
            )
        return self._backend.segment(xyz)
