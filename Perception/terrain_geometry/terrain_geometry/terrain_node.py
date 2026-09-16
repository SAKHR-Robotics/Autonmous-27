#!/usr/bin/env python3
"""
terrain_node.py

Unified ROS 2 node for the `terrain_geometry` package. Combines all 8
perception stages -- TF transform, ground removal, voxel downsampling,
outlier removal, clustering, feature extraction, occupancy grid
rasterization, and costmap inflation -- into a single, single-threaded
PointCloud2 subscriber callback.

DESIGN: ZERO-IPC, IN-MEMORY PIPELINE
    Every stage after the initial PointCloud2 decode operates on plain
    float32 NumPy arrays passed directly from one stage's return value
    into the next stage's call -- there is no intermediate ROS
    publish/subscribe, no re-serialization into PointCloud2, and no
    inter-process hop between Steps 1-6. This eliminates ROS 2 IPC
    (de)serialization latency that would otherwise be paid 5-6 times
    per frame if each stage were its own node.

    The only messages actually constructed mid-pipeline are the final
    outputs (`ObstacleFeatureArray`, `MarkerArray`, `OccupancyGrid`) and,
    optionally, debug PointCloud2 topics when `publish_debug_topics` is
    enabled -- those are for external consumption (RViz2, navigation),
    not internal hand-off, so building them is unavoidable.

PIPELINE STAGES (see the corresponding helper module for the math):
    1. tf_transform.py       -- sensor frame -> base_link (cached 4x4
                                 matrix, vectorized matmul). Finite-value
                                 (NaN/Inf) filtering happens in the same
                                 vectorized pass as the PointCloud2 decode,
                                 in tf_transform.pointcloud2_to_xyz_array.
    2. roi_filter.py         -- vectorized 3D ROI crop (optional, enabled
                                 by default). Runs immediately after the
                                 TF transform and before every expensive
                                 spatial-search stage below, so ground
                                 removal / outlier removal / clustering
                                 never see points outside the configured
                                 box.
    3. ground_removal.py     -- Patchwork++ (or vectorized concentric
                                 slope/height fallback) ground/obstacle
                                 split. Backend is explicitly selectable
                                 via `ground_removal_backend`.
    4. voxel_filter.py       -- per-axis voxel-grid centroid downsample.
    5. outlier_filter.py     -- cKDTree Radius Outlier Removal (ROR).
    6. clustering.py         -- KD-tree DBSCAN + min/max size filter.
    7. obstacle_features.py /
       visualization.py      -- per-cluster centroid/AABB + RViz markers.
    8. occupancy_grid.py     -- vectorized obstacle-footprint rasterization.
    9. costmap_inflation.py  -- distance-transform + exponential cost decay.

Per-stage timing is measured throughout via performance_profiling.py
(negligible overhead -- see that module) and optionally logged as a
compact summary; see `enable_performance_profiling` / `profiling_interval`.

OPTIMIZATION RULE: no per-point or per-cell Python loops anywhere in
this file or the modules it calls; every stage is a vectorized
NumPy/SciPy/scikit-learn call. The only Python-level loops in the
whole pipeline are bounded by the number of *clusters* or *ground
rings* (always small), never by point or cell count.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

from terrain_geometry_msgs.msg import ObstacleFeature as ObstacleFeatureMsg
from terrain_geometry_msgs.msg import ObstacleFeatureArray

from terrain_geometry.tf_transform import CloudFrameTransformer, pointcloud2_to_xyz_array
from terrain_geometry.roi_filter import ROIFilter, ROIFilterError
from terrain_geometry.ground_removal import (
    GroundRemoval,
    GroundNotFoundError,
    GroundRemovalConfigError,
)
from terrain_geometry.voxel_filter import VoxelFilter, VoxelFilterError
from terrain_geometry.outlier_filter import RadiusOutlierFilter, xyz_array_to_pointcloud2
from terrain_geometry.clustering import DBSCANClusterer, xyz_and_colors_to_pointcloud2
from terrain_geometry.obstacle_features import (
    ObstacleFeatureExtractor,
    ObstacleFeature,
    group_points_by_label,
)
from terrain_geometry.visualization import ObstacleMarkerBuilder
from terrain_geometry.occupancy_grid import OccupancyGridGenerator
from terrain_geometry.costmap_inflation import CostmapInflator
from terrain_geometry.performance_profiling import PipelineProfiler
from terrain_geometry.obstacle_tracking import ObstacleTracker, ObstacleTrackerConfigError


def _feature_to_msg(feature: ObstacleFeature) -> ObstacleFeatureMsg:
    """Convert an internal `ObstacleFeature` dataclass into its ROS message."""
    msg = ObstacleFeatureMsg()
    msg.id = int(feature.id)
    msg.num_points = int(feature.num_points)
    msg.centroid = Point(
        x=float(feature.centroid[0]), y=float(feature.centroid[1]), z=float(feature.centroid[2])
    )
    msg.min_point = Point(
        x=float(feature.min_point[0]), y=float(feature.min_point[1]), z=float(feature.min_point[2])
    )
    msg.max_point = Point(
        x=float(feature.max_point[0]), y=float(feature.max_point[1]), z=float(feature.max_point[2])
    )
    msg.width = float(feature.width)
    msg.height = float(feature.height)
    msg.depth = float(feature.depth)
    msg.distance = float(feature.distance)
    return msg


def _feature_to_grid_obstacle(feature: ObstacleFeature) -> SimpleNamespace:
    """Adapt an `ObstacleFeature` to the plain-attribute shape expected by
    `OccupancyGridGenerator` (`.centroid.x/.y`, `.width`, `.height`, `.depth`).

    Avoids a ROS message round trip (constructing an `ObstacleFeatureMsg`
    just to immediately read it back) purely to satisfy Step 7's
    duck-typed input contract.
    """
    return SimpleNamespace(
        centroid=SimpleNamespace(x=float(feature.centroid[0]), y=float(feature.centroid[1])),
        width=feature.width,
        height=feature.height,
        depth=feature.depth,
    )


class TerrainGeometryNode(Node):
    """Unified single-callback terrain/obstacle perception pipeline.

    Subscribes:
        <input_pointcloud_topic> (sensor_msgs/PointCloud2), QoS
            `qos_profile_sensor_data` (Best Effort), in the sensor's own
            optical frame (e.g. camera_depth_optical_frame).

    Publishes:
        /terrain/costmap (nav_msgs/OccupancyGrid): final inflated
            costmap, base output of the pipeline.
        /terrain/obstacle_features (terrain_geometry_msgs/ObstacleFeatureArray):
            per-obstacle geometric features for this frame.
        /terrain/obstacle_markers (visualization_msgs/MarkerArray):
            RViz2 cube + text markers, one pair per obstacle.
        /terrain/debug/ground_cloud, /terrain/debug/voxel_cloud,
        /terrain/debug/clustered_cloud (sensor_msgs/PointCloud2):
            only published when `publish_debug_topics` is True.
    """

    def __init__(self) -> None:
        super().__init__("terrain_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        # --- Pipeline stage objects, constructed once and reused ------
        self._tf = CloudFrameTransformer(self, timeout_sec=self._tf_timeout_sec)

        # ROI filter is optional: only constructed when enabled. Bounds
        # were already validated in _validate_parameters().
        self._roi_filter: ROIFilter | None = None
        if self._enable_roi_filter:
            self._roi_filter = ROIFilter(
                min_x=self._roi_min_x,
                max_x=self._roi_max_x,
                min_y=self._roi_min_y,
                max_y=self._roi_max_y,
                min_z=self._roi_min_z,
                max_z=self._roi_max_z,
            )

        # Ground-removal backend selection is fail-safe: if
        # ground_removal_backend='patchwork' was requested but
        # Patchwork++ is unavailable/broken, this raises
        # GroundRemovalConfigError, which propagates out of __init__ and
        # aborts node startup rather than silently running with a
        # weaker, unrequested backend.
        self._ground_removal = GroundRemoval(
            sensor_height=self._ground_sensor_height,
            num_zones=self._ground_num_zones,
            min_range=self._ground_min_range,
            max_range=self._ground_max_range,
            height_threshold=self._ground_height_threshold,
            backend=self._ground_removal_backend,
            logger=self.get_logger(),
        )
        self._voxel_filter = VoxelFilter(
            leaf_size_x=self._leaf_size_x,
            leaf_size_y=self._leaf_size_y,
            leaf_size_z=self._leaf_size_z,
        )
        self._outlier_filter = RadiusOutlierFilter(
            search_radius=self._search_radius,
            min_neighbors=self._min_neighbors,
            max_workers=self._outlier_removal_max_workers,
        )
        self._clusterer = DBSCANClusterer(
            eps=self._cluster_eps,
            min_points=self._cluster_min_points,
            min_cluster_size=self._min_cluster_size,
            max_cluster_size=self._max_cluster_size,
        )
        self._extractor = ObstacleFeatureExtractor(compute_obb=self._enable_obb)
        self._marker_builder = ObstacleMarkerBuilder()

        # Tracking is optional: only constructed when enabled. Params
        # already validated in _validate_parameters().
        self._tracker: ObstacleTracker | None = None
        if self._enable_obstacle_tracking:
            self._tracker = ObstacleTracker(
                max_association_distance=self._tracking_max_association_distance,
                max_missed_frames=self._tracking_max_missed_frames,
                position_smoothing=self._tracking_position_smoothing,
            )
        # The "known/sensed" region for UNKNOWN-space semantics is the
        # same box the ROI filter already crops to (it IS the region we
        # actually looked at this frame). If ROI filtering is disabled,
        # fall back to a symmetric box sized by ground_max_range, which
        # is the next-best approximation of "region actually sensed".
        if self._enable_roi_filter:
            known_min_x, known_max_x = self._roi_min_x, self._roi_max_x
            known_min_y, known_max_y = self._roi_min_y, self._roi_max_y
        else:
            known_min_x, known_max_x = -self._ground_max_range, self._ground_max_range
            known_min_y, known_max_y = -self._ground_max_range, self._ground_max_range

        self._grid_generator = OccupancyGridGenerator(
            resolution=self._grid_resolution,
            width=self._grid_width_cells,
            height=self._grid_height_cells,
            origin_x=self._grid_origin_x,
            origin_y=self._grid_origin_y,
            origin_z=self._grid_origin_z,
            inflation_radius=0.0,  # binary inflation disabled: Step 8
            frame_id=self._target_frame,  # owns the real (exponential) inflation.
            use_unknown_space=self._use_unknown_space,
            known_region_min_x=known_min_x,
            known_region_max_x=known_max_x,
            known_region_min_y=known_min_y,
            known_region_max_y=known_max_y,
            logger=self.get_logger(),
        )
        self._inflator = CostmapInflator(
            robot_radius=self._robot_radius,
            inflation_radius=self._costmap_inflation_radius,
            cost_scaling_factor=self._cost_scaling_factor,
            inflate_unknown_cells=self._inflate_unknown_cells,
            logger=self.get_logger(),
        )

        # Per-stage timing (cheap to keep on always; only the detailed
        # summary log is throttled by enable_performance_profiling +
        # profiling_interval -- see _cloud_callback).
        self._profiler = PipelineProfiler()
        self._frame_counter = 0

        self.add_on_set_parameters_callback(self._on_parameters_set)

        # --- Subscriber: single PointCloud2 input, sensor QoS ---------
        # Explicit BEST_EFFORT/VOLATILE profile (same reliability/
        # durability as qos_profile_sensor_data, so this remains
        # compatible with the RealSense ROS 2 driver's own publisher
        # QoS) but with a small, configurable queue depth. The goal:
        # if this node's processing ever falls behind the camera's
        # publish rate, the DDS layer drops OLD buffered point clouds
        # in favor of the newest one, rather than working through a
        # backlog of stale frames -- "process the newest available
        # frame", not "process frame 1, then frame 2, then frame 3...".
        # depth=1 (default) means only the single latest cloud is ever
        # queued; raise input_qos_depth if some jitter tolerance is
        # preferred over minimum latency.
        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self._input_qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._cloud_callback,
            input_qos,
        )

        # --- Primary + obstacle-feature publishers ---------------------
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, "/terrain/costmap", reliable_qos
        )
        self._feature_pub = self.create_publisher(
            ObstacleFeatureArray, "/terrain/obstacle_features", reliable_qos
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/terrain/obstacle_markers", reliable_qos
        )

        # --- Debug publishers (only *published to* when enabled) ------
        self._ground_cloud_pub = self.create_publisher(
            PointCloud2, "/terrain/debug/ground_cloud", qos_profile_sensor_data
        )
        self._voxel_cloud_pub = self.create_publisher(
            PointCloud2, "/terrain/debug/voxel_cloud", qos_profile_sensor_data
        )
        self._clustered_cloud_pub = self.create_publisher(
            PointCloud2, "/terrain/debug/clustered_cloud", qos_profile_sensor_data
        )

        self.get_logger().info(
            f"terrain_node initialized. Subscribing to '{self._input_topic}', "
            f"target_frame='{self._target_frame}', "
            f"roi_filter={'enabled' if self._enable_roi_filter else 'disabled'}, "
            f"ground_backend_requested='{self._ground_removal_backend}', "
            f"ground_backend_active='{self._ground_removal.backend_name}', "
            f"publish_debug_topics={self._publish_debug_topics}."
        )

    # ------------------------------------------------------------------ #
    # Parameters
    # ------------------------------------------------------------------ #
    def _declare_parameters(self) -> None:
        self.declare_parameter("input_pointcloud_topic", "/camera/depth/color/points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.2)
        # Small by design -- see the QoS setup in __init__ for why.
        self.declare_parameter("input_qos_depth", 1)

        # ROI filter (applied right after the TF transform, before ground
        # removal / voxel downsampling / outlier removal / clustering).
        # Defaults are a forward-facing box in base_link (REP-103: X
        # forward, Y left, Z up): 0.2-8m ahead, +-4m either side,
        # -0.5..1.5m vertically (below-wheel-height to well over rover
        # height), which comfortably covers the D435's practical depth
        # range on a ground rover.
        self.declare_parameter("enable_roi_filter", True)
        self.declare_parameter("roi_min_x", 0.2)
        self.declare_parameter("roi_max_x", 8.0)
        self.declare_parameter("roi_min_y", -4.0)
        self.declare_parameter("roi_max_y", 4.0)
        self.declare_parameter("roi_min_z", -0.5)
        self.declare_parameter("roi_max_z", 1.5)

        # Step 2: ground removal
        self.declare_parameter("ground_sensor_height", 0.2)
        self.declare_parameter("ground_num_zones", 4)
        self.declare_parameter("ground_min_range", 0.2)
        self.declare_parameter("ground_max_range", 10.0)
        self.declare_parameter("ground_height_threshold", 0.15)
        # One of "patchwork", "fallback", "auto" -- see ground_removal.py.
        self.declare_parameter("ground_removal_backend", "auto")

        # Step 3: voxel downsampling
        self.declare_parameter("leaf_size_x", 0.05)
        self.declare_parameter("leaf_size_y", 0.05)
        self.declare_parameter("leaf_size_z", 0.05)

        # Step 4: outlier removal (ROR)
        self.declare_parameter("enable_radius_outlier_removal", True)
        self.declare_parameter("search_radius", 0.2)
        self.declare_parameter("min_neighbors", 5)
        # -1 = all CPU cores (SciPy's own convention). Lower this on a
        # shared rover compute stack to avoid this stage monopolizing
        # CPU time away from other ROS 2 nodes.
        self.declare_parameter("outlier_removal_max_workers", -1)

        # Step 5: clustering
        self.declare_parameter("enable_clustering", True)
        self.declare_parameter("cluster_eps", 0.3)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("min_cluster_size", 10)
        self.declare_parameter("max_cluster_size", 50000)

        # Step 6: feature extraction / visualization
        self.declare_parameter("enable_obb", False)
        self.declare_parameter("enable_marker_visualization", True)

        # Step 6.5: temporal obstacle tracking (nearest-neighbor + smoothing)
        self.declare_parameter("enable_obstacle_tracking", False)
        self.declare_parameter("tracking_max_association_distance", 0.5)
        self.declare_parameter("tracking_max_missed_frames", 3)
        self.declare_parameter("tracking_position_smoothing", 0.5)

        # Step 7: occupancy grid
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_width_cells", 200)
        self.declare_parameter("grid_height_cells", 200)
        self.declare_parameter("grid_origin_x", -5.0)
        self.declare_parameter("grid_origin_y", -5.0)
        self.declare_parameter("grid_origin_z", 0.0)
        # If True: grid starts UNKNOWN (-1) everywhere except the
        # actively-sensed region (see terrain_node.py's known-region
        # derivation above _grid_generator's construction). If False
        # (default, preserves prior behavior): grid starts fully FREE.
        self.declare_parameter("use_unknown_space", False)

        # Step 8: costmap inflation
        self.declare_parameter("robot_radius", 0.3)
        self.declare_parameter("costmap_inflation_radius", 0.6)
        self.declare_parameter("cost_scaling_factor", 10.0)
        self.declare_parameter("inflate_unknown_cells", False)

        # Debug
        self.declare_parameter("publish_debug_topics", False)

        # Performance profiling: per-stage timing is always measured
        # (negligible overhead), but the detailed summary is only
        # logged when enabled, and only every `profiling_interval`
        # frames, to avoid flooding the console by default.
        self.declare_parameter("enable_performance_profiling", False)
        self.declare_parameter("profiling_interval", 30)

    def _read_parameters(self) -> None:
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self._input_topic = str(gp("input_pointcloud_topic"))
        self._target_frame = str(gp("target_frame"))
        self._tf_timeout_sec = float(gp("tf_timeout_sec"))
        self._input_qos_depth = int(gp("input_qos_depth"))

        self._enable_roi_filter = bool(gp("enable_roi_filter"))
        self._roi_min_x = float(gp("roi_min_x"))
        self._roi_max_x = float(gp("roi_max_x"))
        self._roi_min_y = float(gp("roi_min_y"))
        self._roi_max_y = float(gp("roi_max_y"))
        self._roi_min_z = float(gp("roi_min_z"))
        self._roi_max_z = float(gp("roi_max_z"))

        self._ground_sensor_height = float(gp("ground_sensor_height"))
        self._ground_num_zones = int(gp("ground_num_zones"))
        self._ground_min_range = float(gp("ground_min_range"))
        self._ground_max_range = float(gp("ground_max_range"))
        self._ground_height_threshold = float(gp("ground_height_threshold"))
        self._ground_removal_backend = str(gp("ground_removal_backend"))

        self._leaf_size_x = float(gp("leaf_size_x"))
        self._leaf_size_y = float(gp("leaf_size_y"))
        self._leaf_size_z = float(gp("leaf_size_z"))

        self._enable_radius_outlier_removal = bool(gp("enable_radius_outlier_removal"))
        self._search_radius = float(gp("search_radius"))
        self._min_neighbors = int(gp("min_neighbors"))
        self._outlier_removal_max_workers = int(gp("outlier_removal_max_workers"))

        self._enable_clustering = bool(gp("enable_clustering"))
        self._cluster_eps = float(gp("cluster_eps"))
        self._cluster_min_points = int(gp("cluster_min_points"))
        self._min_cluster_size = int(gp("min_cluster_size"))
        self._max_cluster_size = int(gp("max_cluster_size"))

        self._enable_obb = bool(gp("enable_obb"))
        self._enable_marker_visualization = bool(gp("enable_marker_visualization"))

        self._enable_obstacle_tracking = bool(gp("enable_obstacle_tracking"))
        self._tracking_max_association_distance = float(
            gp("tracking_max_association_distance")
        )
        self._tracking_max_missed_frames = int(gp("tracking_max_missed_frames"))
        self._tracking_position_smoothing = float(gp("tracking_position_smoothing"))

        self._grid_resolution = float(gp("grid_resolution"))
        self._grid_width_cells = int(gp("grid_width_cells"))
        self._grid_height_cells = int(gp("grid_height_cells"))
        self._grid_origin_x = float(gp("grid_origin_x"))
        self._grid_origin_y = float(gp("grid_origin_y"))
        self._grid_origin_z = float(gp("grid_origin_z"))
        self._use_unknown_space = bool(gp("use_unknown_space"))

        self._robot_radius = float(gp("robot_radius"))
        self._costmap_inflation_radius = float(gp("costmap_inflation_radius"))
        self._cost_scaling_factor = float(gp("cost_scaling_factor"))
        self._inflate_unknown_cells = bool(gp("inflate_unknown_cells"))

        self._publish_debug_topics = bool(gp("publish_debug_topics"))

        self._enable_performance_profiling = bool(gp("enable_performance_profiling"))
        self._profiling_interval = int(gp("profiling_interval"))

    def _validate_parameters(self) -> None:
        """Validates configurable parameters up front, with clear errors.

        Most numeric stage parameters are already validated by their
        owning class (VoxelFilter, RadiusOutlierFilter, DBSCANClusterer,
        GroundRemoval, OccupancyGridGenerator, CostmapInflator all raise
        ValueError from their own constructors). This method covers the
        remaining node-level parameters that aren't otherwise validated
        until a downstream object is constructed, so a bad configuration
        is reported clearly and the node refuses to start with silently
        invalid values, rather than defaulting or clamping.

        Raises:
            ValueError: If any parameter is out of its valid range.
        """
        if self._tf_timeout_sec <= 0.0:
            raise ValueError(
                f"tf_timeout_sec must be > 0, got {self._tf_timeout_sec}"
            )
        if self._input_qos_depth < 1:
            raise ValueError(
                f"input_qos_depth must be >= 1, got {self._input_qos_depth}"
            )

        if self._enable_roi_filter:
            if self._roi_min_x >= self._roi_max_x:
                raise ValueError(
                    f"roi_min_x ({self._roi_min_x}) must be < roi_max_x "
                    f"({self._roi_max_x})"
                )
            if self._roi_min_y >= self._roi_max_y:
                raise ValueError(
                    f"roi_min_y ({self._roi_min_y}) must be < roi_max_y "
                    f"({self._roi_max_y})"
                )
            if self._roi_min_z >= self._roi_max_z:
                raise ValueError(
                    f"roi_min_z ({self._roi_min_z}) must be < roi_max_z "
                    f"({self._roi_max_z})"
                )

        if self._ground_removal_backend not in ("patchwork", "fallback", "auto"):
            raise ValueError(
                "ground_removal_backend must be one of 'patchwork', "
                f"'fallback', 'auto' -- got {self._ground_removal_backend!r}"
            )

        if self._profiling_interval <= 0:
            raise ValueError(
                f"profiling_interval must be >= 1, got {self._profiling_interval}"
            )

        if self._grid_width_cells <= 0 or self._grid_height_cells <= 0:
            raise ValueError(
                "grid_width_cells/grid_height_cells must be > 0, got "
                f"width={self._grid_width_cells}, height={self._grid_height_cells}"
            )

        if self._outlier_removal_max_workers == 0 or self._outlier_removal_max_workers < -1:
            raise ValueError(
                "outlier_removal_max_workers must be -1 (all cores) or >= 1, "
                f"got {self._outlier_removal_max_workers}"
            )

        if self._enable_obstacle_tracking:
            if self._tracking_max_association_distance <= 0.0:
                raise ValueError(
                    "tracking_max_association_distance must be > 0, got "
                    f"{self._tracking_max_association_distance}"
                )
            if self._tracking_max_missed_frames < 0:
                raise ValueError(
                    "tracking_max_missed_frames must be >= 0, got "
                    f"{self._tracking_max_missed_frames}"
                )
            if not (0.0 < self._tracking_position_smoothing <= 1.0):
                raise ValueError(
                    "tracking_position_smoothing must be in (0, 1], got "
                    f"{self._tracking_position_smoothing}"
                )

        # Sanity check (warning, not a hard failure): DBSCAN's eps should
        # be comfortably larger than the voxel size, otherwise adjacent
        # voxel centroids of the *same* physical obstacle can end up
        # farther apart than eps and get incorrectly split into separate
        # clusters. A voxel-diagonal's worth of eps is the minimum for
        # same-obstacle centroids to reliably stay connected; below
        # that, this is very likely a misconfiguration.
        voxel_diagonal = (
            (self._leaf_size_x ** 2 + self._leaf_size_y ** 2 + self._leaf_size_z ** 2)
            ** 0.5
        )
        if self._enable_clustering and self._cluster_eps < voxel_diagonal:
            self.get_logger().warning(
                f"cluster_eps ({self._cluster_eps:.3f} m) is smaller than the "
                f"voxel diagonal ({voxel_diagonal:.3f} m from leaf_size_x/y/z); "
                "a single physical obstacle may get split into multiple "
                "DBSCAN clusters. Consider raising cluster_eps or lowering "
                "the voxel leaf sizes."
            )

    def _on_parameters_set(self, params) -> SetParametersResult:
        """Live-reconfigurable toggles: markers, OBB, debug topics.

        Numeric filter parameters (leaf sizes, radii, cluster sizes,
        etc.) are read once at startup only -- their backing objects
        (VoxelFilter, RadiusOutlierFilter, DBSCANClusterer, ...) are
        cheap to reconstruct but are intentionally not hot-swapped
        here to keep the callback's parameter-handling surface small;
        restart the node to change them.
        """
        for param in params:
            if param.name == "enable_marker_visualization":
                self._enable_marker_visualization = bool(param.value)
            elif param.name == "enable_obb":
                new_obb = bool(param.value)
                if new_obb != self._extractor.compute_obb:
                    self._extractor = ObstacleFeatureExtractor(compute_obb=new_obb)
                self._enable_obb = new_obb
            elif param.name == "publish_debug_topics":
                self._publish_debug_topics = bool(param.value)

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    # Main callback: the entire pipeline, one message in/out
    # ------------------------------------------------------------------ #
    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._frame_counter += 1
        profiler = self._profiler
        profiler.start()

        if msg.width * msg.height == 0 or not msg.data:
            self.get_logger().warn("Received empty PointCloud2 -- skipping frame.")
            return

        # --- Decode: PointCloud2 -> raw (N, 3) float32 array, zero-copy
        # read, finite-value filtering (NaN/Inf drop) applied in the same
        # vectorized pass. ---------------------------------------------
        xyz_sensor = pointcloud2_to_xyz_array(msg)
        profiler.mark("decode")
        if xyz_sensor.shape[0] == 0:
            self.get_logger().warn("No valid (finite) points in this frame -- skipping.")
            self._publish_empty(msg.header)
            return

        # --- TF transform (sensor frame -> target_frame) -----------------
        stamp = Time.from_msg(msg.header.stamp)
        xyz_base = self._tf.transform_points(
            xyz_sensor, msg.header.frame_id, self._target_frame, stamp
        )
        profiler.mark("tf")
        if xyz_base is None:
            self.get_logger().warn(
                f"TF transform '{msg.header.frame_id}' -> '{self._target_frame}' "
                "not yet available -- skipping frame (will retry next message)."
            )
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._target_frame

        # --- ROI crop: reduces point count as early as possible, before
        # any of the expensive spatial-search stages (ground removal,
        # KD-tree queries, DBSCAN). ---------------------------------------
        if self._roi_filter is not None:
            roi_xyz = self._roi_filter.filter(xyz_base)
        else:
            roi_xyz = xyz_base
        profiler.mark("roi")

        if roi_xyz.shape[0] == 0:
            self.get_logger().warn(
                f"No points remain after ROI filtering (in={xyz_sensor.shape[0]}) "
                "-- skipping frame."
            )
            self._publish_empty(header)
            return

        # --- Ground removal ------------------------------------------------
        try:
            ground_xyz, obstacle_xyz = self._ground_removal.segment(roi_xyz)
        except GroundNotFoundError as exc:
            self.get_logger().warn(f"Ground segmentation skipped: {exc}")
            self._publish_empty(header)
            return
        profiler.mark("ground")

        if obstacle_xyz.shape[0] == 0:
            self._publish_empty(header)
            if self._publish_debug_topics:
                self._ground_cloud_pub.publish(xyz_array_to_pointcloud2(ground_xyz, header))
            return

        # --- Voxel downsampling --------------------------------------------
        try:
            voxel_xyz = self._voxel_filter.filter(obstacle_xyz)
        except VoxelFilterError as exc:
            self.get_logger().error(f"Voxel downsampling failed: {exc}")
            return
        profiler.mark("voxel")

        # --- Radius outlier removal ------------------------------------------
        if self._enable_radius_outlier_removal:
            inlier_xyz, _outlier_xyz = self._outlier_filter.filter(voxel_xyz)
        else:
            inlier_xyz = voxel_xyz
        profiler.mark("outlier")

        if inlier_xyz.shape[0] == 0:
            self._publish_empty(header)
            if self._publish_debug_topics:
                self._ground_cloud_pub.publish(xyz_array_to_pointcloud2(ground_xyz, header))
                self._voxel_cloud_pub.publish(xyz_array_to_pointcloud2(inlier_xyz, header))
            return

        # --- DBSCAN clustering ------------------------------------------------
        if self._enable_clustering:
            labels, colors, _summaries = self._clusterer.cluster(inlier_xyz)
        else:
            # Clustering disabled: treat every surviving point as noise (no
            # obstacles reported), consistent with the "no clusters" empty
            # case elsewhere in the pipeline -- not a crash, not a fabricated
            # single giant obstacle.
            labels = np.full(inlier_xyz.shape[0], -1, dtype=np.int32)
            colors = np.zeros((inlier_xyz.shape[0], 3), dtype=np.float64)
        profiler.mark("cluster")

        # --- Feature extraction + RViz markers ---------------------------------
        clusters = group_points_by_label(inlier_xyz, labels)
        features = self._extractor.extract_all(clusters) if clusters else []
        profiler.mark("features")

        # --- Temporal obstacle tracking (optional) ------------------------------
        # Only ever returns detections matched THIS frame (see
        # obstacle_tracking.py) -- a missed detection never lingers on as
        # a phantom obstacle in the features/markers/grid/costmap below.
        if self._tracker is not None:
            features = self._tracker.update(features)
        profiler.mark("tracking")

        feature_array_msg = ObstacleFeatureArray()
        feature_array_msg.header = header
        feature_array_msg.obstacles = [_feature_to_msg(f) for f in features]
        self._feature_pub.publish(feature_array_msg)

        markers_to_build = features if self._enable_marker_visualization else []
        marker_array_msg = self._marker_builder.build_marker_array(markers_to_build, header)
        self._marker_pub.publish(marker_array_msg)

        # --- Occupancy grid rasterization --------------------------------------
        grid_obstacles = [_feature_to_grid_obstacle(f) for f in features]
        occupancy_msg, _grid_stats = self._grid_generator.generate(grid_obstacles, header)
        profiler.mark("occupancy")

        # --- Costmap inflation (distance transform + exp decay) ----------------
        costmap_msg, _inflation_stats = self._inflator.inflate(occupancy_msg)
        self._costmap_pub.publish(costmap_msg)
        profiler.mark("costmap")

        # --- Optional debug topics -----------------------------------------
        if self._publish_debug_topics:
            self._ground_cloud_pub.publish(xyz_array_to_pointcloud2(ground_xyz, header))
            self._voxel_cloud_pub.publish(xyz_array_to_pointcloud2(inlier_xyz, header))
            self._clustered_cloud_pub.publish(
                xyz_and_colors_to_pointcloud2(inlier_xyz, colors, header)
            )

        total_ms = profiler.total_ms()
        self.get_logger().info(
            f"Frame processed | in={xyz_sensor.shape[0]} pts, "
            f"roi={roi_xyz.shape[0]}, obstacle={obstacle_xyz.shape[0]}, "
            f"voxel={voxel_xyz.shape[0]}, inlier={inlier_xyz.shape[0]}, "
            f"obstacles={len(features)}, time={total_ms:.2f} ms"
        )

        self._maybe_log_profiling_summary(
            input_count=xyz_sensor.shape[0],
            roi_count=roi_xyz.shape[0],
            non_ground_count=obstacle_xyz.shape[0],
            voxel_count=voxel_xyz.shape[0],
            filtered_count=inlier_xyz.shape[0],
            cluster_count=len(features),
            total_ms=total_ms,
        )

    def _maybe_log_profiling_summary(
        self,
        input_count: int,
        roi_count: int,
        non_ground_count: int,
        voxel_count: int,
        filtered_count: int,
        cluster_count: int,
        total_ms: float,
    ) -> None:
        """Logs a compact per-stage performance summary, throttled by
        `profiling_interval`, only when `enable_performance_profiling` is
        True. Deliberately avoids logging every detailed stage on every
        frame by default, per the profiling requirements."""
        if not self._enable_performance_profiling:
            return
        if self._frame_counter % self._profiling_interval != 0:
            return

        counts = {
            "input": input_count,
            "roi": roi_count,
            "non_ground": non_ground_count,
            "voxel": voxel_count,
            "filtered": filtered_count,
            "clusters": cluster_count,
        }
        summary = self._profiler.format_summary(counts, total_ms)
        stage_breakdown = self._profiler.format_stage_breakdown()
        self.get_logger().info(f"{summary}\nstages: {stage_breakdown}")

    def _publish_empty(self, header: Header) -> None:
        """Publish empty-but-valid outputs so downstream consumers and
        RViz2 both reflect "nothing detected" instead of keeping stale
        data from the previous frame."""
        empty_features = ObstacleFeatureArray()
        empty_features.header = header
        self._feature_pub.publish(empty_features)

        empty_markers = self._marker_builder.build_marker_array([], header)
        self._marker_pub.publish(empty_markers)

        empty_grid_msg, _stats = self._grid_generator.generate([], header)
        empty_costmap_msg, _inflation_stats = self._inflator.inflate(empty_grid_msg)
        self._costmap_pub.publish(empty_costmap_msg)


def main(args: list[str] | None = None) -> None:
    """Standard ROS 2 Python entry point."""
    rclpy.init(args=args)
    node = TerrainGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
