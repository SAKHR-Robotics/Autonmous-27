# terrain_geometry

Single-node ROS 2 Jazzy terrain/obstacle perception pipeline for a rover
carrying an Intel RealSense D435, from raw `PointCloud2` to a costmap ready
for a planner.

## 1. Pipeline architecture

```
sensor_msgs/PointCloud2  (camera optical frame, e.g. camera_depth_optical_frame)
        |
        v
[1] PointCloud2 -> NumPy decode + finite-value (NaN/Inf) filter   (tf_transform.py)
        |            -- single vectorized pass, no Python loop over points
        v
[2] TF transform: sensor frame -> target_frame (base_link)        (tf_transform.py)
        |            -- cached 4x4 matrix, re-looked-up only on failure
        v
[3] 3D ROI crop  (enable_roi_filter)                               (roi_filter.py)
        |            -- vectorized box crop, runs BEFORE every expensive
        |               spatial-search stage below
        v
[4] Ground removal  (ground_removal_backend: patchwork|fallback|auto)  (ground_removal.py)
        |            -- Patchwork++ CZM segmentation, or NumPy fallback
        v
[5] Voxel downsampling  (voxel_filter.py)
        |            -- per-axis voxel-grid centroid downsample
        v
[6] Radius Outlier Removal  (enable_radius_outlier_removal)        (outlier_filter.py)
        |            -- cKDTree radius query, configurable worker count
        v
[7] DBSCAN clustering  (enable_clustering)                         (clustering.py)
        |            -- KD-tree DBSCAN + min/max cluster size filter
        v
[8] Obstacle feature extraction (centroid/AABB/OBB)                (obstacle_features.py)
        |
        v
[9] Temporal obstacle tracking  (enable_obstacle_tracking)         (obstacle_tracking.py)
        |            -- nearest-neighbor association + centroid smoothing
        |               ONLY emits detections matched this frame
        v
[10] Occupancy grid rasterization  (use_unknown_space)             (occupancy_grid.py)
        |            -- vectorized obstacle-footprint rasterization
        v
[11] Costmap inflation (distance transform + exponential decay)    (costmap_inflation.py)
        |
        v
nav_msgs/OccupancyGrid  ->  /terrain/costmap  ->  SLAM / Planner
```

Every stage above `terrain_node.py` is a pure computational module with
**zero rclpy/ROS dependency** -- each is independently importable and
unit-testable (see `PERFORMANCE NOTES` / module docstrings in each file).
`terrain_node.py` is only the ROS wiring: parameters, subscriptions,
publishers, and the per-frame call sequence.

## 2. Coordinate frames

- Sensor data arrives in the camera's own optical frame (e.g.
  `camera_depth_optical_frame`), taken from `msg.header.frame_id`.
- All processing after stage [2] happens in `target_frame` (default
  `base_link`), per REP-103: **X forward, Y left, Z up**.
- The D435 -> `base_link` transform is expected to be a static/rigid
  transform (URDF `<joint type="fixed">` or a `static_transform_publisher`).
  `CloudFrameTransformer` caches the resolved 4x4 matrix and only re-queries
  TF on lookup failure or after a configurable staleness window -- it does
  **not** perform a fresh `lookup_transform()` call on every single frame.
- If the transform isn't available yet (e.g. at startup, before
  `robot_state_publisher` / static TF broadcasters come up), the frame is
  safely skipped (logged as a warning) and retried on the next message --
  the node never processes data against a stale or invalid transform.

## 3. Ground-removal backend selection

`ground_removal_backend` (default `"auto"`):

| Value        | Behavior |
|--------------|----------|
| `patchwork`  | **Requires** Patchwork++ (`pypatchworkpp`). If it's not installed or fails to initialize, the node raises `GroundRemovalConfigError` at startup and refuses to run -- it never silently substitutes the weaker fallback for a caller that explicitly asked for the production backend. |
| `fallback`   | Explicitly uses the vectorized NumPy concentric slope/height-threshold fallback, regardless of whether Patchwork++ is available. Useful for development/CI without native deps. |
| `auto`       | Uses Patchwork++ if importable and it initializes successfully; otherwise falls back to the NumPy implementation and logs a clear warning explaining why. A Patchwork++ init failure (as opposed to it simply not being installed) is always logged as an `ERROR` first, so it's never silently hidden. |

Install Patchwork++ with `pip3 install pypatchworkpp`
(https://github.com/url-kaist/patchwork-plusplus). It's a soft/optional
dependency -- not declared in `package.xml`/`setup.py` -- so the package
builds and runs without it.

## 4. Region-of-interest (ROI) filtering

`enable_roi_filter` (default `True`) crops the cloud to an axis-aligned box
in `target_frame`, immediately after the TF transform and before ground
removal / outlier removal / clustering, since those are the expensive
spatial-search stages. Defaults are a forward-facing box tuned for a
D435 on a ground rover:

| Param | Default | Meaning |
|---|---|---|
| `roi_min_x` / `roi_max_x` | `0.2` / `8.0` m | Forward range (X). |
| `roi_min_y` / `roi_max_y` | `-4.0` / `4.0` m | Lateral range (Y, +left). |
| `roi_min_z` / `roi_max_z` | `-0.5` / `1.5` m | Vertical range (Z, +up). |

Tune these to your rover's actual footprint and the D435's realistic depth
range/FOV. Bounds are validated at startup (`min < max` on every axis);
an empty ROI intersection is handled safely (empty output published, no
crash) rather than treated as an error.

## 5. Voxel downsampling

`leaf_size_x/y/z` (default `0.05` m each) control the voxel-grid centroid
downsample. Keep leaf size in the **0.05-0.08 m** range: large enough to
meaningfully cut point count, small enough that the smallest
safety-relevant obstacle for your rover doesn't get merged away. Handles
empty/invalid input safely and is fully deterministic (no RNG).

## 6. Radius Outlier Removal (ROR)

`enable_radius_outlier_removal` (default `True`) toggles the stage
entirely. `search_radius` / `min_neighbors` control sensitivity.
`outlier_removal_max_workers` (default `-1`, all cores) bounds how many
threads the underlying `cKDTree` radius query may use -- lower this (e.g.
`2`-`4`) on a shared rover compute stack if this stage is starving other
ROS 2 nodes (localization, planning, control) of CPU.

## 7. DBSCAN clustering

`enable_clustering` (default `True`). `cluster_eps` / `cluster_min_points`
/ `min_cluster_size` / `max_cluster_size` as before. At startup, if
`cluster_eps` is smaller than the diagonal of the configured voxel
(`sqrt(leaf_size_x^2 + leaf_size_y^2 + leaf_size_z^2)`), a warning is
logged: adjacent voxel centroids of the *same* physical obstacle can end
up farther apart than `eps` and get incorrectly split into separate
clusters. When clustering is disabled, all surviving points are treated
as unclustered (zero obstacles reported) rather than crashing or
fabricating one giant fake obstacle.

## 8. Temporal obstacle tracking

`enable_obstacle_tracking` (default `False`). A lightweight nearest-
neighbor tracker (`obstacle_tracking.py`), kept fully decoupled from
`clustering.py`:

| Param | Default | Meaning |
|---|---|---|
| `tracking_max_association_distance` | `0.5` m | Max centroid distance to associate a detection with an existing track. |
| `tracking_max_missed_frames` | `3` | Frames a track may go unmatched before being dropped. |
| `tracking_position_smoothing` | `0.5` | Exponential smoothing weight given to each new measurement, in `(0, 1]`. `1.0` = no smoothing. |

Stable IDs reuse the existing `ObstacleFeature.id` field -- no message
schema change. **Only detections matched in the current frame are ever
emitted** -- a missed detection ages internally (so it can re-associate
after a brief occlusion) but never produces a phantom obstacle in the
occupancy grid or costmap on its own.

## 9. Occupancy-grid semantics

`use_unknown_space` (default `False`, preserves prior behavior exactly):

- **`False`**: grid starts entirely `FREE` (0); only obstacle footprints
  are marked `OCCUPIED` (100). "No detected obstacle" reads as "known
  clear". Matches the original pipeline's behavior.
- **`True`**: grid starts entirely `UNKNOWN` (-1); only the region
  actually sensed this frame (reused from the ROI box, or a
  `ground_max_range`-sized box if ROI filtering is disabled) is stamped
  `FREE` before obstacles are rasterized on top. Distinguishes "sensed
  and clear" from "never observed" -- missing depth data can mean
  occlusion, out-of-range, or an invalid return, not necessarily clear
  terrain.

`CostmapInflator`'s `inflate_unknown_cells` param controls whether `-1`
cells are treated as inflatable obstacles or passed through untouched
during the downstream inflation step.

## 10. Costmap inflation

Distance-transform-based (`scipy.ndimage.distance_transform_edt`) +
exponential cost decay, governed by `robot_radius`, `costmap_inflation_radius`,
and `cost_scaling_factor`. Occupied cells always remain occupied after
inflation; empty/all-free grids are handled without error.

## 11. Performance profiling

`enable_performance_profiling` (default `False`) + `profiling_interval`
(default `30` frames). Per-stage timing (`time.perf_counter()`) is always
measured (negligible overhead -- see `performance_profiling.py`); only the
*logging* of a summary is gated by these two parameters, to avoid
flooding the console by default. Example output:

```text
TerrainGeometry:
input=307200
roi=120000
non_ground=28000
voxel=8500
filtered=7200
clusters=14
total=58.4 ms
fps=17.1
stages: decode=1.20ms tf=0.10ms roi=0.80ms ground=4.50ms voxel=2.10ms outlier=1.80ms cluster=3.20ms features=0.90ms tracking=0.15ms occupancy=1.40ms costmap=2.30ms
```

The base per-frame summary line (`Frame processed | in=... roi=...
obstacle=... voxel=... inlier=... obstacles=... time=...`) is still logged
every frame at `INFO`, unchanged from before -- the detailed breakdown
above is additive.

## 12. Real-time QoS behavior

The input subscription uses an explicit `BEST_EFFORT` / `VOLATILE` /
`KEEP_LAST` profile (same reliability/durability as
`qos_profile_sensor_data`, so it stays compatible with the RealSense ROS 2
driver's own publisher QoS) with a small, configurable queue depth
(`input_qos_depth`, default `1`). If processing falls behind the camera's
publish rate, the DDS layer drops old buffered clouds in favor of the
newest one -- the node processes the latest available frame, not a
backlog of stale ones. The node runs under the default single-threaded
executor (`rclpy.spin`); no custom multi-threaded executor or callback
group was introduced, since the pipeline is single-callback and
CPU-bound end to end.

## 13. Parameter reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `input_pointcloud_topic` | string | `/camera/depth/color/points` | Input `PointCloud2` topic. |
| `target_frame` | string | `base_link` | Frame all processing happens in. |
| `tf_timeout_sec` | double | `0.2` | TF lookup timeout (s). Must be > 0. |
| `input_qos_depth` | int | `1` | Subscriber queue depth. Must be >= 1. |
| `enable_roi_filter` | bool | `True` | Enable the 3D ROI crop. |
| `roi_min_x` / `roi_max_x` | double | `0.2` / `8.0` | ROI X bounds (m). min < max. |
| `roi_min_y` / `roi_max_y` | double | `-4.0` / `4.0` | ROI Y bounds (m). min < max. |
| `roi_min_z` / `roi_max_z` | double | `-0.5` / `1.5` | ROI Z bounds (m). min < max. |
| `ground_sensor_height` | double | `0.2` | Sensor height above ground (m). |
| `ground_num_zones` | int | `4` | Concentric zones for ground segmentation. >= 1. |
| `ground_min_range` / `ground_max_range` | double | `0.2` / `10.0` | Radial range considered for ground estimation (m). |
| `ground_height_threshold` | double | `0.15` | Fallback-only height threshold (m). |
| `ground_removal_backend` | string | `auto` | `patchwork` \| `fallback` \| `auto`. |
| `leaf_size_x/y/z` | double | `0.05` | Voxel leaf size per axis (m). > 0. |
| `enable_radius_outlier_removal` | bool | `True` | Enable ROR. |
| `search_radius` | double | `0.2` | ROR neighbor search radius (m). > 0. |
| `min_neighbors` | int | `5` | ROR minimum neighbor count. > 0. |
| `outlier_removal_max_workers` | int | `-1` | ROR KD-tree worker threads. `-1` = all cores, else >= 1. |
| `enable_clustering` | bool | `True` | Enable DBSCAN clustering. |
| `cluster_eps` | double | `0.3` | DBSCAN neighborhood radius (m). > 0. |
| `cluster_min_points` | int | `5` | DBSCAN min samples. >= 1. |
| `min_cluster_size` / `max_cluster_size` | int | `10` / `50000` | Cluster size filter bounds. |
| `enable_obb` | bool | `False` | Compute oriented bounding boxes (else AABB only). |
| `enable_marker_visualization` | bool | `True` | Publish RViz obstacle markers. |
| `enable_obstacle_tracking` | bool | `False` | Enable temporal tracking. |
| `tracking_max_association_distance` | double | `0.5` | Max association distance (m). > 0. |
| `tracking_max_missed_frames` | int | `3` | Frames before a track is dropped. >= 0. |
| `tracking_position_smoothing` | double | `0.5` | Smoothing weight in `(0, 1]`. |
| `grid_resolution` | double | `0.05` | Occupancy grid cell size (m). |
| `grid_width_cells` / `grid_height_cells` | int | `200` / `200` | Grid dimensions in cells. > 0. |
| `grid_origin_x/y/z` | double | `-5.0` / `-5.0` / `0.0` | Grid origin (m, in `target_frame`). |
| `use_unknown_space` | bool | `False` | Three-state (`UNKNOWN`/`FREE`/`OCCUPIED`) vs. binary occupancy semantics. |
| `robot_radius` | double | `0.3` | Robot footprint radius for inflation (m). |
| `costmap_inflation_radius` | double | `0.6` | Inflation radius (m). >= 0. |
| `cost_scaling_factor` | double | `10.0` | Exponential decay rate for inflated cost. |
| `inflate_unknown_cells` | bool | `False` | Whether inflation treats UNKNOWN cells as inflatable. |
| `publish_debug_topics` | bool | `False` | Publish `/terrain/debug/*` intermediate clouds. |
| `enable_performance_profiling` | bool | `False` | Log detailed per-stage timing summaries. |
| `profiling_interval` | int | `30` | Frames between profiling summary logs. >= 1. |

All parameters are exposed as launch arguments in `launch/terrain.launch.py`
under the same names.

## 14. Expected failure cases (all handled without crashing)

| Case | Behavior |
|---|---|
| Empty/zero-point `PointCloud2` | Warning logged, frame skipped. |
| All points non-finite (NaN/Inf) | Warning logged, frame skipped. |
| TF not yet available | Warning logged, frame skipped, retried next message. |
| Zero points remain after ROI crop | Warning logged, empty outputs published. |
| < 3 points for ground segmentation | `GroundNotFoundError` caught, empty outputs published. |
| Zero obstacle (non-ground) points | Empty outputs published; ground debug cloud still published if enabled. |
| Zero points after voxel/ROR/clustering | Empty outputs published at whichever stage produced zero. |
| All DBSCAN points classified as noise | Zero obstacles reported, not a crash. |
| `ground_removal_backend="patchwork"` but Patchwork++ unavailable/broken | `GroundRemovalConfigError` at node startup -- fails loudly, does not run degraded. |
| Invalid parameter (e.g. `roi_min_x >= roi_max_x`, negative radius) | `ValueError` at node startup with a specific message -- node refuses to start rather than run with silently-clamped values. |

## 15. Benchmarking

See `terrain_geometry/benchmark.py` -- a standalone script (not part of
the production node/callback path) that replays either synthetic point
clouds or a recorded `ros2 bag` of `PointCloud2` messages through the
full computational pipeline (ROI -> ground removal -> voxel -> ROR ->
DBSCAN -> features -> tracking) and reports:

- average / min / max / standard deviation of total processing time
- p95 and maximum latency (more informative than average FPS alone)
- point count at every stage, per run

```bash
# Synthetic benchmark (no ROS bag needed):
python3 terrain_geometry/benchmark.py --synthetic --frames 100

# Replay a recorded bag's PointCloud2 topic:
python3 terrain_geometry/benchmark.py --bag /path/to/bag --topic /camera/depth/color/points
```

The benchmark is intentionally kept out of the real-time node/callback --
running it does not affect production latency.

## 16. Known limitations

- **Patchwork++ availability**: the fallback ground-removal backend is a
  simpler concentric slope/height-threshold method; it is not a drop-in
  quality replacement for Patchwork++ on complex/sloped/vegetated terrain.
- **Hardware-specific performance**: profiling numbers depend heavily on
  the rover's onboard compute; no numbers in this document should be
  taken as guaranteed for a different CPU.
- **Depth-camera limitations**: the D435's own range/FOV/noise
  characteristics (especially at range, in direct sunlight, or on
  low-texture/specular surfaces) bound what any downstream stage can
  recover from.
- **Tracking is intentionally simple**: nearest-neighbor + exponential
  smoothing, not a Kalman filter or learned association model. It will
  not gracefully handle fast-crossing obstacles that pass within
  `tracking_max_association_distance` of each other in a single frame.
- **Untested on real hardware in this change**: the changes in this
  revision were validated via unit tests and a synthetic-data
  integration test of the pure-computational pipeline (see the final
  report), not against a live D435 + real ROS 2 graph -- see the
  Testing section of the final report for exactly what was and wasn't
  exercised.
