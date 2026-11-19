# terrain_geometry

Validation status: the native Patchwork++ backend currently fails the existing
synthetic ground/obstacle geometry checks. The reference-frame decision and
measured results are in [the integration review](../INTEGRATION_REVIEW.md).

ROS 2 terrain/obstacle processing package. One `terrain_node` processes each
PointCloud2 frame in memory: timestamped TF, ROI crop, ground removal, voxel
filter, radius outlier filter, DBSCAN, obstacle geometry, optional tracking,
optional occupancy rasterization and inflation. Internal filters remain library
functions. Optional obstacle memory is a separate node in this package.

Whole-Perception orchestration belongs to `perception_bringup`, not terrain.

## Independent startup

```bash
# Inside the sourced Jazzy Docker container; inputs and calibrated TF external:
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=false
ros2 launch terrain_geometry terrain.launch.py --show-args
# Optional grid and visualization:
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=false enable_costmap:=true
ros2 launch terrain_geometry rviz.launch.py use_sim_time:=false
```

`terrain.launch.py` starts only the terrain node. `obstacle_memory.launch.py`
starts only memory; it requires current box observations and odometry TF.
`rviz.launch.py` starts only RViz. The deprecated
`perception_system.launch.py` forwards to `perception_bringup perception.launch.py`
and requires bringup installed. Terrain-only processing does not depend on the
marker package. The compatibility launcher deliberately has no reverse package
dependency that would create a terrain/bringup cycle.

The standalone terrain launch defaults to simulation time and debug clouds on;
combined startup forwards its own clock and defaults debug clouds off. Neither
starts a camera or `/clock`. Select time explicitly in validation commands.

## Inputs and outputs

| Topic | Type / contract |
| --- | --- |
| `/camera/depth/color/points` (configurable) | Best Effort PointCloud2; valid acquisition timestamp and sensor frame |
| `/perception/local_bboxes` | Reliable volatile `Detection3DArray`; current observation in `target_frame` |
| `/perception/obstacles_only` | Same current observation when `publish_to_obstacles_only=true`; optional memory replaces this output |
| `/terrain/obstacle_features` | `terrain_geometry_msgs/msg/ObstacleFeatureArray` |
| `/terrain/obstacle_markers` | Current box/label `MarkerArray`; valid empty observations delete old markers |
| `/terrain/costmap` | Optional inflated `OccupancyGrid`, in `target_frame` |
| `/terrain/diagnostics` | Input/processing freshness and health, published every 0.5 ROS seconds |
| `/terrain/debug/ground_cloud`, `/terrain/debug/voxel_cloud`, `/terrain/debug/clustered_cloud` | Optional sensor-QoS diagnostic clouds |

Cloud decoding honors point layout, byte order and row padding. Malformed fields,
lengths or strides are rejected rather than converted into misleading geometry.
Transforming a full cloud preserves source headers, padding and non-XYZ fields.
Camera-to-target TF resolves at acquisition time by default. Static transform
caching is opt-in for a verified rigid source/target pair, never assumed for
camera-to-odom/map. Clock reversal clears the terrain transform/tracking state.

The default target frame is `base_link`: X forward, Y left, Z up, in metres.
AABB `bbox.size` is X=depth, Y=width, Z=height. The box center is the midpoint of
its min/max bounds, not necessarily the point-sample centroid. OBB output carries
its own orientation. Changing `target_frame` also changes coordinate-dependent
ROI and ground assumptions; configure those deliberately.

## Valid observations versus unavailable processing

`use_unknown_space=true` is the node/launch default. Only cells containing
supplied finite observed-ground returns become free; a rectangular ROI is not
proof that every cell was observed. Obstacle footprints are marked occupied.
Unobserved/occluded cells remain unknown. `false` explicitly selects the legacy
fully-free grid initialization and should not be used to hide sensor loss.

Valid ground-only observations publish empty box arrays and an observed-ground
grid. Empty/invalid/stale clouds, absent TF or segmentation failure report
unavailable processing; an enabled costmap becomes unknown. They do not publish
a fabricated valid empty box observation. Existing consumers may retain earlier
geometry, so check diagnostics AND header age. Silence is not free terrain.

Disabling clustering makes surviving points unclustered and produces no boxes;
that tuning mode does not establish that obstacle detection is working.
No diagnostics topic directly stops the rover; navigation/control must enforce
its observation-loss policy during later integration validation.

## Main configuration defaults

The full launch surface is available through `--show-args`; numeric filters are
read at startup and require a restart. Only documented output toggles are live.

| Setting | Default |
| --- | --- |
| Input topic / target frame | `/camera/depth/color/points` / `base_link` |
| TF timeout / cloud timeout / input queue depth | 0.2 s / 1 s / 1 |
| ROI X / Y / Z | 0.2..8 / -4..4 / -0.5..1.5 m |
| Ground backend / sensor height / height threshold | `auto` / 0.2 m / 0.15 m |
| Voxel leaf size | 0.05 m per axis |
| Outlier radius / minimum neighbors | 0.2 m / 5 |
| DBSCAN eps / min points / cluster size bounds | 0.3 m / 5 / 10..50000 |
| Optional tracking / OBB | Off / off |
| Grid resolution / dimensions / origin | 0.05 m / 200x200 / (0, -5, 0) m |
| Unknown space / costmap generation | On / off |
| Robot radius / inflation radius / scaling | 0.3 m / 0.6 m / 10 |

Verify current values with `ros2 param dump /terrain_node`; measure rover geometry
and camera mounting before tuning safety margins. The grid's default forward
coverage is 10 m. Patchwork++ is used when installed; `auto` otherwise logs a
NumPy fallback. Explicit `ground_removal_backend:=patchwork` fails startup if that
backend cannot initialize. The current project Docker image includes Patchwork++.
Its ground-height assumptions still need physical-data validation.

## Optional memory and surface risk

Memory associates observations in `memory_frame` (default `odom`) and expires
unseen boxes after `ttl_sec` (default 5 s). Its array timestamp is publication
time; each retained detection keeps its acquisition timestamp. Retained geometry
is not a fresh camera observation. In combined startup memory exclusively
publishes obstacle updates; in manual startup set terrain
`publish_to_obstacles_only:=false`. Do not run two producers for that interface.
Ground clearing is off and needs valid debug-ground observations when selected.

Risk checks are off. They require a calibrated gravity-aligned reference plus
positive measured slope, step and roughness limits. They are conditional geometry
checks, not validated crater detection. Missing returns remain unknown.

RViz starts with raw cloud and obstacle markers visible. RobotModel, RGB,
costmap and debug-cloud displays are disabled until their producers are available.
Enable the costmap display after `enable_costmap:=true`, and debug displays after
`publish_debug_topics:=true`. Sensor displays use Best Effort.

The offline `terrain_geometry/benchmark.py` still contains a synthetic generator
and is not called by production nodes. Relocation is reserved for the owner of
the master testing folder; its mirrored pipeline is not full node validation.
See the [testing handoff](../TESTING_GUIDE.md#testing-agent-handoff).

See the [runbook](../RUNBOOK.md), [message definitions](../terrain_geometry_msgs/README.md)
and [validation limits](../INTEGRATION_REVIEW.md).
