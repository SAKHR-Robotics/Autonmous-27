# Perception integration review — Jazzy

Static source review for the core-launch separation. The processing code is
implemented, but end-to-end autonomy is not yet established by the current
interfaces. No findings below were repaired by changing algorithms, schemas,
SLAM, planning or control in this change. Runtime observations are still needed
on Ubuntu 24.04 / ROS 2 Jazzy.

## 1. Build and interface blockers

| Finding | Evidence | Impact and recommended follow-up |
| --- | --- | --- |
| Duplicate message package, incompatible schema | `terrain_geometry_improved/terrain_geometry_msgs/msg/ObstacleFeature.msg` versus `../PathPlaning/src/terrain_geometry_msgs/msg/ObstacleFeature.msg` | Both packages are named `terrain_geometry_msgs`. Perception uses signed `id`, `num_points`, AABB bounds, width/depth and distance; planning's version uses unsigned `id`, radius, classification and confidence. Root colcon discovery cannot select both. Agree on one shared contract and migrate consumers/fixtures together; do not silently delete one version. |
| SLAM obstacle input is disconnected | `../SLAM/rover_slam/config/costmap_params.yaml` subscribes to `/perception/obstacles_only` (`PointCloud2`); `terrain_node.py` publishes costmap, features, markers and optional debug clouds | Real terrain does not supply that production topic. Decide on a timestamped obstacle-cloud interface or a deliberate grid integration. Debug clustered clouds are not a substitute production contract. |
| Planner bridge drops geometry and is not started by planner bringup | `../PathPlaning/src/erc_path_planner/src/costmap_bridge_node.cpp` emits one centroid per obstacle on `/bridge/pointcloud`; its `launch/path_planning.launch.py` starts Nav2 without this bridge | Local costmap expects `/bridge/pointcloud`, but its producer needs an explicit launch. Even then, a centroid represents less than the obstacle footprint. Use obstacle geometry/dense observations and define marking/clearing semantics; verify with large rocks and occlusions. |
| Marker outputs do not match the older SLAM landmark interface | Perception publishes custom `/marker_poses` and `/marker_map`; SLAM's `mock_aruco_publisher.py` publishes `PoseWithCovarianceStamped` on `/perception/aruco_pose` | Synthetic SLAM landmark output is not an adapter from real marker observations. Define marker ID, frame, covariance and landmark semantics with the SLAM consumer before adding a bridge. The mock's existence does not prove RTAB-Map is wired to consume it. |
| Control integration remains unspecified | `../Control/` currently contains documentation only | `/marker_detection/targets` is a usable producer interface, but no implemented control subscriber exists here. Specify consumers and handling of `pose_valid`, `usable_for_action`, staleness and missing TF before actuation. |

Until the shared-message conflict is fixed, build perception with
`colcon build --base-paths Perception --symlink-install`. This is a scoped build,
not a repository-wide integration fix. Existing output stubs in
`../testing/PathPlanner/mock/` remain there. SLAM still has test publishers inside
its package; relocation to top-level `testing/` is a later SLAM/testing change.

## 2. Terrain correctness and observability

**Invalid input can appear clear (high priority).** In
`terrain_geometry_improved/terrain_geometry/terrain_geometry/terrain_node.py`,
`_cloud_callback` calls `_publish_empty` when no finite points remain, ROI is
empty, or `GroundNotFoundError` occurs. `_publish_empty` generates a grid without
obstacles; with `use_unknown_space=false` it becomes free space. For the
no-finite-points branch, it also retains the original sensor header even though
the grid geometry uses the target frame. Conversely, a truly empty message,
unavailable TF or voxel-filter failure returns without an updated output,
potentially leaving downstream consumers with stale data. Distinguish valid
clear observations from unavailable perception; define a diagnostic/freshness
contract and an explicit unknown/error policy with navigation.

**Unknown-space mode is only an approximation.** `occupancy_grid.py` treats
the entire configured ROI rectangle as observed when `use_unknown_space=true`.
A bounding box does not capture camera FOV, occlusion or missing depth pixels.
Add observation-based visibility handling before relying on unknown/free-space
distinctions for navigation.

**Default grid clips detections.** The grid spans 200 × 0.05 m = 10 m per side,
with origin (-5, -5): X coverage is [-5, 5), while ROI accepts X up to 8 m.
Features can therefore exist beyond the published map. Align grid bounds with
the actual sensor coverage and test boundary footprints.

**TF cache assumes a rigid mount.** `tf_transform.py` reuses its first successful
transform for a given source/target frame pair without time-based refresh.
`base_link` is appropriate for a rigid camera; a moving camera, `odom` or `map`
target needs timestamp-aware transforms. Existing README claims about a
configurable staleness window were corrected.

**Terrain classification is incomplete.** Ground removal and above-ground
clustering are implemented; crater, negative-obstacle and traversability/slope
classification are not established by that alone. Ground segmentation is fed
base-frame points while `ground_sensor_height` defaults to 0.2 m; validate the
backend's ground-reference assumption against the actual camera/base geometry.
Benchmark the native and fallback backends separately on recorded rover data.
Per-frame INFO logging and all-core outlier filtering (`max_workers=-1`) also
need CPU/latency measurements alongside SLAM on the target computer.

## 3. Camera, frames and time

RGB-D association requires RGB-aligned depth and RGB CameraInfo at the correct
resolution. YAML/launch defaults use simulation-style `/camera/...` names;
`marker_detection_node.py`'s own defaults use different color/aligned-depth
names. Remap explicitly for the active driver. The RealSense configuration in
`../SLAM/rover_slam/config/realsense_filters.yaml` enables alignment, but verify
the live point-cloud stream and topic names rather than assuming they exist.

`../SLAM/rover_slam/launch/static_transforms.launch.py` hard-codes a
`base_link -> camera_link` transform different from the rover URDF's configurable
mount. Starting both creates competing ownership. Use one calibrated source
per transform; inspect the actual Gazebo optical frame in its image headers.
Core perception intentionally owns neither robot static TF nor camera drivers.

The changed perception launches now propagate `use_sim_time` to all selected
nodes. SLAM/planner configuration still contains `false` clock settings and
planner configuration includes a Humble-specific behavior-tree path. Those
are remaining Jazzy/Gazebo integration issues outside this patch. For clocked
bags, the bag must publish `/clock` and matching TF; changing perception's clock
alone cannot synchronize the whole stack.

## 4. Marker mapping and downstream action

**Potential TF arrival race.** `marker_map_node.py` consumes `/marker_poses` and
immediately looks up `map -> aruco_marker_<id>` at the observation timestamp.
`marker_tf_broadcaster.py` consumes that same topic and broadcasts marker TF
separately. The map callback can run before the TF arrives; its default
single-threaded spin and synchronous lookup timeout do not guarantee the
listener processes it during the callback. Verify under load and startup.
Possible follow-ups are timestamped deferred processing or composition from
the observation and camera-to-map TF, with bounded expiry and no fabricated
poses. No runtime failure is claimed from static inspection alone.

`marker_map_node.py` builds RViz markers with ADD actions; reset clears the
manager and publishes an empty array without DELETE actions. Previously drawn
markers can remain in RViz after reset. A follow-up should delete prior IDs or
use explicit lifetime semantics and verify reset visually.

The action interface already carries validity, quality and confidence gates;
consumers must enforce them, including unavailable TF and stale observations.
Global marker mapping depends on an existing localization frame; it does not
close the rover-localization loop. These are real processing stages and must
not be mislabeled as placeholder/stub logic.

## 5. Verification and recommended order

1. Resolve the shared message package and choose production obstacle/landmark
   contracts with the SLAM/planning owners.
2. Define unavailable-perception behavior, observed-space semantics, TF
   ownership, clock settings and grid coverage before autonomous driving.
3. Run the launch tests on Jazzy, then sensor/bag tests for missing TF, no depth,
   invalid clouds, segmentation failure, occlusion, marker-map reset and startup.
4. Create the separate tools under top-level `testing/`: synthetic sensor inputs
   to test perception, synthetic perception outputs to test consumers, and
   reproducible latency/accuracy benchmarks. Never publish both fake and real
   outputs onto the same production topics unintentionally.

The Windows editing environment lacks ROS 2 launch/rclpy, colcon, NumPy,
OpenCV, SciPy and scikit-learn. Syntax/XML/package-path checks can run here;
live Jazzy launch, DDS, TF, camera and algorithm verification cannot. The new
launch tests intercept process creation, so even a passing Jazzy test run is
not evidence of working sensor transport or obstacle detection accuracy.
