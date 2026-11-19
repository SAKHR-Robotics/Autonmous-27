# Perception

Current status: package/regression checks pass; terrain ground classification
still fails controlled geometry checks, including with a matching height
reference. Positive camera-to-action/landmark acceptance also needs a suitable
scene. See [results and limits](INTEGRATION_REVIEW.md).

`Perception/` is an organizing folder. Its five ROS 2 packages are:

| Package | Owns |
| --- | --- |
| `perception_bringup` | Combined startup, combined RViz view and launch-contract checks |
| `terrain_geometry` | Terrain processing, optional obstacle memory, independent launches and terrain RViz |
| `marker_detection` | Marker detection/tracking, TF, action targets, optional mapping/landmark adapter and marker RViz |
| `terrain_geometry_msgs` | Terrain feature messages |
| `marker_detection_msgs` | Marker observation, tracking, mapping and action-target messages |

A package can contain several nodes. Internal image/cloud filtering helpers stay
in the processing nodes; making each helper a package would add unnecessary
communication and deployment boundaries.

## Start here

Use the software installed in `autonomous27:jazzy`, not host Python or ROS.
The [runbook](RUNBOOK.md) has Docker setup, isolated build commands, independent
and combined startup, sensor requirements and the complete launch inventory.
The [validation guide](TESTING_GUIDE.md) records checks and the testing-agent handoff.

```bash
# Inside the sourced Jazzy container, with sensors and calibrated TF available:
ros2 launch perception_bringup perception.launch.py use_sim_time:=true launch_rviz:=true
```

This starts terrain processing, marker detection/tracking, marker TF and action
targets. The camera driver, clock and localization are external. Use
`use_sim_time:=false` for wall-clock sources. Docker itself does not select time.
Optional mapping, memory, native landmarks, costmap and risk checks are not
started by the default combined launch. Their existing switches are preserved.

Independent entry points:

```bash
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=false
ros2 launch marker_detection marker_branch.launch.py use_sim_time:=false launch_marker_mapping:=false
```

The old `ros2 launch terrain_geometry perception_system.launch.py` command
forwards to the new combined launch and requires `perception_bringup` installed.
It is a compatibility entry point, not terrain's dependency on marker processing.

## Data sent to other modules

| Topic | Type | Meaning |
| --- | --- | --- |
| `/perception/local_bboxes` | `vision_msgs/msg/Detection3DArray` | Current terrain observations in `target_frame` (default `base_link`) |
| `/perception/obstacles_only` | `vision_msgs/msg/Detection3DArray` | Terrain observations normally; retained odom-frame boxes when memory is selected |
| `/marker_detection/targets` | `marker_detection_msgs/msg/MarkerActionTargetArray` | Marker targets in the configured rover base frame; check freshness and usability |
| `/perception/landmark_detections` | `rtabmap_msgs/msg/LandmarkDetections` | Optional native SLAM observations with IDs, per-observation headers and covariance |
| `/perception/aruco_pose` | `geometry_msgs/msg/PoseStamped` | Compatibility pose output; lacks marker ID/covariance and is not the native SLAM interface |
| `/terrain/diagnostics`, `/marker_detection/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Processing health; consumers must also enforce message age |

An empty valid observation is different from unavailable perception. Do not infer
clear terrain from an empty topic, missing updates or a previously clear map.
The planner owns its box-to-cloud bridge. Perception does not start another one.
Current planner and SLAM source configurations match the obstacle and native
landmark topics; full navigation/SLAM behavior needs separate integration checks.

Runtime mock publishers, scenarios and artificial localization belong under the
master `testing/` folder. Per-package `test/` directories contain regression tests,
not runnable replacement dependencies. The offline terrain generator now lives
in `testing/mocks/perception/perception_test_tools`; its timing runner is the
`perception_benchmark_tools` package under `testing/benchmarking/perception/`.

See [terrain details](terrain_geometry/README.md), [marker details](marker_detection/README.md)
and [remaining validation limits](INTEGRATION_REVIEW.md). Older design documents
under `docs/` and `PerPro&Solu.md` describe historical proposals, not current startup
or evidence of physical accuracy.
