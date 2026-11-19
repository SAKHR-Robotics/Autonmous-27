# marker_detection

One ROS 2 package containing marker processing and its downstream nodes. The
RGB detector, depth association, PnP pose estimation and temporal tracker share
one processing node. TF, action targets, global mapping and native landmarks
are separate consumers of `/marker_poses` in the same package.

## Independent startup

Inside a sourced `autonomous27:jazzy` container:

```bash
# Detector and tracker only:
ros2 launch marker_detection marker_detection.launch.py use_sim_time:=false
# Marker branch without external global localization:
ros2 launch marker_detection marker_branch.launch.py use_sim_time:=false launch_marker_mapping:=false
# Marker RViz only (GUI forwarding required):
ros2 launch marker_detection rviz.launch.py use_sim_time:=false
```

The branch's legacy default enables mapping; the combined
`perception_bringup perception.launch.py` explicitly leaves it off. This behavior
is retained. Pass the mapping switch explicitly when validating the marker branch.
Do not start the same consumer twice through both a branch and an individual launch.

| Launch | Starts | External requirements |
| --- | --- | --- |
| `marker_detection.launch.py` | `marker_detection_node`: detection, pose estimation and tracking | RGB; matched calibration for poses; aligned depth for RGB-D fusion |
| `marker_tf.launch.py` | `marker_tf_broadcaster` | `/marker_poses`; no global localization required |
| `marker_action_interface.launch.py` | `marker_action_interface_node` | `/marker_poses` and camera-to-base TF |
| `marker_mapping.launch.py` | `marker_map_node` | `/marker_poses` and genuine map-to-camera TF |
| `landmark_adapter.launch.py` | `marker_landmark_adapter` | `/marker_poses`; native RTAB-Map message dependency |
| `marker_branch.launch.py` | Detector, TF, action targets, mapping by default; optional landmark adapter | Requirements of each selected stage |
| `rviz.launch.py` | RViz only | GUI access and selected topic/TF producers |

Individual processing launches accept `params_file`; the branch has separate
`detection_params_file`, `tf_params_file`, `mapping_params_file`,
`action_params_file` and `landmark_params_file`, with independent stage switches.
Processing-launch time defaults to `true` except the landmark adapter (`false`).
The branch forwards its time value to every selected stage. Always pass time
explicitly; no driver, simulation clock or fake map transform is started here.

## Camera contract

Code, base YAML and launch defaults consistently use `/camera/image_raw`,
`/camera/depth/image_raw` and `/camera/camera_info`. These are workspace aliases;
a name alone does not establish depth alignment. Supply actual physical driver
names through launch arguments, for example:

```bash
ros2 launch marker_detection marker_detection.launch.py use_sim_time:=false \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

RGB and depth subscriptions are independent, Best Effort. Missing depth does
not stop 2D detection or calibrated monocular PnP. Depth fusion requires matching
RGB/depth dimensions and timestamps within `sync_slop` (default 0.05 s).
Calibration must have finite valid intrinsics and match RGB dimensions.
Missing/stale/incompatible calibration prevents calibrated 3D output while 2D
processing continues. Invalid CameraInfo clears prior calibration.

RGB older than `input_timeout_sec` (default 1 s), with zero timestamp, or in the
future is rejected and diagnostics report an error. CameraInfo is considered
stale after `camera_info_max_age_s` (default 5 s) without a new message; this is
receipt-age monitoring, not a guarantee of physical calibration. A ROS clock
reversal resets the depth buffer and marker tracks on the next RGB callback.

Default dictionary is `DICT_4X4_50`; default marker side length is 0.21 m.
Measure actual printed markers and camera mounting. Per-ID sizes are configurable
in `config/marker_detection.yaml`. The printable-marker utility is a calibration
utility, not a mock sensor:

```bash
ros2 run marker_detection generate_aruco_marker.py --id 0 --size 600
```

## Outputs and validity

| Topic | Type / meaning |
| --- | --- |
| `/marker_detections` | `MarkerDetectionArray`: 2D image observations |
| `/marker_3d_detections` | `Marker3DDetectionArray`: depth-associated observations; check `valid_3d` |
| `/marker_poses_raw` | `MarkerPoseArray`: per-frame pose estimates |
| `/marker_poses` | `MarkerTrackedPoseArray`: tracked observations in the RGB header frame |
| `/marker_detection/targets` | `MarkerActionTargetArray`: resolved rover-base targets |
| `/marker_map`, `/marker_map/visualization` | Optional global map and RViz markers |
| `/perception/landmark_detections` | Optional native RTAB-Map landmark array |
| `/perception/aruco_pose` | Compatibility `PoseStamped` for usable targets; no marker ID or covariance |
| `/marker_detection/diagnostics` | Processing/input health, latency, synchronization and calibration availability |

Actionability requires a confirmed valid track, valid pose, adequate confidence,
finite symmetric positive-semidefinite nonzero covariance within the trace limit,
and no disqualifying quality flags. `max_action_age_seconds` defaults to 0.5 s;
publication age includes upstream detection age plus transport delay. Zero/future
stamps, invalid ages and stale targets are never actionable. The header retains
the source observation timestamp. Consumers must still enforce freshness if
updates stop; this node is not a motor interlock.

TF failure never invents a rover-base pose: pose validity is false, distances
are NaN and `TF_UNAVAILABLE` is set. Missing covariance is not zero uncertainty.
Global mapping composes map-to-camera TF with each observed pose; it does not
require the marker TF broadcaster to arrive first.

The native landmark adapter publishes positive IDs (`observed ID + id_offset`,
default offset 1), observation-frame headers and 6x6 covariance. It rejects stale,
future, malformed and low-quality observations. Missing position covariance
uses its documented configured fallback; angular variance is configured, not a
measured orientation uncertainty. Current SLAM uses this interface, not
`/perception/aruco_pose`. No active production subscriber for the compatibility
pose was found in the inspected Perception/SLAM/planner sources; it is retained
for possible external users. Historical diagrams mention it and need separate
whole-stack documentation reconciliation.

Base tuning is in `config/marker_detection.yaml`; competition settings are in
`config/marker_detection_competition.yaml`. Do not treat defaults or synthetic
checks as evidence of outdoor accuracy or competition compliance.

The frontal synthetic scene currently retains `POSE_AMBIGUOUS`: positive
camera-to-action/landmark acceptance remains pending a suitable scene. The
quality gates are preserved; see [measured results](../INTEGRATION_REVIEW.md).

See the [runbook](../RUNBOOK.md), [Docker tests](../TESTING_GUIDE.md),
[message definitions](../marker_detection_msgs/README.md) and
[remaining limits](../INTEGRATION_REVIEW.md).
