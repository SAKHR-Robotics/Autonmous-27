# perception_bringup

Launch real perception on ROS 2 Jazzy. Build instructions and sensor contracts
are in the [perception README](../README.md).

## Selection

```bash
ros2 launch perception_bringup perception_core.launch.py --show-args
ros2 launch perception_bringup perception_core.launch.py use_sim_time:=true
```

| Switch | Default | Node executable |
| --- | --- | --- |
| `enable_terrain` | `true` | `terrain_node` |
| `enable_marker_detection` | `true` | `marker_detection_node` (detection, pose and tracking) |
| `enable_marker_tf` | `true` | `marker_tf_broadcaster` |
| `enable_marker_mapping` | `false` | `marker_map_node` |
| `enable_marker_action` | `true` | `marker_action_interface_node` |

Switches are independent. Turning detection off does not turn its consumers
off; they may consume an externally launched detector. When no producer exists,
consumers wait for observations. Global mapping also needs marker TF and
localization. Action targets need tracked poses and camera-to-base TF, but do
not depend on marker mapping or marker TF broadcasting.

Terrain only:

```bash
ros2 launch perception_bringup perception_core.launch.py \
  enable_marker_detection:=false enable_marker_tf:=false enable_marker_action:=false
```

Marker pipeline including global mapping (localization must already run):

```bash
ros2 launch perception_bringup perception_core.launch.py \
  enable_terrain:=false enable_marker_mapping:=true
```

## Inputs and parameter files

| Argument | Default / purpose |
| --- | --- |
| `use_sim_time` | `false`; propagated as a boolean to every selected node |
| `input_pointcloud_topic` | `/camera/depth/color/points` |
| `target_frame` | `base_link`; terrain processing frame; use a rigid camera-to-target transform |
| `rgb_topic` | `/camera/image_raw` |
| `depth_topic` | `/camera/depth/image_raw`; must be aligned to RGB |
| `camera_info_topic` | `/camera/camera_info`; RGB calibration |
| `marker_detection_params_file` | Installed `marker_detection.yaml` |
| `marker_tf_params_file` | Installed `marker_tf.yaml` |
| `marker_mapping_params_file` | Installed `marker_mapping.yaml` |
| `marker_action_params_file` | Installed `marker_action_interface.yaml` |

Each parameter file is scoped to its node. Explicit camera topic arguments and
`use_sim_time` override YAML values. For example:

```bash
ros2 launch perception_bringup perception_core.launch.py \
  marker_detection_params_file:=/absolute/path/to/detector.yaml \
  marker_action_params_file:=/absolute/path/to/action.yaml
```

Use `ros2 launch terrain_geometry terrain.launch.py --show-args` for all terrain
algorithm settings; the aggregate launch exposes terrain's input topic and
processing frame rather than duplicating its tuning surface.

The legacy full marker branch still launches global mapping by default. Its
file arguments are `detection_params_file`, `tf_params_file`,
`mapping_params_file`, and `action_params_file`. Each individual marker launch
retains its existing `params_file` argument and now accepts `use_sim_time`.

Camera drivers, Gazebo, localization, robot static TF, RViz, synthetic data and
benchmarks are external. Future synthetic publishers belong in top-level
`testing/`; do not add them to this production launch.

## Tests

From the repository root, after a perception-only build and sourcing Jazzy and
the workspace:

```bash
colcon test --base-paths Perception --packages-select perception_bringup
colcon test-result --verbose
```

The tests execute real launch expansion with process creation intercepted.
They check default selection, all five independent switches, clock/topic/frame
forwarding, isolated parameter files, and legacy marker-branch mapping.
On a machine without ROS launch dependencies they skip explicitly. Algorithm,
TF transport and sensor integration still require live Jazzy verification.
