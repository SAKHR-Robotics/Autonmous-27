# perception_test_tools

Test-only `ament_python` package supplying synthetic RGB, aligned 32FC1 depth,
PointCloud2, `CameraInfo`, sensor TF, and machine-readable ground truth to the
real production Perception stack. Production packages do not depend on it.

```bash
# Run real Perception + synthetic sensor suite
ros2 launch perception_test_tools test_stack.launch.py pipeline:=all scenario:=clear_aruco_nav
ros2 launch perception_test_tools test_stack.launch.py pipeline:=terrain scenario:=ground_only ground_z_offset:=-0.255
ros2 launch perception_test_tools test_stack.launch.py pipeline:=markers scenario:=clear_aruco_nav launch_landmark_adapter:=true
ros2 launch perception_test_tools sensor_inputs.launch.py scenario:=missing_depth
```

Supported launch arguments (`test_stack.launch.py`): `pipeline` (`all|terrain|markers`),
`scenario` (`rocks_and_marker`, `clear_aruco_nav`, `ground_only`, `missing_depth`,
`invalid_cloud`, `dropout`, `turn_occlusion`, `terrain_hazard`), `ground_z_offset` (`0.0`,
use `-0.255` to match Rover URDF ground plane below `base_link`), `seed` (`0`),
`rate` (`10.0`), `launch_memory` (`false`), `launch_landmark_adapter` (`false`),
`launch_marker_mapping` (`false`), `launch_rviz` (`true`).

Ground truth is published both visually on `/testing/perception/ground_truth`
(`MarkerArray`) and programmatically on `/testing/perception/ground_truth_json`
(`std_msgs/String`, containing obstacle boxes in `base_link` and 6-DoF marker pose
in `test_camera_optical_frame`).

`perception_test_tools.terrain_scene.synthetic_frames` remains the offline
floor-and-moving-box XYZ generator consumed by `perception_benchmark_tools`.
See [the scenario guide](../Readme.md) and [Perception runbook](../../../../Perception/RUNBOOK.md).
