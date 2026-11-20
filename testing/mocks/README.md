# Dependency mocks and synthetic fixtures

Use the package READMEs below for inputs, outputs, scenarios and limitations.
These packages supply missing upstream/downstream surroundings so the **real production
modules** run unmodified and emit their real outputs for assessment.

| Consumer / purpose | Active guide | Implemented contents and limits |
| --- | --- | --- |
| Planning dependencies | [pathplanner_mock_tools](pathplanning/pathplanner_mock_tools/README.md) | Shared `mock_rover_sim` (with `production_rover` / `nav2_nominal` presets, deterministic seeding, and `freeze_odom` / `drop_tf` / `drop_joints` / live slip overrides), `mock_perception` (`Detection3DArray`, `spawn_obstacle_full`, `clear_obstacles`, `freeze_perception`), map assets, and `mocked_planner.launch.py` |
| Perception inputs | [perception_test_tools](perception/perception_test_tools/README.md), [scenario guide](perception/Readme.md) | Synthetic RGB/depth/cloud/calibration scenes (`rocks_and_marker`, `ground_only`, `missing_depth`, `invalid_cloud`, `dropout`, `turn_occlusion`, `terrain_hazard`, and non-ambiguous tilted `clear_aruco_nav`), configurable `ground_z_offset` (e.g. `-0.255 m`), `/testing/perception/ground_truth_json`, and offline `terrain_scene.py` |
| SLAM dependencies | [slam_mock_tools](slam/slam_mock_tools/README.md) | Shared-rover `rover_sensor_adapter` (`joint_states`, `ticks_array`, `individual_ticks`, slip scaling, rotated IMU mounts, gravity sign, noise/bias, `drop_imu`/`drop_wheels`), `image_gate`, and stamped/PSD-covariance smoke fixtures (`mock_aruco_publisher`, `costmap_test_stub`) |

SLAM's local mocked launch and evidence are in
[slam_test_tools](../benchmarking/slam/slam_test_tools/README.md) and
[SLAM/VALIDATION.md](../../SLAM/VALIDATION.md). Perception's system limits
are in [Perception/INTEGRATION_REVIEW.md](../../Perception/INTEGRATION_REVIEW.md).
Global RTAB-Map mapping requires Gazebo (`final_world_R&A.world`) or `rosbag2`
playback for coherent moving RGB-D imagery.

Do not duplicate dependency producers across packages, and do not run relocated
compatibility aliases (`global_path_benchmarking` or `slam_test_tools` old names)
alongside the primary mock executables.
