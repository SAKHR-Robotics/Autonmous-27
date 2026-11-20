# PathPlanner dependency mocks

Start with the consolidated [module info](../../../../PathPlanning/info.md) for
the complete specification, interface contract, current status and demonstration.

Everything here is test-only. `mocked_planner.launch.py` starts **real `erc_path_planner`/Nav2**,
with synthetic map, localization/kinematic plant, and Perception box inputs. Production
does not depend on this package.

| Asset | Purpose |
| --- | --- |
| `launch/mocked_planner.launch.py` | Wall-time mock dependencies + real production master (fails explicitly if target launch is missing) |
| `launch/test_planner_standalone.launch.py` | Real production master with test map, `launch_bridge`, and optional `map → odom` anchor for external rover odometry |
| `custom_maps/` & `maps/` | Synthetic dummy, dead-end, corridor, rock-field, and Mars Yard scenario maps |
| `rviz/mocked_planner.rviz` | Real planning outputs plus mock rover/obstacle markers |
| `mock_rover_sim` | Integrates `/cmd_vel`, publishes `/odometry/filtered`, always-on `/testing/mock_rover/truth_odom`, and TF; supports `geometry_profile` (`custom`, `production_rover`, `nav2_nominal`), `seed`, and live fault parameters (`freeze_odom`, `drop_tf`, `drop_joints`, `slip_override_linear`, `slip_override_angular`) |
| `mock_perception` | Publishes `Detection3DArray` (`/perception/obstacles_only`) + legacy feature/scan/cloud outputs; supports `clear_default_obstacles`, `freeze_perception`, `/mock_perception/clear_obstacles`, and `/mock_perception/spawn_obstacle_full` |

## Build and launch inside `autonomous27:jazzy`

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/PathPlanning/erc_path_planner \
    /workspace/Perception/terrain_geometry_msgs \
    /workspace/testing/mocks/pathplanning/pathplanner_mock_tools \
    /workspace/testing/benchmarking/pathplanning \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py
```

Key launch arguments: `use_rviz`, `map`, `geometry_profile` (`nav2_nominal`), `seed` (`0`),
`clear_default_obstacles` (`false`), `publish_detections` (`true`), `initial_x`, `initial_y`,
`initial_yaw`, `use_mock_rover`, `use_mock_perception`, `use_costmap_bridge`, `scenario_id`,
`scenarios_file`, `cmd_vel_topic`, `control_mode`, `target_package`, `target_launch`,
`use_external_module`.

Note: `mock_perception` is marking-only (`observation_sources: obstacles`); there is no simulated
camera clearing source in this fixture. `mock_rover_sim` with `nav2_nominal` uses `0.90 × 0.70 m`
footprint and zero noise/slip by default; select `geometry_profile:=production_rover` for `0.06 m`
wheel radius / `0.49 m` track width.

## Fault injection & automated checks

While `mocked_planner.launch.py` is running, trigger stale localization or perception faults:

```bash
ros2 param set /mock_rover_sim freeze_odom true
ros2 param set /mock_perception freeze_perception true
```

Run the mock unit tests (inside sourced Jazzy container):

```bash
python3 -m pytest /workspace/testing/mocks/pathplanning/pathplanner_mock_tools/test -q -p no:cacheprovider
```

For full multi-scenario evaluation and probes, see
[global_path_benchmarking](../../../benchmarking/pathplanning/README.md).
