# Perception — ROS 2 Jazzy / Linux

**Start with the [current runbook](RUNBOOK.md) or the [Linux Testing & Validation Guide](TESTING_GUIDE.md).** It contains build steps,
independent modules, the master launch, testing inputs, Gazebo/real-camera
workflows, RViz, expected outputs and troubleshooting.

## Launch real perception

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch terrain_geometry perception_system.launch.py \
  use_sim_time:=true launch_rviz:=true enable_costmap:=true
```

Camera streams and calibrated camera TF must already exist. The Jazzy Gazebo
rover launch supplies its clock; real cameras require `use_sim_time:=false`.
Global marker mapping, obstacle memory, terrain-risk checks and the RTAB-Map
landmark adapter are optional and off in the master until explicitly selected.

## Test perception only

Build `perception_test_tools` from top-level `testing/Perception/`, then:

```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=all scenario:=rocks_and_marker launch_rviz:=true
```

Use `pipeline:=terrain` or `pipeline:=markers` for one module. This launches
synthetic inputs and **real processing**, without Gazebo, hardware or SLAM.
All runtime stubs and artificial localization fixtures remain under `testing/`.
The [runbook](RUNBOOK.md#3-perception-only-tests-what-to-launch) also shows exactly
how to launch the feeder and production master in separate terminals.

## Packages and evidence

| Package | Details |
| --- | --- |
| Terrain | [Processing and independent launch](terrain_geometry/README.md) |
| Markers | [Detection, tracking and independent consumers](marker_detection/README.md) |
| Terrain interfaces | [Message definitions](terrain_geometry_msgs/README.md) |
| Marker interfaces | [Message definitions](marker_detection_msgs/README.md) |
| Testing | [Scenarios, commands and limits](../testing/Perception/Readme.md) |

Read the [integration review](INTEGRATION_REVIEW.md) before autonomous use.
Unknown/unavailable perception must not be interpreted as clear terrain.
Calibrated hardware accuracy, safe stopping, outdoor performance and final
ERC 2027 compliance still require validation. Historical roadmaps in `docs/`
are not evidence that every proposed feature or benchmark is implemented.
