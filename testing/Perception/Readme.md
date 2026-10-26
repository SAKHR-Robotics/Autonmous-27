# Perception-only testing — ROS 2 Jazzy / Linux

Implemented package: **perception_test_tools**, in this directory. These are
explicit synthetic sensor fixtures and real-node integration checks. The old
benchmark proposal is not an implemented photorealistic or zero-dependency suite.
ROS 2, NumPy, OpenCV and the real perception dependencies are required.

## Build and run

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths Perception PathPlanning/erc_path_planner \
  testing/Perception/perception_test_tools --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths Perception PathPlanning/erc_path_planner \
  testing/Perception/perception_test_tools --symlink-install
source install/setup.bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=all scenario:=rocks_and_marker launch_rviz:=true
```

The launch starts the feeder and real processing, with explicit test-topic
connections and wall-clock ROS time. No Gazebo, physical camera, SLAM, motor
commands or artificial map transform is needed. See the
[production runbook](../../Perception/RUNBOOK.md#3-perception-only-tests-what-to-launch)
for the separate-terminal feeder/master recipe.

## Scenarios and expectations

| Scenario | Expected evidence |
| --- | --- |
| rocks_and_marker | A rock near base (2,.6,.25) m; marker ID 0 at about optical depth 2 m; occupied grid cells and healthy diagnostics |
| ground_only | Observed ground cells only; no fabricated rock or marker |
| missing_depth | RGB/PnP continues; depth fusion unavailable and quality flags reflect it |
| invalid_cloud | Terrain unavailable; no successful clear-terrain claim |
| dropout | Intermittent input loss; freshness times out and optional grid becomes unknown |
| turn_occlusion | Object leaves the synthetic view; compare memory on/off in terrain-only mode |
| terrain_hazard | Geometric raised/ramped patch; risk interpretation requires explicitly calibrated limits and gravity TF |

Select `pipeline:=terrain` or `pipeline:=markers`. Optional memory test:

```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=terrain scenario:=turn_occlusion launch_memory:=true
```

Only this explicit terrain-memory mode publishes synthetic odom->base_link.
Never run it beside real odometry. RGB/depth fixtures are fixed-camera fixtures;
moving-cloud tests intentionally restrict the selected pipeline to terrain.

## Inputs, frames and truth

Inputs are `/testing/perception/camera/{points,image_raw,depth,image_info}`.
The optical frame is `test_camera_optical_frame`; its known fixture mount is
published under base_link. These are fixture coordinates, not physical rover
calibration values. Depth is aligned 32FC1 metres; intrinsics are 640x480 with
fx=fy=600. The 0.21 m DICT_4X4_50 marker is generated deterministically.

Green expected boxes are published on `/testing/perception/ground_truth`. The
test view overlays these with actual detector results. No synthetic publisher
writes perception outputs. Existing SLAM output mocks are separately packaged
under `testing/SLAM/slam_test_tools` and must never be mixed with live producers.

`legacy_map_anchor.launch.py` preserves the relocated explicit fake-map fixture;
it is not the recommended test path and must not run beside SLAM. Normal tests
require neither map TF nor global marker mapping.

## Automated checks

```bash
python3 -m pytest Perception/terrain_geometry/test Perception/marker_detection/test \
  testing/Perception/perception_test_tools/test -q
```

Run ROS integration tests on an isolated ROS_DOMAIN_ID, with no rover hardware
attached. Tests start only their own process groups and stop those groups; no
broad process killing is used. Live tests inspect the real pipelines and bounded
C++ bridge, not mocked algorithm outputs. Logs use `/tmp/perception-*.log`.

A future benchmarker may add recorded-data scoring, ground-truth error metrics,
CPU/latency distributions and reports. Those are not claimed here. Physical
sunlight/dust, drop-off detection, calibrated accuracy, RViz rendering and full
navigation must be checked separately. Antigravity can run the broader Linux
validation handoff without modifying or pushing source.
