# Linux Testing & Validation Guide for Perception (`perception-jazzy-improvements`)

This guide provides exact, reproducible steps to build and validate the Perception subsystem on a **Linux machine (Ubuntu 24.04 / ROS 2 Jazzy)** or inside the project's Linux Docker container (`apex_autonomous_lp:jazzy`).

---

## 1. Prerequisites & Environment Setup

- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: Jazzy Jalisco
- **Workspace Root**: The root of the repository (`Autonmous-27`).

### Network & Process Isolation
Set an isolated `ROS_DOMAIN_ID` to prevent cross-talk with other nodes or simulations on the local network:
```bash
export ROS_DOMAIN_ID=87
source /opt/ros/jazzy/setup.bash
```

> [!WARNING]
> **Clean Stale Build Symlinks**:
> If packages or launch files were relocated (such as `test_perception_standalone.launch.py` moving to `testing/Perception/perception_test_tools`), always build in a fresh or cleaned build/install directory to prevent setuptools symlink collisions:
> ```bash
> rm -rf /tmp/perception-build /tmp/perception-install
> ```

---

## 2. Dependency Resolution

From the repository root, install all declared package dependencies:
```bash
rosdep update
rosdep install --from-paths \
  Perception \
  PathPlanning/erc_path_planner \
  SLAM/rover_slam \
  testing/Perception/perception_test_tools \
  testing/SLAM/slam_test_tools \
  --ignore-src --rosdistro jazzy -r -y
```

---

## 3. Isolated Build (`colcon`)

Build the perception stack and its test tools into dedicated isolated directories (`/tmp/perception-build` and `/tmp/perception-install`) with `--symlink-install`:

```bash
colcon build \
  --base-paths \
    Perception \
    PathPlanning/erc_path_planner \
    SLAM/rover_slam \
    testing/Perception/perception_test_tools \
    testing/SLAM/slam_test_tools \
  --build-base /tmp/perception-build \
  --install-base /tmp/perception-install \
  --symlink-install
```

After compilation succeeds, source the overlay:
```bash
source /tmp/perception-install/setup.bash
```

---

## 4. Automated Pytest Test Suites

Run all unit and integration test suites:
```bash
python3 -m pytest \
  Perception/terrain_geometry/test/ \
  Perception/marker_detection/test/ \
  testing/Perception/perception_test_tools/test/ \
  -v --tb=short
```

### Expected Coverage
- **`Perception/terrain_geometry/test/`**:
  - `test_launch_contract.py`: Launch arguments, scoping, parameter file boundaries, and isolation.
  - `test_observation_semantics.py`: Unknown space vs observed ground clearing, missing depth handling.
  - `test_obstacle_memory.py`: Bounded odom-frame tracking and obstacle expiration.
  - `test_surface_risk.py`: Slope, roughness, and step edge detection.
  - `test_tf_temporal.py`: Dynamic TF lookup and clock-reversal recovery.
- **`Perception/marker_detection/test/`** (109 unit tests):
  - ArUco PnP / RGB-D detection, tracking, Kalman filtering, marker map composition, RTAB-Map landmark adapter, and action targets.
- **`testing/Perception/perception_test_tools/test/`**:
  - `test_scene.py`: Synthetic test fixtures.
  - `test_bridge_integration.py`: Nav2 costmap bridge bounds, nan/inf handling, and rotated 3D box projection.
  - `test_live_stack.py`: Headless live sensor-to-grid pipeline execution.

---

## 5. Live Pipeline & Scenario Testing

The `perception_test_tools` package provides reproducible scenarios without requiring Gazebo, hardware, or artificial `map` transforms.

### A. Combined Perception (Terrain + Markers)
```bash
# With RViz GUI (on a system with active Linux display):
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=all scenario:=rocks_and_marker launch_rviz:=true

# Headless / Terminal only:
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=all scenario:=rocks_and_marker launch_rviz:=false
```

### B. Terrain-Only Pipeline
```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=terrain scenario:=rocks_and_marker launch_rviz:=false
```
Verify topic outputs:
```bash
ros2 topic hz /terrain/costmap
ros2 topic echo /perception/local_bboxes --once
ros2 topic echo /terrain/diagnostics --once
```

### C. Marker-Only Pipeline
```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=markers scenario:=rocks_and_marker launch_rviz:=false
```
Verify marker detection:
```bash
ros2 topic echo /marker_poses --once
ros2 topic echo /marker_detections --once
```

### D. Specific Robustness Scenarios
Test failure semantics and edge cases:
- `scenario:=dropout`: Simulates intermittent sensor loss; verify nodes don't crash.
- `scenario:=invalid_cloud`: Simulates malformed sensor pointclouds; verify terrain marks unknown rather than free.
- `scenario:=missing_depth`: Color image present without depth; verify RGB PnP fallback with diagnostics.
- `scenario:=turn_occlusion`: Rotates the rover; verify obstacle memory persists obstacles outside current FOV.
- `scenario:=terrain_hazard`: Steep slope/drop-off; verify hazard detection.

Example:
```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=terrain scenario:=invalid_cloud launch_rviz:=false
```

---

## 6. Nav2 Costmap Bridge Verification

Verify that the C++ bridge properly bounds point output from 3D bounding boxes:
```bash
ros2 run erc_path_planner costmap_bridge_node --ros-args \
  -p input_topic:=/testing/bridge/boxes \
  -p output_topic:=/testing/bridge/cloud \
  -p max_points:=100
```

---

## 7. Production Launch Cleanliness Check

Ensure production launch configurations do not contain synthetic publishers or artificial transforms:

```bash
# Check available launch arguments for production perception system
ros2 launch terrain_geometry perception_system.launch.py --show-args
```

Verify that:
1. `perception_system.launch.py` starts real sensor subscribers (default `/camera/depth/color/points` and `/camera/color/image_raw`).
2. No `sensor_inputs` mock node or fake `static_transform_publisher` (e.g. `map -> odom`) is started by production scripts.
3. Test harnesses remain strictly isolated under `testing/`.

---

## 8. RViz Visual Checks (Linux Display)

If a physical or virtual X11/Wayland display is available (`echo $DISPLAY` is not empty):
```bash
ros2 launch terrain_geometry perception_system.launch.py \
  launch_rviz:=true enable_costmap:=true use_sim_time:=false
```
Inspect:
1. **Camera Frame Alignment**: Point clouds and marker axes align with `camera_depth_optical_frame`.
2. **Occupancy Grid**: Occupied cells (purple/red) accurately envelop detected rock bounding boxes.
3. **Marker Display**: Marker bounding boxes and ID text render in RViz.

*Note: In headless environments (such as Docker without X-forwarding), report GUI visual checks as untested and rely on ROS 2 topic echoes.*
