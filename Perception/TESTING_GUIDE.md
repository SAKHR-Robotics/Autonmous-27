# Perception validation and testing handoff

Validation targets the production Perception packages. Master-folder testing
assets only provide controlled dependencies. Work here uses `autonomous27:jazzy`;
no host Python/ROS installations or shared build directories are changed.

## Production regression checks

Build the five packages using the isolated Docker [runbook](RUNBOOK.md#1-open-an-isolated-validation-container).
Inside that container, after sourcing `/validation/install/setup.bash`:

```bash
python3 -m pytest /workspace/Perception/perception_bringup/test \
  /workspace/Perception/terrain_geometry/test \
  /workspace/Perception/marker_detection/test -q -p no:cacheprovider
```

The package-local tests cover actual processing helpers and node contracts.
They are not installed mock dependency providers. The optional test-launch
compatibility check skips when `perception_test_tools` has not been built.

Coverage includes cloud row padding/endianness/layout rejection, transform header
preservation, unknown versus observed ground, unavailable terrain, dynamic TF,
surface-risk and memory math, marker detection/PnP/tracking, calibration freshness
and resolution, stale/future RGB rejection, action-target freshness and covariance,
native landmark contracts, independent launch selection, configuration scoping,
legacy forwarding and production RViz defaults.

Check the installed launch entry points:

```bash
ros2 launch perception_bringup perception.launch.py --show-args
ros2 launch terrain_geometry terrain.launch.py --show-args
ros2 launch marker_detection marker_branch.launch.py --show-args
# No sensors: check startup only, then Ctrl+C and inspect clean shutdown.
ros2 launch perception_bringup perception.launch.py use_sim_time:=false
```

Startup is not proof of correct processing. Test actual message flow separately.
Logs/artifacts belong in the dedicated validation directory, not source packages
or the other agent's master testing folder. Never use broad ROS/Gazebo kill commands.
Stop only the processes started by this validation run.

## Controlled production message flow

**Current status:** the default Patchwork++ geometry check fails with the existing
ground-at-base-origin fixture. These commands reproduce the case; they are not a
passing terrain acceptance recipe. The frontal marker also retains
`POSE_AMBIGUOUS`, so positive action/landmark acceptance needs a different coherent
scene. See [the measured results and decision](INTEGRATION_REVIEW.md).

The existing feeder is `testing/mocks/perception/perception_test_tools`.
Its layout is owned by the testing agent. Use it as-is or snapshot it into a
scratch directory before building; do not change it as part of Perception work.
Build that package into the same isolated overlay when deliberately selected:

```bash
colcon --log-base /validation/log build \
  --base-paths /workspace/testing/mocks/perception/perception_test_tools \
  --build-base /validation/build --install-base /validation/install --symlink-install
source /validation/install/setup.bash
```

Terminal A supplies only controlled sensor inputs and its explicitly test-only
camera mount TF; Terminal B runs the real production processing:

```bash
# A: the existing testing package, no production algorithms replaced.
ros2 launch perception_test_tools sensor_inputs.launch.py scenario:=rocks_and_marker
```

```bash
# B:
ros2 launch perception_bringup perception.launch.py use_sim_time:=false \
  enable_costmap:=true launch_landmark_adapter:=true \
  input_pointcloud_topic:=/testing/perception/camera/points \
  rgb_topic:=/testing/perception/camera/image_raw \
  depth_topic:=/testing/perception/camera/depth \
  camera_info_topic:=/testing/perception/camera/image_info
```

Never attach these synthetic providers to real rover hardware. The isolated
container keeps this ROS graph separate. Expected scene evidence includes rock
truth geometry near base (2.0, 0.6, 0.25) m, marker ID 0 near optical depth 2 m, occupied
grid cells and healthy terrain diagnostics. For an accepted marker ID 0, native landmark ID is 1 with the
default offset; the current frontal fixture is correctly rejected as ambiguous. Exact acceptance tolerances must account for sampled surfaces
and the selected ground backend; these fixtures are not calibrated physical truth.

Additional feeder scenarios already include `ground_only`, `missing_depth`,
`invalid_cloud` and `dropout`. Ground-only is required to be a valid empty obstacle update (currently failing
with native Patchwork++ and this fixture);
invalid cloud/dropout must report unavailable perception and an unknown enabled
grid. Missing depth must preserve RGB detection/PnP with calibration. A resumed
stream must recover without restarting production. Inspect both data and health.
Do not run two feeders simultaneously on the same topics/TF.

## Testing-agent handoff

The offline generator and runner have now been relocated into master `testing/`
at Saif's request. Existing live sensor-feeder behavior is unchanged.

| Item | Status / action |
| --- | --- |
| Offline terrain generator and runner | Relocation complete: generator in `testing/mocks/perception/perception_test_tools/perception_test_tools/terrain_scene.py`, runner in `testing/benchmarking/perception/perception_benchmark_tools`. The old production module is removed. Use `ros2 run perception_benchmark_tools terrain_benchmark --synthetic --frames 3`; this measures helper computation, not production-node accuracy. |
| Combined launch references | Prefer `perception_bringup perception.launch.py`. Existing `terrain_geometry perception_system.launch.py` includes still work after bringup is built. |
| Combined view | Production view now lives in `perception_bringup/rviz/`. Truth overlays stay in the testing package's own RViz view. |
| Dependency mocks | Match the documented topic types, QoS, units, timestamps and frames; provide only selected missing dependencies with one TF/output authority. |
| Scenarios/referees | Cover normal inputs, missing/stale inputs, calibration mismatch, TF loss, recovery and clock reset; keep expected truth separate from algorithm inputs. |

The printable ArUco image generator is a calibration utility, not a sensor mock;
it remains in `marker_detection`. Test-only publisher nodes already reside in
master `testing/`. The offline benchmark relocation preserves the generator and
processing settings; it does not resolve the known ground-classification error.
See the [offline runner README](../testing/benchmarking/perception/perception_benchmark_tools/README.md)
for isolated build and execution commands.

## Limits and next integration checks

Passing regression tests and synthetic message flow does not establish hardware
accuracy, outdoor performance, map quality, safe stopping or ERC compliance.
Validate physical camera alignment/mounting, real scenes and target-computer
latency. Then validate Nav2 obstacle marking/clearing and observation-loss behavior,
RTAB-Map localization/loop closure, and mission target handling in their module
reviews. Diagnostic timers follow ROS time; consumers need a policy for clock loss
and stopped publishers. See [current evidence](INTEGRATION_REVIEW.md).
