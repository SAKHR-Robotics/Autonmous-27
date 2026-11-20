# Production SLAM staged review

Review dates: 2026-11-19/20. Branch: `TestingValidation`; initial revision
`5916db1`. Other agents' later commits on this shared branch are preserved.
This record supersedes historical task completion claims for current readiness.
Read alongside [the running guide](rover_slam/README.md) and
[Perception's measured limits](../Perception/INTEGRATION_REVIEW.md).

## Scope, environment and coordination

The production implementation is one `ament_python` package, `rover_slam`.
It installs encoder conversion and slip checking executables, ten launches,
six parameter YAML files and local/global RViz views. Wheel math is a library.
`robot_localization`, `rtabmap_slam`, Nav2, TF and the optional RealSense driver
are external production packages. Normal bringup does not import testing tools.
The explicit `ekf_validator.launch.py` compatibility entry requires testing.

The testing owner confirmed these scoped locations may be edited here:
`SLAM/**`, `testing/mocks/slam/slam_mock_tools/**`, and
`testing/benchmarking/slam/**`. Shared testing READMEs, other mock packages and
`testing/validation_bringup/` remain theirs. The latter has documentation only,
not a ROS bringup package. Existing mock implementations have already moved;
do not repeat that migration or duplicate shared dependencies.

Runtime is the existing `autonomous27:jazzy` image, ID
`sha256:d952a9381d1f0b15c8212df0a78e023f3058235c659c44267bb782500df235ad`.
Docker Python is 3.12.3, ROS is Jazzy. No host ROS/Python or package installation
is used. Container `codex-slam-validation-20261119` uses `--network none`,
ROS domain 88, a read-only `/workspace`, and isolated `/validation` build,
install, log and output paths. Host artifacts are
`/tmp/codex-slam-validation.FoJgcD`, outside source packages.

Saif selected `final_world_R&A.world` for Gazebo and **retaining EKF on
`base_link`**, with a coordinated Rover URDF correction rather than changing
SLAM to `base_footprint`. Hardware geometry remains uncalibrated. Keep the
existing planar estimator; old proposals to enable full 3D are not accepted
implementation requirements. Accuracy tolerances need an explicit requirement
before a numeric trajectory/map acceptance verdict.

## Launch inventory

Production launch defaults use wall time (`use_sim_time=false`). Selecting true
requires an external `/clock`; none of these production launches supplies one.
All nodes and selected providers must share the same clock.

| Launch in `rover_slam` | Starts | External inputs | TF assumptions |
| --- | --- | --- | --- |
| `encoder.launch.py` | Encoder converter; one selected source | Default `/joint_states`; alternatively `/wheel/ticks` or four individual count topics | Frames default `odom`, `base_link`; launch forces TF publication off |
| `slip.launch.py` | Slip covariance filter | Selected raw wheel Odometry (default `/wheel/odom_raw`), `/imu/data`; optional `/wheel/single_wheel_slip` | IMU mounting transform into odometry child frame at measurement time; publishes no TF |
| `ekf.launch.py` | EKF + slip by default; encoder optional | Wheel input and IMU; enabling converter requires `/wheel/odom_raw` | EKF owns `odom -> base_link`; external description owns sensor transforms |
| `rtabmap.launch.py` | Real RGB-D RTAB-Map | `/odometry/filtered`, RGB, aligned depth, CameraInfo; optional native landmarks | External odom/body/sensor TF; owns `map -> odom`; database preserved by default |
| `slam_bringup.launch.py` | EKF, slip and RTAB-Map by default | Default raw wheel source `/odom`, IMU, RGB-D/calibration, mounts; no simulator | Combines the preceding ownership; optional camera, static TF, costmap and RViz default off |
| `static_transforms.launch.py` | No broadcasters by default; opt-in mount/optical fallbacks | Explicitly chosen missing geometry | Never enable a child already owned by URDF/driver; mount values are generic, not calibration |
| `vision_helper.launch.py` | RealSense driver, filters and normalized camera aliases | Physical supported camera and measured base mount; separate IMU | Driver owns camera internal frames; external base mount; driver is missing in this image |
| `costmap.launch.py` | Obstacle inspection costmap + lifecycle manager | Marking `/bridge/pointcloud`, clearing `/camera/depth/color/points`, valid global robot TF | Needs `map -> base_link`; no static map layer; do not duplicate navigation costmaps |
| `rviz.launch.py` | RViz only | GUI, selected topics, description and TF | Global fixed frame `map`; local `odom`; accepts explicit overrides |
| `ekf_validator.launch.py` | Forwards to testing EKF + monitor | Installed `slam_test_tools`, wheel/IMU inputs | Same EKF ownership; never run alongside a second EKF |

Testing launches remain in `slam_test_tools`: `ekf_validation.launch.py` starts
real EKF/slip plus a health monitor; `test_slam_standalone.launch.py` starts
the real master and RViz, with both smoke mocks off. The latter defaults to
simulation time and needs external Gazebo/bag sensors, mounts and `/clock`.
It does not supply complete synthetic SLAM inputs or independent trajectory truth.

`slam_test_tools/local_mocked.launch.py` is a separate, wall-time-only nominal
local-estimator profile. It runs real encoder conversion, slip filtering and
EKF. The existing shared mock rover has TF disabled and all its outputs/control
inputs isolated. Its synthetic odometry is remapped to `/testing/slam/truth`,
never `/odometry/filtered`. The SLAM-owned `rover_sensor_adapter` expands its two
side-joint positions into four wheel observations and derives coherent planar
IMU observations. It includes centripetal acceleration, uses source timestamps,
rebaselines on clock reversal/gaps and supports explicit sensor dropout flags.
This profile has zero noise/slip, radius 0.06 m and track 0.49 m. It is a
synthetic consistency check; sensors and comparison truth share a kinematic
model, so it cannot establish physical accuracy or slip classification quality.

Rover's `gazebo.launch.py` starts Gazebo, spawning, robot/joint state publishers
and bridges; its world default is `world1.world`. Its three optional TF switches
must remain false with SLAM. `spawn_rover.launch.py` expects an existing world;
its implementation hardcodes world name `marsyard` despite a world argument.
Both launches bridge `/clock`, wheel odometry, IMU, joint states and RGB-D.
Do not start both bridge sets. The selected world must actually expose the
expected world/sensor names. Their camera optical correction is applied in
launch processing; it is not supplied by the bare URDF alone.

## Interface and logic audit

| Boundary | Current contract and finding |
| --- | --- |
| Wheel conversion | JointState positions are radians; counts use 1024 ticks/rev, radius 0.06 m and track 0.49 m, matching Gazebo parameters. Default count order is left-front, right-front, left-rear, right-rear; a legacy six-count array selects indices 0,2,3,5. Measurement intervals drive m/s and rad/s. Rejects incomplete/nonfinite, stale/future, duplicate and implausible measurements; gaps rebaseline. Unstamped count messages use receipt time; zero JointState stamps also fall back to receipt time, so original acquisition age cannot be verified. |
| Encoder outputs | Reliable depth-10 Odometry, Float64MultiArray wheel speeds and Bool single-wheel flag. Odom pose is integrated from wheels; only velocity/yaw rate are fused. Nominal twist variances are 0.02 and 0.05. Raw covariance is a fixed heuristic, not a calibrated accuracy model. No fresh zero-motion messages are manufactured after input loss. |
| Slip inputs/output | Raw Odometry reliable depth 10; IMU Best Effort sensor QoS. Preserves original stamp, frames, measured twist and input message; scales covariance including cross terms. IMU must be fresh, time-aligned, finite and transformable. Missing IMU inflates wheel covariance and reports degraded status; false slip Bool alone is not proof of health. |
| Slip limits | No open-loop IMU velocity integration remains. Heuristics detect selected blocked-turn, spin, collision, takeoff and wheel disagreement cases. Same-direction yaw discrepancies can be exempted deliberately. This is suspected slip, not measured ground speed. Equal steady wheel slip is not generally observable from these sensors. IMU translational lever-arm acceleration is not compensated. |
| Covariance validation | Slip now rejects nonfinite, negative-diagonal, asymmetric and indefinite matrices, plus overflow. Symmetry/eigenvalue checks allow numerical roundoff at 1e-12 scale; valid rank-deficient matrices and cross terms are preserved. New regressions failed before the fix and pass afterward. NumPy is declared as a runtime dependency. |
| Local EKF | 30 Hz planar output, sensor timeout 0.2 s. Fuses wheel X velocity/yaw rate, IMU yaw/yaw rate and X/Y acceleration; removes gravity and resets on backward ROS time. No RTAB-Map feedback into local filter. Long prediction after input loss must be measured; a configured sensor timeout is not a motor-stop policy. |
| RGB-D mapping | Reliable EKF odometry, Best Effort image/calibration subscriptions, approximate synchronization bounded to 0.05 s. `odom_frame_id` is empty to consume covariance-bearing messages. Depth must be registered to RGB and metric (float metres or supported uint16 millimetres). Current RTAB-Map settings force planar registration/optimization; backend strategy 1 is g2o. Gazebo sensor flow is observed; physical sensor compatibility and map-surface accuracy remain untested. |
| Landmarks | Native array `/perception/landmark_detections` and singular `/perception/landmark_detection`, with RTAB-Map message types. Perception adapter preserves observation headers/covariance and offsets IDs by +1. Historical `/perception/aruco_pose` PoseStamped is not the native mapping input. Positive camera-to-SLAM acceptance remains unvalidated; do not relax gates. |
| Planner | Uses `/odometry/filtered`, `/map`, `map`, `odom`, `base_link`. Nav2 owns navigation costmaps. SLAM's optional inspection costmap marks bridge clouds and clears measured camera rays; empty boxes/TTL do not establish clearing. Terrain classification remains faulty, so healthy SLAM does not establish safe terrain/planning behavior. |

Clock reversal clears converter measurement baselines and slip cached inputs.
A real DDS regression now confirms wheel loss produces no new wheel output,
backward `/clock` requires a new wheel baseline, old IMU data is discarded,
degraded covariance is emitted until fresh IMU returns, and preprocessing then
recovers. The separate mocked run exercises actual EKF input loss/recovery.
Vendor EKF/RTAB-Map reset recovery, TF loss and stopped-clock policy remain
untested. ROS-time watchdogs pause with a stopped
clock; separate consumers need a wall-time clock-loss policy. No hardware
interlock or physical calibration is established by this review.

## Findings and bounded implementation sequence

| ID | Location / evidence | Cause and next action | Status |
| --- | --- | --- | --- |
| S01 | Rover URDF `base_footprint_joint` versus EKF `base_link_frame` | Reversed the fixed footprint edge per Saif's choice, preserving the 0.255 m offset and chassis-relative mounts; Gazebo odometry child now matches `base_link`. Expanded-URDF tests and live TF samples confirm the intended chain. | Fixed; regression and Gazebo recheck passed |
| S02 | Encoder/slip `main()` and testing monitor | Ctrl+C produces `rcl_shutdown already called` after Jazzy signal handler. Use `try_shutdown`; actual installed launches now exit cleanly. | Fixed; live recheck passed |
| S03 | Production READMEs and historical topic/task records | Corrected active testing paths/build boundaries and marked historical 100 Hz/full-3D/PoseStamped claims. Retained the 30 Hz planar/native-landmark design. | Corrected |
| S04 | `test_camera_optical_frame.py` requires Rover package index | Production-only build passes; first tests were 29 pass/1 fail due missing Rover index. Declare test dependency and build prerequisite; all 47 then pass, no skips. | Dependency declared; recheck passed |
| S05 | Optional `costmap.launch.py`, installed Nav2 | With no map TF, activation waits; Ctrl+C during activation leads to context-invalid error and child abort -6. Reproduced with only launch-parent SIGINT; investigate lifecycle cancellation separately. Do not fabricate map TF to obtain a pass. | Failed, unresolved |
| S06 | Optional RealSense helper | `realsense2_camera` absent from unchanged image. `--show-args` does not resolve/start that executable. Physical driver validation requires a separately approved image/device stage. | Blocked |
| S07 | Rover Gazebo bridges and IMU sensor | Live run found two IMU endpoints and an untransformable `my_robot/base_link/imu_sensor` header. Removed redundant entries from both Rover launches and set the sensor's `gz_frame_id` to its existing `imu_link` mount. Rerun observes one publisher, transformable headers and healthy slip diagnostics. | Fixed; Gazebo recheck passed |
| S08 | Legacy SLAM mocks | Landmark stamp is zero and covariance unset; fixed sensor-relative scene is not moving truth. Obstacle marker stamps also use zero. Keep these out of accuracy/positive landmark acceptance; repair separately if used. | Unvalidated smoke fixtures |
| S09 | Slip covariance helper | Symmetry/PSD validation fixed, before the observation can mutate collision state. A regression reproduced invalid covariance latching a collision despite rejected output, then passed with early validation. IMU lever-arm handling and wider mismatched-rate/delayed/out-of-order sensor scenarios remain outside passing cases. | Covariance/state isolation fixed; broader scenarios untested |
| S10 | Rover `gazebo.launch.py` | Selected world filename contains `&`; unquoted `gz_args` caused the shell to run `A.world` and exit 127. Quote the resolved path; actual-launch construction regression checks spaces and ampersands. | Fixed; selected world runs |
| S11 | Slip collision heuristic | Matched braking in either direction was classified as collision and permanently latched obstacle stall. Compare IMU impact with measured wheel deceleration; retain existing blocked-wheel impact/takeoff checks and expose the slip reason. Two new braking regressions failed before the fix; the real nominal run now recovers. | Fixed for tested cases |
| S12 | RViz profiles/launch | `/mapPath` is volatile, but views requested transient durability. Corrected both views and forwarded the selected raw wheel topic into RViz (`/odom` in normal Gazebo bringup). | QoS corrected; both views render |
| S13 | Shared `mock_rover_sim.py` shutdown | Testing owner explicitly cleared only the duplicate shutdown call. Replaced it with `try_shutdown`; shared mock defaults/algorithms remain unchanged. All local-profile children now stop cleanly. | Fixed; live recheck passed |
| S14 | EKF acceleration fusion | Controlled turn error reproduces specifically with lateral acceleration fusion. Independent Gazebo motion also shows remaining drift. See profile comparison below; the production default remains unchanged pending the fusion decision. | Confirmed accuracy concern; default decision open |

Stage 1 closes package installation, launch expansion and sensor-free startup;
failures stay listed rather than becoming accuracy passes. Stage 2 addresses
confirmed production defects and tests real encoder/slip/EKF with selected
dependency mocks and independent truth. Reuse shared mock rover inputs; do not
consume its synthetic `/odometry/filtered` as a SLAM output or publish its TF.
It supplies two wheel joint names and no IMU; the new adapter supplies only
those missing interfaces without changing or duplicating the shared rover.
The existing Perception stationary feeder is also unsuitable for moving RGB-D.

Stage 3 uses the selected Gazebo world, real rover sensors and independent
Gazebo entity truth, never wheel-integrated `/odom` as accuracy truth. Measure
timestamp-aligned local/global position/yaw error, drift per distance, turn
response, loop closure, mapped geometry, dropout and recovery. Store raw data,
rates, coverage and errors; agree acceptance bounds before pass/fail accuracy.
Stage 4 renders both production RViz views and checks whole-module contracts.
RViz now forwards the chosen raw-wheel topic and matches `/mapPath` durability.
Both views rendered using image-installed RViz, Qt and software OpenGL 4.5,
with window captures outside source. Sensor-free rendering is not a visual
acceptance of localization/map contents; full data-attached visual review remains.

## Fusion investigation and measured motion

The unchanged production profile fuses wheel forward speed/yaw rate and IMU
yaw/yaw rate plus forward/lateral acceleration. Encoder + IMU fusion itself
works through turns; selecting acceleration channels is the issue under review.
No zero-sideways-velocity constraint has been added, since it could hide real
skid-steer lateral slip. No production fusion mask was changed.

Four scratch configurations were exercised with the same nominal fixture,
commands, truth matching (within 0.05 s), loss/recovery checks and output logic:

| EKF acceleration inputs | Max planar position error in stationary/straight/turn segment | Loss/recovery contracts |
| --- | --- | --- |
| Forward + lateral, current default | 0.326461 m | 8/8 pass after braking fix |
| Neither | 0.002834 m | 8/8 pass |
| Forward only | 0.002803 m | 8/8 pass |
| Lateral only | 0.326448 m | 8/8 pass |

These are nominal synthetic consistency measurements, not physical accuracy
passes. Heading error stayed around 6.4e-5 rad. The fixture's lateral
acceleration during steady turning is physically `v * yaw_rate`.
The image runs robot_localization 3.8.3. Its
[versioned prediction implementation](https://github.com/cra-ros-pkg/robot_localization/blob/3.8.3/src/ekf.cpp)
integrates the lateral acceleration state into lateral velocity. With the
present unconstrained lateral velocity, the observed growth is consistent with
feeding centripetal acceleration into that path. This is an inference from
the source and isolated channel comparisons, not a claim that all IMU fusion
fails on turns. Further modelling/calibration or a separately justified motion
constraint is needed before treating lateral acceleration as validated.

The selected Gazebo world ran with real rover sensors and a fresh RTAB-Map
database per measured run. Independent truth came directly from Gazebo
`/world/marsyard/dynamic_pose/info`; model identity was resolved using the scene
service (`my_robot`, entity 190 in these runs). It did not come from wheel
odometry, estimated TF or a SLAM output. The generic Pose_V-to-TF bridge did not
preserve usable model identity in the first attempt; that attempt is recorded
as unavailable, not a truth measurement.

Measurements below compare planar trajectories after one initial rigid
translation/yaw alignment, matching timestamps within 0.05 s. No scale fitting
or per-sample alignment was used. They are relative drift measurements, not
surveyed global position. The separate Gazebo runs are diagnostic comparisons,
not identical-sensor replay: simulation timing and IMU noise can differ.

| Gazebo profile | Matched local/global samples | Max local/global position error |
| --- | --- | --- |
| Current production profile | 607 / 17 | 0.5840 / 0.5367 m |
| Scratch forward-acceleration-only profile | 771 / 23 | 0.2548 / 0.2512 m |

The candidate still has material error on the actual terrain. Wheel slip,
calibration, planar modelling on sloped terrain and visual/global correction
behavior require further isolation. The planar estimate does not establish
physical roll/pitch; 3D map/terrain accuracy cannot be inferred from these
planar trajectory measurements. No competition/hardware accuracy pass is
assigned, no acceptance bound was relaxed, and no lateral-slip measurement was
invented. Map publication was observed, but map-surface error, successful loop
closure and closed-loop repeatability remain unvalidated. The later bounded
revisit below checks RGB loss/recovery and exposes failed clock-reset recovery.
Positive Perception camera-to-landmark acceptance remains unvalidated.

Artifacts include the per-sample JSON, scratch YAML profiles, executed drivers,
logs and `fusion-comparison.png` under the isolated run directory. The default
fusion decision remains open; continuing analysis is not approval to change it.

## Executed evidence

### Bounded Gazebo revisit and input recovery

`/validation/gazebo_revisit_check.py` used real Gazebo sensors and the scratch
forward-acceleration-only EKF profile, a fresh database and native entity truth.
The test-only `image_gate` forwards original RGB messages unchanged, except
during the deliberate ten-second interruption. Both outbound and return targets
were reached; native Gazebo return distance was 0.03530 m. This is actual robot
return distance, not estimator accuracy. Mapping Info updates stopped during RGB
loss, local odometry continued, and mapping resumed after RGB restoration.
Neither loop-closure nor proximity-closure IDs were reported: revisit occurred,
but successful global relocalization/loop closure was not demonstrated.

The simulator accepted `WorldControl.reset.all`. During the next ten wall
seconds the observer received no new-clock EKF odometry or RTAB-Map Info updates.
Both recovery checks failed. TF old-data warnings were logged; the exact cause
requires isolation across simulator reset, bridge, TF buffers and vendor nodes.
Native truth matching also stops after reset because entity identity can change;
post-reset truth must be rediscovered before any accuracy measurement. Do not
infer post-reset physical accuracy or blame one node from this run alone.

Per-message TF publisher GID is absent from this image's rclpy callback metadata.
Full TF-authority tracing is therefore untested, not passed. Two initial recorder
attempts failed on this metadata assumption before motion; the final attempt
records the limitation explicitly. Final artifacts are
`gazebo-revisit-results.json`, `gazebo-revisit-driver.log`, `revisit-*.log` and a
fresh RTAB-Map database. Reproduce inside the existing isolated Docker environment:

```bash
source /opt/ros/jazzy/setup.bash
source /validation/install/setup.bash
python3 /validation/gazebo_revisit_check.py > /validation/gazebo-revisit-driver.log 2>&1
```

The driver stops only its recorded launch/server process groups. A zero driver
exit is not an acceptance verdict: inspect every contract, null/untested value,
closure ID list and raw samples. Clock-reset recovery currently fails.

| Passed | Evidence |
| --- | --- |
| Production independence | Fresh `rover_slam` symlink build with no testing packages in overlay |
| Optional helper installation | Rover + production + both relocated SLAM testing packages built in scratch |
| Regression baseline | Initial complete environment: 47 passed, zero skips |
| Current regression suite | 60 passed, zero skips, Docker Python; includes real DDS clock reset, invalid-input state isolation and mock-adapter physics/dropout checks |
| Launch argument expansion | Nine normal production launches expanded without test packages; compatibility wrapper correctly required absent testing package |
| Installed startup and shutdown | Encoder, slip, EKF, RTAB-Map, production master and compatibility validator started with no sensors and stopped cleanly after shutdown fix |
| Current production-only overlay | Fresh separate build/install, testing package absent, real slip/EKF/RTAB-Map started and stopped cleanly |
| Nominal real local stages | 8/8 truth/input-loss/recovery contracts pass; measured current-profile turn error remains an accuracy concern |
| Gazebo input/TF checks | RGB, 32FC1 depth, CameraInfo, IMU, wheels and map arrive; corrected IMU frame/publisher and TF chain observed |
| RViz rendering | Both packaged profiles rendered and exited cleanly in Docker; captures inspected |

**Failed:** full Gazebo clock-reset recovery in the bounded observation window;
missing-TF optional costmap activation/shutdown; some headless
Gazebo servers required scoped shutdown escalation; current-profile
nominal turning consistency concern and remaining Gazebo trajectory error.
**Blocked:** physical RealSense driver/device check; final accuracy acceptance
requires an agreed requirement. **Untested:** map-surface accuracy, loop closure,
full TF-authority tracing, wider sensor-rate
faults, positive camera landmarks, data-attached visual acceptance and hardware.
No SLAM output has
been replaced by a mock and called validated. Known Perception failures remain.

## Reproduction commands and shutdown

Host commands create an isolated container from the existing image; no image
build or dependency replacement was performed. Use a fresh run directory/name
if this retained container or artifacts are still needed.

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
validation_dir=$(mktemp -d /tmp/codex-slam-validation.XXXXXX)
chmod 777 "$validation_dir"
docker run -d --name codex-slam-validation-20261119 --network none \
  -e ROS_DOMAIN_ID=88 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/validation/ros-log \
  -v "$PWD":/workspace:ro -v "$validation_dir":/validation -w /workspace \
  autonomous27:jazzy sleep infinity
docker exec -it codex-slam-validation-20261119 bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
# First: production alone, proving independence from testing packages.
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/SLAM/rover_slam \
  --build-base /validation/build --install-base /validation/install
# Then: declared optical-test prerequisite and optional testing tools.
colcon --log-base /validation/log-deps build --symlink-install \
  --base-paths /workspace/SLAM/rover_slam /workspace/Rover/my_robot_description \
  /workspace/testing/mocks/slam/slam_mock_tools \
  /workspace/testing/benchmarking/slam/slam_test_tools \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
python3 -m pytest /workspace/SLAM/rover_slam/test \
  /workspace/testing/benchmarking/slam/slam_test_tools/test \
  -q -rs -p no:cacheprovider
ros2 launch rover_slam slam_bringup.launch.py --show-args
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=false \
  database_path:=/validation/manual-startup.db
```

The original baseline was 47 passes; the current complete suite is 60 passes
when the SLAM mock package's `test/` directory is included. Master starts slip, EKF and
RTAB-Map. With no sensor providers there is no valid localization/map evidence.
Stop the foreground launch with **one Ctrl+C**; every child should exit cleanly.
Repeat individual launches from the inventory with the same explicit time.
Any traceback, child crash, missing executable, duplicate publisher/TF owner,
or claimed healthy output with invalid inputs fails that stage. The optional
costmap is currently a known failing case without its TF prerequisite.

The executed bounded launch driver is `/validation/startup_checks.py`, with
before/final logs and JSON summaries in `/validation`. It signals only its
launch parent and waits for its children. An initial whole-group signal driver
sent duplicate SIGINTs; its shutdown traces are not the evidence used to assign
S02. The parent-only rerun independently reproduced S02. Launch exit code zero
alone was not used: child logs were reviewed and exposed the costmap abort.

```bash
# Host: stop only this review's container after its foreground runs are stopped.
docker stop --time 10 codex-slam-validation-20261119
```

Do not use global pkill/killall, shared build/install directories, bulk git
staging, branch switches or another agent's uncommitted files. Generated logs,
databases and captures remain outside source packages.

### Current local mocked stage

Inside the same container, build the additional shared fixture dependency and
its message package before running the new profile:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /validation/log-mocked build --symlink-install \
  --base-paths /workspace/SLAM/rover_slam /workspace/Rover/my_robot_description \
  /workspace/Perception/terrain_geometry_msgs \
  /workspace/testing/mocks/pathplanning/pathplanner_mock_tools \
  /workspace/testing/mocks/slam/slam_mock_tools \
  /workspace/testing/benchmarking/slam/slam_test_tools \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
python3 -m pytest /workspace/SLAM/rover_slam/test \
  /workspace/testing/benchmarking/slam/slam_test_tools/test \
  /workspace/testing/mocks/slam/slam_mock_tools/test -q -rs -p no:cacheprovider
# Terminal A:
ros2 launch slam_test_tools local_mocked.launch.py
# Terminal B, with the same overlay sourced:
ros2 run slam_test_tools local_estimator_probe --output /validation/local-run.json
```

Expected: 60 regression passes, no skips; eight truth/input-loss/recovery
contracts true. The unchanged default still reports the approximately 0.326 m
nominal turn error. Do not interpret a contract-only exit code zero as accuracy
acceptance. A missing package, false contract, stale/fabricated wheel output,
or unhealthy settled recovery fails the corresponding contract check.

The controlled candidate comparison changes only an external scratch YAML:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
profile = yaml.safe_load(Path('/workspace/SLAM/rover_slam/config/ekf.yaml').read_text())
profile['ekf_filter_node']['ros__parameters']['imu0_config'][13] = False
Path('/validation/ekf-forward-accel.yaml').write_text(yaml.safe_dump(profile))
PY
# Stop the prior local launch before starting the alternative.
ros2 launch slam_test_tools local_mocked.launch.py \
  ekf_params_file:=/validation/ekf-forward-accel.yaml
# Another sourced terminal:
ros2 run slam_test_tools local_estimator_probe --output /validation/local-forward.json
```

Expected nominal candidate result is about 0.0028 m max position error, with
the same eight contracts. Both profiles retain encoder + IMU heading/turn-rate
fusion. This comparison does not change production defaults or establish
hardware/Gazebo accuracy. Stop the observer/launch with Ctrl+C, then confirm
each child exits cleanly. The recorded parent-only driver is
`/validation/run_local_mocked.py`; profile-specific drivers and JSON are beside it.

### Selected Gazebo world and independent truth

The world package was built with:

```bash
colcon --log-base /validation/log-worlds build --symlink-install \
  --base-paths /workspace/worlds --build-base /validation/build \
  --install-base /validation/install
source /validation/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/workspace/worlds:/workspace/worlds/models:/workspace/worlds/models/rocks:/workspace/worlds/models/aruco:/validation/install/my_robot_description/share
export LIBGL_ALWAYS_SOFTWARE=1
# A: headless Gazebo with real rendering sensors, no synthetic sensor feeder.
ros2 launch ros_gz_sim gz_sim.launch.py \
  "gz_args:=-s --headless-rendering -r '/workspace/worlds/worlds/final_world_R&A.world'"
# B: one sensor/clock bridge set and the real rover.
ros2 launch my_robot_description spawn_rover.launch.py world:=marsyard \
  bridge_sim_tf:=false publish_map_tf:=false publish_camera_tf:=false
# C: actual production SLAM; use a fresh absolute database path per run.
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  database_path:=/validation/your-fresh-run.db
```

The bounded measured drivers use the direct equivalent `gz sim -s
--headless-rendering -r <world-path>` to supervise the server process, then the
Rover spawn and production master above. Reproduce the measured default and
scratch-profile cases sequentially, with no other ROS/Gazebo processes in this
review container:

```bash
python3 /validation/gazebo_truth_check.py
python3 /validation/gazebo_forward_accel_check.py
```

Each commands stationary, straight, turning and stopped phases for 32 wall
seconds, logs ROS stamps and actual Gazebo entity poses, writes JSON, then stops
only its owned process groups. Output is `gazebo-truth-results.json` or
`gazebo-forward-accel-results.json`; fresh databases include a run identifier.
The source command trajectory is not independent truth. No maps/odometry/TF
from a mock are supplied to the SLAM nodes in these Gazebo cases.

Expected input contract: one clock bridge, one `/imu/data` publisher with
`imu_link`, RGB/depth/CameraInfo with the optical header, wheel Odometry with
child `base_link`, EKF odometry and RTAB-Map output. Missing model identity,
missing inputs, TF ambiguity or absent timestamp-matched truth prevents an
accuracy measurement; do not substitute wheel odometry as truth. Existing
measured errors are listed above, not classified as a physical accuracy pass.

Headless Gazebo did not always exit after SIGINT/TERM. The drivers now escalate
only their saved process group to SIGKILL after bounded waits. One earlier
orphaned review server was stopped by its verified PID; this is recorded as a
shutdown failure, not a clean shutdown pass. Stopping the dedicated review
container is the final scoped cleanup. Broader process-name cleanup is forbidden.

### RViz evidence

`/validation/rviz_capture.py` ran both packaged views using the existing image,
an authorized host X socket/authority mount and software rendering. It captured
only each RViz window, not the whole desktop. `rviz-local.png`, `rviz-global.png`
and logs were inspected. Both windows render, the 3D scene remains visible,
and source-absent fixed-frame warnings are expected in this sensor-free check.
For a data-attached inspection, run a GUI-enabled container in the same ROS
graph and choose the matching clock:

```bash
ros2 launch rover_slam rviz.launch.py use_sim_time:=true rviz_view:=global \
  raw_odom_topic:=/odom
ros2 launch rover_slam rviz.launch.py use_sim_time:=true rviz_view:=local \
  raw_odom_topic:=/odom
```

Run each view independently and stop it with Ctrl+C. Full sensor/map visual
acceptance remains separate from the passing rendering check.
