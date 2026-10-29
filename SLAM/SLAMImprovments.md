# SLAM improvements: implementation and resume record

Approved base: `origin/jazzy-port` at `d9c39d7`. Feature branch:
`slam-jazzy-improvements`. Target: Ubuntu 24.04 / ROS 2 Jazzy on Linux.

## Boundaries

Keep robot_localization EKF + RTAB-Map, existing ROS message contracts and
planar estimation. Do not import the reference IESKF/ZED backend. Synthetic
inputs, evaluation helpers and future benchmarks belong under top-level
`testing/`. Do not create a PR or merge; push this branch for the owner.

Local `jazzy-port` has three separate documentation commits not on the remote
base. Preserve them on that branch. Its TestingPlan.md edits must be reconciled
after that documentation work lands; do not overwrite another agent's work.

## Ordered implementation checklist

- [x] Repair master launch; add encoder/slip standalone launches; scope flags,
      topic/frame overrides, clock and YAML profiles; fix interactive script.
- [x] Select one encoder source; compute speeds from measurement intervals;
      stop valid-looking stale output; reject jumps and rebaseline on resets.
- [x] Validate IMU age/values/frame; preserve measured wheel twist on slip;
      inflate covariance; remove implicit source fallback; publish diagnostics.
- [x] Feed RTAB-Map EKF odometry with covariance; expose database/profile;
      correct RealSense settings and optional costmap configuration.
- [x] Make TF ownership explicit; add opt-in Gazebo optical bridge; update
      local/global RViz and Perception integration documentation.
- [x] Move EKF evaluation helper to testing; repair logging, clock, expected
      frequency and dropout monitoring; retain explicit compatibility launch.
- [x] Rewrite SLAM and package READMEs: build, individual stages, standalone
      Gazebo with testing helpers, hardware, whole system and troubleshooting.
- [x] Verify syntax/manifests/install resources, focused regression tests and
      actual launch construction where ROS exists. Record missing evidence.
- [x] Review each milestone diff, make focused descriptive commits, push branch.

## Acceptance and deferred testing requirements

Production master starts no mock or artificial localization. One owner per TF
edge: EKF odom->base_link; RTAB-Map map->odom; URDF/driver calibrated mounts.
Only one wheel source; stale/invalid measurements cannot look healthy. Slip
changes uncertainty, not reported ground speed. Databases survive by default.
RViz display types match actual topics. Readme commands match installed launches.

The testing work must cover launch combinations/parameter forwarding; sensor
rate mismatch, stale/missing wheels and IMU, counter resets, clock reversal,
rotated IMU, thresholds and covariance; native Perception landmarks and obstacle
cloud clearing; textured coherent moving RGB-D stationary/straight/turn/loop,
dropout and recovery. Truth must not publish estimated TF. Missing dependencies
must never count as passes. No new tester/referee/benchmark is implemented here.

## Verification / continuation

At start Docker's Linux engine is unavailable. Do not claim Linux runtime,
Gazebo GUI, physical sensor or full autonomy validation unless executed.
Keep optional Perception gravity-based terrain risk disabled: planar EKF TF
does not establish real rover roll/pitch. Known landmark observations are not
surveyed-map anchoring. Diagnostics are not a controller stopping interlock.

Resume by reading this file, `git status`, and this branch's log. Complete the
unchecked tasks only; preserve unrelated edits. Update this record at milestones.

## Corrected audit finding

The current Gazebo launch rewrites its camera sensor joint into optical
coordinates and robot_state_publisher already owns that child. No extra bridge
is needed for the documented Gazebo workflow. An explicit fallback switch is
provided for external incomplete descriptions only; it defaults off. Earlier
claims that the standard Gazebo workflow lacked this TF were incomplete.

## Testing-plan merge handoff

The other documentation writer has finished its commits on local jazzy-port
(`8d5de8b`), but they are not on origin/jazzy-port at implementation time.
Do not silently include its three unrelated commits in this PR. After that
work merges, update TestingPlan.md section 9 with these exact requirements:

- Converter scope selects exactly one source: joint states, combined tick array
  or individual ticks. Test cadence mismatch, missing wheels, stale/future/
  duplicate samples, counter discontinuity, clock reset, stationary and turns.
- Slip scope preserves measured twist and cross-covariance while increasing
  uncertainty. Exercise fresh/stale/missing IMU, rotated sensor mounting,
  gravity conventions, effective yaw thresholds and clock reset recovery.
- Real master launch construction and stage/clock/frame/profile forwarding are
  prerequisites. Validate exclusive TF and publisher ownership; ROS absence
  must be skip/unavailable, not success.
- RTAB-Map consumes covariance-bearing /odometry/filtered. Verify coherent
  moving RGB-D, graph updates, landmark arrays on /perception/landmark_detections,
  adapter ID offset and legacy singular fixture separately. Judge global map
  pose separately from local odometry, with independent timestamp-aligned truth.
- Validate typed local/global RViz and the optional obstacle inspection costmap;
  box clouds mark, measured camera rays clear. Test memory TTL and real clearing
  together without double-publishing the bridge or synthetic observations.

The legacy testing README is marked as a proposal. The actual package guide
contains only commands implemented by this branch. No new benchmark/referee
or coherent moving sensor tester was added.

## Verification record — 2026-10-29

Implemented source and documentation are ready for review. Executed on the
available host (Python 3.13; target remains Linux/Jazzy):

- `python -m pytest SLAM/rover_slam/test testing/SLAM/slam_test_tools/test -q`
  with source PYTHONPATH, no bytecode/cache writes: **37 passed, 3 skipped**.
  Skips are entire ROS/optical-dependent modules: missing NumPy, ROS launch,
  and rclpy. They are not passing runtime checks.
- Parsed all **30 Python files** in the production/testing packages; parsed
  both XML manifests; checked package resource markers and setup.cfg files.
- Parsed every SLAM YAML and both RViz profiles; verified current guide links;
  `git diff --check` passed.
- Git Bash `bash -n scripts/launch_slam.sh` passed. Four isolated checks with
  a temporary mock ros2 executable verified package selection, argument/clock
  forwarding and invalid-option rejection. No ROS nodes ran in these checks.

Not executed: Linux colcon installation; ROS-dependent node/launch tests;
installed RealSense/RTAB-Map parameter acceptance; Gazebo moving RGB-D and
optical/IMU TF; native landmark graph optimization; RViz GUI; Nav2 lifecycle/
clearing; physical hardware or safe autonomous operation. Docker Linux engine
was unavailable. Do not interpret source completion as deployment approval.

### Linux continuation commands

Run from the repository root in an isolated environment with no rover hardware
attached. Install dependencies from the running guide first, then:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87
colcon --log-base /tmp/slam-review-log build --symlink-install \
  --base-paths SLAM/rover_slam testing/SLAM/slam_test_tools \
  --build-base /tmp/slam-review-build --install-base /tmp/slam-review-install
source /tmp/slam-review-install/setup.bash
python3 -m pytest SLAM/rover_slam/test testing/SLAM/slam_test_tools/test -q -rs
ros2 launch rover_slam slam_bringup.launch.py --show-args
```

Use fresh artifact paths if another review owns these directories. Follow the
package README's real Gazebo and integrated Perception sequences after these
checks. Record exact results here; do not infer map accuracy from stationary
mock inputs. TestingPlan.md reconciliation remains the separate merge handoff
above because its file is not in this branch's remote base.

## Delivery status

Feature branch `slam-jazzy-improvements` was pushed successfully to origin on
2026-10-29. Source implementation, guides and available static/pure checks are
complete; the Linux/runtime checks and TestingPlan.md merge handoff above remain
explicitly pending. Create a PR with base `jazzy-port`. Do not merge or claim
hardware readiness automatically. Existing local jazzy-port documentation
commits were preserved unchanged on that branch.
