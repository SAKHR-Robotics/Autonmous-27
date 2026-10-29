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

- [ ] Repair master launch; add encoder/slip standalone launches; scope flags,
      topic/frame overrides, clock and YAML profiles; fix interactive script.
- [ ] Select one encoder source; compute speeds from measurement intervals;
      stop valid-looking stale output; reject jumps and rebaseline on resets.
- [ ] Validate IMU age/values/frame; preserve measured wheel twist on slip;
      inflate covariance; remove implicit source fallback; publish diagnostics.
- [ ] Feed RTAB-Map EKF odometry with covariance; expose database/profile;
      correct RealSense settings and optional costmap configuration.
- [ ] Make TF ownership explicit; add opt-in Gazebo optical bridge; update
      local/global RViz and Perception integration documentation.
- [ ] Move EKF evaluation helper to testing; repair logging, clock, expected
      frequency and dropout monitoring; retain explicit compatibility launch.
- [ ] Rewrite SLAM and package READMEs: build, individual stages, standalone
      Gazebo with testing helpers, hardware, whole system and troubleshooting.
- [ ] Verify syntax/manifests/install resources, focused regression tests and
      actual launch construction where ROS exists. Record missing evidence.
- [ ] Review each milestone diff, make focused descriptive commits, push branch.

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
