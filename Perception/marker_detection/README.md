# marker_detection

ROS 2 Jazzy marker-perception package for an ERC (European Rover Challenge)
rover, built around the Intel RealSense D435 and OpenCV ArUco. It provides
**trustworthy observations, not just detections**: every marker pose carries
split confidence scores, a measurement covariance, and explicit quality flags
so a navigation/mission layer can make its own risk decisions instead of
trusting a single opaque number.

## Architecture / data flow

```
RGB + depth + CameraInfo (RealSense D435)
        |
        v
ArUco detection + 2D geometric validation         (Stage 1: aruco_detector.py, detection_validator.py)
        |
        v
2D image-quality gating (blur / contrast / geometry)  (image_quality.py)
        |
        v
RGB-D depth association (robust median + MAD)      (Stage 2: depth_processor.py, camera_model.py)
        |                                            RGB and depth are independently subscribed
        v                                            (sync_buffer.py); depth loss only skips this
Robust multi-candidate solvePnP                    (Stage 3: pose_estimator.py)     stage, never Stage 1.
        |
        v
Time-aware tracking + depth/PnP Kalman fusion       (Stage 4: tracker_manager.py, position_kalman.py, quaternion_filter.py)
        |
        +----------------------------------------------------+
        v                                                    v
Dynamic camera -> marker TF (Stage 5: marker_tf_broadcaster.py) Clean base_link marker interface (Stage 7:
        |                                                    marker_action_interface_node.py, frame_transform.py,
        v                                                    action_target_builder.py)
Global marker map via TF2 lookups (Stage 6: marker_map_node.py, its own node)      |
        |                                                                          v
        v                                                              /marker_detection/targets
/marker_map, /marker_poses, diagnostics
```

Stages 1-4 run in a single node, `marker_detection_node`, because they form
one tight per-frame pipeline; Stages 5, 6, and 7 are separate nodes because
they consume the tracked-pose *topic*, not internal pipeline state, and none
of them may block Stage 1-4 detection if TF, the map, or a downstream
consumer is unavailable. Stage 7 is a pure TF2 consumer exactly like Stage 6
(read-only lookups, no map/pose is synthesized on failure); it does not
depend on Stage 5 or Stage 6 running.

### Downstream-interface hardening pass (RGB/depth decoupling, base_link interface)

This pass makes two structural changes on top of everything below, both
additive and topic/message-preserving:

13. **RGB detection no longer depends on depth arriving** (`sync_buffer.py`,
    `marker_detection_node.py`). Previously, `message_filters.
    ApproximateTimeSynchronizer` fused RGB and depth *before* either stream
    reached the detection callback, so a depth stall silently stopped even 2D
    ArUco detection. RGB and depth are now subscribed to independently: every
    RGB frame runs 2D detection, image-quality gating, and monocular PnP
    unconditionally; `DepthSyncBuffer` finds the closest-in-time buffered
    depth frame (if any) within the existing `sync_slop` window for that
    frame's 3D association. When no match exists, depth-derived 3D position
    is marked unavailable via the existing `LOW_DEPTH_QUALITY` flag (reusing
    the same code path as a depth/RGB shape mismatch) -- it is never
    fabricated -- and 2D detections, `MarkerDetection`/`MarkerDetectionArray`,
    and PnP-only poses keep publishing normally. When depth resumes, the very
    next frame simply finds a match again; no explicit "resume" step exists
    anywhere in the pipeline. `sync_queue_size`/`sync_slop` keep their
    previous meaning and values.

14. **Clean, high-level marker interface for manipulation/planning**
    (`action_target_builder.py`, `frame_transform.py`,
    `marker_action_interface_node.py`, new `MarkerActionTarget`/
    `MarkerActionTargetArray` messages, topic `/marker_detection/targets`).
    A manipulation/planning node no longer needs to understand ArUco, depth
    filtering, PnP, tracking, covariance, or TF lookups: it subscribes to one
    topic and gets, per marker, `detected`, `stable`, `usable_for_action`,
    `tracking_state`, the pose in `base_link` (`pose_valid` +
    `position`/`orientation`), `distance_to_marker`/`forward_distance`/
    `lateral_distance`/`vertical_distance`, `confidence`, `quality_flags`,
    `position_covariance` (rotated into `base_link`), and `age_seconds`.
    `stable` and `usable_for_action` are derived entirely from the *existing*
    `TrackState`/`QualityFlag` systems -- no second tracking or quality system
    was introduced. The node performs exactly one TF2 lookup per track
    (`base_link <- <tracked pose's own frame>`, i.e. the RGB optical frame, at
    that pose's own timestamp) and never fabricates a `base_link` pose if the
    lookup fails: `pose_valid` becomes `false`, distances become `NaN`,
    `TF_UNAVAILABLE` is folded into `quality_flags`, and `usable_for_action` is
    forced `false`. See "Clean downstream marker interface" below for the
    exact `usable_for_action` conditions and the two new configurable
    thresholds it adds.

15. **RGB/depth synchronization diagnostics** (`diagnostics.py`,
    `/marker_detection/diagnostics`). In addition to the existing latency
    percentiles, the diagnostic status now reports `sync_delta_p50_ms`/
    `sync_delta_p95_ms`/`sync_delta_max_ms` (the RGB-depth timestamp-delta
    distribution over matched pairs), `frames_with_depth`/
    `frames_without_depth` counts, `camera_info_age_s`, and RGB liveness
    (`seconds_since_last_rgb_frame`) reported separately from depth-pair
    liveness (`seconds_since_last_sync_pair`) -- a depth stall alone no
    longer reads as "the whole pipeline is unhealthy" the way an RGB stall
    does, matching the decoupling above.

16. **Optional adaptive fallback detection pass** (`use_adaptive_multiscale_
    detection`, default `false`). When enabled, if the primary detection pass
    finds zero markers, a single secondary pass with CLAHE-enhanced
    preprocessing (reusing `image_processor.bgr_to_gray`'s existing CLAHE
    support, not a new algorithm) is tried; if it finds markers, those are
    used, otherwise the empty result stands. Never more than one extra pass
    per frame, and disabled by default so behavior is unchanged unless
    explicitly turned on.

**Deliberately not implemented in this sub-pass:** an optional temporal-ROI
detection optimization (track a marker's predicted pixel location and search
only a cropped region around it, falling back to full-frame detection
periodically for reacquisition) was considered but not implemented. Doing it
safely would mean projecting the Kalman-tracked 3D position back into pixel
space using the current camera intrinsics and changing which region of the
frame Stage 1 detection runs on every frame it's active -- a change to the
hot detection path itself, not an additive one, and one that needs on-hardware
validation of reacquisition behavior this environment cannot provide. Given
the explicit guidance to prefer correctness over optimization and skip
anything that risks destabilizing detection, this is left for a follow-up
pass on real hardware rather than implemented speculatively here.

### What changed in this hardening pass, and why

This is a targeted upgrade of an already-modular package, not a rewrite.
Nothing here changes topic names, TF direction/ownership, or Stage
boundaries; every existing message field is preserved and new fields are
additive. Breaking parameter changes (only in `marker_detection_node`'s
tracking parameters) are called out under **Migration notes** below.

1. **Robust planar pose selection** (`pose_estimator.py`). `IPPE_SQUARE` can
   return two valid solutions for a planar marker. Every candidate is now
   scored on reprojection quality, 2D corner geometry, agreement with the
   independently measured depth position, and agreement with the *previous*
   tracked pose (scaled by elapsed time). The highest-scoring valid candidate
   is kept, not simply the one with the lowest reprojection error. With no
   depth or previous-track context (a marker's first-ever detection) those
   terms are neutral and selection reduces to reprojection + geometry, so
   temporal disambiguation kicks in from the second detection onward as
   intended.

2. **Depth+PnP fusion** (`tracker_manager.py`, `position_kalman.py`). Rather
   than comparing a PnP pose and a depth pose and picking one, the tracked
   (Stage 4) position is a genuine two-measurement Kalman fusion: the full 3D
   PnP position updates the filter first (with its own derived covariance),
   then, if independent depth is available that frame, a dedicated *Z-only*
   partial update (`PositionKalman.update_axis`) folds in the depth sensor's
   own measurement noise. This is standard sequential-update Kalman theory,
   not an ad hoc average, and it correctly leaves X/Y untouched by the
   depth-only update. `MarkerTrackedPose.depth_fused` reports whether a given
   update used both sources. The Stage 3 raw-pose message (`/marker_poses_raw`)
   is unchanged and still reports PnP-only, so nothing downstream that reads
   the raw topic is affected.

3. **Pose covariance, not a constant**. `PoseEstimate.position_noise` (Stage
   3) and `TrackedPose.position_covariance` (Stage 4) are real 3x3
   covariances: lateral (X, Y) variance is back-projected pixel-localization
   error at the measured range; Z variance uses a configurable
   distance-proportional term (monocular planar-PnP range accuracy degrades
   with distance); depth variance comes from the median filter's MAD and
   sample count. Both are published in `MarkerPose.position_covariance` and
   `MarkerTrackedPose.position_covariance` (row-major 3x3, m^2).

4. **Split confidence types** (`quality_flags.py` values feed into
   `tracker_manager.py`). `detection_confidence` (2D detection quality:
   blur/contrast/geometry), `pose_quality` (3D candidate score from Stage 3),
   and `tracking_confidence` (temporal-track health: detection density in the
   confirmation window, freshness, filter-innovation stability) are now
   separate fields. `confidence` is kept for compatibility and is now
   `min(detection_confidence, pose_quality, tracking_confidence)` — a
   conservative choice: one weak link is enough to distrust an observation.

5. **Time-aware track confirmation** (`tracker_manager.py`). Confirmation and
   loss are judged against elapsed wall-clock time on the *measurement*
   timestamp, not a fixed frame count, so behavior no longer depends on
   camera FPS. States are `NEW -> CONFIRMED -> DEGRADED -> LOST`: a track
   needs `min_confirmations` detections inside a rolling
   `confirmation_window_seconds`; a single missed frame within
   `degraded_after_seconds` keeps a `CONFIRMED` track `CONFIRMED`; beyond that
   but within `lost_timeout_seconds` it becomes `DEGRADED`; beyond that it is
   `LOST` (and deleted only after `track_timeout_seconds`). Reacquisition of
   the same ID reuses the same track object and requires a fresh confirmation
   window, not an instant snap back to `CONFIRMED`.

6. **Image-quality gating** (`image_quality.py`), independent of pose/depth:
   Laplacian-variance blur, grey-level contrast, quadrilateral convexity, and
   side-length consistency, evaluated inside the marker's interior. Below a
   hard floor a detection is rejected outright; between the floor and a
   "good" threshold it is kept but flagged `LOW_IMAGE_QUALITY`.

7. **Explicit quality flags** (`quality_flags.py`), a `uint32` bitmask on
   every detection/pose/tracked-pose message:
   `LOW_DEPTH_QUALITY, LOW_IMAGE_QUALITY, LOW_MARKER_RESOLUTION,
   HIGH_VIEWING_ANGLE, HIGH_REPROJECTION_ERROR, DEPTH_MISMATCH,
   TEMPORAL_OUTLIER, POSE_AMBIGUOUS, STALE_DATA, TRACK_DEGRADED, TRACK_LOST,
   TF_UNAVAILABLE, MULTIPLE_INSTANCES_SAME_ID`. A mission layer can act on the
   specific reason rather than inferring it from a single scalar.

8. **Viewing angle** (`PoseEstimator.viewing_angle_deg`): angle between the
   marker's surface normal and the camera's optical axis, 0=fronto-parallel,
   90=edge-on. Hard-rejected beyond `max_viewing_angle_deg`, flagged beyond
   `high_viewing_angle_warn_deg`.

9. **Per-marker physical size** (Improvement #21): `marker_size_m` remains
   the default; `use_per_marker_sizes` + parallel `per_marker_size_ids` /
   `per_marker_size_values` arrays override it for specific IDs. A
   `PoseEstimator` instance is cached per unique size value, not per marker,
   to avoid redundant objects.

   `marker_size_m` defaults to **0.21 m**, matching the current competition
   marker set: `aruco.zip`'s cube models (`model.sdf`) each place a
   0.21 x 0.21 m marker face on the head box, and this is the side length
   `marker_size_m` must equal (the full printed square used by PnP, not the
   cube's overall dimensions or any inner code-only region). `aruco_dictionary`
   defaults to **`DICT_4X4_50`**: every marker in `aruco.zip` (IDs 1-15) was
   verified to decode uniquely under that dictionary. Both defaults must stay
   identical across `marker_detection_node.py`'s declared defaults,
   `config/marker_detection.yaml`, `scripts/generate_aruco_marker.py`, and the
   simulation models -- there is intentionally only one place that says "0.21"
   and one place that says "DICT_4X4_50" that a human is expected to edit
   (this YAML file); everywhere else exists only as a matching default so a
   node launched with no params file still behaves correctly.

9a. **Corner refinement** (`cornerRefinementMethod` / `Win­Size` /
   `MaxIterations` / `MinAccuracy`): previously declared nowhere, so OpenCV
   silently ran with `CORNER_REFINE_NONE` regardless of intent. Defaults to
   `CORNER_REFINE_SUBPIX`, the standard choice for reducing corner-localization
   jitter (and therefore PnP/distance jitter); declared as a readable string
   name (translated in `aruco_detector.resolve_corner_refinement_method`) the
   same way `aruco_dictionary` is, rather than an opaque OpenCV integer.
   `test_aruco_detector.py::test_corner_refinement_parameters_are_actually_applied_to_opencv`
   asserts the values reach `cv2.aruco.DetectorParameters`, not just the ROS
   parameter server.

10. **Competition vs development mode** (Improvement #20):
    `competition_mode: true` disables all `*_debug_image` publishers and
    drops periodic stats logging to `DEBUG`, without touching any detection,
    pose, fusion, or tracking threshold. See `config/marker_detection_competition.yaml`.

11. **Diagnostics** (`diagnostics.py` + `/marker_detection/diagnostics`,
    `diagnostic_msgs/DiagnosticArray`, throttled to `diagnostic_rate_hz`):
    processing FPS, P50/P95/P99/max latency, seconds since the last
    synchronized RGB/depth pair, and rolling counts of detections/rejections/
    track states. The percentile and health-level logic lives in
    `diagnostics.py`, kept free of ROS imports so it is unit-testable without
    a ROS environment.

12. **dt-aware outlier rejection** (`tracker_manager.py`): the existing
    Mahalanobis gate (whose covariance already grows with `dt` via the Kalman
    predict step) is combined with an explicit
    `position_jump_tolerance_m + max_linear_velocity_mps * dt` jump budget, so
    a given absolute jump is judged against how much motion the elapsed time
    could plausibly explain.

### Deliberately not implemented in this pass, and why

- **Full joint 6-DoF (non-diagonal) measurement covariance / EKF.** The
  fusion above uses axis-independent (diagonal) covariances. RGB
  reprojection noise and RealSense depth noise are physically close to
  independent, so diagonal fusion is a defensible, real-time-cheap
  approximation; a fully joint EKF would add real complexity for limited
  accuracy gain here. Flagged as a natural follow-up if trajectory smoothness
  under fast rover motion turns out to need it.
- **Marker-surface plane fitting** in Stage 2. The existing robust
  median/MAD interior-pixel sampling already meets the "real-time capable,
  robust marker-center estimate" requirement; plane fitting adds cost for a
  center-position estimate that would not materially change.
- **Bag-replay test harness / recorded-dataset evaluation.** No ROS bags or
  recorded RealSense data were available in this environment to build or
  validate one against. The pipeline's synchronization, message shapes, and
  QoS are unchanged, so existing bags should still replay; a `ros2 bag play`
  + `ros2 bag record` workflow against the topics below is the recommended
  next step on real hardware.
- **CPU/RAM and on-hardware FPS/latency numbers.** No ROS 2 install or
  physical D435 was available in this sandbox (see Testing report below).

### Engineering audit pass (dictionary/marker-size ground truth, duplicate IDs, topics)

A follow-up audit cross-checked this package against `aruco.zip` (the
project's physical/simulation marker models) and fixed the following
config/code mismatches. None of these change the pipeline's algorithms;
they correct defaults and one tracking bug so the existing algorithms run
against the right physical constants and the right observations.

- **Dictionary and marker size were re-derived from `aruco.zip`, not
  assumed.** Every marker texture (`aruco_1`..`aruco_15`) was decoded with
  OpenCV: all 15 detect uniquely under `DICT_4X4_50` with IDs matching their
  filenames. Each cube's `model.sdf` places a 0.21 x 0.21 m marker face on
  the head box. `marker_detection_node.py`'s *declared parameter defaults*
  previously disagreed with `config/marker_detection.yaml` on both counts
  (`DICT_6X6_250`/`0.10 m` in code vs. `DICT_4X4_50`/`0.21 m` in YAML) --
  code defaults now match the YAML and the physical models everywhere:
  the node, the YAML, and `scripts/generate_aruco_marker.py`.
- **RealSense topic defaults.** `config/marker_detection.yaml` and the
  `rgb_topic`/`depth_topic`/`camera_info_topic` launch-argument defaults in
  `marker_detection.launch.py`/`marker_branch.launch.py` pointed at generic
  `/camera/image_raw`-style topics that do not exist on a real D435 driver
  and are not resolution/frame-compatible with RGB-based detection (raw
  depth vs. depth aligned to the color optical frame). Both now default to
  `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, and
  `/camera/color/camera_info`, matching the node's own Python defaults (which
  were already correct) and the README's usage examples (which were also
  already correct -- only the YAML/launch defaults were wrong).
- **`use_sim_time` defaulted to `true` in every launch file**, including
  the ones with no simulation counterpart in this package -- a real-hardware
  competition run that forgot to pass `use_sim_time:=false` would silently
  run on simulation time. All launch files now default to `false`; pass
  `use_sim_time:=true` explicitly for a Gazebo run.
- **Critical: duplicate-same-ID overwrite bug** in
  `TrackerManager.process()` (`tracker_manager.py`). The measurement-selection
  line was `observations = {item.marker_id: item for item in measurements}`,
  a plain dict comprehension that silently keeps whichever detection is
  *last* in OpenCV's per-frame detection order when the same ID is seen on
  two physical faces at once (legitimate on a marker cube) -- not the best
  one, and not a stable choice frame-to-frame, which could make the global
  marker map (Stage 6) jitter between two different physical locations under
  one ID. Replaced with `TrackerManager._select_observations` /
  `_candidate_rank_key`: a fixed, value-based ranking (valid pose > valid
  depth > lower viewing angle > lower reprojection error > the existing
  composite `pose_quality` > `detection_confidence`, with the 3D position
  itself as a final deterministic tiebreak) that never depends on input
  order, plus a new `MULTIPLE_INSTANCES_SAME_ID` quality flag so downstream
  consumers can see this happened rather than it being silent. The losing
  candidate(s) are neither averaged in (which would fabricate a pose between
  two different physical faces) nor tracked separately (per ID, one logical
  track). 8 new tests in `test_tracking.py` cover two/three simultaneous
  candidates, both possible input orders producing the same winner,
  invalid-beats-high-score, appear/disappear across frames, and the
  depth-quality tiebreak.
- **Corner refinement was declared nowhere**, so OpenCV always ran with its
  own default (`CORNER_REFINE_NONE`) regardless of any intent to configure
  it. Added `cornerRefinementMethod` (default `CORNER_REFINE_SUBPIX`),
  `cornerRefinementWinSize`, `cornerRefinementMaxIterations`,
  `cornerRefinementMinAccuracy` to both the node and
  `config/marker_detection.yaml`, and a test
  (`test_corner_refinement_parameters_are_actually_applied_to_opencv`) that
  asserts the values land on `cv2.aruco.DetectorParameters`, not just the ROS
  parameter server.
- **Startup self-check** (`MarkerDetectionNode._log_startup_self_check`):
  logs the topics, dictionary, marker size, corner-refinement method, depth
  scale/range, and `use_sim_time` that actually took effect, once, at node
  startup -- so a configuration mismatch is visible in the log immediately
  rather than discovered later from bad poses. `CameraInfo`/TF readiness are
  covered by the existing periodic diagnostics instead, since neither is
  known synchronously at construction time.
- **Not changed, and confirmed not bugs on inspection:** `min_depth_m`/
  `max_depth_m` (raw per-pixel depth-sample acceptance range, `depth_processor.py`)
  and `min_marker_depth_m`/`max_marker_depth_m` (valid range for the PnP-derived
  `tvec.z`, `pose_estimator.py`) are two intentionally distinct concepts that
  happen to share the same numeric defaults, not a duplicate parameter; the
  depth-scale conversion (`DepthProcessor.to_meters`) applies `depth_scale`
  exactly once, only to `16UC1`, never to `32FC1`.

### Consistency/hardening pass (debug-image dedup, camera-frame Z, CameraInfo freshness)

A third, narrowly-scoped pass, building on the audit above, without touching
the ArUco dictionary, marker size, same-ID selection algorithm, or tracking
architecture:

- **`/marker_detection/debug_image` now draws the canonical (same-ID
  deduplicated) observations, not the raw pre-dedup list.** Previously
  `_publish_debug` drew from `markers: list[ValidatedMarker]`, the 2D
  detections straight out of the quality gate -- before `TrackerManager`'s
  same-ID candidate selection ever runs. If two faces of a cube shared an ID
  in one frame, the debug image showed both, even though tracking had
  already picked one. `_track_poses` now also calls a new pure helper,
  `tracker_manager.select_canonical_with_source(measurements, sources)`,
  which wraps `TrackerManager.select_canonical` (the exact same static
  method `process()` uses) to recover which `AssociatedPose` (2D corners +
  quality) each canonical `RawPose` pick came from, by object identity. This
  is the same selection algorithm called from one more place, not a second,
  independent reimplementation -- `select_canonical` itself is unchanged.
  `_publish_debug`'s signature changed from `markers: list[ValidatedMarker]`
  to `poses: list[AssociatedPose]` accordingly; the topic, message type, and
  drawing style (boxes + ID label) are unchanged.
- **The primary debug image now shows camera-frame Z** (`Z: 1.42 m`) next to
  each marker, preferring the PnP-derived Z and falling back to the
  depth-sensor Z if PnP is invalid, via the new `_camera_frame_z_m` helper.
  This is explicitly labeled `Z:`, not `Distance:`, to avoid ever being
  confused with `distance_to_marker` -- the base_link Euclidean distance on
  `/marker_detection/targets`, which remains computed exactly as before, in
  `marker_action_interface_node.py`/`action_target_builder.py`, and remains
  the only authoritative rover-relative distance. No RViz/visualization
  component for `distance_to_marker` was added in this pass (see "Not done"
  below).
- **CameraInfo freshness is now enforced, not just reported.** A new
  `camera_info_max_age_s` parameter (default 5.0 s, matching this file's
  existing RGB/depth staleness convention) is checked once per frame in
  `_estimate_poses`: if the most recently received CameraInfo is older than
  this, it is treated as absent for that frame's pose estimation only (2D
  detection is entirely unaffected) and the existing `STALE_DATA` quality
  flag is set -- no new parallel quality mechanism was introduced. The
  decision itself is a new pure function, `diagnostics.camera_info_is_stale(age_s,
  max_age_s)` (`age_s=None` means never received, which is a distinct,
  already-correctly-handled case and is never "stale"), independently unit
  tested for fresh/stale/missing/boundary in `test_diagnostics.py`. A
  `camera_info_stale` diagnostics KeyValue and a one-shot warning log were
  added; `camera_info_age_s` (already existed) is unchanged.
- **The leftover misleading `per_marker_size_values: [0.10]` in
  `config/marker_detection.yaml` was fixed to `[0.21]`.** The previous audit
  pass had already fixed the *node's* Python default for this value but
  missed the YAML, which still shipped a non-0.21 placeholder even though
  `use_per_marker_sizes: false` makes it inert; the comment block above it
  was also expanded to state plainly that all current competition markers
  are 0.21 m and that toggling the flag alone (without editing the arrays)
  is a no-op, not a silent scale error. `_build_marker_size_overrides()`
  itself (which already returns `{}` whenever the flag is false) was not
  changed -- the runtime guarantee that a different size cannot silently
  take effect already existed in code; only the YAML/comments were misleading.
- **Not done, and left for a future pass:** an RViz/visualization component
  specifically for `distance_to_marker` (Priority 2 Requirement B suggested
  this only "if practical" -- this pass avoided adding a new visualization
  component to stay inside the "clean up, don't start a new dev cycle"
  instruction; `/marker_detection/targets` remains directly inspectable via
  `ros2 topic echo` in the meantime). `_publish_pose_debug` and
  `_publish_3d_debug` (the two other debug image topics) were intentionally
  left drawing from their existing, non-deduplicated inputs -- only
  `/marker_detection/debug_image` ("the primary debug image") was in scope.

## Build and run on Ubuntu 24.04 / ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select marker_detection
source install/setup.bash
```

---

## 🧪 Standalone Testing Guide in Simulation (World + Rover + Teleop + ArUco)

You can validate ArUco detection, 6-DoF pose estimation, and target generation in isolation with the simulated rover:

### Method A: Interactive Launcher (Fastest)
```bash
bash scripts/launch_perception.sh
# Select Option 3 (ArUco Tag Marker Detection Only) or Option 1 (Full Perception)
```

---

### Method B: Manual Step-by-Step Terminal Playbook

#### Step 1: Launch Mars Yard World & Spawn Rover
Open Terminal 1:
```bash
source install/setup.bash
# When testing standalone without SLAM, pass publish_map_tf:=true to provide map -> odom
ros2 launch my_robot_description gazebo.launch.py publish_map_tf:=true
```

#### Step 2: Launch Teleoperation GUI
Open Terminal 2:
```bash
source install/setup.bash
ros2 run my_robot_description teleop_gui.py
```
*(Drive the rover to face ArUco marker posts in Mars Yard simulation).*

#### Step 3: Launch Marker Detection Node
Open Terminal 3:
```bash
source install/setup.bash
ros2 launch marker_detection marker_detection.launch.py
```

#### Step 4: Verification Commands
Open Terminal 4:
```bash
# 1. Verify 2D and 3D detection streams
ros2 topic echo /marker_detections
ros2 topic echo /marker_poses

# 2. Verify high-level target output for navigation & manipulation
ros2 topic echo /marker_detection/targets

# 3. View visual debug stream with bounding boxes and estimated Z distance
rqt_image_view /marker_detection/debug_image
```

---

## Hardware Deployment with Physical RealSense D435

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_sync:=true align_depth.enable:=true
```

In a second terminal (after sourcing the overlay), inspect the actual graph and start the branch:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo --once /camera/color/camera_info

# Recommended: launch all six stages (detection+tracking, TF, global mapping) together.
ros2 launch marker_detection marker_branch.launch.py

ros2 topic echo /marker_detections
ros2 topic echo /marker_3d_detections
ros2 topic echo /marker_poses
ros2 topic echo /marker_poses_raw
ros2 topic echo /marker_map
ros2 topic echo /marker_detection/targets
ros2 topic echo /marker_detection/diagnostics
rqt_image_view /marker_detection/debug_image
rqt_image_view /marker_detection/debug_3d_image
rqt_image_view /marker_detection/pose_debug_image
rqt_image_view /marker_tracking/debug_image
```

`marker_branch.launch.py` composes the three per-stage launch files below (it
does not start the RealSense driver). Launch stages individually only if you
need to run a subset, e.g. while iterating on one stage in isolation.

If the RealSense namespace is `/camera/camera/...`, pass actual topic names:

```bash
ros2 launch marker_detection marker_detection.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

Generate a printable marker with the configured default dictionary:

```bash
ros2 run marker_detection generate_aruco_marker.py --id 0 --size 600
```

### Competition mode

For a competition run, launch with the competition profile layered on top of
the base config (`ros2 launch` does not merge multiple `--params-file`
values from different files onto the same node in older Jazzy releases —
merge the two YAML files into one before the run, or pass the merged result):

```bash
python3 - <<'PY'
import yaml
base = yaml.safe_load(open("src/marker_detection/config/marker_detection.yaml"))
override = yaml.safe_load(open("src/marker_detection/config/marker_detection_competition.yaml"))
base["marker_detection"]["ros__parameters"].update(override["marker_detection"]["ros__parameters"])
yaml.safe_dump(base, open("/tmp/marker_detection_competition_merged.yaml", "w"))
PY
ros2 launch marker_detection marker_branch.launch.py \
  --ros-args --params-file /tmp/marker_detection_competition_merged.yaml
```

Competition mode disables debug-image publishing and drops periodic stats
logging to `DEBUG`; it does not relax any detection, pose, fusion, or
tracking threshold. Diagnostics (`/marker_detection/diagnostics`) stay on —
they are a handful of scalars per second, not images.

### Per-marker sizes

If your printed markers are not all the same physical size:

```yaml
use_per_marker_sizes: true
per_marker_size_ids: [7]
per_marker_size_values: [0.15]   # marker 7 is 15cm; every other ID uses marker_size_m
```

## Confidence, covariance, and quality flags

Every `MarkerPose` (Stage 3) and `MarkerTrackedPose` (Stage 4) carries:

| Field | Meaning |
| --- | --- |
| `detection_confidence` | 2D-detection reliability (blur, contrast, geometry), `[0,1]` |
| `pose_quality` | 3D-pose reliability (reprojection, viewing angle, depth/temporal agreement), `[0,1]` |
| `tracking_confidence` *(TrackedPose only)* | Temporal-track health, `[0,1]` |
| `confidence` *(TrackedPose only)* | `min()` of the three above — the conservative overall score |
| `viewing_angle_deg` | Angle between the marker normal and the camera optical axis; 0=best, 90=worst |
| `distance_m` *(TrackedPose only)* | `\|filtered position\|` |
| `quality_flags` | `uint32` bitmask, see `marker_detection/quality_flags.py` |
| `position_covariance` | Row-major 3x3 (m^2); all-zero means "not computed" |
| `depth_fused` *(TrackedPose only)* | Whether this update included a depth-Z Kalman fusion step |

Decode flags for logging/debugging with:

```python
from marker_detection.quality_flags import names
names(message.quality_flags)  # -> e.g. ['HIGH_VIEWING_ANGLE', 'LOW_MARKER_RESOLUTION']
```

Important parameters are in `config/marker_detection.yaml`, grouped and
commented by improvement area (candidate scoring weights, covariance
derivation, image-quality gates, time-aware tracking, competition/development
mode, diagnostics). Every parameter the node declares appears there with its
default; nothing is hardcoded in Python.

Step 5 runs as `marker_tf_broadcaster`. It consumes only `/marker_poses` and
uses each valid pose's own header frame and timestamp to dynamically
broadcast `camera_optical_frame -> aruco_marker_<ID>`. Translation and
normalized quaternion are used directly as the Stage 4 `T_camera_marker`
pose; they are not inverted or axis-flipped. By default only `CONFIRMED`
tracks broadcast TF; a `DEGRADED` track with a fresh detection this frame is
also eligible when `publish_only_confirmed` is disabled, but a *predicted*
(undetected) `DEGRADED`/`LOST` track is only broadcast if
`publish_predicted_lost` is enabled. Set `expected_parent_frame` only after
inspecting the live RealSense TF tree if you want strict parent-frame checking.

```bash
ros2 launch marker_detection marker_tf.launch.py
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo <rgb_optical_frame> aruco_marker_17
```

Step 6 runs as a separate `marker_map_node`. It consumes `/marker_poses`,
then uses the existing Step 5 TF tree to look up `map_frame ->
aruco_marker_<ID>` at the original observation timestamp, weighting fusion by
each observation's `tracking_confidence`. It does not synthesize a
transform, manually multiply camera transforms, or publish any `map ->
marker` TF. If global TF is missing or extrapolated, the observation is
rejected and no false global pose is created. Valid map-frame observations
are fused with a per-ID static constant-position Kalman estimator and
quaternion SLERP. The in-memory map is published on `/marker_map`; optional
RViz `MarkerArray` output is published on `/marker_map/visualization`. Call
`/marker_map/reset` (`std_srvs/Empty`) to clear only the map, never the
Stage 4 tracker.

```bash
ros2 launch marker_detection marker_mapping.launch.py
ros2 topic echo /marker_map
ros2 service call /marker_map/reset std_srvs/srv/Empty '{}'
```

## Clean downstream marker interface

Step 7 runs as a separate `marker_action_interface_node`. It consumes only
`/marker_poses` (the existing Stage 4 output) and, for each track, performs
exactly one read-only TF2 lookup -- `base_link <- <the pose's own header
frame>` (the RGB optical frame), at that pose's own timestamp -- then
composes it with the pose's existing camera-frame position/orientation/
covariance (`frame_transform.transform_to_base_link`). It broadcasts no TF of
its own and never fabricates a `base_link` pose: if the lookup fails, the
corresponding fields are marked invalid instead. Results publish as
`MarkerActionTarget` on `/marker_detection/targets`:

| Field | Meaning |
| --- | --- |
| `detected` | Mirrors the Stage 4 track's `detection_valid` this update |
| `stable` | `true` only when `tracking_state == "CONFIRMED"` -- the *existing* Stage 4 state machine, not a new one |
| `pose_valid` | `true` only if the `base_link` TF2 lookup succeeded; `position`/`orientation`/all three distances are meaningless when `false` |
| `position`, `orientation` | Marker pose in `base_link` |
| `distance_to_marker` | `sqrt(x^2+y^2+z^2)` of `position` above |
| `forward_distance`, `lateral_distance`, `vertical_distance` | `position.x`, `.y`, `.z` in `base_link` (REP-103: +X forward, +Y left, +Z up) |
| `usable_for_action` | `true` only if `pose_valid`, `stable`, `confidence >= min_action_confidence`, the resolved covariance trace is `<= max_action_covariance_trace_m2`, and none of `TRACK_LOST`/`TRACK_DEGRADED`/`TEMPORAL_OUTLIER`/`POSE_AMBIGUOUS`/`TF_UNAVAILABLE`/`STALE_DATA` are set -- all existing flags, reused as-is |
| `confidence`, `quality_flags`, `position_covariance`, `age_seconds` | Same meaning as the Stage 4 fields, with covariance rotated into `base_link` |

`min_action_confidence` and `max_action_covariance_trace_m2` (see
`config/marker_action_interface.yaml`) are the only two new configurable
thresholds this interface adds; every other condition above reuses existing
tracking/quality state verbatim.

```bash
ros2 launch marker_detection marker_action_interface.launch.py
ros2 topic echo /marker_detection/targets
```

## Testing

Unit tests cover detection, image-quality gating, depth validation, pose
estimation/reprojection/candidate-scoring/covariance, quaternion filtering,
depth+PnP Kalman fusion, time-aware tracking lifecycle, quality-flag bit
operations, diagnostics percentile/threshold logic (including the new
RGB-depth sync-delta distribution), the RGB/depth matching buffer, the
camera->base_link frame composition, the clean downstream marker-interface
builder (including an integration-level test through tracking -> TF
composition -> `usable_for_action`), TF construction/rejection, and
global-map fusion/outlier rejection (see `test/`). Run them either directly:

```bash
pytest src/marker_detection/test
```

or as part of the package build:

```bash
colcon test --packages-select marker_detection
colcon test-result --verbose
```

Tests that only need Python/NumPy/OpenCV (detector, validator, depth
processor, Kalman filter, quaternion filter, tracker, marker map manager,
image quality, quality flags, diagnostics, the RGB/depth sync buffer, the
frame-transform composition, and the action-target builder) run without a
ROS environment. Tests that construct ROS messages or nodes (`camera_model`,
`pose_estimator`, `marker_tf_broadcaster`) require `sensor_msgs`/`rclpy`/the
generated `marker_detection.msg` Python classes, i.e. a sourced ROS 2 Jazzy
environment.

### What was actually executed in this environment

This sandbox has no ROS 2 install (`rclpy`, `sensor_msgs`, `diagnostic_msgs`,
`tf2_ros`, and the `marker_detection.msg` generated classes are all
unavailable), so `colcon build`/`colcon test` could not run. What *was*
executed and passed:

- `py_compile` on every Python file in the package: **clean**.
- `pyflakes` static analysis on the whole tree: **clean** (no unused imports,
  no undefined names).
- Cross-checks (via `ast`/`yaml` parsing, not just eyeballing) that every
  parameter each node declares (`marker_detection_node`,
  `marker_action_interface_node`) appears in its corresponding YAML config
  with no orphans in either direction, and that every keyword argument
  `PoseEstimator`/`TrackerManager` accept is exactly the set the node passes,
  and that the `msg/` directory and `CMakeLists.txt`'s
  `rosidl_generate_interfaces` list match exactly: **exact match** in every
  case.
- **103 of 103** runnable unit tests (up from 93 after the previous
  engineering-audit pass: +2 new tests in `test_tracking.py`
  (`select_canonical_with_source` giving the debug image the same pick as
  tracking, and its length-mismatch guard) and +4 in `test_diagnostics.py`
  for `camera_info_is_stale` (fresh/stale/missing/boundary), covering this
  pass's debug-image-consistency and CameraInfo-freshness changes. 109 total
  test functions in `test/`, of which only the 6 in
  `test_marker_tf_broadcaster.py` need real `rclpy`/`tf2_ros` and could not
  run here (`test_action_interface_integration.py`'s 4 tests, previously
  listed here as also needing real `rclpy`, do not -- they exercise pure
  functions directly per their own module docstring, and were confirmed
  passing alongside everything else with the same `sensor_msgs` stub used
  for `test_camera_model.py`/`test_pose_estimator.py`), using a minimal local
  stub for
  `sensor_msgs.msg.CameraInfo`/`Image`, `std_msgs.msg.Header`,
  `geometry_msgs.msg.Point`/`Quaternion`/`Pose`, and
  `builtin_interfaces.msg.Time` (data-only classes, no ROS runtime behavior):
  **all pass**.
- `test_marker_tf_broadcaster.py` requires real `rclpy`/`tf2_ros`/generated
  messages and could not be executed here; it is included, unchanged by this
  pass, for a real ROS 2 environment to run and was validated by static
  analysis only. `marker_action_interface_node.py` itself (the ROS wiring
  around the newly-tested pure functions) equally requires real
  `rclpy`/`tf2_ros` and was validated by static analysis and the parameter/
  config cross-check above, not by execution.

**Not executed, and require a real ROS 2 Jazzy environment:** `colcon build`,
`colcon test`, `ros2 launch`, and everything under Performance report below.

## Performance report

No physical D435 or ROS 2 runtime was available in this sandbox, so no FPS,
latency, CPU, or RAM numbers are reported here — reporting fabricated numbers
would be worse than reporting none. `diagnostics.py`/`/marker_detection/diagnostics`
give you P50/P95/P99/max latency and FPS live once running on real hardware;
`latency_warn_p95_ms` / `latency_error_p95_ms` are a starting point to tune
against your actual measurements, not validated figures.

## RViz2 setup

```bash
rviz2
```

Then add:

- **TF** — to see `camera_color_optical_frame -> aruco_marker_<ID>` (Step 5)
  and the full rover tree.
- **MarkerArray** on `/marker_map/visualization` — persistent global marker
  poses and labels (Step 6). Set the Fixed Frame to `map`.
- **Image** on `/marker_detection/debug_image`, `/marker_detection/debug_3d_image`,
  `/marker_detection/pose_debug_image`, or `/marker_tracking/debug_image` for
  per-stage visual debugging (disabled in `competition_mode`).

## Expected TF tree

```
existing rover TF tree
          |
          v
camera_color_optical_frame (or actual RGB optical frame from RealSense TF)
      |
      +---- aruco_marker_1
      |
      +---- aruco_marker_7
      |
      +---- aruco_marker_17
```

Step 5 is the only owner of `camera -> aruco_marker_<ID>`. Step 6 never
publishes `map -> aruco_marker_<ID>`; it only performs read-only TF2 lookups
through the existing rover localization tree and reports results on
`/marker_map`. Step 7 (`marker_action_interface_node`) similarly never
publishes any TF of its own; it only looks up `base_link <- <RGB optical
frame>` and reports results on `/marker_detection/targets`.

## Failure handling

| Situation | Behavior |
| --- | --- |
| RGB disappears | `_report_sync_health` warns (throttled to once/5s) once >5s without an RGB frame at all; diagnostics `seconds_since_last_rgb_frame` climbs and the diagnostic level moves WARN -> ERROR. |
| Depth disappears (RGB keeps arriving) | 2D ArUco detection, image-quality gating, and monocular PnP **keep running and publishing** every RGB frame (Part 2 fix) -- they are no longer gated on depth at all. `DepthSyncBuffer.match()` simply returns no match; 3D depth-derived position is marked unavailable via the existing `LOW_DEPTH_QUALITY` flag (never fabricated); tracking continues off PnP alone (`depth_fused=false`). `seconds_since_last_sync_pair` climbs independently of `seconds_since_last_rgb_frame` and is reported, but does not by itself move the overall diagnostic level to ERROR the way an RGB stall does. |
| Depth returns after a stall | The very next RGB frame simply finds a depth match again; 3D association/PnP-depth fusion resume automatically, no explicit "resume" step anywhere. |
| CameraInfo disappears / never arrives | 2D detection and `/marker_detections` keep publishing; pose estimation is skipped (no valid camera matrix) and a one-time warning is logged; tracks age out via the normal DEGRADED/LOST path; `camera_info_age_s` in diagnostics reports `nan` until one is received. |
| TF unavailable (Stage 6) | The specific observation is dropped with a rate-limited warning; the map entry is left untouched (not deleted, not moved); Stage 1-4 detection is completely unaffected — mapping never blocks detection. |
| TF unavailable (Stage 7, `base_link <- camera`) | That marker's `pose_valid` becomes `false`, all three distance fields become `NaN`, `TF_UNAVAILABLE` is folded into `quality_flags`, and `usable_for_action` is forced `false` — no `base_link` pose is fabricated; Stage 1-4 detection and Stage 5/6 are completely unaffected. |
| Marker partially occluded | Fewer than 4 corners -> ArUco simply does not report that marker; a temporarily-undetected `CONFIRMED` track survives the `degraded_after_seconds` grace period unchanged. |
| Marker too small | Detection-level `min_marker_size_px`/`min_marker_area_px` reject it outright (Stage 1); if it passes those but is under `min_pose_quality_marker_size_px`, it is kept but flagged `LOW_MARKER_RESOLUTION`. |
| Marker blurred | Below `min_blur_score_hard`/`min_contrast_score_hard`: rejected before depth/pose ever run. Between that and the "good" threshold: kept but flagged `LOW_IMAGE_QUALITY`. |
| Depth invalid at the marker | `DepthEstimate.valid=False`; the position falls back to PnP-only (no depth fusion that frame, `depth_fused=False`) and `LOW_DEPTH_QUALITY` is flagged. |
| PnP candidates ambiguous | Both scored candidates are kept close in score (within `ambiguity_score_margin`); the higher-scoring one is used but `POSE_AMBIGUOUS` is flagged so a consumer can discount it. |
| Marker suddenly jumps | Rejected by the combined Mahalanobis + dt-scaled jump-budget gate (`TEMPORAL_OUTLIER`); track state is unaffected by a single rejected measurement inside the grace period. |
| Marker disappears | Track transitions `CONFIRMED -> DEGRADED -> LOST` purely by elapsed time, never deleted before `track_timeout_seconds`; TF broadcast for `LOST`/undetected `DEGRADED` stops unless `publish_predicted_lost` is set. |
| Tracking lost | `TrackedPose.state == LOST`, `TRACK_LOST` flagged, `tracking_confidence` decays toward 0 via the freshness term; reacquisition of the same ID reuses the track and requires a fresh confirmation window before returning to `CONFIRMED`. |

## Migration notes (breaking parameter changes)

Time-aware tracking (Improvement #5) replaces frame-counted confirmation and
loss. If you have an existing parameter override file:

- **Removed**: `max_missed_frames`, `confirmation_gain`, `miss_penalty`,
  `rejection_penalty` (the old running-score confidence accumulator is
  replaced by the measured `detection_confidence` / `pose_quality` /
  `tracking_confidence` composite described above).
- **Added, required**: `confirmation_window_seconds`, `degraded_after_seconds`,
  `lost_timeout_seconds`. Must satisfy
  `0 < degraded_after_seconds <= lost_timeout_seconds <= track_timeout_seconds`
  (the node validates this at startup and raises a clear error otherwise).
- **Changed default**: `track_timeout_seconds` moved from `1.0` to `5.0` to
  stay consistent with the new `lost_timeout_seconds` default of `1.5`.

Everything else (topic names, TF frames, message field names) is unchanged
and additive-only.

### This downstream-interface hardening pass specifically

Fully additive, no removals or renames:

- `message_filters` is no longer a runtime dependency of
  `marker_detection_node` (removed from `package.xml`); RGB/depth are now
  subscribed independently. `rgb_topic`/`depth_topic`/`camera_info_topic`/
  `sync_queue_size`/`sync_slop` parameters are unchanged in name and meaning.
- `MarkerTrackedPose.msg` gained one new field, `age_seconds` (appended at
  the end); existing fields and their order are unchanged.
- New, optional: `use_adaptive_multiscale_detection` parameter (default
  `false`, zero behavior change unless enabled).
- New: `MarkerActionTarget`/`MarkerActionTargetArray` messages,
  `marker_action_interface_node` executable, `/marker_detection/targets`
  topic, `config/marker_action_interface.yaml`,
  `launch/marker_action_interface.launch.py` (also included from
  `marker_branch.launch.py`). None of this touches any existing node,
  message, topic, or config file's behavior — a workspace that does not
  launch the new node is unaffected.

## Known limitations

- Static/code validation only in this environment (see Testing report above):
  `colcon build`/`colcon test` and all on-hardware validation with a live
  D435 were **not** performed here and must be run on a Jazzy machine.
- Depth+PnP fusion and pose covariance use axis-independent (diagonal)
  covariances rather than a full joint EKF — see "Deliberately not
  implemented" above.
- The global marker map is in-memory only; it is not persisted to disk and
  resets on node restart (`/marker_map/reset` also clears it manually).
- Global fusion assumes a static marker; a confirmed marker that is
  physically moved will be treated as inconsistent observations and gated
  out rather than automatically relocated (by design, see
  `position_gate_threshold`).
- No marker-based localization feedback, SLAM, or Nav2/AMCL integration is
  implemented or intended — this branch only detects, tracks, and maps
  markers. No ML/neural detector was added or is needed; classical ArUco
  remains fast, deterministic, and interpretable for known markers.
- Temporal-ROI detection (Part 9 of this pass) was deliberately not
  implemented; see "Deliberately not implemented in this sub-pass" above.
- `marker_action_interface_node`'s `base_link` resolution depends on a
  `base_link <- <RGB optical frame>` TF being published by something else in
  the rover's TF tree (e.g. `robot_state_publisher` / a static transform
  publisher); this package does not publish that transform itself, by
  design (Part 4 explicitly requires using TF2, not computing it manually).
- **Advanced features intentionally not implemented in the engineering-audit
  pass**, per its own "do not overengineer" instruction (implement the
  smallest robust fix for a confirmed problem, not every suggested feature):
  IR-stream detection (Section 19), ego-motion compensation using
  odometry/IMU (Section 21), a rigid multi-face/Board object-pose mode
  (Section 9), and a rosbag benchmark/evaluation script (Section 35). None
  of these address a confirmed bug; `aruco.zip` also gives no evidence the
  physical cubes use *different* IDs per face, which is the main case a
  Board mode would help with. The current single-constant-ID duplicate
  handling (this pass's tracker fix) remains correct and is a prerequisite
  either way.
- The duplicate-same-ID candidate ranking (this pass) does not have access to
  a per-candidate apparent marker pixel size or a separately-tracked
  normalized reprojection error on `RawPose` -- only the pose-stage output
  carries those. It uses the existing composite `pose_quality` as a proxy
  (its geometry term already penalizes small/skewed detections) rather than
  threading two more fields through the pose->tracking boundary for a
  same-frame edge case; flagged here as a possible future refinement, not a
  known defect in current behavior.
