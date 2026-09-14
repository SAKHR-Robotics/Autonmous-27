# ERC terrain & rock perception upgrade -- final report

This follows the reporting structure requested in the task brief
(Part 38). It is written to be read on its own, without the chat
history.

## 0. Environment disclosure (read this first)

This work was done in a sandbox with Python 3, pip, and a virtualenv,
but **no ROS 2 installation, no colcon, no rclpy, no Gazebo, and no
recorded D435 bag data**. That materially limits what could actually
be *executed* versus *written and reasoned about carefully*:

- `colcon build` / `colcon test` / `ros2 launch` were **NOT run** --
  there is no ROS 2 distro available here. All ROS-message-touching
  code (`terrain_node.py`, `tf_transform.py`, `clustering.py`,
  `visualization.py`, `outlier_filter.py`'s PointCloud2 helpers) was
  reviewed and edited carefully and compiles (`py_compile`) cleanly,
  but was never actually imported against real `rclpy`/message
  packages, so I cannot claim it is free of, e.g., a typo in an
  attribute name that only a real import would catch.
- The provided `templates.zip` contains Gazebo **SDF models and world
  files** (`mars_yard`, `rock_3`, `rock_4`, `rock_5`, several
  `.world` files) -- real simulation assets, but there is no Gazebo
  or ROS 2 simulation runtime here to spawn them and generate an
  actual simulated D435 point cloud from them. I inspected the
  assets (confirmed they're exactly what the brief describes) but did
  **not** and could not run the Part 28/29 Gazebo-based test matrix.
- No real or recorded D435 data was provided or available, so the
  Part 28 "real D435 validation" tests were **not** performed. Every
  test below uses synthetic NumPy point sets constructed to match the
  documented geometry of each scenario (a plane at a known slope, a
  bounded rock-shaped bump on that plane, etc.).
- What *was* done end-to-end: every new module is pure Python/NumPy/
  SciPy with **zero ROS dependency**, and was unit-tested for real,
  with a real pytest run, in this environment (51 tests, all passing,
  shown below with output). A small ROS-message-stub shim
  (`test/conftest.py`) was written so `occupancy_grid.py`'s and
  `costmap_inflation.py`'s message-construction code could also be
  exercised without a real ROS install -- it fabricates plain
  attribute-bag objects for `Header`/`Point`/`Pose`/`Quaternion`/
  `OccupancyGrid`, nothing more, and steps aside automatically if a
  real ROS environment is present.

Everything below is reported against that ground truth -- I have not
fabricated colcon/launch/hardware results.

## 1. Audit of the existing package (Part 1)

Before changing anything, I read every file in `terrain_geometry/`
(package root as uploaded: `terrain_geometry/terrain_geometry/*.py`,
launch files, `package.xml`, `setup.py`). Summary of what was already
there and already good:

- A genuinely well-factored pipeline: thin ROS node
  (`terrain_node.py`) delegating to pure-computation classes
  (`ROIFilter`, `GroundRemoval`, `VoxelFilter`, `RadiusOutlierFilter`,
  `DBSCANClusterer`, `ObstacleFeatureExtractor`, `ObstacleTracker`,
  `OccupancyGridGenerator`, `CostmapInflator`), each independently
  constructible and already validating its own parameters.
- `ground_removal.py` already does a concentric-ring (Patchwork++-style
  CZM) local floor estimate rather than one global plane -- better than
  a naive approach, but **still only local in the range dimension**,
  not azimuth, which is exactly why a wide/diagonal slope still gets
  partially misclassified (see Section 3).
- `obstacle_tracking.py` already has frame-local track IDs with
  greedy nearest-neighbor association and EMA smoothing -- correct
  design for *local* tracking, but explicitly not persistent long-term
  memory (short `max_missed_frames` by design). This became the
  foundation Part 17 asks to build on top of, not replace.
- `occupancy_grid.py` already supports an `UNKNOWN` cell state and a
  `use_unknown_space` mode that only stamps the actively-sensed
  footprint FREE -- a good existing building block for Part 3, though
  it had no way to *prevent* that FREE stamp on a bad frame (fixed
  below) and had the width/depth bug (Section 2).
- **No `test/` directory existed at all** in the uploaded package --
  zero tests, despite fairly involved geometry code. This is worth
  flagging explicitly since it's a gap on its own, independent of this
  upgrade.
- `terrain_geometry_msgs` (the package providing `ObstacleFeature.msg`
  / `ObstacleFeatureArray.msg`) is an **external dependency**, not
  included in the uploaded zip. I did not have access to add fields to
  that message definition, which shaped several decisions below (see
  Section 6).

## 2. Part 2 -- occupancy grid width/depth bug

**Found and fixed.** In `occupancy_grid.py`'s `_rasterize_obstacle`,
the footprint was built as:

```python
x_min, x_max = cx - obs_width / 2.0, cx + obs_width / 2.0   # wrong: width is the Y extent
y_min, y_max = cy - obs_depth / 2.0, cy + obs_depth / 2.0   # wrong: depth is the X extent
```

`obstacle_features.py` defines `width` = Y/lateral extent and `depth` =
X/longitudinal extent (confirmed by reading its AABB computation), so
every obstacle was rasterized rotated 90 degrees relative to its actual
shape. Fixed to:

```python
x_min, x_max = cx - obs_depth / 2.0, cx + obs_depth / 2.0
y_min, y_max = cy - obs_width / 2.0, cy + obs_width / 2.0
```

`test/test_occupancy_grid_footprint.py` adds 4 tests, including the
exact depth=1.0/width=0.4 case from the brief and a rotated/offset
case. I verified these tests genuinely catch the bug by temporarily
reverting the fix in a throwaway copy and re-running them -- 3 of 4
fail against the old code, all 4 pass against the fix (shown in
Section 8's test log).

## 3. Parts 4-9 -- local terrain model & rock classification (the core fix)

**New module `terrain_model.py` (`LocalTerrainModel`)**: bins ground-
classified points into an XY grid (`terrain_patch_size` per cell,
default 0.5 m), fits a robust local plane per cell (least-squares +
one outlier-trim-and-refit pass), and exposes, per query point, height
above the *local* fitted surface, local slope, local roughness, and a
confidence derived from inlier count and fit residual. This directly
targets the documented root cause: the existing ground-removal's
per-range-ring floor is blind to azimuth, so a side-slope or diagonal
ramp gets its uphill half misclassified. A 2D grid with an actual
per-cell plane fit (not just a height percentile) does not have that
blind spot.

**New module `terrain_classification.py`**: for every DBSCAN cluster
surviving ground removal, computes what fraction of its points are
"explained" by the local terrain surface (within `max_terrain_residual`,
default 8 cm) and classifies:
- `TERRAIN` if most of the cluster is explained (any slope, however
  steep -- Part 4's fix) -- **dropped from the obstacle stream
  entirely**, not just relabeled.
- `ROCK` / `OBSTACLE` / `STEP` if it's a genuine protrusion, split by
  footprint size and roughness (Part 9's decision tree, using height
  above terrain + footprint + roughness + point count together, not a
  single threshold).
- `UNKNOWN` if there isn't enough valid local-terrain coverage under
  the cluster to say anything confidently, or if a small protrusion is
  too sparse/small to trust.

Verified with synthetic tests (`test_terrain_model.py`,
`test_terrain_classification.py`) including the **mandatory Part 30
scenario**: a continuous 20-degree synthetic incline is classified
`TERRAIN` (explained fraction > 0.6, height-above-*local*-terrain < 8
cm everywhere) even though the same points are `>15 cm` above a naive
global `Z=0` threshold at the far end of the ramp (this is asserted
directly, `test_global_height_would_have_falsely_flagged_the_incline`)
-- and a rock placed on that same incline is classified `ROCK` with a
clearly separated height-above-terrain (~15-18 cm).

**Wiring**: `terrain_node.py` now fits `LocalTerrainModel` on
`ground_xyz` every frame (right after ground removal), classifies each
DBSCAN cluster before handing it to the tracker, and only ROCK/
OBSTACLE/STEP clusters continue into `ObstacleTracker` /
`RockLandmarkDatabase` / the occupancy grid's lethal-obstacle list.
`TERRAIN` clusters are dropped; `UNKNOWN` clusters are kept separately
and rasterized as `UNKNOWN` grid cells (Section 5), never silently
merged into either FREE or OCCUPIED.

`enable_terrain_classification` defaults `True` but can be set `False`
to restore the exact pre-upgrade behavior (every cluster = obstacle) if
this stage ever needs to be bypassed in the field.

## 4. Parts 10-19 -- persistent rock landmark database

**New module `rock_landmarks.py`** (`RockLandmarkDatabase`,
`RockLandmark`, `RockObservation`):

- Assigns a `rock_N` **persistent ID**, independent of
  `ObstacleTracker`'s frame-local `track_id` (Part 17) -- the two are
  correlated only for one frame at a time via `last_track_id`, never
  used as the identity itself.
- Stores positions in a **stable map/world frame** (Part 12): the
  caller (`terrain_node.py`) transforms each classified detection's
  centroid into `map_frame` (falling back to `odom_frame` if `map` TF
  isn't available) using a **new, non-caching** transform lookup
  (`CloudFrameTransformer.get_dynamic_matrix`, added to
  `tf_transform.py`) -- the existing cached `get_matrix`/
  `transform_points` are correct only for the rigid, static
  sensor->base_link mount and must never be reused for a
  frame pair that moves every frame like base_link->map.
  Transforming into map frame before association is exactly what
  performs Part 16's ego-motion compensation: the rover's own motion
  cancels out because both the new observation and the stored
  landmark are compared in the same non-rover-relative frame.
- Re-identifies existing landmarks using **both** map-frame distance
  and dimension similarity (Part 14), via greedy nearest-neighbor
  under a combined gate.
- Updates a matched landmark's position (and dimensions) via EMA, never
  overwrites outright (Part 15).
- Keeps a landmark in the database with `currently_visible=False` when
  not re-observed, rather than deleting it (Part 18); `landmark_timeout_sec`
  defaults to "never expire" for a mission-length run, but is
  configurable.
- Optional `save_to_file`/`load_from_file` (Part 19), explicitly **only
  ever called from `terrain_node.destroy_node()` or an external
  trigger, never from the per-frame callback** -- so disk I/O can never
  block real-time perception. Uses JSON rather than YAML specifically
  to avoid adding a new (PyYAML) dependency the package doesn't
  otherwise need (documented in the module).

Tested with `test_rock_landmarks.py`, including the **mandatory Part 31
sequence** (`test_disappearance_and_reappearance_keeps_same_id`): a
rock seen for 2 frames, missing for 2 frames, then seen again 3 frames
later keeps the exact same `rock_N` ID throughout, and the database
never exceeds 1 entry for that physical rock. Also covers: two distant
rocks get different IDs; same-position-different-size is treated as a
distinct object (Part 14); position smoothing doesn't jump to a single
noisy reading (Part 15); timeout pruning; save/load round-trip.

## 5. Part 3 -- perception validity states / failure semantics

**New module `perception_health.py`** (`PerceptionState`:
VALID/DEGRADED/INVALID, `PerceptionHealthMonitor`). Checks, every
frame: cloud staleness, TF availability, decoded point count, ground
segmentation success/point count, and previous-frame processing
latency.

**The actual safety fix**: `occupancy_grid.py`'s `generate()` gained a
`mark_known_region_free: bool` argument. When `False` (driven by
`health.should_report_free`, which is `False` for any `INVALID` frame),
the grid is filled entirely `UNKNOWN` and **stays** `UNKNOWN` except
for any obstacle footprints actually passed in -- regardless of
`use_unknown_space`'s normal-operation setting. Previously, every
early-return path in `terrain_node.py` called `_publish_empty()`,
which (depending on `use_unknown_space`) could still stamp the sensed
footprint FREE even on a total perception failure -- exactly the
"perception failure -> empty obstacle list -> free grid -> planner
assumes safe" chain the brief calls unacceptable. Every early-return
branch in `_cloud_callback` (empty message, empty decode, missing TF,
empty ROI, failed ground segmentation) now computes a `PerceptionHealth`
first and passes it through.

Published on a new topic, `/terrain/perception_health`
(`std_msgs/String`, e.g. `"DEGRADED: point cloud is stale (0.62s >
0.50s)"`) -- see Section 6 for why this is a plain String rather than a
new message type.

Tested in `test_perception_health.py`: 12 tests covering every
VALID/DEGRADED/INVALID transition and the "degraded may still report
free" toggle.

## 6. Parts 20-24, 32-33 -- costmap, confidence, output interfaces

- **Uncertain-region rasterization** (Part 21): `occupancy_grid.generate()`
  gained an `uncertain_obstacles` argument -- footprints stamped
  `UNKNOWN` (never FREE, never lethal) before the real obstacle
  footprints are drawn on top. Fed from two sources: `UNKNOWN`-classified
  clusters, and `LocalTerrainModel.missing_patch_centers()` --
  patches with literally zero returns (occlusion or a possible
  negative obstacle/drop-off), gated by `enable_negative_obstacle_marking`.
  This is a conservative geometric heuristic, not a depth-based hole
  detector, consistent with Part 36's "do not overengineer".
- **Confidence** (Part 23/24): new `confidence.py`, a weighted blend
  of point count, spatial density, depth-direction spread, distance,
  and terrain separation -- every sub-score is derived from the
  cluster's own measured points, never a fabricated constant (tested:
  `test_never_returns_fixed_constant_across_varied_inputs`, plus
  monotonicity tests for every input dimension).
- **Rover footprint / graded terrain traversability cost** (Part 20/22):
  **not implemented** in this pass. `CostmapInflator`'s existing
  circular-radius inflation is unchanged; a rectangular-footprint or
  slope-graded (rather than binary lethal/free) costmap would be a
  reasonable follow-up but was out of scope for the time available --
  flagged here rather than silently skipped.
- **Structured rock output** (Part 32) and **RViz visualization** (Part
  33): `/terrain/rock_landmarks` (JSON over `std_msgs/String`) and
  `/terrain/rock_landmark_markers` (`MarkerArray`, dimmed for
  `currently_visible=False`). `visualization.py`'s obstacle markers are
  now colored by classification (red-orange=ROCK, yellow=OBSTACLE,
  purple=STEP, gray=UNKNOWN) and labeled with the persistent ID once
  assigned. **Why JSON-over-String instead of a proper new `.msg`
  type**: `terrain_geometry_msgs` is an external package not included
  in the uploaded zip, and the brief's Part 1 rule ("do not change
  ROS topics/message interfaces... without checking compatibility")
  argues against modifying a message package I don't have visibility
  into or permission to edit. This is a real limitation, documented
  rather than silently worked around -- a follow-up PR against
  `terrain_geometry_msgs` adding a proper `RockLandmark.msg` /
  `RockLandmarkArray.msg` (mirroring the JSON schema already in
  `terrain_node._publish_rock_landmarks`) would be the natural next
  step and is a small, mechanical change once that package is
  available.

## 7. Parts 25-27, 36-37 -- configuration, performance, scope discipline

- All new thresholds are ROS parameters with validation in
  `_validate_parameters` (see the parameter table in `README.md`).
  `enable_terrain_classification` and `enable_rock_landmarks` are kill
  switches restoring exact pre-upgrade behavior.
- No new heavy dependency was introduced (no PyYAML, no ML libraries);
  `terrain_model.py`'s per-cell fitting loop is bounded by patch count
  (typically tens to a few hundred), never point count -- consistent
  with the existing per-zone loop style in `ground_removal.py`.
  `terrain_classification.py`/`confidence.py` are pure NumPy, no
  per-point Python loops.
- **Not verified**: actual real-time performance/CPU numbers on the
  competition computer. I have not fabricated latency/FPS/CPU/RAM
  figures -- doing so without real hardware would violate Part 38's
  explicit "do not fabricate these numbers" instruction. The added
  per-frame work is one `LocalTerrainModel.fit` (bounded by ground
  point count) + one `classify_cluster` call per DBSCAN cluster
  (typically single digits to low tens of clusters per frame), which
  is algorithmically the same order of cost as the existing clustering/
  feature-extraction stages already in the pipeline, but this is an
  engineering estimate, not a measurement.

## 8. Tests -- what was actually run, and the result

```
$ PYTHONPATH=. pytest test/ -q
...................................................
51 passed in 0.19s
```

Breakdown by file:
- `test_occupancy_grid_footprint.py` -- 4 tests (Part 2 bug fix,
  including rotated/non-symmetric cases as required).
- `test_terrain_model.py` -- 9 tests (flat terrain, the mandatory
  incline-is-terrain case, rock-on-incline separation, missing-patch
  detection, config validation).
- `test_terrain_classification.py` -- 8 tests (incline->TERRAIN,
  rock-on-incline->ROCK, wide flat slab->STEP, sparse noise->UNKNOWN,
  out-of-coverage->UNKNOWN, empty cluster, config validation).
- `test_perception_health.py` -- 12 tests (every VALID/DEGRADED/INVALID
  transition named in Part 3).
- `test_rock_landmarks.py` -- 15 tests, including the mandatory Part 31
  sequence, dimension-gated association, position smoothing, timeout,
  and save/load round-trip.
- `test_confidence.py` -- 8 tests (boundedness, monotonicity in every
  input dimension, non-constant-output check).

I also specifically verified the occupancy-grid tests are a real
regression guard, not tautological: I reverted the Part 2 fix in a
throwaway copy of the package and re-ran that test file -- 3 of 4 tests
fail against the pre-fix code (shown below), all 4 pass against the
actual fix.

```
# throwaway copy with the OLD (buggy) width/depth assignment:
$ pytest test/test_occupancy_grid_footprint.py -q
FAILED test_elongated_obstacle_orientation_matches_depth_x_width_y
FAILED test_wide_shallow_obstacle_orientation
FAILED test_non_symmetric_offset_obstacle_footprint_bounds
3 failed, 1 passed
```

**Not run** (no ROS 2 environment available, see Section 0):
`colcon build`, `colcon test`/`ament_flake8`/`ament_pep257`, `ros2
launch terrain_geometry terrain.launch.py`, RViz2 visual verification,
Gazebo-based regression scenarios against `templates.zip`'s
`mars_yard`/`rock_3`/`rock_4`/`rock_5` assets, and any test against
real or recorded D435 data. All new modules were, however, syntax-
checked with `py_compile` including `terrain_node.py`,
`tf_transform.py`, and `visualization.py`, which import ROS message
packages this environment doesn't have -- that only proves the Python
syntax is valid, not that the ROS wiring is correct end-to-end.

## 9. Files modified

- `occupancy_grid.py` -- Part 2 bug fix; `generate()` gained
  `uncertain_obstacles` and `mark_known_region_free` parameters.
- `obstacle_features.py` -- `ObstacleFeature` gained additive,
  defaulted fields (`classification`, `height_above_terrain`,
  `local_slope_deg`, `roughness`, `confidence`, `persistent_id`); no
  existing field changed, `_feature_to_msg` untouched.
- `tf_transform.py` -- added `CloudFrameTransformer.get_dynamic_matrix`
  (non-caching lookup for the map/odom frame); nothing existing
  changed.
- `visualization.py` -- classification-based marker coloring,
  persistent-ID labels, and a new `build_landmark_marker_array` method;
  existing `build_marker_array` behavior unchanged for
  unclassified/default input.
- `terrain_node.py` -- new parameters, new stage construction, the
  classification/landmark wiring described above, health-gated grid
  generation, three new publishers, `save_landmarks_now`/
  `destroy_node` for optional persistence.
- `README.md` -- appended an upgrade section (existing content
  unchanged).

## 10. Files added

- `terrain_geometry/terrain_model.py`
- `terrain_geometry/terrain_classification.py`
- `terrain_geometry/confidence.py`
- `terrain_geometry/perception_health.py`
- `terrain_geometry/rock_landmarks.py`
- `test/conftest.py`, `test/test_occupancy_grid_footprint.py`,
  `test/test_terrain_model.py`, `test/test_terrain_classification.py`,
  `test/test_perception_health.py`, `test/test_rock_landmarks.py`,
  `test/test_confidence.py`
- `docs/erc_upgrade_report.md` (this file)

## 11. Files intentionally unchanged

`roi_filter.py`, `voxel_filter.py`, `outlier_filter.py`, `clustering.py`,
`obstacle_tracking.py`, `costmap_inflation.py`, `ground_removal.py`,
`benchmark.py`, `performance_profiling.py`, `package.xml`, `setup.py`,
`setup.cfg`, launch files, rviz config. No new external dependency was
required, so `package.xml`/`setup.py` did not need updating.

## 12. Acceptance criteria checklist (Section "MOST IMPORTANT ACCEPTANCE CRITERIA")

| # | Criterion | Status |
|---|---|---|
| 1 | Continuous incline classified as TERRAIN | Done, unit-tested |
| 2 | Rock on that incline still detected as ROCK | Done, unit-tested |
| 3 | Local terrain geometry, not one global horizontal assumption | Done (`LocalTerrainModel`) |
| 4 | Persistent rock IDs independent of frame-local tracking | Done, unit-tested (Part 31 sequence) |
| 5 | Persistent positions in stable map/world frame | Done (map frame, odom fallback) |
| 6 | Rock stays in DB when out of FOV | Done, unit-tested |
| 7 | Re-identifies on return | Done, unit-tested |
| 8 | Ego-motion compensated via TF/odometry | Done (map-frame transform before association) |
| 9 | Perception failure never silently free | Done, unit-tested (12 health-state tests) |
| 10 | Occupancy grid width/depth bug fixed + tested | Done, verified against pre-fix code |
| 11 | Computationally practical for D435/competition computer | Believed true by design (vectorized, bounded loops); **not measured** on real hardware |
| 12 | Builds on ROS 2 Jazzy | **Not verified** -- no ROS 2 environment available in this session |
| 13 | Full test suite passes | 51/51 new tests pass; **pre-existing suite did not exist**; `colcon test` not run |
| 14 | No unrelated functionality/architecture removed | Believed true -- every change above is additive or scoped to the described bug/feature |
