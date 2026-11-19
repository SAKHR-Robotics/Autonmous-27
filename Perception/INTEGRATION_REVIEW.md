# Perception validation evidence and open decisions

Review date: 2026-11-19. Source branch: `TestingValidation`.
Environment: existing `autonomous27:jazzy` image, ID `d952a9381d1f`, isolated ROS
network namespaces and scratch build/install directories. The host ROS/Python
installation was not used for builds, tests or perception processing. No image
packages were installed or replaced. Master `testing/` source files are unchanged.

## Implemented

Combined orchestration and its RViz view now belong to `perception_bringup`.
Terrain and marker packages retain independent launches. The old terrain combined
launch forwards arguments to bringup. Terrain-only processing no longer declares
a marker-package dependency; the optional compatibility launcher requires bringup
installed separately, avoiding a circular package dependency.

Cloud decoding validates field layouts and honors row padding and byte order.
Full-cloud transforms preserve input headers and non-XYZ bytes. Invalid layouts
report unavailable terrain rather than escaping the callback. When memory owns
the obstacle output, terrain also relinquishes its DDS publisher endpoint.

Marker action targets reject stale/future/invalid-age observations and malformed,
unknown or excessive covariance. Age includes transport delay; headers retain
observation time. Invalid CameraInfo clears previous calibration; stale or
wrong-resolution calibration is not used for depth projection or PnP. Old/future
RGB is rejected. Clock reversal clears tracks and buffered depth. Diagnostics
report unusable calibration/invalid input and measure FPS within its reporting
window instead of reusing a growing history count.

Native landmark arrays preserve both array and observation headers. The adapter
validates IDs, frames, combined observation age, finite confidence and covariance.
The legacy PoseStamped interface remains; no active subscriber was found in the
inspected production Perception/SLAM/planner sources. Native landmark topics and
message types remain unchanged. Shutdown handling is safe after ROS SIGINT.

## Checked results

| Check | Result |
| --- | --- |
| Fresh scoped build | Five production packages built with `--symlink-install` |
| Package test runners | `colcon test-result`: 180 checks, zero errors/failures/skips; includes 178 pytest cases plus two CTest wrapper results |
| Optional testing dependency | Existing sensor feeder was copied to scratch and built unchanged; production regression checks also work without it with one explicit skip |
| Launch argument expansion | All 13 production and compatibility launch files passed |
| Actual startup/shutdown | Core combined (4 nodes), all selected processing stages (7), terrain-only (1), markers-only (3) started and stopped cleanly |
| Native landmark adapter | Installed production executable accepted a controlled valid tracked observation, preserved headers/covariance and emitted positive ID 1; future observations were rejected |
| Actual Docker-installed RTAB-Map | Running node exposed `/perception/landmark_detections` as `rtabmap_msgs/msg/LandmarkDetections`; this is a subscription contract check, not mapping/optimization validation |
| Missing depth | Real marker detection/tracking continued without depth fusion |
| Invalid cloud | Terrain published unavailable diagnostics and unknown grids, with no fabricated valid empty box observation |
| Dropout/recovery | Terrain changed to unavailable/unknown during loss, then recovered on resumed input |
| RViz | Combined, terrain and marker views rendered through Docker with software OpenGL; inspected window captures, sensor remapping and global fixed-frame status. Layouts were corrected to keep the main scene visible. |

The command sequence is in [TESTING_GUIDE.md](TESTING_GUIDE.md). Scratch logs,
message summaries and window captures are outside the repository. No sensor
mock, artificial localization fixture or generated validation output was added
to production packages.

## Terrain geometry fails controlled ground-segmentation checks

The existing synthetic feeder places the ground at `base_link` Z=0. With the
image's native Patchwork++ backend, the exact float32 transformed cloud labels
2,227 known ground samples as non-ground. After clustering, ground and rock merge
into a box centered near (1.275, -0.0125, 0.2244) m rather than the known rock near
(2.0, 0.6, 0.25) m: approximately 0.95 m center error. The ground-only scene also
produces false obstacle boxes. These are FAILED geometry checks, even though
processing diagnostics can be healthy.

The upstream native algorithm uses plane position relative to its origin when
classifying near-zone ground (see the
[Patchwork++ implementation](https://github.com/url-kaist/patchwork-plusplus/blob/master/cpp/patchworkpp/src/patchworkpp.cpp)).
The observed failure is consistent with its below-origin ground assumption and
the fixture's ground-at-origin frame. Changing only the configured sensor-height
value between 0, 0.2 and 0.4 m did not resolve the classification. The NumPy
backend was compared diagnostically and did not show this ground-as-obstacle
case; it has NOT been substituted for the default to claim validation passed.

A follow-up isolated check in the same Docker image fed known XYZ points directly
to the production `GroundRemoval` wrapper, bypassing ROS decoding, TF and box
visualization. Each fresh native instance processed five identical observations;
the counts below describe the fifth observation of a 9,120-point floor with no
rock. All points were inside the configured ROI and radial range.

| Floor Z (m) | Configured origin height (m) | Ground points labelled non-ground |
| --- | --- | --- |
| 0 | 0.2 | 2,173 |
| -0.2 | 0.2 | 605 |
| -0.255 | 0.255 | 605 |

These checks demonstrate that correcting the height reference alone does not
resolve the failure. Sloped and noisy diagnostic variants changed the error
counts; variants containing a rock also lost some known rock samples to the
ground class. Reducing floor false positives alone is therefore insufficient
evidence of a fix. The exact native cause, including patch support, plane-fitting
numerics and classification settings, remains unconfirmed. The scratch driver
and full results are outside the repository in
`/tmp/codex-perception-validation.kXgjr9/check_ground_sensitivity.py` and
`ground_sensitivity.json`; these are diagnostic inputs, not rover calibration or
an approved traversability envelope.

Next validation must distinguish raw-cloud/TF alignment, ground classification,
and downstream box generation using the existing Gazebo world's camera stream.
That world has not yet been validated by these synthetic checks. Replay the same
recorded observations for before/after comparisons, including flat ground,
gentle slopes, steps and real obstacles. Ground classification and safe
traversability are separate decisions: acceptable slope and step limits need
the rover's agreed operating requirements. No invented offset, hidden fallback
or guessed physical calibration has been introduced. Overall Perception
validation remains incomplete until ground-only and obstacle-geometry checks
pass without hiding real obstacles.

## Positive camera-to-action/landmark acceptance needs a suitable scene

The existing frontal marker fixture reaches CONFIRMED and depth fusion, but keeps
`POSE_AMBIGUOUS`. Action and landmark quality gates correctly reject it. This
scene therefore verifies detection/tracking and rejection, not positive usable
targets or accepted SLAM landmarks. A non-ambiguous geometrically coherent scene
or real recording is needed for that positive end-to-end check. The controlled
adapter contract and action-interface regression tests pass separately; quality
gates have not been loosened to force fixture acceptance.

## Coordination and later integration

The offline synthetic terrain benchmark still resides in production source and
is not invoked by production nodes. Its relocation awaits the testing agent's
chosen package destination. The [testing handoff](TESTING_GUIDE.md#testing-agent-handoff)
records source paths and compatibility details. Perception changes do not write
to master `testing/` or change optional feature selection defaults.

Physical camera alignment/mounting, ground reference, marker size, outdoor data,
target-computer latency, Nav2 marking/clearing and safe stopping, RTAB-Map map
quality/loop closure and mission target behavior remain later integration checks.
Diagnostic timers use ROS time; stopped publishers/clocks require consumer-side
freshness policies. No hardware readiness or ERC compliance is claimed.
