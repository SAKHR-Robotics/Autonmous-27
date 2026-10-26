# Perception validation record — 2026-10-26

Base: jazzy-port, d05282b. Branch: perception-jazzy-improvements.
Target: Linux, Ubuntu 24.04 / ROS 2 Jazzy. No Windows runtime validation required.

## Executed

- Built eight packages in an isolated Jazzy Docker environment: perception
  messages, terrain, markers, planner bridge, rover_slam, and both testing packages.
- Rebuilt terrain, markers and perception test tools after adding installed YAML
  configuration. Build artifacts were isolated under /tmp, with ROS_DOMAIN_ID=87.
- Final pytest run: **135 passed in 8.66 seconds**, with no skipped tests:

```bash
python3 -m pytest Perception/terrain_geometry/test Perception/marker_detection/test \
  testing/Perception/perception_test_tools/test -q --disable-warnings
```

The run includes the original 109 marker tests, terrain failure/free-space tests,
dynamic TF lookup, surface risk, memory, marker-map composition, native landmarks,
launch selection/configuration, deterministic fixtures, the real headless
sensor-to-terrain/marker pipeline and a live C++ bridge regression.

The live pipeline observed a rock, occupied costmap cells, a depth-fused marker
pose and healthy terrain diagnostics. The bridge checked empty arrays, invalid
geometry, preserved frame headers and the configured point cap.

Python syntax, XML manifests, RViz YAML, current-guide links and shell syntax
were checked separately. The deployed marker executables' Linux shebangs were
verified after reinstalling normalized LF sources.

## Not established

RViz GUI rendering, physical camera calibration/accuracy, outdoor lighting/dust,
target-computer latency/thermal behavior, native Patchwork++ results, complete
RTAB-Map/Nav2 operation and safe full-rover autonomous driving were not verified.
The validation image used the logged NumPy ground-removal fallback.

Memory/risk thresholds are not competition-certified values. Diagnostics do not
replace the controller's required freshness and stop policy. Repeat the broader
validation below before merging or deployment.

## Antigravity validation handoff

```text
Validate branch perception-jazzy-improvements against jazzy-port on Linux/Jazzy.
Read AGENT.md and GEMINI.md. Never recursively scan worlds/, build/, install/,
log/ or .git/. Do not create AI setup files.

Do not edit source, commit, push, reset or clean. Use isolated build/install
directories and ROS_DOMAIN_ID. Do not attach rover hardware or publish motion.

Build with --symlink-install using these base paths:
Perception/
PathPlanning/erc_path_planner/
SLAM/rover_slam/
testing/Perception/perception_test_tools/
testing/SLAM/slam_test_tools/

Source Jazzy and the resulting overlay. Run pytest on:
Perception/terrain_geometry/test/
Perception/marker_detection/test/
testing/Perception/perception_test_tools/test/

Exercise terrain-only, markers-only and combined perception. Check invalid
input, dropout recovery, optional terrain memory through turns, calibrated-risk
configuration rejection, landmark gates and bounded bridge output. Verify no
synthetic publishers or artificial localization start from production launches.

Inspect terrain, marker and master RViz views on a Linux display. Check actual
Gazebo camera inputs, clock and TF separately. Report unavailable GUI/Gazebo/
hardware checks as untested; do not infer readiness from synthetic results.

Report exact commands, pass/fail/skip counts and failure logs. If a check fails,
identify file/cause/proposed fix and stop before editing source.
```
