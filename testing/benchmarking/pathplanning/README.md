# Navigation mission benchmarking: PathPlanner

This package measures planning and controller behavior to explain navigation
mission performance. **The evaluator/reporting stage is checked; the complete
mission benchmark is not validated yet.** A published path does not establish
that the rover reached its goal.

ROS package name: `global_path_benchmarking`. Detailed findings and evidence:
[VALIDATION.md](VALIDATION.md). The former architecture/usage proposal is preserved
as [LEGACY_DESIGN.md](LEGACY_DESIGN.md); its capabilities, thresholds and launch
instructions are not acceptance evidence.

## Current responsibilities and layout

```text
pathplanning/
├── global_path_benchmarking/
│   ├── benchmarking_node.py       # Observer and provisional scenario diagnostics
│   ├── testing_node.py            # Existing mission/scenario runner and report compilation
│   ├── report_generator.py        # Measurement calculations and report exporters
│   └── mock_*.py                  # Compatibility adapters; implementations live under mocks/
├── config/                       # Existing scenarios and provisional score settings
├── launch/                       # Existing launches; defaults unchanged in this step
├── rviz/                         # Existing view; GUI appearance not checked in this step
├── test/                         # Known-answer evaluator, observer and export checks
└── reports/                      # Historical output; not current validation evidence
```

Dependency mocks and static maps live in
`testing/mocks/pathplanning/pathplanner_mock_tools`. Keep producers separate from
observers. Existing legacy orchestration still publishes scenario inputs; its
full separation into mission runner/bringup remains pending.

## What the current fixes establish

1. Missing or invalid observations produce `None`/JSON `null`, displayed as `N/A`
   in CSV, HTML, Markdown and PDF. No invented clearance, goal error, cadence or
   controller score replaces missing evidence. CSV now includes an `Outcome` column.
2. Empty, one-pose and nonfinite paths are rejected. The observer preserves that
   result and marks detected sampled collisions as failures. Missing maps,
   unsupported grid geometry/unknown cells and frame mismatches leave safety
   evaluation incomplete rather than apparently safe.
3. Command cadence counts intervals correctly; heading uses observed odometry
   orientation. Metrics require their actual input streams. The legacy fields
   `linear_jerk_std` and `angular_jerk_std` measure acceleration standard deviation,
   not jerk; current text labels identify them accordingly.
4. A controller score requires both sufficient measurements and an explicit
   execution-complete signal to the calculator. The current runner does not
   establish completion, so its combined result remains `INCOMPLETE` unless a
   known invalid path makes it `FAIL`. Last observed goal distance is not arrival.
5. Exporters tolerate unavailable values. Reports show recorded outcomes rather
   than claiming the system is stable. Missing PDF support is reported as an
   unavailable export, never a successfully generated PDF.

## Reproduce the checked stage

From the repository root on the host, using the existing Jazzy image:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
docker run --rm --network none --read-only --tmpfs /tmp \
  -v "$PWD:/workspace:ro" \
  -e MPLCONFIGDIR=/tmp/matplotlib \
  -e PYTHONPATH=/workspace/testing/benchmarking/pathplanning:/workspace/testing/mocks/pathplanning/pathplanner_mock_tools \
  autonomous27:jazzy bash -lc 'source /opt/ros/jazzy/setup.bash && python3 -m pytest /workspace/testing/benchmarking/pathplanning/test -q -p no:cacheprovider'
```

Expected for the checked image: **35 passed, 2 skipped**. The skips are positive
PDF-export checks because that image lacks ReportLab; a separate test verifies
honest reporting when the dependency is absent. These tests do not start ROS
nodes, Gazebo, Nav2 or hardware; observer methods use real ROS message types.

For the calculator and report checks on a Python environment with NumPy, pytest
and ReportLab installed:

```bash
PYTHONPATH="$PWD/testing/benchmarking/pathplanning" python3 -m pytest \
  testing/benchmarking/pathplanning/test/test_evaluator_contract.py \
  testing/benchmarking/pathplanning/test/test_reports.py -q -p no:cacheprovider
```

Host result: **25 passed**, including both PDF exports. Extracted PDF text was
checked for `N/A` and `INCOMPLETE`. This is not PDF visual-layout approval or proof
that PDF export is available in the Jazzy image.

## Remaining mission-validation gates

| Gate | Remaining work |
| --- | --- |
| Path safety | Validate endpoints, bounds and the swept rover footprint; sampled collision checks are insufficient |
| Trial accounting and timing | One terminal result per requested trial, including timeout/no-path; correlate replans with obstacle events |
| Full controller execution | Wait for actual completion/abort/timeout and measure the entire execution interval |
| Mission profile and integration | Agree objectives, inputs, clock, footprint, truth and failure conditions; then validate real planning with selected mocks |
| Runtime and views | Reconcile dependencies/configuration, duplicate publishers and cleanup, then build, launch and inspect RViz in Jazzy |

Do not use the existing 85-point threshold as an accepted mission requirement.
Do not run the legacy master shell script yet: broad process cleanup remains an
open defect. Launch choices must be agreed before changing launch behavior.

Next bounded step: define valid path/request outcomes and trial correlation,
then validate them against independent scene truth before running a mission.
