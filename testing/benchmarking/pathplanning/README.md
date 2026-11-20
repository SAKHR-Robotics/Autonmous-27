# PathPlanner validation tools

ROS package: `global_path_benchmarking`. These tools observe the **real production
PathPlanner**; dependency mocks live in
[`pathplanner_mock_tools`](../../mocks/pathplanning/pathplanner_mock_tools/README.md).
The current [production validation record](../../../PathPlanning/VALIDATION.md)
separates passing checks from failed navigation gates and blocked integration.

## Supported entry points

| Entry point | Behavior |
| --- | --- |
| `navigation_probe` | Sends one real Nav2 planning/navigation action; records its goal ID, terminal result, latency, raw motion observations and optional independent map check |
| `planner_map_probe` | Generates independent maps, requests planning only, and checks swept rectangular geometry; a successful action with an invalid path fails the gate |
| `live_test.launch.py` | Compatibility include of the mock-owned launch; starts real production PathPlanner by default with one bridge/map source |
| `run_testing_suite.sh` | Thin launcher for the supported mock profile after building/sourcing Jazzy; no rebuild, directory deletion or broad process cleanup |

Build and source the overlay using the
[mock package guide](../../mocks/pathplanning/pathplanner_mock_tools/README.md).
Run only one dependency profile in a ROS graph. In a sourced container terminal:

```bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py
```

In another sourced terminal:

```bash
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 2 -1 0 --timeout 45 --output /validation/navigation.json
PATHPLANNER_RUNTIME_TESTS=1 python3 -m pytest \
  /workspace/testing/benchmarking/pathplanning/test -q -rs -p no:cacheprovider
```

The recorded Jazzy image passes **47 tests**, with two positive PDF tests skipped
because ReportLab is absent. The opt-in bridge integration tests start a separately
named real C++ bridge on distinct test topics. Tests do not require hardware.
The probe's estimated goal error is not independent localization accuracy, and a
published path is not navigation completion. No mission score is invented.

To reproduce the known map-boundary failure in a disposable mocked run:

```bash
ros2 run global_path_benchmarking planner_map_probe --output-dir /validation/map-probe
```

This changes the map server's map and executes no motion. In the recorded image,
blocked-wall and too-narrow gates fail, the wide-passage gate passes, and the
command returns exit code 1. Restart the mocked launch before another profile.

## Legacy evaluator and reports

`benchmark.launch.py`, `testing_node` and `benchmarking_node` remain legacy
diagnostic tools. Their combined scores are not release gates: they do not yet
provide a validated full-duration, request-correlated mission benchmark.
Historical evidence and outstanding issues are in [VALIDATION.md](VALIDATION.md),
and the old proposal is preserved in [LEGACY_DESIGN.md](LEGACY_DESIGN.md).

Existing calculators/exporters preserve missing measurements as `None`/JSON
`null`/`N/A`, reject invalid path geometry and avoid invented success claims.
Positive PDF export still requires ReportLab. Historical reports under `reports/`
are not evidence of the current production planner's correctness.

The legacy scenario runner's artificial truth now uses
`/testing/pathplanner/scenario_truth`, never Nav2's output costmap topic. Its
`/map` input uses transient-local durability. Broad `--clean` process killing is
disabled; requesting it fails without killing other processes. The shell wrapper
rejects obsolete batch/verify flags and directs callers to the supported probes.
The old compatibility mock executables/imports delegate to `pathplanner_mock_tools`;
do not run both executable names simultaneously.
