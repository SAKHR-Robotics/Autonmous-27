# PathPlanner validation tools

ROS package: `global_path_benchmarking`. These tools observe the **real production
PathPlanner**; dependency mocks live in
[`pathplanner_mock_tools`](../../mocks/pathplanning/pathplanner_mock_tools/README.md).
The current [production validation record](../../../PathPlanning/VALIDATION.md)
separates passing checks from failed navigation gates and blocked integration.

## Supported entry points

| Entry point | Behavior |
| --- | --- |
| `scenario_benchmark_runner` | Action-correlated multi-scenario Two-Tier benchmarker (`planner_only`, `nav_closed_loop`, `mixed`); awaits `ComputePathToPose`, `NavigateToPose`, and `NavigateThroughPoses`, checks `0.90 × 0.70 m` `swept_collision()` on independent YAML/PGM truth maps, runs negative/cancel/stale-input gates, and writes v2.0.0 JSON, CSV, HTML, MD, optional PDF, and PNG overlays to `--output-dir` outside source |
| `navigation_probe` | Single-shot Nav2 probe (`plan`, `navigate`, `navigate_through_poses`); records `goal_id`, terminal status, latency, swept clearance, raw motion observations, and embedded `schema_v2` block |
| `planner_map_probe` | Generates independent barrier maps (`blocked-wall`, `too-narrow`, `wide-passage`), requests planning only, and checks swept rectangular geometry |
| `benchmark.launch.py` | Launches `mocked_planner.launch.py` (unless `start_mocks:=false`) and runs `scenario_benchmark_runner` into `output_dir` (default `/tmp/pathplanner_benchmark`); rejects `clean:=true` |
| `live_test.launch.py` | Compatibility wrapper forwarding all documented arguments to `pathplanner_mock_tools/launch/mocked_planner.launch.py` |
| `run_testing_suite.sh` | Launcher supporting live RViz, `--headless`, `--benchmark --output-dir ...`, and `--map-probe`; rejects legacy `--clean` / `--batch` |

## Run real PathPlanner + completing mock

Build and source the overlay using the
[mock package guide](../../mocks/pathplanning/pathplanner_mock_tools/README.md).
Run only one dependency profile in a ROS graph. In a sourced `autonomous27:jazzy` container terminal:

```bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py clear_default_obstacles:=true
```

In another sourced terminal, run either the full multi-scenario Two-Tier benchmark suite or individual probes:

```bash
# Full scenario suite (outputs outside source tree)
ros2 run global_path_benchmarking scenario_benchmark_runner \
  --mode nav_closed_loop --output-dir /validation/pp_suite

# Or single-shot probe / independent barrier check
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 2 -1 0 --timeout 45 --output /validation/navigation.json
ros2 run global_path_benchmarking planner_map_probe --output-dir /validation/map-probe

# Unit & runtime checks
PATHPLANNER_RUNTIME_TESTS=1 python3 -m pytest \
  /workspace/testing/benchmarking/pathplanning/test -q -rs -p no:cacheprovider
```

Or launch mocks + batch benchmark in one command:

```bash
ros2 launch global_path_benchmarking benchmark.launch.py \
  mode:=nav_closed_loop output_dir:=/validation/pp_suite
```

The negative gates (`blocked_wall`, `too_narrow_0_6m`) and `stale_input_stop` (`PP10`/`PP11`)
intentionally return `FAIL` (exit code 1) until the underlying production Smac boundary and
stale-input stopping behaviors are resolved. ReportLab remains optional for positive PDF export.

## Legacy evaluator and reports

`testing_node` and `benchmarking_node` remain for legacy compatibility (`legacy:=true` in
`benchmark.launch.py`) and should not be used as release gates. Historical reports under `reports/`
are archival only and do not establish current planner correctness. The `--clean` flag remains
disabled and fails without killing unrelated processes.
