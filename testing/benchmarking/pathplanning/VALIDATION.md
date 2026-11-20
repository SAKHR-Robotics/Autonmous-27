# PathPlanner benchmark validation

## Scope and status — 2026-11-19

Initial source review on `TestingValidation`, followed by the offline checks
recorded below. No builds or ROS launches have run for this validation. Existing reports are not evidence of a validated
benchmarker. Mock implementation is assigned to the other agent; this work does
not move or modify those files.

The goal is navigation mission performance with module-level explanations.
Start by validating planning observations used during that mission, then add
full-duration controller evaluation. Reuse the observations across isolated,
partially integrated and full-stack mission runs. Mission objectives, failure
conditions and thresholds still need agreement. ERC scoring requires the chosen
rulebook and edition; this review makes no compliance claim.

| Layer | What it establishes | What remains outside its claim |
| --- | --- | --- |
| Planner benchmark, first | Request outcome, endpoint validity, path geometry, map/footprint validity, timing and replanning for named scenarios | Goal arrival, controller performance, real perception or localization accuracy |
| Controller benchmark, next | Completion or explicit terminal failure, tracking, clearance and motion metrics over the full execution interval | Physical actuator performance when motion is simulated |
| Mission profile, later | Ordered mission objectives, time limits, interventions and mission scoring from agreed requirements | General reliability outside the tested conditions |

Use reusable observers and metric calculations, with scenario-specific expected
outcomes. Do not build a universal all-stack benchmarker or two independent
measurement systems now. Report which modules are real, mocked, unavailable or
untested. A missing module must not silently become a successful measurement.

## Findings from existing source

| ID | Location and cause | Required correction / acceptance evidence |
| --- | --- | --- |
| P01 | `global_path_benchmarking/report_generator.py`, `calculate_control_metrics`: empty command and odometry histories return controller score 95, frequency 20 Hz and goal error 0.05 m. `testing_node.py`, `compile_reports`, also supplies favorable defaults. | Missing evidence must produce unavailable metrics and an explicit incomplete/invalid result, never an invented measured value. Check empty and partial streams. |
| P02 | `benchmarking_node.py`, `path_callback`/`process_scenario_results`: scores on path arrival. `testing_node.py`, `run_scenario`: stops processes after a result file appears. Navigation action results are not awaited in `publish_start_goal`. | Separate planner response from controller execution. Controller evaluation must end on confirmed completion, abort, cancellation or timeout; a published path does not prove arrival. |
| P03 | `benchmarking_node.py`, `evaluate_path`: overwrites success with true for every nonempty path, including a one-pose path. Collision is a weighted score component rather than an unconditional validity gate. | Define success per mode; validate finite coordinates, endpoint tolerances and collision/footprint checks. Invalid paths must fail regardless of other scores. Handle expected no-path cases separately. |
| P04 | `report_generator.py`, `calculate_path_metrics`, and `benchmarking_node.py`, `evaluate_path`: checks path samples against obstacle points without swept-segment validation; missing map defaults to 10 m clearance; grid origin rotation/frame alignment and unknown-space policy are not validated. | Require valid map/frame inputs for safety metrics, define unknown-space policy, and check the agreed footprint along segments. Missing geometry must be unavailable, not safe. |
| P05 | `benchmarking_node.py`: treats second/subsequent paths as dynamic replans; timing can be goal-to-received-path elapsed time or an uncorrelated internal-time topic. | Associate requests, scenario phases and results; distinguish end-to-end latency from internal compute time. A periodic plan before obstacle injection must not count as obstacle response. |
| P06 | `config/benchmark_config.yaml` declares interface and control options, but the observer uses fixed topics and `Twist`, and does not read those sections. It always combines planner/controller scores 50/50. | Agree planner-only versus controller modes and supported interfaces, wire supported configuration, and reject unsupported choices. Do not penalize planner-only runs for absent controller data. |
| P07 | `testing_node.py` publishes `/global_costmap/costmap` as scenario input while the observer also treats it as the evaluation map. | Separate mock inputs, actual stack outputs and independent scenario truth. Agree authoritative publishers with the mock agent before runtime integration. |
| P08 | `benchmarking_node.py`, `path_callback`, stores received paths as history entries; `report_generator.py` derives report success from those entries. A run that produces no path has no corresponding path callback. | Record one terminal result for every scheduled trial, including no-response and timeout. Repeated plan publications must not inflate the number of successful trials. |

## Bounded implementation sequence

1. Define planner-only result states and remove fabricated measurements, including
   report fallbacks. Validate the evaluator itself using known expected examples.
2. Repair path validity and timing/trial correlation. Include valid path, invalid
   path, blocked route, missing input and no-response cases.
3. Integrate the other agent's mock package through an agreed interface contract;
   repair package/runtime setup and supply exact launch and observation commands.
4. Add full-duration controller evaluation, then an agreed RViz view. Keep GUI
   observation separate from numerical validation.
5. Add a mission profile only after the underlying measurements are trustworthy
   and mission requirements are available.

Before changing launch files, confirm the ROS distribution and host/container,
clock source, action/topic contract, footprint and mock package interfaces.
Existing 85-point pass thresholds are configuration values, not accepted system
requirements. Record raw measurements first; agree thresholds before using them
as release gates.

## Layout and evaluator evidence — 2026-11-19

The folder migration preserves original ROS package names and launch defaults.
Existing planning mocks moved to `testing/mocks/pathplanning/pathplanner_mock_tools`;
SLAM output stubs moved to `testing/mocks/slam/slam_mock_tools`. Legacy Python
imports and executables delegate to the new packages. Seven relocated node
implementations were compared with the previous commit: text content unchanged.
The offline asset generator's destination is now the caller's `custom_maps/`
directory rather than an assumed source-tree depth.

| Check executed on host | Result |
| --- | --- |
| `colcon list --base-paths testing` | Five packages discovered as `ros.ament_python`; no duplicate names |
| Python AST parsing | All 52 Python files parsed |
| Manifest, setup resource and entry-module checks | All five packages passed; this is not an installation/import check |
| `bash -n` on the two existing shell entry points | Passed; scripts were not executed |
| Known-answer evaluator tests | **3 passed, 5 failed**, 1 dependency deprecation warning; no skips |

The three passing checks establish polyline length, a known 1 m tracking offset,
and a known 1 m final-position error. The failures reproduce existing defects:
no-data controller score 95, invented goal error 0.08 m without odometry,
invented clearance 10 m without a map, and success for both NaN and infinite
path geometry. The tests assert the intended honest behavior and deliberately
remain failing until the corresponding implementation is repaired.

These failures are benchmarker defects, not evidence against the real planner.
No mission run, GUI/RViz inspection, ROS node integration or package build was
performed. Host `/opt/ros` contains Humble and no Docker container was running
at inspection; the intended runtime still needs confirmation.

## Reproduce this step

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
colcon list --base-paths testing
PYTHONPATH="$PWD/testing/benchmarking/pathplanning${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m pytest testing/benchmarking/pathplanning/test/test_evaluator_contract.py -q -p no:cacheprovider
```

Expected: five discovered packages; **3 passed, 5 failed** from the evaluator
checks. The test command returns exit code 1 until the defects are fixed. It
requires Python, NumPy and pytest, starts no ROS nodes, and needs no mocks.

Next correction: replace invented measurements with explicit unavailability,
reject invalid geometry, and propagate those states through observer scoring and
all report formats. Do not repair only the calculator while leaving reports or
combined scores dependent on numeric defaults.


## Evaluator and report corrections — 2026-11-20

Preserved and extended the partial uncommitted `report_generator.py` edits found
at the start of this step. Their owner was unknown; Saif did not know, and the
SLAM agent confirmed no active overlap. The pre-edit file was retained at
`/tmp/codex-pathplanner-validation.JqAhLh/report_generator.before.py`.

P01 is corrected for unavailable inputs and exports. P03 now rejects empty,
one-pose and nonfinite geometry in both calculator and observer and prevents a
known sampled collision from counting as success. Endpoint and swept-footprint
checks remain open. P02 is contained by requiring explicit execution completion
before controller scoring; the runner still does not wait for mission completion.
P04 now leaves unsupported maps/frames incomplete; full collision geometry and
unknown-space policy still require implementation/agreement. P05–P08 remain open.

The original eight calculator checks pass, but were insufficient on their own:
partial-stream measurements, report exporters and observer propagation needed
additional regressions. Controller inputs are checked for finite values and
strictly increasing sample times. Missing fields stay null in JSON and N/A in
human-readable exports. Reports no longer derive an overall-system stability
claim from path-record success frequency. A failed/missing exporter is not logged
as a successful export. Legacy diagnostic score thresholds are unchanged and
remain provisional, not mission requirements.

| Executed check | Result |
| --- | --- |
| Isolated `autonomous27:jazzy` container, sourced Jazzy, all package tests | **35 passed, 2 skipped**, no warnings; skips are positive PDF tests because ReportLab is absent |
| Host calculator and report tests, with ReportLab | **25 passed**, no skips; one ReportLab/import deprecation warning |
| PDF text extraction on both host-generated reports | `N/A` and `INCOMPLETE` present; no `SYSTEM STABLE` claim |
| Scoped diff whitespace check | Passed |

Container: `codex-pathplanner-validation-20261120`, isolated network, source mounted
read-only, removed after each run. No ROS nodes, hardware, Gazebo or Nav2 started.
Image ID: `sha256:d952a9381d1f0b15c8212df0a78e023f3058235c659c44267bb782500df235ad`.
Artifacts: `/tmp/codex-pathplanner-validation.JqAhLh/`, including `evaluator.xml`
and `host-report-tests.xml`. No generated reports were added to source packages.
The SLAM agent's container/domain/build outputs were left alone.

Current reproducible commands are in [README.md](README.md#reproduce-the-checked-stage).
The older commands/results above remain historical evidence. No full package
build, real planner run, mission completion, RViz rendering or PDF visual-layout
verification was performed in this step.
