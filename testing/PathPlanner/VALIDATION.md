# PathPlanner benchmark validation

## Scope and status — 2026-11-19

Initial source review on `TestingValidation`; no builds, tests or ROS launches
executed for this review. Existing reports are not evidence of a validated
benchmarker. Mock implementation is assigned to the other agent; this work does
not move or modify those files.

Recommended scope: a development benchmark for the global planner first, followed
by a separately gated controller benchmark. Add an ERC navigation mission profile
later using the same observations and results. This recommendation does not
establish compliance with any ERC rulebook; mission requirements and their
edition must be supplied and reviewed before implementing mission scoring.

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

## Review commands for this source-review step

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
git show --stat HEAD
git show HEAD -- testing/PathPlanner/VALIDATION.md
```

Expected: documentation only. No runtime behavior has been corrected yet.
Implementation steps will include their exact executable validation commands,
expected results and any checks not run.
