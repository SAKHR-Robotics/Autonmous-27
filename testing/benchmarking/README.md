# Mission and module evaluation

The goal is navigation mission performance, explained by module observations.
Reuse observations across mocked, partially integrated and full-stack runs.
Record which components were real or mocked in every result.

| Area | Current implementation | Status and guide |
| --- | --- | --- |
| `pathplanning/` | `global_path_benchmarking` | Missing-data evaluator/report fixes checked; full mission execution remains pending. [README](pathplanning/README.md), [evidence](pathplanning/VALIDATION.md). |
| `slam/slam_test_tools/` | `slam_test_tools` | Health monitor, local-estimator probe, mocked launch and external-sensor wrapper. [README](slam/slam_test_tools/README.md), [production evidence](../../SLAM/VALIDATION.md). Drift/reset recovery and wider mission gates remain open. |
| `perception/perception_benchmark_tools/` | `perception_benchmark_tools` | Relocated offline helper timing runner; not a production-node or mission accuracy observer. [README](perception/perception_benchmark_tools/README.md). |

Mock implementations live under `testing/mocks/`. Planning orchestration still
mixes scenario input publishing with observation. Future shared orchestration is
documented in [validation_bringup](../validation_bringup/README.md); no shared
bringup package or GUI exists yet. Historical design documents and generated
reports are not current acceptance evidence.
