# Mission and module evaluation

The goal is navigation mission performance, explained by module observations.
Keep observations reusable between mocked, partially integrated and full-stack
runs. Record which components were real or mocked in every result.

| Area | Current implementation | Status |
| --- | --- | --- |
| `pathplanning/` | `global_path_benchmarking` | Existing runner, observer and reports; known scoring defects. See [validation](pathplanning/VALIDATION.md). |
| `slam/slam_test_tools/` | `slam_test_tools` | Existing EKF monitor and launch wrappers, not a complete mission benchmark |
| `perception/` | Documentation only | Mission observer not implemented |

Mock implementations live under `testing/mocks/`. Existing planning orchestration
still mixes scenario input publishing with observation; its future separation
must be validated rather than claimed complete by this directory move.
