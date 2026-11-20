# Dependency mocks and synthetic fixtures

Use the package READMEs below for inputs, outputs, scenarios and limitations.
These packages supply dependencies; observers evaluate the real module outputs.

| Consumer / purpose | Active guide | Implemented contents and limits |
| --- | --- | --- |
| Perception inputs | [perception_test_tools](perception/perception_test_tools/README.md), [scenario guide](perception/Readme.md) | Synthetic RGB/depth/cloud/calibration scenes, ground-truth markers and offline floor/box generator. Not calibrated physical sensors or full mission truth. |
| Planning dependencies | [pathplanner_mock_tools](pathplanning/pathplanner_mock_tools/README.md) | Shared mock rover, obstacle feeder, sample planner, map assets and generators. Full mission mock contracts remain pending. |
| SLAM dependencies | [slam_mock_tools](slam/slam_mock_tools/README.md) | Shared-rover wheel/IMU adapter, real-image dropout gate, legacy landmark/cloud stubs. Nominal synthetic consistency differs from physical accuracy; legacy stubs remain unvalidated. |

SLAM's local mocked launch and detailed evidence are in
[slam_test_tools](../benchmarking/slam/slam_test_tools/README.md) and
[SLAM/VALIDATION.md](../../SLAM/VALIDATION.md). Perception's current system limits
are recorded in [Perception/INTEGRATION_REVIEW.md](../../Perception/INTEGRATION_REVIEW.md);
[the older synthetic validation](perception/VALIDATION.md) is historical evidence.
Neither package is a complete, accepted navigation mission simulator.

The mock-development agents can extend their packages without editing shared
observers. Reuse a dependency producer between consumers; do not duplicate it.
Legacy executable/import adapters remain in the original ROS packages. They
delegate to the relocated implementations; running both would duplicate publishers.

Installed static maps belong to `pathplanner_mock_tools`, in its `maps/` and
`custom_maps/` share directories. Legacy scene generators still create `maps/`,
`config/` and `reports/` relative to the run directory. Use a dedicated output
directory, never installed package data; path/output cleanup remains open.
