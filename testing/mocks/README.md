# Dependency mocks and synthetic fixtures

| Consumer / purpose | Package or directory | Contents |
| --- | --- | --- |
| Perception inputs | `perception/perception_test_tools` | Existing sensor scenes, feeder, integration checks and RViz view |
| Planning dependencies | `pathplanning/pathplanner_mock_tools` | Existing mock rover, obstacle feeder, sample planner, obstacle injection, map assets and generators |
| SLAM dependencies | `slam/slam_mock_tools` | Existing landmark and bridge-cloud stubs; neither is a complete SLAM sensor simulation |

These are relocated existing implementations, not newly validated mission mocks.
The mock-development agent can extend these packages without editing observers.
Group fixtures by the dependency they simulate as new shared contracts emerge;
do not duplicate a producer simply because two modules consume it.

Legacy executable/import adapters remain in the old ROS packages so launch
commands do not change during the folder migration. They delegate here. Running
both the legacy and new executable would duplicate publishers.

Installed static map assets belong to `pathplanner_mock_tools`, in its `maps/`
and `custom_maps/` share directories. Legacy scene generators still create
`maps/`, `config/` and `reports/` relative to the run directory; correcting this
behavior remains part of benchmark validation. Generate only in a dedicated run
directory, never inside installed package data.
