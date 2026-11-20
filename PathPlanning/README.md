# PathPlanning

Production navigation lives in [`erc_path_planner`](erc_path_planner/README.md).
It runs the real obstacle bridge and Nav2 Smac Hybrid A*, MPPI, costmaps,
navigation/recovery behaviors and velocity smoother. Motor hardware drivers,
SLAM, Perception, sensor drivers and robot TF are external dependencies.

There are no runtime mocks, synthetic maps or artificial localization launches
inside this module. Those live in
[`testing/mocks/pathplanning/pathplanner_mock_tools`](../testing/mocks/pathplanning/pathplanner_mock_tools/README.md).
Testing includes the production launch; it does not replace Smac with a toy planner.

Read the [current validation record](VALIDATION.md) before interpreting a
successful launch as navigation correctness. Boundary collision and stale-input
safe-stopping checks currently fail; hardware readiness is not established.
Older material in `docs/` and `Prop&Solu.md` is historical, not the current runbook.
