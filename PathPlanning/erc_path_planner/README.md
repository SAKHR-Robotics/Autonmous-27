# ERC PathPlanner — production only

Read **[PathPlanning/info.md](../info.md)** first. It consolidates this package's
specifications, node/topic ownership, publisher/subscriber contracts, parameters,
production and test launch commands, RViz procedure, required mocks and open work.

`erc_path_planner` is the production ROS 2 Jazzy `ament_cmake` package. It starts
the real obstacle bridge and Nav2 planning/control/navigation components.
Synthetic inputs, maps and artificial localization remain in
`testing/mocks/pathplanning/pathplanner_mock_tools`; production does not depend on it.

Use the [mocked demonstration](../info.md#8-see-real-pathplanner-in-action-with-mocked-dependencies)
for isolated operation and the [real integration procedure](../info.md#9-real-production-and-gazebo-profile)
for Gazebo. Full motion validation remains blocked by the failures documented in
[the module status](../info.md#1-current-status-and-next-work).

Detailed experiment history remains in [VALIDATION.md](../VALIDATION.md).
