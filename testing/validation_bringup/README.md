# Validation bringup

Reserved for mission profiles, launch orchestration and RViz configuration shared
across modules. No new ROS package, GUI or launch behavior is introduced here yet.

Existing launches stay installed by their original ROS packages during this
structural migration. Their commands still use `global_path_benchmarking`,
`perception_test_tools` and `slam_test_tools` after rebuilding the overlay.

Before moving or changing launch behavior, agree the ROS environment, clock,
real versus mocked dependencies, topic/action contracts and TF owners. The
mission runner sends objectives; mocks supply inputs; observers evaluate outputs.
A future GUI should operate those profiles.
