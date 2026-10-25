# perception_test_tools

Test-only ament_python package. See [the testing guide](../Readme.md) and the
[perception runbook](../../../Perception/RUNBOOK.md) for build prerequisites,
inputs, expected observations, isolated ROS-domain usage and limitations.

```bash
ros2 launch perception_test_tools test_stack.launch.py pipeline:=all
ros2 launch perception_test_tools test_stack.launch.py pipeline:=terrain
ros2 launch perception_test_tools test_stack.launch.py pipeline:=markers
ros2 launch perception_test_tools sensor_inputs.launch.py scenario:=missing_depth
```

`test_stack` supplies explicit test topics to real perception. `sensor_inputs`
starts only the feeder. Scenarios are deterministic, not photorealistic. Runtime
fixtures stay in this testing package; production packages do not depend on it.
