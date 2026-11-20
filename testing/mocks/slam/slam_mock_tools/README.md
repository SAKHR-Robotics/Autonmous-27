# slam_mock_tools

Test-only SLAM dependency fixtures. Production SLAM does not require this package.

Installed executables: `mock_aruco_publisher`, `costmap_test_stub`,
`rover_sensor_adapter`, `image_gate`.

`image_gate` forwards real RGB images unchanged from `/camera/image_raw` to
`/testing/slam/camera/image_raw`, using sensor QoS. Setting its Boolean `enabled`
parameter false stops forwarding to exercise camera loss; true resumes it.
It preserves original image timestamps and data and never produces SLAM output.

The old `slam_test_tools` executable names/imports remain compatibility entry points.
Both names start the same implementation; do not run both together.

The landmark/obstacle stubs remain fixed smoke fixtures, not moving-scene truth.
Their zero-stamped landmark/marker behavior is not a valid positive landmark or
trajectory acceptance test. Both remain off in the existing standalone wrapper.

## Shared-rover sensor adapter

`rover_sensor_adapter` is used by
`ros2 launch slam_test_tools local_mocked.launch.py`. It reuses the existing
`pathplanner_mock_tools/mock_rover_sim` rather than implementing another rover.
The launch disables mock TF, isolates its outputs and selects the explicit
nominal geometry: radius 0.06 m, track 0.49 m, zero noise and zero slip.

| Interface | Type / semantics |
| --- | --- |
| Input `/testing/slam/truth` | Odometry from the shared kinematic rover, never the production EKF |
| Input `/testing/slam/side_joints` | JointState with `left_wheel_joint` and `right_wheel_joint` positions in radians |
| Output `/joint_states` | Four wheel positions in the converter's documented order; original header retained |
| Output `/imu/data` | Best Effort Imu in the truth body frame; yaw orientation/rate, differentiated forward speed, lateral `v * yaw_rate`, and +9.81 m/s² gravity |

No input is published as `/odometry/filtered`, and the adapter publishes no TF.
It emits no acceleration sample before a second valid timestamp, or across a
backward clock jump/gap over 0.5 s. It does not integrate acceleration into truth.
The `drop_imu` and `drop_wheels` Boolean parameters deliberately stop the selected
output. IMU history continues during deliberate loss, avoiding an artificial
acceleration impulse at recovery. Incomplete/nonfinite wheel sides are rejected.

Source timestamps and coherent fixture physics are regression checked, including
centripetal acceleration and dropout/clock-reset baselines. This is a nominal
synthetic consistency fixture: IMU and comparison truth originate in the same
model. It does not establish physical accuracy, real slip detection quality,
sensor noise/calibration robustness, moving RGB-D or loop closure.

```bash
python3 -m pytest testing/mocks/slam/slam_mock_tools/test -q -p no:cacheprovider
```

Use the isolated Docker build and evidence commands in
[SLAM/VALIDATION.md](../../../../SLAM/VALIDATION.md). Generated results belong
outside source packages.
