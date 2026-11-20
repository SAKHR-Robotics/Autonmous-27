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

`mock_aruco_publisher` and `costmap_test_stub` remain fixed sensor-relative smoke
fixtures (never moving-scene truth). Both now default `use_current_stamp:=true` (`S08`),
and `mock_aruco_publisher` emits valid PSD diagonal `position_variance`/`orientation_variance`
plus optional `publish_array:=true` (`/perception/landmark_detections`).

## Shared-rover sensor adapter

`rover_sensor_adapter` is used by `ros2 launch slam_test_tools local_mocked.launch.py`.
It reuses `pathplanner_mock_tools/mock_rover_sim` (with TF disabled and `0.06 m`
radius / `0.49 m` track) rather than duplicating another rover plant.

| Interface | Type / semantics |
| --- | --- |
| Input `/testing/slam/truth` | Odometry from the shared kinematic rover, never the production EKF |
| Input `/testing/slam/side_joints` | `JointState` with `left_wheel_joint` and `right_wheel_joint` positions in radians |
| Output `/joint_states` or `/wheel/ticks*` | Selected via `wheel_output_mode`: `joint_states` (4-wheel `JointState`), `ticks_array` (`/wheel/ticks`), or `individual_ticks` (`/wheel/ticks/{left_front,right_front,left_rear,right_rear}`) |
| Output `/imu/data` | Best Effort `Imu` in `imu_frame_id` (defaults to `base_link`); yaw orientation/rate, forward accel, centripetal `v * yaw_rate`, and `gravity_sign * 9.81 m/s²`, transformed by `imu_mount_{roll,pitch,yaw}_rad` with optional `imu_accel_noise_std`, `imu_gyro_noise_std`, `imu_gyro_bias_z` |
| Output `/testing/slam/fault_state` | `std_msgs/String` JSON summarizing active `drop_imu`, `drop_wheels`, `wheel_slip_scale`, `single_wheel_slip_index`, and `wheel_output_mode` |

Additional adapter parameters: `seed` (`0`), `ticks_per_revolution` (`1024`),
`wheel_slip_scale` (`1.0`), `single_wheel_slip_index` (`-1` for all wheels, `0..3` for
single-wheel slip injection), `drop_imu` (`false`), and `drop_wheels` (`false`).

It emits no IMU acceleration sample before a second valid timestamp, or across a
backward clock jump/gap over 0.5 s. It publishes no `/odometry/filtered` and no TF.

```bash
python3 -m pytest testing/mocks/slam/slam_mock_tools/test -q -p no:cacheprovider
```

Use the isolated Docker build and evidence commands in
[SLAM/VALIDATION.md](../../../../SLAM/VALIDATION.md). Generated results belong
outside source packages.
