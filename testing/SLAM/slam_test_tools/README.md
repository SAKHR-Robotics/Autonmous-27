# slam_test_tools

Existing runtime mocks relocated out of rover_slam. Build this package explicitly
with rover_slam and source the overlay. Never use these publishers beside live
producers or hardware; use an isolated ROS_DOMAIN_ID.

```bash
ros2 run slam_test_tools costmap_test_stub
ros2 run slam_test_tools mock_aruco_publisher
ros2 launch slam_test_tools test_slam_standalone.launch.py launch_aruco_stub:=true
```

The costmap mock publishes PointCloud2 on /bridge/pointcloud, not on the
Detection3DArray topic /perception/obstacles_only. The landmark mock publishes
native RTAB-Map LandmarkDetection on /perception/landmark_detection (ID 1).
Do not enable the real landmark adapter with the same landmark IDs concurrently.
The historical PoseStamped fixture is retained as a non-installed reference.

The standalone helper still requires real camera/odometry inputs for SLAM; a
synthetic landmark alone does not make RTAB-Map work. Production slam_bringup no
longer accepts mock-startup options. Costmap artificial localization belongs in
explicit testing fixtures; production costmap.launch only accepts standalone:=false.
