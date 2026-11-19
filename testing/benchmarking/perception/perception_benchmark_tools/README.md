# perception_benchmark_tools

Relocated offline timing runner from `terrain_geometry/benchmark.py`. The
artificial floor-and-box generator now belongs to
`perception_test_tools.terrain_scene` under `testing/mocks/perception/`.
Production `terrain_geometry` does not depend on either testing package.

Build from the workspace root inside `autonomous27:jazzy`, using an isolated
overlay so an existing installation does not hide packaging errors:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/perception-benchmark-validation/log build \
  --base-paths Perception testing/mocks/perception/perception_test_tools \
    testing/benchmarking/perception/perception_benchmark_tools \
  --packages-up-to perception_benchmark_tools --symlink-install \
  --build-base /tmp/perception-benchmark-validation/build \
  --install-base /tmp/perception-benchmark-validation/install
source /tmp/perception-benchmark-validation/install/setup.bash
```

Then check the installed executable and run a short sample:

```bash
ros2 pkg executables perception_benchmark_tools
ros2 run perception_benchmark_tools terrain_benchmark --synthetic --frames 3
ros2 run perception_benchmark_tools terrain_benchmark --synthetic --frames 3 --enable-tracking
```

This command creates no ROS publishers, TF, clock, simulator or RViz process.
It runs production helper classes and reports computation latency and point
counts. It does not test the production node's decoding, TF, ROS transport,
health behavior or output messages, and does not establish obstacle accuracy.
The existing generator and processing settings are preserved during relocation;
the artificial ground near Z=0 is not calibrated rover geometry. The known
ground-classification failure remains open. FPS here excludes sensor transport
and ROS overhead and is not a measured production node rate.

`--bag PATH --topic TOPIC` retains the existing SQLite3 reader. It decodes XYZ
without replaying TF or transforming the points. Inputs must already use the
intended Z-up processing coordinates; raw camera optical coordinates are not
equivalent. Bag replay and geometry accuracy are not validated by this move.

The old `terrain_geometry.benchmark` module is removed rather than forwarding
production imports into testing. Use the installed command above.
