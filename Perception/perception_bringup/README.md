# perception_bringup

Owns whole-Perception startup and the combined RViz configuration. It includes
the independent terrain and marker launches; it contains no detection logic,
camera driver, mock publisher or localization transform producer.

```bash
ros2 launch perception_bringup perception.launch.py use_sim_time:=true
ros2 launch perception_bringup perception.launch.py --show-args
```

Use wall time (`use_sim_time:=false`) for physical/synthetic wall-clock inputs.
The inherited default is `true`; a simulation/bag clock must actually be present.
Pass time explicitly in validation commands. All selected nodes use that value.

| Selection | Current default |
| --- | --- |
| Terrain processing | On |
| Marker detection/tracking, marker TF, marker action interface | On |
| Marker mapping, memory, landmark adapter | Off |
| RViz, terrain costmap, terrain-risk checks, terrain debug clouds | Off |

`launch_terrain` and `launch_marker_branch` select the two branches. Existing
marker stage switches and their individual `*_params_file` arguments are forwarded
to the marker package. `memory_params_file` configures optional memory. Camera
input topics, target frame and risk parameters are declared launch arguments.
Use the independent terrain launch for the full filter-tuning surface.

Memory in the combined launch requires the terrain branch. When selected,
terrain stops publishing obstacle updates on `/perception/obstacles_only` and
memory owns them. The current-frame `/perception/local_bboxes` output remains.
Missing localization is never replaced by fabricated TF.

## Visualization

```bash
# Processing plus combined RViz:
ros2 launch perception_bringup perception.launch.py use_sim_time:=true launch_rviz:=true
# Combined RViz only:
ros2 launch perception_bringup rviz.launch.py use_sim_time:=true
```

RViz accepts `fixed_frame`, `rviz_config`, `rgb_topic` and
`input_pointcloud_topic`. The combined launch exposes `rviz_fixed_frame` and
forwards its selected camera topics. Sensor displays use Best Effort.
RobotModel, raw/pose-debug images, costmap, debug clouds and global marker map displays start disabled
because their external/optional producers may be absent. Enable a display only
after selecting its producer and checking TF. There are no testing truth overlays.

The old terrain combined-launch filename forwards here and preserves its
arguments. Build this package for that compatibility command; terrain-only
users need neither marker processing nor bringup installed.

See the [launch inventory and sensor setup](../RUNBOOK.md) and
[Docker validation commands](../TESTING_GUIDE.md).
