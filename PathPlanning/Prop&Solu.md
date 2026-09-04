# ERC Path Planning: Problems & Complete Solutions Guide (Prop&Solu)

This document provides an exhaustive breakdown of every architectural, algorithmic, and configuration problem identified in the **ERC 2026 Path Planning Subsystem**, along with the exact, step-by-step engineering solutions to resolve them.

---

## Summary Matrix of Problems & Solutions

| # | Problem Area | Root Cause | Impact | Solution |
|---|---|---|---|---|
| **1** | **Launch File** | `costmap_bridge_node` is not included in `path_planning.launch.py`. | Perception data never reaches Nav2 costmaps. | Add `Node(package='erc_path_planner', executable='costmap_bridge_node')` to launch description. |
| **2** | **Perception Bridge** | `costmap_bridge_node.cpp` mismatched with perception and centroid-only. | Bypasses persistent memory; obstacles vanish in blind spots & rover clips edges. | Subscribe to `/perception/obstacles_only` (`vision_msgs/Detection3DArray`) and sample full 3D bounding box footprint. |
| **3** | **Global Costmap** | `global_costmap` lacks an `obstacle_layer` in `nav2_params.yaml`. | Global planner paths directly through new obstacles; local controller gets trapped. | Add `obstacle_layer` to `global_costmap` plugins and configure observation sources. |
| **4** | **Robot Geometry** | No `robot_radius` or `footprint` defined in costmaps. | Inflation layer uses tiny default radius; rover body collisions occur. | Define exact polygon `footprint: "[[0.5, 0.4], [0.5, -0.4], [-0.5, -0.4], [-0.5, 0.4]]"` or `robot_radius: 0.5`. |
| **5** | **Collision Monitor** | Configured for 2D laser `scan` instead of 3D RealSense pointcloud. | Collision monitor fails/timeouts or stays permanently inactive. | Update observation source from `scan` to `pointcloud` on `/bridge/pointcloud`. |
| **6** | **Velocity Stamping** | `enable_stamped_cmd_vel: true` outputs `TwistStamped` instead of `Twist`. | Motor driver expecting `geometry_msgs/Twist` cannot parse commands. | Align stamping across Nav2 (`enable_stamped_cmd_vel: false`) or add conversion bridge. |
| **7** | **Behavior Trees** | Hardcoded `/opt/ros/jazzy/...` absolute paths in `nav2_params.yaml`. | System breaks across different environments and lacks custom replanning logic. | Store custom BT XML inside `behavior_trees/` and reference via package share directory. |
| **8** | **Costmap Clearing** | `clearing: true` on centroid-only pointcloud creates ghost obstacles. | Cleared obstacles linger forever because no raytracing occurs through empty space. | Configure proper `raytrace_max_range`, voxel decay, or costmap clearance recovery. |

---

## Detailed Problems & Step-by-Step Solutions

### Problem 1: `costmap_bridge_node` is Not Started in Launch File

#### Root Cause
In `launch/path_planning.launch.py`, only `nav2_bringup/bringup_launch.py` is invoked. The executable `costmap_bridge_node` built from `src/costmap_bridge_node.cpp` is never instantiated.

#### Engineering Solution
Modify `PathPlanning/erc_path_planner/launch/path_planning.launch.py` to import `Node` from `launch_ros.actions` and launch `costmap_bridge_node`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    params_file = os.path.join(erc_path_planner_dir, 'config', 'nav2_params.yaml')
    default_map = os.path.join(erc_path_planner_dir, 'config', 'dummy_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    # Launch Nav2 Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'use_composition': 'False',
            'autostart': 'true'
        }.items()
    )

    # Launch Costmap Perception Bridge Node
    bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        nav2_launch,
        bridge_node
    ])
```

---

### Problem 2: Perception Interface Mismatch & Centroid-Only Representation

#### Root Cause
1. **Interface Mismatch:** `src/costmap_bridge_node.cpp` was originally listening to `/terrain/obstacle_features` (`terrain_geometry_msgs/ObstacleFeatureArray`), which bypasses the Persistent Memory Node (`persistent_memory_node`). When the camera rotates away into blind spots, obstacles disappear immediately.
2. **Point Volume Loss:** The conversion loop only wrote a single point at obstacle centroids, treating a $1.5\text{ m}$ boulder as a single $5\text{ cm}$ cell.

#### Standardized Engineering Solution (Option 1 - `vision_msgs` Standard)
Update `costmap_bridge_node.cpp` to subscribe to the standardized persistent topic **`/perception/obstacles_only`** (`vision_msgs/msg/Detection3DArray`) published by the Persistent Memory Node, and sample the full 3D bounding box footprint into `sensor_msgs/msg/PointCloud2`:

```cpp
#include "vision_msgs/msg/detection3_d_array.hpp"

void obstacle_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::Point> sampled_points;
  const float step = 0.05f; // 5cm grid resolution matching costmap

  for (const auto & detection : msg->detections) {
    const auto & center = detection.bbox.center.position;
    const auto & size = detection.bbox.size;

    float min_x = center.x - size.x / 2.0f;
    float max_x = center.x + size.x / 2.0f;
    float min_y = center.y - size.y / 2.0f;
    float max_y = center.y + size.y / 2.0f;

    // Sample bounding box volume/perimeter into point grid
    for (float x = min_x; x <= max_x; x += step) {
      for (float y = min_y; y <= max_y; y += step) {
        geometry_msgs::msg::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = center.z;
        sampled_points.push_back(pt);
      }
    }
  }

  // Create PointCloud2 with all sampled boundary points
  sensor_msgs::msg::PointCloud2 pointcloud;
  pointcloud.header = msg->header;
  pointcloud.height = 1;
  pointcloud.width = static_cast<uint32_t>(sampled_points.size());

  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(sampled_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

  for (const auto & pt : sampled_points) {
    *iter_x = static_cast<float>(pt.x);
    *iter_y = static_cast<float>(pt.y);
    *iter_z = static_cast<float>(pt.z);
    ++iter_x; ++iter_y; ++iter_z;
  }

  pointcloud_publisher_->publish(pointcloud);
}
```

---

### Problem 3: Global Costmap Blind to Real-Time Obstacles

#### Root Cause
In `config/nav2_params.yaml`, `global_costmap` only includes `static_layer` and `inflation_layer`. When the rover's camera detects an unmapped rock or crater, the obstacle is added ONLY to the local costmap. The Global Planner continues to plan straight through the obstacle, creating an unresolvable conflict between global route and local controller.

#### Engineering Solution
Add the `obstacle_layer` to `global_costmap` in `nav2_params.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      always_send_full_costmap: true
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map
        subscribe_to_updates: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: obstacles
        obstacles:
          topic: /bridge/pointcloud
          data_type: PointCloud2
          marking: true
          clearing: false
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
          obstacle_max_range: 8.0
          raytrace_max_range: 10.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.75
```

---

### Problem 4: Missing Robot Dimensions & Footprint

#### Root Cause
Neither costmap defines the physical extent of the rover (`robot_radius` or `footprint`), causing Nav2 to assume a default minimal circle. The inflation layer cannot provide adequate clearance.

#### Engineering Solution
Measure the rover's physical dimensions (including wheels and frame) and specify the bounding polygon in `nav2_params.yaml`:

```yaml
# Add inside global_costmap.global_costmap.ros__parameters
# and inside local_costmap.local_costmap.ros__parameters:
footprint: "[[0.60, 0.45], [0.60, -0.45], [-0.60, -0.45], [-0.60, 0.45]]"
footprint_padding: 0.05
```

---

### Problem 5: Collision Monitor Sensor Configuration Mismatch

#### Root Cause
`collision_monitor` is configured with `type: "scan"` subscribing to topic `scan` (`sensor_msgs/LaserScan`). The rover does not possess a 2D LiDAR; perception produces a 3D PointCloud.

#### Engineering Solution
Change the `collision_monitor` source to `pointcloud` in `nav2_params.yaml`:

```yaml
collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.5
    source_timeout: 5.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      enabled: true
    observation_sources: ["pointcloud_source"]
    pointcloud_source:
      type: "pointcloud"
      topic: "/bridge/pointcloud"
      min_height: 0.15
      max_height: 2.0
      enabled: true
```

---

### Problem 6: `cmd_vel` Stamping Mismatch (`TwistStamped` vs `Twist`)

#### Root Cause
ROS 2 Jazzy Nav2 defaults to `geometry_msgs/msg/TwistStamped` when `enable_stamped_cmd_vel: true`. The rover's `motor_driver` expects `geometry_msgs/msg/Twist`.

#### Engineering Solution
Choose one of two standard approaches:
* **Option A (Simpler)**: Set `enable_stamped_cmd_vel: false` across all nodes in `nav2_params.yaml` (`controller_server`, `velocity_smoother`, `behavior_server`, `collision_monitor`).
* **Option B (Recommended for Jazzy)**: Update `motor_driver` to subscribe to `geometry_msgs/msg/TwistStamped` for synchronized timestamp checking.

---

### Problem 7: Hardcoded Behavior Tree Paths

#### Root Cause
`nav2_params.yaml` references `/opt/ros/jazzy/share/nav2_bt_navigator/...` directly, which causes runtime errors if run on other installations or containerized setups.

#### Engineering Solution
1. Create a custom behavior tree file: `behavior_trees/erc_navigate_w_replanning.xml`.
2. Configure `bt_navigator` in `nav2_params.yaml` with clean package-relative substitutions or launch-level parameter overrides.

---

### Problem 8: Phantom / Ghost Obstacle Accumulation

#### Root Cause
Because `costmap_bridge_node` generates obstacle points without raytracing empty space between the camera origin and the obstacle, dynamic obstacles or false positives will get marked as lethal and never cleared.

#### Engineering Solution
1. In `local_costmap`, ensure `rolling_window: true` so obstacles outside the local vicinity naturally scroll off.
2. Configure Nav2 `ClearEntireCostmap` or `ClearCostmapExceptRegion` recovery behaviors in the Behavior Tree when the rover fails to progress.
3. For static terrain, allow the perception module's persistent memory node (`EMA filter`) to confirm/decay obstacles before sending them to the bridge node.
