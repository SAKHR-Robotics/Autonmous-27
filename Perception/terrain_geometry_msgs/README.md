# Terrain Geometry Message Interfaces (`terrain_geometry_msgs`)

Custom ROS 2 message package providing geometric obstacle feature definitions extracted by the `terrain_geometry` perception pipeline.

---

## 📌 Message Definitions

### 1. `ObstacleFeature.msg`
Defines 3D geometric properties for an individual segmented terrain cluster/rock:

```
int32 id                 # Stable cluster ID within current frame
int32 num_points         # Number of inlier point cloud points in this cluster

geometry_msgs/Point centroid   # Arithmetic mean (x, y, z) in target frame
geometry_msgs/Point min_point  # Axis-aligned bounding box minimum corner
geometry_msgs/Point max_point  # Axis-aligned bounding box maximum corner

float32 width            # Lateral extent along Y (left-right, meters)
float32 height           # Vertical extent along Z (up-down, meters)
float32 depth            # Longitudinal extent along X (front-back, meters)

float32 distance         # Euclidean distance from robot origin to centroid
```

### 2. `ObstacleFeatureArray.msg`
Headered collection of all detected obstacles in the current point cloud frame:

```
std_msgs/Header header
ObstacleFeature[] obstacles
```

---

## 🔄 Relationship with `vision_msgs`

- `ObstacleFeatureArray` is published on `/terrain/obstacle_features` for internal perception diagnostics and custom visualization.
- For downstream consumption by Nav2 (`erc_path_planner/costmap_bridge_node`) and Persistent Memory, `terrain_node.py` also standardizes and publishes `vision_msgs/msg/Detection3DArray` on `/perception/local_bboxes` and `/perception/obstacles_only`.

---

## 🛠️ Build & Installation

```bash
colcon build --symlink-install --packages-select terrain_geometry_msgs
source install/setup.bash
```
Verify message registration:
```bash
ros2 interface show terrain_geometry_msgs/msg/ObstacleFeature
ros2 interface show terrain_geometry_msgs/msg/ObstacleFeatureArray
```
