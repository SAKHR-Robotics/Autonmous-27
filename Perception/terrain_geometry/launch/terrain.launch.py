"""
terrain.launch.py

Launches the unified terrain_node with configurable parameters for
every one of its 8 pipeline stages.

Example override:
    ros2 launch terrain_geometry terrain.launch.py \\
        input_pointcloud_topic:=/camera/depth/color/points \\
        search_radius:=0.15 min_neighbors:=6 \\
        cluster_eps:=0.25 publish_debug_topics:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("input_pointcloud_topic", default_value="/camera/depth/color/points"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("input_qos_depth", default_value="1"),

        # ROI filter (applied after TF transform, before ground removal)
        DeclareLaunchArgument("enable_roi_filter", default_value="true"),
        DeclareLaunchArgument("roi_min_x", default_value="0.2"),
        DeclareLaunchArgument("roi_max_x", default_value="8.0"),
        DeclareLaunchArgument("roi_min_y", default_value="-4.0"),
        DeclareLaunchArgument("roi_max_y", default_value="4.0"),
        DeclareLaunchArgument("roi_min_z", default_value="-0.5"),
        DeclareLaunchArgument("roi_max_z", default_value="1.5"),

        # Step 2: ground removal
        DeclareLaunchArgument("ground_sensor_height", default_value="0.2"),
        DeclareLaunchArgument("ground_num_zones", default_value="4"),
        DeclareLaunchArgument("ground_min_range", default_value="0.2"),
        DeclareLaunchArgument("ground_max_range", default_value="10.0"),
        DeclareLaunchArgument("ground_height_threshold", default_value="0.15"),
        # One of "patchwork", "fallback", "auto"
        DeclareLaunchArgument("ground_removal_backend", default_value="auto"),

        # Step 3: voxel downsampling
        DeclareLaunchArgument("leaf_size_x", default_value="0.05"),
        DeclareLaunchArgument("leaf_size_y", default_value="0.05"),
        DeclareLaunchArgument("leaf_size_z", default_value="0.05"),

        # Step 4: radius outlier removal
        DeclareLaunchArgument("enable_radius_outlier_removal", default_value="true"),
        DeclareLaunchArgument("search_radius", default_value="0.2"),
        DeclareLaunchArgument("min_neighbors", default_value="5"),
        DeclareLaunchArgument("outlier_removal_max_workers", default_value="-1"),

        # Step 5: clustering
        DeclareLaunchArgument("enable_clustering", default_value="true"),
        DeclareLaunchArgument("cluster_eps", default_value="0.3"),
        DeclareLaunchArgument("cluster_min_points", default_value="5"),
        DeclareLaunchArgument("min_cluster_size", default_value="10"),
        DeclareLaunchArgument("max_cluster_size", default_value="50000"),

        # Step 6: feature extraction / visualization
        DeclareLaunchArgument("enable_obb", default_value="false"),
        DeclareLaunchArgument("enable_marker_visualization", default_value="true"),

        # Tracking
        DeclareLaunchArgument("enable_obstacle_tracking", default_value="false"),
        DeclareLaunchArgument("tracking_max_association_distance", default_value="0.5"),
        DeclareLaunchArgument("tracking_max_missed_frames", default_value="3"),
        DeclareLaunchArgument("tracking_position_smoothing", default_value="0.5"),

        # Step 7: occupancy grid
        DeclareLaunchArgument("grid_resolution", default_value="0.05"),
        DeclareLaunchArgument("grid_width_cells", default_value="200"),
        DeclareLaunchArgument("grid_height_cells", default_value="200"),
        DeclareLaunchArgument("grid_origin_x", default_value="-5.0"),
        DeclareLaunchArgument("grid_origin_y", default_value="-5.0"),
        DeclareLaunchArgument("grid_origin_z", default_value="0.0"),
        DeclareLaunchArgument("use_unknown_space", default_value="false"),

        # Step 8: costmap inflation
        DeclareLaunchArgument("robot_radius", default_value="0.3"),
        DeclareLaunchArgument("costmap_inflation_radius", default_value="0.6"),
        DeclareLaunchArgument("cost_scaling_factor", default_value="10.0"),
        DeclareLaunchArgument("inflate_unknown_cells", default_value="false"),

        # Debug
        DeclareLaunchArgument("publish_debug_topics", default_value="false"),

        # Performance profiling
        DeclareLaunchArgument("enable_performance_profiling", default_value="false"),
        DeclareLaunchArgument("profiling_interval", default_value="30"),
    ]

    param_names = [
        "input_pointcloud_topic", "target_frame", "tf_timeout_sec", "input_qos_depth",
        "enable_roi_filter", "roi_min_x", "roi_max_x",
        "roi_min_y", "roi_max_y", "roi_min_z", "roi_max_z",
        "ground_sensor_height", "ground_num_zones", "ground_min_range",
        "ground_max_range", "ground_height_threshold", "ground_removal_backend",
        "leaf_size_x", "leaf_size_y", "leaf_size_z",
        "enable_radius_outlier_removal", "search_radius", "min_neighbors",
        "outlier_removal_max_workers",
        "enable_clustering", "cluster_eps", "cluster_min_points",
        "min_cluster_size", "max_cluster_size",
        "enable_obb", "enable_marker_visualization",
        "enable_obstacle_tracking", "tracking_max_association_distance",
        "tracking_max_missed_frames", "tracking_position_smoothing",
        "grid_resolution", "grid_width_cells", "grid_height_cells",
        "grid_origin_x", "grid_origin_y", "grid_origin_z", "use_unknown_space",
        "robot_radius", "costmap_inflation_radius", "cost_scaling_factor",
        "inflate_unknown_cells", "publish_debug_topics",
        "enable_performance_profiling", "profiling_interval",
    ]

    terrain_node = Node(
        package="terrain_geometry",
        executable="terrain_node",
        name="terrain_node",
        output="screen",
        emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name in param_names}],
    )

    return LaunchDescription(args + [terrain_node])
