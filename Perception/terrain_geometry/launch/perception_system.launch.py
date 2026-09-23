"""
perception_system.launch.py

Unified Perception Subsystem Launcher.
Spins up both core perception pipelines concurrently:
  1. terrain_geometry (terrain_node): TF transform, ground segmentation,
     voxel downsampling, DBSCAN rock clustering, 2D costmap inflation.
  2. marker_detection (marker_branch): 2D ArUco detection, SolvePnP 3D pose,
     Kalman tracking, marker TF broadcaster, and global marker mapping.
  3. Optional: RViz2 visualization.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    terrain_share = get_package_share_directory("terrain_geometry")
    marker_share = get_package_share_directory("marker_detection")

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz2 visualizer if true.",
    )
    launch_marker_branch_arg = DeclareLaunchArgument(
        "launch_marker_branch",
        default_value="true",
        description="Launch complete marker branch (detection + TF + map + action).",
    )

    # Unified Camera & Sensor Topics
    input_pointcloud_topic_arg = DeclareLaunchArgument(
        "input_pointcloud_topic",
        default_value="/camera/depth/color/points",
        description="Raw 3D PointCloud2 input topic for terrain geometry.",
    )
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/image_raw",
        description="RGB image topic for ArUco marker detection.",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/depth/image_raw",
        description="Depth image topic for marker 3D position estimation.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera_info",
        description="Camera calibration CameraInfo topic.",
    )
    enable_costmap_arg = DeclareLaunchArgument(
        "enable_costmap",
        default_value="false",
        description="Generate and inflate 2D costmap on /terrain/costmap if true (default false to save CPU).",
    )

    # 1. Terrain Geometry Pipeline
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_share, "launch", "terrain.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "input_pointcloud_topic": LaunchConfiguration("input_pointcloud_topic"),
            "enable_costmap": LaunchConfiguration("enable_costmap"),
        }.items(),
    )

    # 2. Marker Detection & Tracking Pipeline
    marker_branch_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(marker_share, "launch", "marker_branch.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("launch_marker_branch")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }.items(),
    )

    # 3. Optional RViz2
    rviz_config = os.path.join(terrain_share, "rviz", "terrain_geometry_view.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_perception",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        launch_marker_branch_arg,
        input_pointcloud_topic_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        enable_costmap_arg,
        terrain_launch,
        marker_branch_launch,
        rviz_node,
    ])
