#!/usr/bin/env python3
"""
test_perception_standalone.launch.py - Standalone Testing Launcher for Perception Subsystem

Spins up the perception system (terrain_geometry + marker_detection) with
costmap rasterization enabled for visual inspection in RViz2, plus a static
identity transform (map -> odom) so RViz's default fixed frame 'map' works
without SLAM running.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('terrain_geometry')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_map_anchor = LaunchConfiguration('publish_map_anchor')
    enable_costmap = LaunchConfiguration('enable_costmap')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_publish_map_anchor = DeclareLaunchArgument(
        'publish_map_anchor',
        default_value='true',
        description='Publish static map -> odom transform so RViz map frame resolves without SLAM'
    )
    declare_enable_costmap = DeclareLaunchArgument(
        'enable_costmap',
        default_value='true',
        description='Generate and inflate 2D costmap on /terrain/costmap for visual verification'
    )
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 visualizer'
    )

    # Static identity anchor: map -> odom (for RViz visualization without SLAM)
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='standalone_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_map_anchor),
        output='screen'
    )

    perception_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception_system.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_costmap': enable_costmap,
            'launch_rviz': launch_rviz,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_publish_map_anchor,
        declare_enable_costmap,
        declare_launch_rviz,
        map_to_odom_node,
        perception_system,
    ])
