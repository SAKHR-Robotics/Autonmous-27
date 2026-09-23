#!/usr/bin/env python3
"""
test_planner_standalone.launch.py - Standalone Testing Launcher for Nav2 Path Planner

Spins up the Nav2 stack (Smac Hybrid A*, MPPI Controller, costmap bridge)
in standalone testing mode with map_server running dummy_map.yaml (or custom test map)
and an optional static identity anchor (map -> odom) so navigation works without SLAM.
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
    pkg_share = get_package_share_directory('erc_path_planner')
    default_map = os.path.join(pkg_share, 'config', 'dummy_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    publish_map_anchor = LaunchConfiguration('publish_map_anchor')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load for standalone navigation'
    )
    declare_publish_map_anchor = DeclareLaunchArgument(
        'publish_map_anchor',
        default_value='true',
        description='Publish static map -> odom transform for standalone navigation without SLAM'
    )

    # Static anchor: map -> odom (for standalone planning tests without SLAM)
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='standalone_planner_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_map_anchor),
        output='screen'
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'path_planning.launch.py')
        ),
        launch_arguments={
            'use_slam': 'false',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_publish_map_anchor,
        map_to_odom_node,
        planner_launch,
    ])
