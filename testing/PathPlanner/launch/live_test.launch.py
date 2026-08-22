#!/usr/bin/env python3
"""
live_test.launch.py

Master Launch file for Standalone Path Planner & MPPI Controller Live Testing.
Integrates:
- erc_path_planner Nav2 bringup (Smac Hybrid A* + MPPI Controller)
- costmap_bridge_node
- mock_rover_sim (Kinematic Rover Physics, 50Hz Odom & dynamic TF broadcaster)
- mock_perception (Terrain geometry obstacle feature generator)
- RViz2 dashboard with real-time path tracking and moving rover visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_benchmarking_dir = get_package_share_directory('global_path_benchmarking')
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    default_rviz_config = os.path.join(
        pkg_benchmarking_dir, 'rviz', 'live_path_tracking.rviz'
    )
    # Fallback to local rviz if running from source
    if not os.path.exists(default_rviz_config):
        default_rviz_config = os.path.join(
            erc_path_planner_dir, 'rviz', 'nav2_default_view.rviz'
        )

    default_map = os.path.join(
        erc_path_planner_dir, 'config', 'dummy_map.yaml'
    )

    # Launch Arguments
    use_mock_rover_arg = DeclareLaunchArgument(
        'use_mock_rover',
        default_value='true',
        description='Launch mock rover kinematics & TF broadcaster'
    )

    use_mock_perception_arg = DeclareLaunchArgument(
        'use_mock_perception',
        default_value='true',
        description='Launch mock perception obstacle feature generator'
    )

    use_costmap_bridge_arg = DeclareLaunchArgument(
        'use_costmap_bridge',
        default_value='true',
        description='Launch costmap_bridge_node for perception to pointcloud conversion'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 live visualizer'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map yaml file'
    )

    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.0')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.0')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    # 1. Nav2 Path Planning Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(erc_path_planner_dir, 'launch', 'path_planning.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # 2. Costmap Bridge Node
    costmap_bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_costmap_bridge'))
    )

    # 3. Mock Rover Simulator Node
    mock_rover_node = Node(
        package='global_path_benchmarking',
        executable='mock_rover_sim',
        name='mock_rover_sim',
        output='screen',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'update_rate': 50.0,
            'publish_tf': True
        }],
        condition=IfCondition(LaunchConfiguration('use_mock_rover'))
    )

    # 4. Mock Perception Node
    mock_perception_node = Node(
        package='global_path_benchmarking',
        executable='mock_perception',
        name='mock_perception',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock_perception'))
    )

    # 5. RViz2 Live Dashboard
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_mock_rover_arg,
        use_mock_perception_arg,
        use_costmap_bridge_arg,
        use_rviz_arg,
        map_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,

        nav2_bringup,
        costmap_bridge_node,
        mock_rover_node,
        mock_perception_node,
        rviz_node
    ])
