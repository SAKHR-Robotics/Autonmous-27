#!/usr/bin/env python3
"""
live_test.launch.py

Master Universal Launch File for Path Planner & Control Module Live Testing.
Can test ANY path planner and controller module:
- Integrated Nav2 stack (erc_path_planner by default)
- OR any custom external ROS 2 planner/controller node running in another terminal
- Mock rover kinematic physics & TF broadcaster (50 Hz)
- Universal mock perception (PointCloud2, LaserScan, ObstacleFeatureArray)
- Live RViz2 dashboard with real-time trajectory visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_benchmarking_dir = get_package_share_directory('global_path_benchmarking')

    try:
        erc_path_planner_dir = get_package_share_directory('erc_path_planner')
        default_map = os.path.join(erc_path_planner_dir, 'config', 'dummy_map.yaml')
        default_fallback_rviz = os.path.join(erc_path_planner_dir, 'rviz', 'nav2_default_view.rviz')
    except Exception:
        erc_path_planner_dir = ''
        default_map = ''
        default_fallback_rviz = ''

    default_rviz_config = os.path.join(
        pkg_benchmarking_dir, 'rviz', 'live_path_tracking.rviz'
    )
    if not os.path.exists(default_rviz_config) and default_fallback_rviz:
        default_rviz_config = default_fallback_rviz

    # =========================================================================
    # Universal Target Module Arguments
    # =========================================================================
    use_external_module_arg = DeclareLaunchArgument(
        'use_external_module',
        default_value='false',
        description='Set true to test an external planner/controller node without launching erc_path_planner'
    )

    target_package_arg = DeclareLaunchArgument(
        'target_package',
        default_value='erc_path_planner',
        description='ROS 2 package of the planner/controller to launch'
    )

    target_launch_arg = DeclareLaunchArgument(
        'target_launch',
        default_value='path_planning.launch.py',
        description='Launch file name of the target planner/controller'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Command velocity topic to monitor and simulate'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='cmd_vel',
        description='Control evaluation mode: "cmd_vel" (local planner) or "motor_rpm" (motor driver)'
    )

    scenario_id_arg = DeclareLaunchArgument(
        'scenario_id',
        default_value='',
        description='Scenario ID for timed obstacle injection (optional)'
    )

    # =========================================================================
    # Simulation & Visualizer Arguments
    # =========================================================================
    use_mock_rover_arg = DeclareLaunchArgument(
        'use_mock_rover',
        default_value='true',
        description='Launch universal mock rover kinematics & TF broadcaster'
    )

    use_mock_perception_arg = DeclareLaunchArgument(
        'use_mock_perception',
        default_value='true',
        description='Launch universal mock perception generator (PointCloud2, LaserScan, Features)'
    )

    use_costmap_bridge_arg = DeclareLaunchArgument(
        'use_costmap_bridge',
        default_value='true',
        description='Launch costmap_bridge_node if using terrain_geometry_msgs with Nav2'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 live visualizer dashboard'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map yaml file'
    )

    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.0')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.0')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    # =========================================================================
    # Nodes & Subsystems
    # =========================================================================

    # 1. Target Planner Bringup (Skipped if use_external_module is true)
    launch_target_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('use_external_module'), "' == 'false' and '",
            LaunchConfiguration('target_package'), "' != ''"
        ])
    )

    def get_target_launch_action():
        if erc_path_planner_dir and os.path.exists(os.path.join(erc_path_planner_dir, 'launch', 'path_planning.launch.py')):
            return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(erc_path_planner_dir, 'launch', 'path_planning.launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'false',
                    'autostart': 'true'
                }.items(),
                condition=launch_target_condition
            )
        return GroupAction([])

    # 2. Costmap Bridge Node (Only if using internal erc_path_planner stack)
    costmap_bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_costmap_bridge'), "' == 'true' and '",
                LaunchConfiguration('use_external_module'), "' == 'false'"
            ])
        )
    )

    # 3. Universal Mock Rover Simulator Node
    mock_rover_node = Node(
        package='global_path_benchmarking',
        executable='mock_rover_sim',
        name='mock_rover_sim',
        output='screen',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'control_mode': LaunchConfiguration('control_mode'),
            'update_rate': 50.0,
            'publish_tf': True
        }],
        condition=IfCondition(LaunchConfiguration('use_mock_rover'))
    )

    # 4. Universal Mock Perception Node
    mock_perception_node = Node(
        package='global_path_benchmarking',
        executable='mock_perception',
        name='mock_perception',
        output='screen',
        parameters=[{
            'scenario_id': LaunchConfiguration('scenario_id'),
            'publish_rate': 10.0
        }],
        condition=IfCondition(LaunchConfiguration('use_mock_perception'))
    )

    # 5. RViz2 Live Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_external_module_arg,
        target_package_arg,
        target_launch_arg,
        cmd_vel_topic_arg,
        control_mode_arg,
        scenario_id_arg,

        use_mock_rover_arg,
        use_mock_perception_arg,
        use_costmap_bridge_arg,
        use_rviz_arg,
        map_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,

        get_target_launch_action(),
        costmap_bridge_node,
        mock_rover_node,
        mock_perception_node,
        rviz_node
    ])
