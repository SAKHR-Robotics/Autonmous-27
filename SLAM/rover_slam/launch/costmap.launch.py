import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    costmap_config_path = os.path.join(pkg_share, 'config', 'costmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    standalone = LaunchConfiguration('standalone')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 lifecycle nodes'
    )

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        choices=['false'], description='Deprecated: test TF belongs to slam_test_tools'
    )

    # Nav2 Costmap 2D Lifecycle Node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
             'robot_base_frame':LaunchConfiguration('base_frame'), 'global_frame':LaunchConfiguration('map_frame'),
             'obstacle_layer.rock_obstacles.topic': LaunchConfiguration('obstacle_topic'),
             'obstacle_layer.clearing_depth.topic': LaunchConfiguration('clearing_cloud_topic')}
        ],
        remappings=[
            ('costmap', '/global_costmap/costmap'),
            ('costmap_updates', '/global_costmap/costmap_updates'),
            ('/costmap/costmap', '/global_costmap/costmap'),
            ('/costmap/costmap_updates', '/global_costmap/costmap_updates'),
        ]
    )

    # Nav2 Lifecycle Manager to automatically configure and activate costmap
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='costmap_lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'autostart': ParameterValue(autostart, value_type=bool)},
            {'bond_timeout': 0.0},
            {'node_names': ['costmap']}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('params_file', default_value=costmap_config_path),
        DeclareLaunchArgument('obstacle_topic', default_value='/bridge/pointcloud'),
        DeclareLaunchArgument('clearing_cloud_topic', default_value='/camera/depth/color/points'),
        declare_use_sim_time,
        declare_autostart,
        declare_standalone,
        costmap_node,
        lifecycle_manager_node
    ])
