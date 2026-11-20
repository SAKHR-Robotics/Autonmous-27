import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('erc_path_planner')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'odom_topic': LaunchConfiguration('odom_topic')},
        convert_types=True), allow_substs=True)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('manage_lifecycle', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params, {
                 'default_nav_to_pose_bt_xml': os.path.join(share, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(share, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             condition=IfCondition(LaunchConfiguration('manage_lifecycle')),
             parameters=[{'use_sim_time': clock, 'autostart': autostart,
                          'node_names': ['behavior_server', 'velocity_smoother', 'bt_navigator'], 'bond_timeout': 4.0}]),
    ])
