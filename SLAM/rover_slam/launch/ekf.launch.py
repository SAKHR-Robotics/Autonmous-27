import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Raw odometry topic (e.g. /odom for Gazebo or /wheel/odom_raw for physical robot)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_topic,
        # Encoder Ticks Pre-Processor
        Node(
            package='rover_slam',
            executable='encoder_ticks_to_odom',
            name='encoder_ticks_to_odom',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # Heuristic Slip Checker Pre-Filter
        Node(
            package='rover_slam',
            executable='heuristic_slip_checker',
            name='heuristic_slip_checker',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/wheel/odom_raw', odom_topic)]
        ),
        # Local EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/wheel/odom_raw', odom_topic)
            ]
        )
    ])
