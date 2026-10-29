import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rover_slam')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'slip.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='/wheel/odom_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        Node(package='rover_slam', executable='heuristic_slip_checker', name='heuristic_slip_checker', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'raw_odom_topic': LaunchConfiguration('odom_topic'), 'imu_topic': LaunchConfiguration('imu_topic')}])
    ])
