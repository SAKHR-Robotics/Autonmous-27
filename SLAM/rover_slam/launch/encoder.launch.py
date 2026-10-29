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
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'encoder.yaml')),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('input_mode', default_value='joint_states', choices=['joint_states', 'ticks_array', 'individual_ticks']),
        Node(package='rover_slam', executable='encoder_ticks_to_odom', name='encoder_ticks_to_odom', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'input_mode': LaunchConfiguration('input_mode'), 'publish_tf': False,
                 'odom_frame_id':LaunchConfiguration('odom_frame'), 'base_frame_id':LaunchConfiguration('base_frame')}])
    ])
