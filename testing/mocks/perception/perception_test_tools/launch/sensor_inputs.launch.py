from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='rocks_and_marker'),
        DeclareLaunchArgument('synthetic_motion', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ground_z_offset', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('rate', default_value='10.0'),
        Node(
            package='perception_test_tools',
            executable='sensor_inputs',
            name='perception_test_inputs',
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'synthetic_motion': ParameterValue(LaunchConfiguration('synthetic_motion'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'ground_z_offset': ParameterValue(LaunchConfiguration('ground_z_offset'), value_type=float),
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                'rate': ParameterValue(LaunchConfiguration('rate'), value_type=float),
            }],
        ),
    ])
