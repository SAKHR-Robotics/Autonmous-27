import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'use_sim_time':'false','odom_topic':'/wheel/odom_raw','launch_encoder':'false',
          'launch_slip':'true','encoder_input_mode':'joint_states','imu_topic':'/imu/data',
          'expected_frequency_hz':'30.0'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[
        GroupAction(scoped=True,actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_slam'),'launch','ekf.launch.py')),
            launch_arguments={k:LaunchConfiguration(k) for k in defaults if k!='expected_frequency_hz'}.items())]),
        Node(package='slam_test_tools',executable='ekf_validator',name='ekf_validator',output='screen',
            parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                'expected_frequency_hz':ParameterValue(LaunchConfiguration('expected_frequency_hz'),value_type=float)}])])
