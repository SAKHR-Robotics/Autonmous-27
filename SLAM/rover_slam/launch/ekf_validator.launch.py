"""Compatibility launch: explicit evaluation belongs to slam_test_tools."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    keys={'use_sim_time':'false','odom_topic':'/wheel/odom_raw','launch_encoder':'false',
          'launch_slip':'true','encoder_input_mode':'joint_states','imu_topic':'/imu/data',
          'expected_frequency_hz':'30.0'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in keys.items()]+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_test_tools'),'launch','ekf_validation.launch.py')),
            launch_arguments={k:LaunchConfiguration(k) for k in keys}.items())])
