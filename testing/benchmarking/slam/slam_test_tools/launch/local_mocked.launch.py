"""Real converter, slip filter and EKF; shared rover provides only dependencies."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ekf_params_file', default_value=os.path.join(
            get_package_share_directory('rover_slam'), 'config', 'ekf.yaml')),
        DeclareLaunchArgument('wheel_output_mode', default_value='joint_states',
                              choices=['joint_states', 'ticks_array', 'individual_ticks']),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('imu_mount_yaw_rad', default_value='0.0'),
        DeclareLaunchArgument('gravity_sign', default_value='1.0'),
        Node(package='pathplanner_mock_tools', executable='mock_rover_sim',
             name='slam_dependency_rover', output='screen', parameters=[{
                 'use_sim_time': False, 'publish_tf': False, 'wheel_radius': .06,
                 'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                 'cmd_vel_topic': '/testing/slam/cmd_vel',
                 'track_width': .49, 'noise_std_v': 0., 'noise_std_w': 0.,
                 'slip_ratio_linear': 0., 'slip_ratio_angular': 0.}],
             remappings=[('/odometry/filtered', '/testing/slam/truth'),
                         ('/rover/joint_states', '/testing/slam/side_joints'),
                         ('/rover_marker', '/testing/slam/mock_rover_marker'),
                         ('/rover/motor_rpm', '/testing/slam/motor_rpm'),
                         ('/motor_driver/rpm_feedback', '/testing/slam/rpm_feedback'),
                         ('/initialpose', '/testing/slam/initialpose'),
                         ('/start_pose', '/testing/slam/start_pose')]),
        Node(package='slam_mock_tools', executable='rover_sensor_adapter',
             name='slam_sensor_adapter', output='screen', parameters=[{
                 'use_sim_time': False,
                 'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                 'wheel_output_mode': LaunchConfiguration('wheel_output_mode'),
                 'imu_mount_yaw_rad': ParameterValue(LaunchConfiguration('imu_mount_yaw_rad'), value_type=float),
                 'gravity_sign': ParameterValue(LaunchConfiguration('gravity_sign'), value_type=float),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_slam'), 'launch', 'ekf.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'launch_encoder': 'true',
                              'encoder_input_mode': LaunchConfiguration('wheel_output_mode'),
                              'ekf_params_file': LaunchConfiguration('ekf_params_file'),
                              'odom_topic': '/wheel/odom_raw'}.items()),
    ])
