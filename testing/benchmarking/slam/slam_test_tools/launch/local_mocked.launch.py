"""Real converter, slip filter and EKF; shared rover provides only dependencies."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ekf_params_file', default_value=os.path.join(
            get_package_share_directory('rover_slam'), 'config', 'ekf.yaml')),
        Node(package='pathplanner_mock_tools', executable='mock_rover_sim',
             name='slam_dependency_rover', output='screen', parameters=[{
                 'use_sim_time': False, 'publish_tf': False, 'wheel_radius': .06,
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
             name='slam_sensor_adapter', output='screen', parameters=[{'use_sim_time': False}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_slam'), 'launch', 'ekf.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'launch_encoder': 'true',
                              'encoder_input_mode': 'joint_states',
                              'ekf_params_file': LaunchConfiguration('ekf_params_file'),
                              'odom_topic': '/wheel/odom_raw'}.items()),
    ])
