"""Production-only SLAM launch description.

Starts state estimation and RTAB-Map. Costmaps, RViz, validators, and synthetic
publishers are intentionally excluded; those belong to integration tests or
other subsystem launch files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_static_tf = LaunchConfiguration('launch_static_tf')
    standalone_tf = LaunchConfiguration('standalone_tf')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'static_transforms.launch.py')),
        launch_arguments={
            'standalone': standalone_tf,
            'publish_optical_tf': publish_optical_tf,
        }.items(),
        condition=IfCondition(launch_static_tf),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ekf.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
        }.items(),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vision_helper.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_camera),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock when true',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='false',
            description='Launch the RealSense driver when true',
        ),
        DeclareLaunchArgument(
            'launch_static_tf',
            default_value='true',
            description='Include the static sensor TF launch file',
        ),
        DeclareLaunchArgument(
            'standalone_tf',
            default_value='false',
            description='Publish base_link sensor TFs when robot_state_publisher is absent',
        ),
        DeclareLaunchArgument(
            'publish_optical_tf',
            default_value='false',
            description='Publish optical TFs when robot_state_publisher is absent',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/wheel/odom_raw',
            description='Raw odometry input for the slip checker',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/image_raw',
            description='RGB image input for RTAB-Map',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Depth image input for RTAB-Map',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
            description='Camera calibration input for RTAB-Map',
        ),
        static_tf_launch,
        ekf_launch,
        camera_launch,
        rtabmap_launch,
    ])