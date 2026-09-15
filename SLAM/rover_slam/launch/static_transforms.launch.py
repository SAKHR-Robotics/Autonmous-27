import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    standalone = LaunchConfiguration('standalone')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='If true, publish base_link->camera_link and base_link->imu_link transforms (use only when robot_state_publisher is NOT running)'
    )

    declare_publish_optical_tf = DeclareLaunchArgument(
        'publish_optical_tf',
        default_value='true',
        description='If true, publish camera_link->camera_depth_optical_frame transform for optical sensor alignment'
    )

    return LaunchDescription([
        declare_standalone,
        declare_publish_optical_tf,

        # base_link -> camera_link (Only published when standalone=true to avoid colliding with robot_state_publisher)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '--x', '0.2', '--y', '0.0', '--z', '0.3',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ],
            condition=IfCondition(standalone)
        ),

        # base_link -> imu_link (Only published when standalone=true to avoid colliding with robot_state_publisher)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ],
            condition=IfCondition(standalone)
        ),

        # camera_link -> camera_depth_optical_frame (ROS REP-103 standard Optical Frame rotation)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_optical',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '-1.57079632679', '--pitch', '0.0', '--yaw', '-1.57079632679',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_depth_optical_frame'
            ],
            condition=IfCondition(publish_optical_tf)
        ),

        # camera_link -> camera_color_optical_frame (ROS REP-103 standard Optical Frame rotation for RGB camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_color_optical',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '-1.57079632679', '--pitch', '0.0', '--yaw', '-1.57079632679',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_color_optical_frame'
            ],
            condition=IfCondition(publish_optical_tf)
        ),

        # camera_depth_optical_frame -> my_robot/camera_link/camera (Bridge Gazebo sensor frame to REP-103 optical frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_optical_to_gz',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'camera_depth_optical_frame',
                '--child-frame-id', 'my_robot/camera_link/camera'
            ],
            condition=IfCondition(publish_optical_tf)
        )
    ])


