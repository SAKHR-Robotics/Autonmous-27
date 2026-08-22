import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # base_link -> camera_link (Camera mounted 0.2m forward, 0.3m high on rover chassis: x y z yaw pitch roll frame_id child_frame_id)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=['0.2', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
        ),

        # base_link -> imu_link (BNO055 IMU mounted at chassis center, 0.1m high)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
        ),

        # camera_link -> camera_depth_optical_frame (ROS REP-103 standard Optical Frame rotation: yaw=-pi/2, pitch=0, roll=-pi/2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_optical',
            arguments=['0.0', '0.0', '0.0', '-1.57079632679', '0.0', '-1.57079632679', 'camera_link', 'camera_depth_optical_frame']
        )
    ])

