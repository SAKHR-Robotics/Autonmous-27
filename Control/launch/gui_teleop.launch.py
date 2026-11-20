"""Phase 1: Gazebo, manual timeout guard, and the existing external rover GUI."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    control = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gui_dir = LaunchConfiguration('gui_dir').perform(context)
    gui_python = LaunchConfiguration('gui_python').perform(context)
    entry = os.path.join(gui_dir, 'control_station.py')
    if not os.path.isfile(entry):
        raise ValueError('gui_dir must contain the existing GUI control_station.py')
    if not os.path.isfile(gui_python):
        raise ValueError('gui_python must point to Python with rclpy and GUI dependencies')
    share = get_package_share_directory('my_robot_description')
    guard = ExecuteProcess(cmd=[sys.executable, os.path.join(control, 'scripts', 'teleop_guard.py')],
                           output='screen')
    gui = ExecuteProcess(cmd=[gui_python, entry, '--gazebo-teleop'], cwd=gui_dir, output='screen')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'publish_map_tf': 'true',
                              'bridge_sim_tf': 'true', 'publish_camera_tf': 'true'}.items()),
        RegisterEventHandler(OnProcessExit(target_action=guard,
            on_exit=[EmitEvent(event=Shutdown(reason='Teleop guard exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason='Rover GUI closed'))])),
        guard, gui,
    ]


def generate_launch_description():
    # Typical layout: MESEKET/Autonmous-27/Autonmous_Ws/Control and MESEKET/GUI.
    control = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gui_dir = os.path.abspath(os.path.join(control, '..', '..', '..', 'GUI'))
    venv = os.path.join(gui_dir, '.venv', 'bin', 'python')
    return LaunchDescription([
        DeclareLaunchArgument('gui_dir', default_value=gui_dir),
        DeclareLaunchArgument('gui_python', default_value=venv if os.path.isfile(venv) else sys.executable),
        DeclareLaunchArgument('world', default_value='empty_with_sensors.sdf'),
        OpaqueFunction(function=setup),
    ])
