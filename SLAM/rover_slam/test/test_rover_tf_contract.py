"""The actual rover description must leave the EKF's body frame unparented."""
from pathlib import Path
import importlib.util
import shlex
import xml.etree.ElementTree as ET

import pytest
import xacro


def test_rover_root_and_gazebo_odometry_match_ekf():
    workspace = Path(__file__).resolve().parents[3]
    description = ET.fromstring(xacro.process_file(str(
        workspace / 'Rover/my_robot_description/urdf/my_robot.urdf.xacro')).toxml())
    links = {link.attrib['name'] for link in description.findall('link')}
    children = [joint.find('child').attrib['link'] for joint in description.findall('joint')]
    assert len(children) == len(set(children)), 'URDF children must have one parent'
    assert links - set(children) == {'base_link'}
    footprint = description.find("joint[@name='base_footprint_joint']")
    assert footprint.find('parent').attrib['link'] == 'base_link'
    assert footprint.find('child').attrib['link'] == 'base_footprint'
    assert [float(v) for v in footprint.find('origin').attrib['xyz'].split()] == pytest.approx([0., 0., -.255])
    plugin = description.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert plugin.findtext('frame_id') == 'odom'
    assert plugin.findtext('child_frame_id') == 'base_link'
    imu = description.find("gazebo[@reference='imu_link']/sensor[@name='imu_sensor']")
    assert imu.findtext('gz_frame_id') == 'imu_link'
    # Mount geometry remains relative to the same chassis, not the footprint.
    camera = description.find("joint[@name='camera_joint']")
    assert camera.find('parent').attrib['link'] == 'base_link'
    assert [float(v) for v in camera.find('origin').attrib['xyz'].split()] == pytest.approx([.265, 0., 0.])


def test_gazebo_launch_quotes_selected_world_path():
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    workspace = Path(__file__).resolve().parents[3]
    path = workspace / 'Rover/my_robot_description/launch/gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('rover_gazebo_contract', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    selected = '/tmp/selected Mars Yard_R&A.world'
    context.launch_configurations.update(world=selected, bridge_sim_tf='false',
                                         publish_map_tf='false', publish_camera_tf='false')
    actions = module.launch_setup(context)
    gazebo = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    arguments = dict(gazebo.launch_arguments)
    command = perform_substitutions(context, normalize_to_list_of_substitutions(arguments['gz_args']))
    assert shlex.split(command) == ['-r', selected]
    # ros_gz_sim passes gz_args through a shell: '&' must stay inside a quoted word.
    lexer = shlex.shlex(command, posix=True, punctuation_chars='&')
    lexer.whitespace_split = True
    assert list(lexer) == ['-r', selected]
