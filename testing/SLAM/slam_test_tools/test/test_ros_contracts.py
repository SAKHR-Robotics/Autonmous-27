"""Requires Linux ROS 2 Jazzy; missing ROS is an explicit skip, never a pass."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock
import pytest
rclpy=pytest.importorskip('rclpy',reason='ROS 2 runtime required')
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
from rover_slam.heuristic_slip_checker import HeuristicSlipCheckerNode


@pytest.fixture
def ros():
    owned=not rclpy.ok()
    if owned: rclpy.init(args=[])
    yield
    if owned: rclpy.shutdown()


def test_encoder_subscribes_to_only_selected_source(ros):
    node=EncoderTicksToOdomNode()
    try:
        assert [s.topic_name for s in node.subscriptions_owned]==['/joint_states']
        node.pub_odom=Mock()
        node._watchdog()
        node.pub_odom.publish.assert_not_called()
    finally: node.destroy_node()


def test_slip_preserves_twist_and_input_message(ros):
    node=HeuristicSlipCheckerNode()
    try:
        msg=Odometry(); msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='odom'; msg.child_frame_id='base_link'
        msg.twist.twist.linear.x=.4; msg.twist.twist.angular.z=1.
        msg.twist.covariance[0]=.2; msg.twist.covariance[35]=.3
        imu=Imu(); imu.header.stamp=msg.header.stamp; imu.header.frame_id='base_link'
        imu.orientation.w=1.; imu.linear_acceleration.z=9.81
        node._imu_callback(imu)
        node.pub_filtered_odom=Mock()
        node._wheel_odom_callback(msg)
        output=node.pub_filtered_odom.publish.call_args.args[0]
        assert output.twist.twist==msg.twist.twist
        assert output.twist.covariance[0]>=20.
        assert msg.twist.covariance[0]==.2
        assert output.header==msg.header
    finally: node.destroy_node()


def test_stale_odom_is_not_republished(ros):
    node=HeuristicSlipCheckerNode()
    try:
        msg=Odometry(); msg.header.frame_id='odom'; msg.child_frame_id='base_link'
        node.pub_filtered_odom=Mock()
        node._wheel_odom_callback(msg)
        node.pub_filtered_odom.publish.assert_not_called()
    finally: node.destroy_node()


def test_real_master_constructs_and_selects_stages(ros):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from ament_index_python.packages import get_package_share_directory,PackageNotFoundError
    try: get_package_share_directory('rover_slam')
    except PackageNotFoundError: pytest.skip('Build/source rover_slam overlay for launch checks')
    path=Path(__file__).resolve().parents[4]/'SLAM/rover_slam/launch/slam_bringup.launch.py'
    spec=importlib.util.spec_from_file_location('slam_master_test',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    description=module.generate_launch_description()
    context=LaunchContext()
    for entity in description.entities:
        if isinstance(entity,DeclareLaunchArgument): entity.execute(context)
    assert len(module.setup(context))==2
    context.launch_configurations['launch_rtabmap']='false'
    assert len(module.setup(context))==1
    context.launch_configurations['launch_ekf']='false'
    assert module.setup(context)==[]
    context.launch_configurations['launch_encoder']='true'
    with pytest.raises(ValueError): module.setup(context)
