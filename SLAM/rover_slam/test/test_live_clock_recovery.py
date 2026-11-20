"""Real DDS input/output through production preprocessing across a ROS clock reset."""
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
from rover_slam.heuristic_slip_checker import HeuristicSlipCheckerNode


def test_preprocessors_recover_from_missing_inputs_and_clock_reset():
    rclpy.init(args=[])
    encoder, slip = EncoderTicksToOdomNode(), HeuristicSlipCheckerNode()
    inputs = rclpy.create_node('slam_clock_contract_inputs')
    executor = SingleThreadedExecutor()
    for node in (encoder, slip):
        node.set_parameters([Parameter('use_sim_time', value=True)])
    for node in (encoder, slip, inputs):
        executor.add_node(node)
    clock = inputs.create_publisher(Clock, '/clock', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    joints = inputs.create_publisher(JointState, '/joint_states', 10)
    imu = inputs.create_publisher(Imu, '/imu/data', 10)
    outputs = []
    inputs.create_subscription(Odometry, '/wheel/odom_filtered', outputs.append, 10)

    def spin(seconds=.08):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)

    def advance(seconds):
        message = Clock()
        message.clock.sec = int(seconds)
        message.clock.nanosec = round((seconds-int(seconds))*1e9)
        clock.publish(message)
        spin()
        return message.clock

    def observe(seconds, position, include_imu=True):
        stamp = advance(seconds)
        if include_imu:
            message = Imu()
            message.header.stamp, message.header.frame_id = stamp, 'base_link'
            message.orientation.w = 1.
            message.linear_acceleration.z = 9.81
            imu.publish(message)
            spin()
        message = JointState()
        message.header.stamp = stamp
        message.name = ['left_front_wheel_joint', 'right_front_wheel_joint',
                        'left_rear_wheel_joint', 'right_rear_wheel_joint']
        message.position = [position] * 4
        joints.publish(message)
        spin()

    try:
        spin(.3)
        observe(10., 0.)
        observe(10.02, .1)
        assert outputs and outputs[-1].twist.twist.linear.x > 0
        count = len(outputs)
        advance(11.)
        assert len(outputs) == count, 'Input loss must not manufacture wheel odometry'
        assert slip.health == 'wheel odometry unavailable or stale'
        observe(1., 0., include_imu=False)
        assert len(outputs) == count, 'Clock reset requires a new wheel baseline'
        observe(1.02, .1, include_imu=False)
        assert len(outputs) == count + 1
        assert outputs[-1].header.stamp.sec == 1
        assert outputs[-1].twist.covariance[0] >= 2., 'Old IMU must not survive clock reset'
        assert 'degraded' in slip.health
        observe(1.04, .2)
        assert slip.health == 'healthy'
        assert outputs[-1].twist.covariance[0] < 2.
    finally:
        executor.shutdown()
        for node in (encoder, slip, inputs):
            node.destroy_node()
        rclpy.try_shutdown()
