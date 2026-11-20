"""Test-only sensors from the shared mock rover; never publishes estimated TF."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


class RoverSensorAdapter(Node):
    def __init__(self):
        super().__init__('slam_sensor_adapter')
        self.declare_parameter('drop_imu', False)
        self.declare_parameter('drop_wheels', False)
        self.previous = None
        self.imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.wheels = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/testing/slam/truth', self.observe_truth, 10)
        self.create_subscription(JointState, '/testing/slam/side_joints', self.observe_wheels, 10)

    def observe_truth(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        velocity, yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if not all(math.isfinite(v) for v in (stamp, velocity, yaw_rate)):
            self.previous = None
            return
        previous, self.previous = self.previous, (stamp, velocity)
        if previous is None or not 0 < stamp - previous[0] <= .5:
            return
        if self.get_parameter('drop_imu').value:
            return
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = msg.child_frame_id
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity.z = yaw_rate
        imu.linear_acceleration.x = (velocity - previous[1]) / (stamp - previous[0])
        imu.linear_acceleration.y = velocity * yaw_rate
        imu.linear_acceleration.z = 9.81
        for covariance in (imu.orientation_covariance, imu.angular_velocity_covariance,
                           imu.linear_acceleration_covariance):
            for index in (0, 4, 8):
                covariance[index] = 1e-4
        self.imu.publish(imu)

    def observe_wheels(self, msg):
        if self.get_parameter('drop_wheels').value:
            return
        positions = dict(zip(msg.name, msg.position))
        names = ['left_front', 'right_front', 'left_rear', 'right_rear']
        if not all(name in positions for name in ('left_wheel_joint', 'right_wheel_joint')):
            return
        values = [positions['left_wheel_joint' if name.startswith('left')
                            else 'right_wheel_joint'] for name in names]
        if not all(math.isfinite(v) for v in values):
            return
        wheels = JointState()
        wheels.header = msg.header
        wheels.name = [name + '_wheel_joint' for name in names]
        wheels.position = values
        self.wheels.publish(wheels)


def main(args=None):
    rclpy.init(args=args)
    node = RoverSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
