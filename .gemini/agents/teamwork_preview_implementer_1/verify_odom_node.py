import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode

def test_node_live():
    rclpy.init()
    node = EncoderTicksToOdomNode()
    
    # Track received messages
    received_odom = []
    received_speeds = []

    test_sub_node = rclpy.create_node("test_listener")
    test_sub_node.create_subscription(
        Odometry,
        "/wheel/odom_raw",
        lambda msg: received_odom.append(msg),
        10
    )
    test_sub_node.create_subscription(
        Float64MultiArray,
        "/wheel/per_wheel_speeds",
        lambda msg: received_speeds.append(msg),
        10
    )

    # Publish joint states for 4 wheels driving forward at 5 rad/s
    pub_js = test_sub_node.create_publisher(JointState, "/joint_states", 10)
    
    start = time.time()
    angle = 0.0
    while time.time() - start < 1.0:
        angle += 0.1
        js = JointState()
        js.header.stamp = test_sub_node.get_clock().now().to_msg()
        js.name = [
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_rear_wheel_joint",
            "right_rear_wheel_joint",
        ]
        js.position = [angle, angle, angle, angle]
        pub_js.publish(js)

        rclpy.spin_once(node, timeout_sec=0.02)
        rclpy.spin_once(test_sub_node, timeout_sec=0.02)

    print(f"Received odom messages: {len(received_odom)}")
    print(f"Received per-wheel speed messages: {len(received_speeds)}")
    assert len(received_odom) > 0, "No odom messages received!"
    assert len(received_speeds) > 0, "No wheel speeds messages received!"

    last_odom = received_odom[-1]
    last_speeds = received_speeds[-1]
    print(f"Last odom linear velocity x: {last_odom.twist.twist.linear.x}")
    print(f"Last per-wheel speeds (4 wheels): {last_speeds.data}")
    assert len(last_speeds.data) == 4, f"Expected 4 wheel speeds, got {len(last_speeds.data)}"
    assert last_odom.twist.twist.linear.x > 0.0, "Expected positive forward velocity!"

    node.destroy_node()
    test_sub_node.destroy_node()
    rclpy.shutdown()
    print("\n✓ LIVE ENCODER TICKS TO ODOM TEST PASSED!")

if __name__ == "__main__":
    test_node_live()
