#!/usr/bin/env python3
"""Step 5: broadcast dynamic camera-optical-frame to tracked-marker TF edges."""
from __future__ import annotations
import re
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from marker_detection_msgs.msg import MarkerTrackedPose, MarkerTrackedPoseArray
from marker_detection.quaternion_filter import normalize


class MarkerTFBroadcaster(Node):
    """Consumes Step 4 poses only; does not estimate, filter, or transform poses."""
    def __init__(self) -> None:
        super().__init__("marker_tf_broadcaster")
        for name, value in {
            "tracked_pose_topic": "/marker_poses", "marker_frame_prefix": "aruco_marker_",
            "publish_tf": True, "publish_only_confirmed": True,
            "publish_predicted_lost": False, "expected_parent_frame": "", "debug_logging": False,
        }.items():
            self.declare_parameter(name, value)
        self.prefix = self.get_parameter("marker_frame_prefix").value
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", self.prefix):
            raise ValueError("marker_frame_prefix must contain only safe TF frame characters and begin with a letter.")
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            MarkerTrackedPoseArray, self.get_parameter("tracked_pose_topic").value,
            self._poses_callback, QoSProfile(depth=10))

    def _poses_callback(self, message: MarkerTrackedPoseArray) -> None:
        if not self.get_parameter("publish_tf").value:
            return
        transforms: list[TransformStamped] = []
        published_ids: set[int] = set()
        for pose in message.markers:
            if pose.id in published_ids or not self._publishable(pose):
                continue
            transform = self._to_transform(pose)
            if transform is not None:
                transforms.append(transform)
                published_ids.add(pose.id)
        if transforms:
            self.broadcaster.sendTransform(transforms)

    def _publishable(self, pose: MarkerTrackedPose) -> bool:
        if pose.id < 0 or not pose.tracking_valid:
            return False
        state = pose.track_state.upper()
        if self.get_parameter("publish_only_confirmed").value:
            return pose.detection_valid and state == "CONFIRMED"
        if pose.detection_valid:
            return state in ("NEW", "CONFIRMED", "DEGRADED")
        return self.get_parameter("publish_predicted_lost").value and state in ("DEGRADED", "LOST")

    def _to_transform(self, pose: MarkerTrackedPose) -> TransformStamped | None:
        parent = pose.header.frame_id.strip()
        if not parent or (pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0):
            self.get_logger().warning("Skipping marker TF with empty parent frame or invalid pose timestamp.")
            return None
        expected_parent = self.get_parameter("expected_parent_frame").value
        if expected_parent and parent != expected_parent:
            self.get_logger().warning(
                f"Skipping marker TF: pose parent '{parent}' does not match expected_parent_frame '{expected_parent}'.")
            return None
        position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
        quaternion = normalize(np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        if not np.isfinite(position).all() or quaternion is None:
            self.get_logger().warning(f"Skipping invalid tracked pose for marker {pose.id}.")
            return None
        transform = TransformStamped()
        # Step 4 position/orientation are T_camera_marker, so neither is inverted.
        transform.header = pose.header
        transform.child_frame_id = f"{self.prefix}{pose.id}"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position.tolist()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion.tolist()
        if self.get_parameter("debug_logging").value:
            self.get_logger().debug(f"Broadcast {parent} -> {transform.child_frame_id}")
        return transform


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
