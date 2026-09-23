#!/usr/bin/env python3
"""Clean, high-level marker interface for manipulation/planning nodes
(Parts 3, 4, 5, 6, 7, 17 of the ERC downstream-interface hardening pass).

Subscribes to the existing Stage 4 tracked-pose topic (`/marker_poses`) and
republishes a single `MarkerActionTargetArray` on `/marker_detection/targets`
with each marker's pose in `base_link`, forward/lateral/vertical/Euclidean
distances, and an explicit `usable_for_action` safety gate -- so a
manipulation/planning node never needs to subscribe separately to raw
detection, depth, PnP, tracking, or TF to decide whether a marker is usable.

This node performs exactly one TF2 lookup per track (`base_link <-
<tracked-pose frame>`, i.e. the RGB optical frame, at the pose's own
timestamp) and otherwise reuses:

  - the existing tracking state machine (tracker_manager.TrackState) for
    `stable`,
  - the existing quality-flag bitmask (quality_flags.QualityFlag) for
    `usable_for_action`,
  - the existing TF tree (no new frames are broadcast; this node is a
    read-only TF2 consumer, exactly like marker_map_node.py).

No new tracking, quality, or PnP system is introduced. If the TF lookup
fails, no base_link pose is fabricated: position/orientation/distances are
marked invalid and usable_for_action is forced false (Part 19).
"""
from __future__ import annotations
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import PoseStamped
from marker_detection_msgs.msg import MarkerActionTarget, MarkerActionTargetArray, MarkerTrackedPose, MarkerTrackedPoseArray
from marker_detection.action_target_builder import ActionTarget, build_action_target
from marker_detection.frame_transform import ResolvedPose, transform_to_base_link


def _covariance_matrix(flat9) -> np.ndarray | None:
    """MarkerTrackedPose.position_covariance convention: all-zero means
    'not computed' (see MarkerPose.msg/MarkerTrackedPose.msg docstrings)."""
    values = np.asarray(flat9, dtype=np.float64)
    if values.size != 9 or not np.any(values):
        return None
    return values.reshape(3, 3)


class MarkerActionInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_action_interface")
        self._declare_parameters()
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.tf_buffer = Buffer(cache_time=Duration(seconds=float(self.get_parameter("tf_cache_seconds").value)))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.targets_pub = self.create_publisher(
            MarkerActionTargetArray, self.get_parameter("targets_topic").value, QoSProfile(depth=10))
        self.slam_landmark_pub = self.create_publisher(
            PoseStamped, self.get_parameter("slam_landmark_topic").value, QoSProfile(depth=10))
        self.pose_sub = self.create_subscription(
            MarkerTrackedPoseArray, self.get_parameter("tracked_pose_topic").value,
            self._poses_callback, QoSProfile(depth=10))
        self._last_tf_warning = 0.0

    def _declare_parameters(self) -> None:
        for name, value in {
            "tracked_pose_topic": "/marker_poses", "targets_topic": "/marker_detection/targets",
            "slam_landmark_topic": "/perception/aruco_pose",
            "base_link_frame": "base_link", "tf_cache_seconds": 10.0, "tf_lookup_timeout_seconds": 0.05,
            # Part 6/16: the only two new configurable thresholds this
            # interface needs; everything else is read from the existing
            # tracking state, quality flags, and confidence already computed
            # upstream.
            "min_action_confidence": 0.6, "max_action_covariance_trace_m2": 0.01,
        }.items():
            self.declare_parameter(name, value)

    def _poses_callback(self, message: MarkerTrackedPoseArray) -> None:
        output = MarkerActionTargetArray()
        output.header.stamp, output.header.frame_id = message.header.stamp, self.base_link_frame
        for pose in message.markers:
            msg = self._build_message(pose)
            output.markers.append(msg)
            if msg.usable_for_action and msg.pose_valid:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = msg.header.stamp
                pose_stamped.header.frame_id = self.base_link_frame
                pose_stamped.pose.position.x = msg.position.x
                pose_stamped.pose.position.y = msg.position.y
                pose_stamped.pose.position.z = msg.position.z
                pose_stamped.pose.orientation.x = msg.orientation.x
                pose_stamped.pose.orientation.y = msg.orientation.y
                pose_stamped.pose.orientation.z = msg.orientation.z
                pose_stamped.pose.orientation.w = msg.orientation.w
                self.slam_landmark_pub.publish(pose_stamped)
        self.targets_pub.publish(output)

    def _build_message(self, pose: MarkerTrackedPose) -> MarkerActionTarget:
        resolved = self._resolve_base_link(pose)
        target = build_action_target(
            marker_id=pose.id, detection_valid=pose.detection_valid, tracking_valid=pose.tracking_valid,
            tracking_state=pose.track_state, confidence=pose.confidence, quality_flags=pose.quality_flags,
            age_seconds=pose.age_seconds,
            base_link_position=resolved.position if resolved else None,
            base_link_orientation=resolved.orientation if resolved else None,
            base_link_position_covariance=resolved.position_covariance if resolved else None,
            min_action_confidence=float(self.get_parameter("min_action_confidence").value),
            max_action_covariance_trace_m2=float(self.get_parameter("max_action_covariance_trace_m2").value))
        return _to_message(target, self.base_link_frame, pose.header.stamp)

    def _resolve_base_link(self, pose: MarkerTrackedPose) -> ResolvedPose | None:
        """Part 4: camera frame -> marker pose -> TF2 -> base_link. Looks up
        the (normally static) base_link <- camera_optical_frame transform via
        TF2 -- never manually computed -- and composes it with the marker's
        existing camera-frame pose (see frame_transform.transform_to_base_link).
        Returns None (never a fabricated pose) if the lookup fails."""
        parent = pose.header.frame_id.strip()
        if not parent or (pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0):
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link_frame, parent, Time.from_msg(pose.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter("tf_lookup_timeout_seconds").value)))
        except TransformException as error:
            now = time.monotonic()
            if now - self._last_tf_warning >= 5.0:
                self.get_logger().warning(
                    f"Cannot resolve {self.base_link_frame} <- {parent} at the tracked-pose timestamp ({error}); "
                    "reporting this marker's base_link pose as unavailable.")
                self._last_tf_warning = now
            return None
        t, r = transform.transform.translation, transform.transform.rotation
        camera_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        camera_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return transform_to_base_link(
            transform_translation=np.array([t.x, t.y, t.z]), transform_rotation=np.array([r.x, r.y, r.z, r.w]),
            camera_position=camera_position, camera_orientation=camera_orientation,
            camera_position_covariance=_covariance_matrix(pose.position_covariance))


def _to_message(target: ActionTarget, frame_id: str, stamp) -> MarkerActionTarget:
    message = MarkerActionTarget()
    message.header.stamp, message.header.frame_id = stamp, frame_id
    message.marker_id = target.marker_id
    message.detected, message.stable, message.usable_for_action = (
        target.detected, target.stable, target.usable_for_action)
    message.tracking_state, message.pose_valid = target.tracking_state, target.pose_valid
    if target.position is not None:
        message.position.x, message.position.y, message.position.z = target.position.tolist()
    if target.orientation is not None:
        (message.orientation.x, message.orientation.y,
         message.orientation.z, message.orientation.w) = target.orientation.tolist()
    message.distance_to_marker, message.forward_distance = target.distance_to_marker, target.forward_distance
    message.lateral_distance, message.vertical_distance = target.lateral_distance, target.vertical_distance
    message.confidence, message.quality_flags = target.confidence, target.quality_flags
    if target.position_covariance is not None:
        message.position_covariance = np.asarray(target.position_covariance).reshape(9).tolist()
    message.age_seconds = target.age_seconds
    return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerActionInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
