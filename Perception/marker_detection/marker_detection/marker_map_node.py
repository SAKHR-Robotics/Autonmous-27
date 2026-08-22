#!/usr/bin/env python3
"""Step 6 global marker mapping from Step 4 poses and the existing TF tree."""
from __future__ import annotations
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from marker_detection.msg import MarkerMap, MarkerMapEntry, MarkerTrackedPoseArray
from .marker_map_manager import GlobalObservation, MarkerMapManager


class MarkerMapNode(Node):
    """Fuses TF-resolved map observations without publishing any map-to-marker TF."""
    def __init__(self) -> None:
        super().__init__("marker_map")
        self._declare_parameters()
        self.map_frame = self.get_parameter("map_frame").value
        self.marker_frame_prefix = self.get_parameter("marker_frame_prefix").value
        self.manager = MarkerMapManager(
            min_confirmations=int(self.get_parameter("min_global_confirmations").value),
            position_measurement_noise=float(self.get_parameter("position_measurement_noise").value),
            initial_covariance=float(self.get_parameter("position_initial_covariance").value),
            position_gate_threshold=float(self.get_parameter("position_gate_threshold").value),
            orientation_gate_threshold_deg=float(self.get_parameter("orientation_gate_threshold_deg").value),
            orientation_alpha=float(self.get_parameter("orientation_fusion_alpha").value),
            stale_timeout_seconds=float(self.get_parameter("stale_timeout_seconds").value),
            min_tracking_confidence=float(self.get_parameter("min_tracking_confidence").value),
            max_reprojection_error_px=float(self.get_parameter("max_reprojection_error_px").value))
        self.tf_buffer = Buffer(cache_time=Duration(seconds=float(self.get_parameter("tf_cache_seconds").value)))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_pub = self.create_publisher(MarkerMap, self.get_parameter("marker_map_topic").value, QoSProfile(depth=10))
        self.rviz_pub = self.create_publisher(MarkerArray, self.get_parameter("rviz_topic").value, QoSProfile(depth=10))
        self.pose_sub = self.create_subscription(MarkerTrackedPoseArray, self.get_parameter("tracked_pose_topic").value,
                                                 self._poses_callback, QoSProfile(depth=10))
        self.reset_service = self.create_service(Empty, "/marker_map/reset", self._reset)
        self._last_tf_warning = 0.0
        self.create_timer(1.0, self._maintenance)

    def _declare_parameters(self) -> None:
        for name, value in {
            "tracked_pose_topic": "/marker_poses", "marker_map_topic": "/marker_map",
            "map_frame": "map", "marker_frame_prefix": "aruco_marker_", "tf_cache_seconds": 10.0,
            "tf_lookup_timeout_seconds": 0.05, "min_global_confirmations": 5,
            "position_measurement_noise": 0.04, "position_initial_covariance": 1.0,
            "position_gate_threshold": 11.34, "orientation_gate_threshold_deg": 30.0,
            "orientation_fusion_alpha": 0.20, "stale_timeout_seconds": 30.0,
            "min_tracking_confidence": 0.30, "max_reprojection_error_px": 3.0,
            "publish_rviz_markers": True, "rviz_topic": "/marker_map/visualization",
        }.items():
            self.declare_parameter(name, value)

    def _poses_callback(self, message: MarkerTrackedPoseArray) -> None:
        stamp = Time.from_msg(message.header.stamp)
        timestamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        if timestamp <= 0.0:
            self.get_logger().warning("Ignoring marker poses with an invalid zero timestamp.")
            return
        for pose in message.markers:
            if not (pose.detection_valid and pose.tracking_valid):
                continue
            transform = self._lookup_global_marker(pose.id, stamp)
            if transform is None:
                continue
            translation, rotation = transform.transform.translation, transform.transform.rotation
            observation = GlobalObservation(
                pose.id, np.array([translation.x, translation.y, translation.z]),
                np.array([rotation.x, rotation.y, rotation.z, rotation.w]), pose.tracking_confidence,
                pose.reprojection_error_px, timestamp)
            self.manager.update(observation)
        self._publish_map()

    def _lookup_global_marker(self, marker_id: int, stamp: Time):
        """Use TF2's composed map->marker transform at the original observation stamp."""
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, f"{self.marker_frame_prefix}{marker_id}", stamp,
                timeout=Duration(seconds=float(self.get_parameter("tf_lookup_timeout_seconds").value)))
        except TransformException as error:
            now = time.monotonic()
            if now - self._last_tf_warning >= 5.0:
                self.get_logger().warning(
                    f"Global mapping unavailable: cannot resolve {self.map_frame} to marker TF at observation time ({error}).")
                self._last_tf_warning = now
            return None

    def _maintenance(self) -> None:
        now = self.get_clock().now()
        self.manager.update_staleness(now.nanoseconds * 1e-9)
        self._publish_map()

    def _publish_map(self) -> None:
        now = self.get_clock().now().to_msg()
        output = MarkerMap()
        output.header.stamp, output.header.frame_id = now, self.map_frame
        for entry in self.manager.entries.values():
            message = MarkerMapEntry()
            message.id, message.confidence = entry.marker_id, entry.confidence
            message.pose.position.x, message.pose.position.y, message.pose.position.z = entry.position.tolist()
            message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = entry.orientation.tolist()
            message.observation_count, message.accepted_observations, message.rejected_observations = (
                entry.observation_count, entry.accepted_observations, entry.rejected_observations)
            seconds = int(entry.last_seen)
            message.state, message.last_seen = entry.state.value, Time(seconds=seconds, nanoseconds=int((entry.last_seen - seconds) * 1e9)).to_msg()
            message.position_covariance = entry.covariance.reshape(9).tolist()
            output.markers.append(message)
        self.map_pub.publish(output)
        if self.get_parameter("publish_rviz_markers").value:
            self.rviz_pub.publish(self._rviz_markers(now))

    def _rviz_markers(self, stamp) -> MarkerArray:
        output = MarkerArray()
        for entry in self.manager.entries.values():
            axis = Marker()
            axis.header.frame_id, axis.header.stamp, axis.ns, axis.id = self.map_frame, stamp, "aruco_marker_axes", entry.marker_id * 2
            axis.type, axis.action = Marker.ARROW, Marker.ADD
            axis.pose.position.x, axis.pose.position.y, axis.pose.position.z = entry.position.tolist()
            axis.pose.orientation.x, axis.pose.orientation.y, axis.pose.orientation.z, axis.pose.orientation.w = entry.orientation.tolist()
            axis.scale.x, axis.scale.y, axis.scale.z = 0.20, 0.025, 0.025
            axis.color.r, axis.color.g, axis.color.b, axis.color.a = 0.0, 1.0, 0.0, 0.9
            text = Marker()
            text.header.frame_id, text.header.stamp, text.ns, text.id = self.map_frame, stamp, "aruco_marker_labels", entry.marker_id * 2 + 1
            text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = (entry.position + np.array([0.0, 0.0, 0.12])).tolist()
            text.scale.z, text.color.r, text.color.g, text.color.b, text.color.a = 0.08, 1.0, 1.0, 1.0, 1.0
            text.text = f"ID: {entry.marker_id}\n{entry.state.value} conf: {entry.confidence:.2f}"
            output.markers.extend([axis, text])
        return output

    def _reset(self, _request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.manager.clear()
        self._publish_map()
        self.get_logger().info("Marker map reset; tracking was not affected.")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
