import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from marker_detection.marker_map_node import MarkerMapNode
from marker_detection_msgs.msg import MarkerTrackedPose, MarkerTrackedPoseArray
from std_srvs.srv import Empty


def test_map_consumes_camera_pose_without_marker_tf():
    rclpy.init()
    node = MarkerMapNode()
    try:
        pose = MarkerTrackedPose()
        pose.id = 7
        pose.header.frame_id = 'camera_optical'
        pose.header.stamp.sec = 1
        pose.position.z = 2.
        pose.orientation.w = 1.
        pose.detection_valid = pose.tracking_valid = True
        pose.tracking_confidence = 0.9
        transform = TransformStamped()
        transform.transform.translation.x = 1.
        transform.transform.rotation.w = 1.
        requested = []
        node._lookup_global_marker = lambda frame, stamp: (requested.append(frame) or transform)
        observations = []
        node.manager.update = observations.append
        message = MarkerTrackedPoseArray()
        message.header = pose.header
        message.markers = [pose]
        node._poses_callback(message)
        assert requested == ['camera_optical']
        np.testing.assert_allclose(observations[0].position, [1., 0., 2.])
        deletion = []
        node._delete_visuals = lambda: deletion.append(True)
        node._reset(Empty.Request(), Empty.Response())
        assert deletion == [True]
    finally:
        node.destroy_node()
        rclpy.shutdown()
