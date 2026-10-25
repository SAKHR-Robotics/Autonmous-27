import numpy as np
from marker_detection.landmark_adapter import landmark_from_track
from marker_detection_msgs.msg import MarkerTrackedPose


def track():
    value=MarkerTrackedPose()
    value.id=0
    value.header.stamp.sec=10
    value.header.frame_id='camera_optical'
    value.detection_valid=value.tracking_valid=True
    value.track_state='CONFIRMED'
    value.tracking_confidence=.9
    value.orientation.w=1.
    value.position.z=2.
    value.position_covariance=(np.eye(3)*.01).ravel().tolist()
    return value


def test_ids_frames_and_covariance_preserved():
    value=landmark_from_track(track(),now=10.1)
    assert value.id==1
    assert value.header.frame_id=='camera_optical'
    assert value.pose.covariance[0]==.01
    assert value.pose.covariance[35]==.1


def test_stale_lost_and_nonfinite_observations_rejected():
    assert landmark_from_track(track(),now=12.) is None
    value=track(); value.track_state='LOST'
    assert landmark_from_track(value,now=10.) is None
    value=track(); value.position.x=float('nan')
    assert landmark_from_track(value,now=10.) is None
