import numpy as np
import pytest
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


@pytest.mark.parametrize('defect', ['negative_id', 'empty_frame', 'zero_stamp', 'future', 'negative_age', 'bad_covariance', 'combined_age', 'infinite_confidence'])
def test_invalid_landmark_contract_is_rejected(defect):
    value=track()
    now=10.
    if defect=='negative_id':
        value.id=-1
    elif defect=='empty_frame':
        value.header.frame_id=''
    elif defect=='zero_stamp':
        value.header.stamp.sec=0
    elif defect=='future':
        now=9.
    elif defect=='negative_age':
        value.age_seconds=-1.
    elif defect=='combined_age':
        value.age_seconds=.3
        now=10.3
    elif defect=='infinite_confidence':
        value.tracking_confidence=float('inf')
    else:
        value.position_covariance=(-np.eye(3)).ravel().tolist()
    assert landmark_from_track(value,now=now) is None
