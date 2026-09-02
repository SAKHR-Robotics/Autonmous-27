import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from marker_detection.pose_estimator import PoseEstimator
from marker_detection.quality_flags import QualityFlag


def camera_info() -> CameraInfo:
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    return info


def test_square_object_points_match_documented_ippe_order():
    estimator = PoseEstimator(0.10, "IPPE_SQUARE", 3.0, 0.15, 10.0)
    assert np.allclose(estimator.object_points, [[-0.05, 0.05, 0.0], [0.05, 0.05, 0.0],
                                                   [0.05, -0.05, 0.0], [-0.05, -0.05, 0.0]])


def test_synthetic_pose_is_recovered_with_zero_reprojection_error():
    estimator = PoseEstimator(0.10, "IPPE_SQUARE", 3.0, 0.15, 10.0)
    matrix, distortion = PoseEstimator.camera_parameters(camera_info())
    source_rvec = np.array([[0.10], [-0.05], [0.20]], dtype=np.float64)
    source_tvec = np.array([[0.20], [0.10], [2.0]], dtype=np.float64)
    image_points, _ = cv2.projectPoints(estimator.object_points, source_rvec, source_tvec, matrix, distortion)
    recovered = estimator.estimate(image_points.reshape(4, 2), camera_info())
    assert recovered.valid
    assert np.allclose(recovered.tvec, source_tvec, atol=1e-4)
    assert recovered.mean_reprojection_error_px < 1e-3


def test_identity_quaternion_is_normalized_ros_xyzw():
    quaternion = PoseEstimator.rotation_to_quaternion(np.eye(3))
    assert quaternion is not None
    assert np.allclose(quaternion, (0.0, 0.0, 0.0, 1.0))
    assert np.isclose(np.linalg.norm(quaternion), 1.0)


def test_missing_camera_info_is_safely_invalid():
    estimator = PoseEstimator(0.10, "IPPE_SQUARE", 3.0, 0.15, 10.0)
    assert not estimator.estimate(np.zeros((4, 2), dtype=np.float32), None).valid


def _synthetic_estimate(rvec, tvec, *, estimator=None, **estimate_kwargs):
    estimator = estimator or PoseEstimator(0.10, "IPPE_SQUARE", 3.0, 0.15, 10.0)
    matrix, distortion = PoseEstimator.camera_parameters(camera_info())
    image_points, _ = cv2.projectPoints(estimator.object_points, rvec, tvec, matrix, distortion)
    return estimator.estimate(image_points.reshape(4, 2), camera_info(), **estimate_kwargs)


def test_fronto_parallel_marker_has_near_zero_viewing_angle():
    result = _synthetic_estimate(np.zeros((3, 1)), np.array([[0.0], [0.0], [2.0]]))
    assert result.valid
    assert result.viewing_angle_deg < 1.0


def test_steeply_angled_marker_is_rejected_beyond_max_viewing_angle():
    estimator = PoseEstimator(0.10, "IPPE_SQUARE", 3.0, 0.15, 10.0, max_viewing_angle_deg=75.0)
    # ~85 degree tilt about Y: well past the 75 degree gate.
    steep_rvec = np.array([[0.0], [np.deg2rad(85)], [0.0]])
    result = _synthetic_estimate(steep_rvec, np.array([[0.0], [0.0], [2.0]]), estimator=estimator)
    assert not result.valid


def test_position_noise_is_diagonal_and_grows_with_distance():
    near = _synthetic_estimate(np.zeros((3, 1)), np.array([[0.0], [0.0], [1.0]]))
    far = _synthetic_estimate(np.zeros((3, 1)), np.array([[0.0], [0.0], [5.0]]))
    assert near.position_noise.shape == (3, 3)
    assert np.count_nonzero(near.position_noise - np.diag(np.diagonal(near.position_noise))) == 0
    assert np.trace(far.position_noise) > np.trace(near.position_noise)


def test_depth_mismatch_flag_is_set_when_depth_and_pnp_disagree():
    tvec = np.array([[0.0], [0.0], [2.0]])
    consistent = _synthetic_estimate(np.zeros((3, 1)), tvec, depth_position=np.array([0.0, 0.0, 2.0]))
    inconsistent = _synthetic_estimate(np.zeros((3, 1)), tvec, depth_position=np.array([0.0, 0.0, 3.0]))
    assert not (consistent.quality_flags & QualityFlag.DEPTH_MISMATCH)
    assert inconsistent.quality_flags & QualityFlag.DEPTH_MISMATCH
    assert inconsistent.pose_quality < consistent.pose_quality


def test_temporal_consistency_favours_the_candidate_matching_the_previous_pose():
    tvec = np.array([[0.0], [0.0], [2.0]])
    previous_position = np.array([0.0, 0.0, 2.0])
    previous_orientation = np.array(PoseEstimator.rotation_to_quaternion(np.eye(3)))
    result = _synthetic_estimate(np.zeros((3, 1)), tvec, previous_position=previous_position,
                                 previous_orientation=previous_orientation, dt_since_previous_s=0.03)
    assert result.valid
    assert result.pose_quality > 0.9  # Reprojection, geometry, and temporal terms all near-perfect.
