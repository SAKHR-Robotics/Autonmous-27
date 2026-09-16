"""Pure math for composing a TF2-resolved (translation, quaternion) transform
with a camera-frame marker pose, to obtain that pose in base_link (Part 4).

Expected pipeline (see marker_action_interface_node.py):

    camera frame marker pose (Stage 4, MarkerTrackedPose)
        -> TF2 lookup of base_link <- camera_optical_frame at the pose's own
           stamp (a normally-static rover extrinsic, looked up -- never
           manually computed, per Part 4)
        -> transform_to_base_link() (this module): pure composition
        -> base_link marker pose, ready for the clean downstream interface

Kept ROS-free (translation/quaternion are plain numpy arrays, not
geometry_msgs types) so the composition itself is unit-testable without a
TF2 buffer or rclpy -- see test/test_frame_transform.py. The node supplies
the TF2-resolved (translation, quaternion) pair; this module never performs
a TF lookup itself and never fabricates one when the lookup fails (the node
passes `None` upstream in that case; see action_target_builder.py, which
never invents a base_link pose from missing inputs).
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .quaternion_filter import multiply, normalize, to_rotation_matrix


@dataclass(frozen=True)
class ResolvedPose:
    position: np.ndarray                       # base_link-frame marker position (m).
    orientation: np.ndarray                     # base_link-frame marker orientation (x, y, z, w).
    position_covariance: np.ndarray | None      # base_link-frame 3x3 position covariance (m^2), if available.


def transform_to_base_link(*, transform_translation: np.ndarray, transform_rotation: np.ndarray,
                           camera_position: np.ndarray, camera_orientation: np.ndarray,
                           camera_position_covariance: np.ndarray | None) -> ResolvedPose | None:
    """Compose base_link <- camera_optical_frame with a camera-frame marker pose.

    `transform_translation`/`transform_rotation` are the TF2-resolved
    base_link <- camera_optical_frame transform (translation in metres,
    ROS-order (x, y, z, w) rotation quaternion). `camera_position`/
    `camera_orientation` are the marker's own pose in the camera frame
    (MarkerTrackedPose.position/.orientation). Returns None if any input is
    invalid -- callers must never substitute a fabricated pose in that case.
    """
    rotation = to_rotation_matrix(transform_rotation)
    orientation = normalize(camera_orientation)
    position = np.asarray(camera_position, dtype=np.float64)
    if rotation is None or orientation is None or position.shape != (3,) or not np.isfinite(position).all():
        return None
    composed_orientation = multiply(transform_rotation, camera_orientation)
    if composed_orientation is None:
        return None
    base_link_position = rotation @ position + np.asarray(transform_translation, dtype=np.float64)
    base_link_covariance = (rotation @ camera_position_covariance @ rotation.T
                            if camera_position_covariance is not None else None)
    return ResolvedPose(base_link_position, composed_orientation, base_link_covariance)
