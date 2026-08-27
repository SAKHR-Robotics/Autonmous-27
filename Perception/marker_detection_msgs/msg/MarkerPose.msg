std_msgs/Header header
int32 id
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float32 roll
float32 pitch
float32 yaw
float32 reprojection_error_px
float32 max_reprojection_error_px
float32 depth_position_x
float32 depth_position_y
float32 depth_position_z
float32 position_difference_m
bool pose_valid
# Improvement #1: angle between the marker normal and the camera optical axis (deg); 0=fronto-parallel.
float32 viewing_angle_deg
# Improvement #4: split confidence types, [0,1] each. See marker_detection/pose_estimator.py and tracker_manager.py.
float32 detection_confidence
float32 pose_quality
# Improvement #22: bitmask, see marker_detection/quality_flags.py QualityFlag.
uint32 quality_flags
# Improvement #3: row-major 3x3 position-measurement covariance (m^2), derived from
# reprojection error and range, not a fixed constant. All-zero means "not computed".
float64[9] position_covariance
