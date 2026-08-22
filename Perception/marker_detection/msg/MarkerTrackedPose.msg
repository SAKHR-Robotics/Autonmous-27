std_msgs/Header header
int32 id
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
bool detection_valid
bool tracking_valid
string track_state
float32 confidence
float32 reprojection_error_px
float32 position_innovation_m
float32 orientation_innovation_deg
uint32 detection_count
uint32 missed_frames
# Improvement #4: split confidence types, [0,1] each. `confidence` above is
# min(detection_confidence, pose_quality, tracking_confidence): the conservative overall score.
float32 detection_confidence
float32 pose_quality
float32 tracking_confidence
# Improvement #1: angle between the marker normal and the camera optical axis (deg).
float32 viewing_angle_deg
float32 distance_m
# Improvement #22: bitmask, see marker_detection/quality_flags.py QualityFlag.
uint32 quality_flags
# Improvement #3: row-major 3x3 fused position covariance (m^2) from the tracking Kalman filter.
float64[9] position_covariance
# Improvement #2: true if this update included a depth-Z fusion step, not PnP alone.
bool depth_fused
# Downstream-interface Part 3: seconds since the last valid detection (0.0 if
# detected this update); purely derived from existing tracker timestamps.
float32 age_seconds
