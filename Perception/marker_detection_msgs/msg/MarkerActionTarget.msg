# Clean, high-level marker interface for manipulation/planning nodes (Parts
# 3, 4, 5, 6, 7). Published on /marker_detection/targets by
# marker_action_interface_node, derived entirely from the existing Stage 4
# MarkerTrackedPose plus a TF2-resolved base_link pose -- consumers of this
# message never need to know about ArUco, depth filtering, PnP, tracking,
# covariance, or TF lookups.
#
# header.frame_id is always "base_link" (or the configured base_link_frame);
# header.stamp is the *originating* tracked-pose timestamp, not "now".
std_msgs/Header header
int32 marker_id

# Part 7: NEW/CONFIRMED/DEGRADED/LOST, reusing tracker_manager.TrackState
# verbatim -- no second tracking state machine.
string tracking_state
bool detected
# True only when tracking_state == "CONFIRMED" (Part 7); the existing
# tracking system's own confirmation, not a new one.
bool stable

# Part 4: marker pose in base_link. Never fabricated: if pose_valid is
# false (TF lookup failed/unavailable), position/orientation/distances below
# are all invalid and must be ignored.
bool pose_valid
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# Part 5: distances derived from `position` above (base_link, REP-103:
# +X forward, +Y left, +Z up). NaN when pose_valid is false.
float32 distance_to_marker    # sqrt(x^2 + y^2 + z^2)
float32 forward_distance      # position.x
float32 lateral_distance      # position.y
float32 vertical_distance     # position.z

# Part 6: the single safety/quality gate downstream planning needs -- derived
# from the existing quality/tracking system (quality_flags.py,
# tracker_manager.py), not a new detection algorithm. See
# action_target_builder.build_action_target for the exact conditions.
bool usable_for_action

float32 confidence
# Improvement #22 bitmask, see marker_detection/quality_flags.py QualityFlag.
# TF_UNAVAILABLE is set whenever pose_valid is false.
uint32 quality_flags
# Row-major 3x3 base_link-frame position covariance (m^2), rotated from the
# Stage 4 camera-frame covariance; all-zero/absent means "not computed".
float64[9] position_covariance
# Seconds since the last valid detection (0.0 if detected this update).
float32 age_seconds
