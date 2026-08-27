# Geometric features extracted for a single obstacle cluster (Step 6).
#
# Convention (matches ObstacleFeatureExtractor in terrain_geometry):
#   width  <- Y extent (lateral, left-right)
#   height <- Z extent (vertical, up-down)
#   depth  <- X extent (longitudinal, front-back)

int32 id
int32 num_points

geometry_msgs/Point centroid
geometry_msgs/Point min_point
geometry_msgs/Point max_point

float32 width
float32 height
float32 depth

float32 distance
