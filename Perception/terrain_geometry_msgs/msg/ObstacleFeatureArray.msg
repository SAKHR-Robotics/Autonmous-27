# A stamped collection of ObstacleFeature, one per detected obstacle
# cluster in the current frame. Published by terrain_node.py on
# /terrain/obstacle_features.

std_msgs/Header header
ObstacleFeature[] obstacles
