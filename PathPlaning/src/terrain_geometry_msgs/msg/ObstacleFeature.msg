# A single detected obstacle/terrain feature.

# Unique identifier for this obstacle (useful for tracking across frames)
uint32 id

# Center point of the obstacle in the message's frame_id
geometry_msgs/Point centroid

# Approximate radius of the obstacle (meters), 0 if unknown
float32 radius

# Approximate height of the obstacle (meters), 0 if unknown
float32 height

# Optional classification label (e.g. "rock", "crater", "unknown")
string classification

# Detection confidence, range [0.0, 1.0]
float32 confidence
