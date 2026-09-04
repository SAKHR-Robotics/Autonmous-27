int32 id
float32 center_x
float32 center_y
float32 corner_1_x
float32 corner_1_y
float32 corner_2_x
float32 corner_2_y
float32 corner_3_x
float32 corner_3_y
float32 corner_4_x
float32 corner_4_y
float32 area_px
# Improvement #4/#7: 2D-detection-only quality (blur, contrast, geometry); [0,1].
float32 detection_confidence
# Improvement #22: bitmask, see marker_detection/quality_flags.py QualityFlag.
uint32 quality_flags
