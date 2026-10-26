# marker_detection_msgs

ROS 2 Jazzy message definitions used by the real marker pipeline:

| Group | Messages |
| --- | --- |
| Image observations | MarkerDetection / MarkerDetectionArray |
| Depth and raw poses | Marker3DDetection / Marker3DDetectionArray, MarkerPose / MarkerPoseArray |
| Tracked observations | MarkerTrackedPose / MarkerTrackedPoseArray |
| Global map | MarkerMapEntry / MarkerMap |
| Action interface | MarkerActionTarget / MarkerActionTargetArray |

Existing schemas are preserved. Consumers must use headers, tracking/detection
validity, age, quality flags and covariance rather than treating any published
pose as usable. The optional SLAM adapter uses RTAB-Map's native messages;
it does not change these interfaces. See the [runbook](../RUNBOOK.md).
