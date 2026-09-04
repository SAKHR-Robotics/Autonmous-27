cmake_minimum_required(VERSION 3.8)
project(marker_detection_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetection.msg"
  "msg/MarkerDetectionArray.msg"
  "msg/Marker3DDetection.msg"
  "msg/Marker3DDetectionArray.msg"
  "msg/MarkerPose.msg"
  "msg/MarkerPoseArray.msg"
  "msg/MarkerTrackedPose.msg"
  "msg/MarkerTrackedPoseArray.msg"
  "msg/MarkerMapEntry.msg"
  "msg/MarkerMap.msg"
  "msg/MarkerActionTarget.msg"
  "msg/MarkerActionTargetArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
