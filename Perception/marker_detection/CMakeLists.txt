cmake_minimum_required(VERSION 3.8)
project(marker_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(marker_detection_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/generate_aruco_marker.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS marker_detection/marker_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marker_detection_node)
install(PROGRAMS marker_detection/marker_map_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marker_map_node)
install(PROGRAMS marker_detection/marker_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marker_tf_broadcaster)
install(PROGRAMS marker_detection/marker_action_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marker_action_interface_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(marker_detection_pytest test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
