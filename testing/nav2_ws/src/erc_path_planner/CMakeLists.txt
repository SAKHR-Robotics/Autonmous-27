cmake_minimum_required(VERSION 3.8)
project(erc_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(terrain_geometry_msgs REQUIRED)


# --------------------------------------------------
# Costmap Bridge Node
# --------------------------------------------------

add_executable(costmap_bridge_node
  src/costmap_bridge_node.cpp
)

ament_target_dependencies(costmap_bridge_node
  rclcpp
  sensor_msgs
  terrain_geometry_msgs
)


# --------------------------------------------------
# Install executable
# --------------------------------------------------

install(
  TARGETS
    costmap_bridge_node
  DESTINATION
    lib/${PROJECT_NAME}
)


# --------------------------------------------------
# Install package directories
# --------------------------------------------------

install(
  DIRECTORY
    config
    launch
    rviz
    behavior_trees
  DESTINATION
    share/${PROJECT_NAME}
)


# --------------------------------------------------
# Testing / Linting
# --------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright check
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()


ament_package()