cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_aggregator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Resolve Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(occupancy_grid_aggregator_srv REQUIRED)
set(DEPENDENCIES rclcpp tf2_ros tf2 tf2_geometry_msgs geometry_msgs nav_msgs occupancy_grid_aggregator_srv)


add_executable(aggregator_node src/aggregator_node.cpp src/aggregator.cpp)
ament_target_dependencies(aggregator_node ${DEPENDENCIES})
install(TARGETS aggregator_node
        DESTINATION lib/${PROJECT_NAME})


# Resolve and install include filepaths
include_directories(
  src
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
