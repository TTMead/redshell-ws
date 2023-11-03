cmake_minimum_required(VERSION 3.8)
project(beboop_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Resolve Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies rclcpp OpenCV sensor_msgs cv_bridge)


# Install executables
add_executable(track_error_driver src/track_error_driver.cpp src/vision_driver.cpp)
ament_target_dependencies(track_error_driver ${dependencies})
install(TARGETS track_error_driver
        DESTINATION lib/${PROJECT_NAME})


# Resolve and install include filepaths
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
install(
  DIRECTORY include
  DESTINATION include
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
