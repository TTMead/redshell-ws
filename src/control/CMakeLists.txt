cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
set(DEPENDENCIES rclcpp geometry_msgs std_msgs nav_msgs tf2_ros)

# Create Pure Pursuit Executable
add_executable(pure_pursuit src/pure_pursuit.cpp src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit ${DEPENDENCIES})
target_include_directories(pure_pursuit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pure_pursuit PUBLIC cxx_std_17)
install(TARGETS pure_pursuit DESTINATION lib/${PROJECT_NAME})

# Create VS Control Executable
add_executable(vs_control src/vs_control/vs_control.cpp)
ament_target_dependencies(vs_control ${DEPENDENCIES})
target_include_directories(vs_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/vs_control)
install(TARGETS vs_control
        DESTINATION lib/${PROJECT_NAME})

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
