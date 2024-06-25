cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
set(DEPENDENCIES rclcpp robot_localization tf2_ros geometry_msgs)

# Create Ekf Supervisor Executable
add_executable(ekf_supervisor src/ekf_supervisor.cpp src/ekf_supervisor_node.cpp)
ament_target_dependencies(ekf_supervisor ${DEPENDENCIES})
target_include_directories(ekf_supervisor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(ekf_supervisor PUBLIC cxx_std_17)
install(TARGETS ekf_supervisor DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
