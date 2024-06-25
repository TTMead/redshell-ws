cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_aggregator_srv)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetAggregateGrid.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
