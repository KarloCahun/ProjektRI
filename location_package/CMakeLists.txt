cmake_minimum_required(VERSION 3.5)
project(location_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteMission.srv"
)

# Install Python package
install(DIRECTORY location_package/
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  location_package/coordinate_logger.py
  location_package/goal_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
