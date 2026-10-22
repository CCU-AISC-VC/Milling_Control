cmake_minimum_required(VERSION 3.8)
project(m710id_70_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
