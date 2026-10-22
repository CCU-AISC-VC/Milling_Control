cmake_minimum_required(VERSION 3.8)
project(m710id_description_spindle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.py"
)

# Install config files  
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh files - CRITICAL FOR VISUALIZATION
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install robot files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/robots")
  install(DIRECTORY robots
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()