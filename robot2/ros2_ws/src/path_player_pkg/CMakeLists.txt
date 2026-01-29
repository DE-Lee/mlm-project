cmake_minimum_required(VERSION 3.8)
project(path_player_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

# Install Python package
install(
  DIRECTORY path_player_pkg/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  path_player_pkg/path_player_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME path_player_server
)

# path_visualizer.py removed (moved to trash)
# install(PROGRAMS
#   path_player_pkg/path_visualizer.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME path_visualizer
# )

install(PROGRAMS
  path_player_pkg/motion_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_controller
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install action files
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Install data files
install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource marker file
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
