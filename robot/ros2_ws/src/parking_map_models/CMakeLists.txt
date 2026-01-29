cmake_minimum_required(VERSION 3.8)
project(parking_map_models)

find_package(ament_cmake REQUIRED)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
