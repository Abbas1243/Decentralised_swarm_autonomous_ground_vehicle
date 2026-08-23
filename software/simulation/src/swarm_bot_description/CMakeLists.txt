cmake_minimum_required(VERSION 3.8)
project(swarm_bot_description)

find_package(ament_cmake REQUIRED)

# Install all directories
install(
  DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()