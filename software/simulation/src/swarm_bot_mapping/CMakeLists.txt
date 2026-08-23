cmake_minimum_required(VERSION 3.8)
project(swarm_bot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    swarm_bot_mapping/map_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()