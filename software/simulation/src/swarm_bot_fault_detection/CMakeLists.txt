cmake_minimum_required(VERSION 3.8)
project(swarm_bot_fault_detection)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS swarm_bot_fault_detection/heartbeat_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()