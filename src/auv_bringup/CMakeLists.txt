cmake_minimum_required(VERSION 3.8)
project(auv_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES INTERFACE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
