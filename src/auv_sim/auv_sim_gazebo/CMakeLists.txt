cmake_minimum_required(VERSION 3.8)
project(auv_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch models
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/spawn_harmonic.py
  scripts/build_ros_gz_harmonic.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
