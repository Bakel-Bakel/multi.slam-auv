cmake_minimum_required(VERSION 3.8)
project(auv_sim_stonefish)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scenarios launch
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/build_stonefish.sh
        DESTINATION lib/${PROJECT_NAME})

ament_package()
