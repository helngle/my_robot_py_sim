cmake_minimum_required(VERSION 3.8)
project(my_robot_maps)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
