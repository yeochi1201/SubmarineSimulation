cmake_minimum_required(VERSION 3.8)
project(submarine_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


ament_package()