cmake_minimum_required(VERSION 3.8)
project(submarine_simulation)


#---------------------------------------
# Base Compile Option
#---------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#VSCode IntelliSence
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#---------------------------------------
# ROS 2 Build System
#---------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#---------------------------------------
# Gazebo Fortress (Ignition Gazebo 6) Dependency
# gazebo6 : sim core (System, Link, Model)
# math6 : Vector3D
# common4 : ignmsg/ignerr
# plugin1 : IGNITION_ADD_PLUGIN MACRO
#---------------------------------------
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

#---------------------------------------
# Plugin Library Build
#---------------------------------------
set(M1_PLUGIN_NAME m1_thruster_system)

add_library(${M1_PLUGIN_NAME} SHARED
  plugin/m1_plugins/source/m1_thruster_system.cpp
)

target_include_directories(${M1_PLUGIN_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugin/m1_plugins/header>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${M1_PLUGIN_NAME}
  rclcpp
  std_msgs
  ignition-gazebo6
  ignition-math6
  ignition-common4
  ignition-plugin1
)

#---------------------------------------
# Plugin Install
#---------------------------------------
install(TARGETS ${M1_PLUGIN_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY plugin/m1_plugins/header/
  DESTINATION include
)

#---------------------------------------
# Model & World Install
#---------------------------------------
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()