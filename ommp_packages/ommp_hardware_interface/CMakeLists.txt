cmake_minimum_required(VERSION 3.0.2)
project(ommp_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  trajectory_msgs
  transmission_interface
  urdf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)


add_library(ommp_hardware_interface src/ommp_hardware_interface.cpp)

target_link_libraries(ommp_hardware_interface
  ${catkin_LIBRARIES})
  
add_executable(ommp_hardware_interface_node src/ommp_hardware_interface_node.cpp)
add_dependencies(ommp_hardware_interface_node ommp_hardware_interface)

target_link_libraries(ommp_hardware_interface_node
	ommp_hardware_interface
  ${catkin_LIBRARIES})
  
#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS ommp_hardware_interface_node ommp_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
