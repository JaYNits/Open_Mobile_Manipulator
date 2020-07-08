cmake_minimum_required(VERSION 2.8.3)
project(ommp_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  tf
  control_msgs
  sensor_msgs
  rtabmap_ros
  amcl
  map_server
  move_base
  robot_localization



)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_Pub_to_PCA9685_Driver src/pub_to_pca9685_driver.cpp)
add_executable(${PROJECT_NAME}_Cmd_Vel_to_Arduino src/cmd_vel_mux.cpp)
add_executable(${PROJECT_NAME}_Diff_TF src/diff_tf.cpp)
add_executable(${PROJECT_NAME}_Camera_Relay src/camera_relay.cpp)


set_target_properties(${PROJECT_NAME}_Pub_to_PCA9685_Driver PROPERTIES OUTPUT_NAME Pub_to_PCA9685_Driver PREFIX "")
set_target_properties(${PROJECT_NAME}_Cmd_Vel_to_Arduino PROPERTIES OUTPUT_NAME Cmd_Vel_to_Arduino PREFIX "")
set_target_properties(${PROJECT_NAME}_Diff_TF PROPERTIES OUTPUT_NAME Diff_TF PREFIX "")

set_target_properties(${PROJECT_NAME}_Camera_Relay PROPERTIES OUTPUT_NAME Camera_Relay PREFIX "")


add_dependencies(${PROJECT_NAME}_Pub_to_PCA9685_Driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_Cmd_Vel_to_Arduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_Diff_TF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_Camera_Relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_Pub_to_PCA9685_Driver
   ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_Cmd_Vel_to_Arduino
   ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_Diff_TF
   ${catkin_LIBRARIES}
 )
 
 target_link_libraries(${PROJECT_NAME}_Camera_Relay
 ${catkin_LIBRARIES}
)



