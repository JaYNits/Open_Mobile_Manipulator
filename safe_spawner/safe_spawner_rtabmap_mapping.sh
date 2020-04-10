
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=mydesigned_world 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e rostopic pub -1 /kinect_controller/command std_msgs/Float64 "data: -0.5" 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e roslaunch ommp_bringup rtabmap_mapping.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_rtabmap_mapping 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup teleop.launch 2>/dev/null &



