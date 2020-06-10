
#! /bin/bash

x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=navigation 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch moveit_interface moveit.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e rosrun moveit_interface set_start_pos.py 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e rostopic pub -1 /kinect_controller/command std_msgs/Float64 "data: 0.0" 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e roslaunch ommp_bringup gmapping.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup move_base.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch explore_lite explore.launch 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=navigation 2>/dev/null &



