
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup amcl.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup move_base.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_navigation 2>/dev/null &



