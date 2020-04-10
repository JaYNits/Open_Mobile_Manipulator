
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=simple_world 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup gmapping.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup move_base.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch explore_lite explore.launch 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_navigation 2>/dev/null &



