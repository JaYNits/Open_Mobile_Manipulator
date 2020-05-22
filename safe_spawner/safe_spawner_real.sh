
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup real_bringup.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_view_urdf 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup serial.launch 2>/dev/null &
#sleep 3 &&

#x-terminal-emulator -e  rqt 2>/dev/null &

