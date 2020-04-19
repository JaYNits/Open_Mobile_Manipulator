
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=object_rec_test 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch sensor_stick object_recognition.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_object_recognition 2>/dev/null &



