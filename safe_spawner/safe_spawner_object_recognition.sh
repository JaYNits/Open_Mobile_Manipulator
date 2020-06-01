
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=simple_world 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup moveit.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch sensor_stick object_recognition.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e rostopic pub -1 /kinect_controller/command std_msgs/Float64 "data: 0.0" 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_bringup rviz.launch rviz_config:=rviz_object_recognition 2>/dev/null &



