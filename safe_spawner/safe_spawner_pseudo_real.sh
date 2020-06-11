
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup real_bringup.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_viz rviz.launch config:=basic 2>/dev/null &



