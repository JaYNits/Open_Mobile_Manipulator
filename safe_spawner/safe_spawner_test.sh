#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup real_bringup.launch 2>/dev/null &&


x-terminal-emulator -e rosrun ommp_bringup set_start_pos.py 2>/dev/null &


