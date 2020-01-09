#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

pose_target = geometry_msgs.msg.Pose()
q = quaternion_from_euler(0,1.57,0.0)
orientation_list =[0,0,0,1]#[0.0,0.70688,0.0,0.70733]
(roll, pitch, yaw) = euler_from_quaternion (orientation_list) 
print (roll, pitch, yaw)
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]

pose_target.position.x = 0.39044#0.34044
pose_target.position.y = 0.004467
pose_target.position.z = 0.328661
group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait=True)
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
