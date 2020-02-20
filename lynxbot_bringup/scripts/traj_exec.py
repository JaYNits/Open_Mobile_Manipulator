#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math
#from tf.transformations import quaternion_from_euler, euler_from_quaternion
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#get the pose
listener = tf.TransformListener()
#listener.waitForTransform("/odom", "/target", rospy.Time(), rospy.Duration(4.0))

#(trans,rot) = listener.lookupTransform('/odom', '/target', rospy.Time(0))
#if listener.frameExists("/odom") and listener.frameExists("/target"):
#    (trans,rot) = listener.lookupTransform('/odom', '/target', rospy.Time(0))
#else:
#	print("hmm WTF WTF WTF")
'''
group_variable_values = arm_group.get_current_joint_values()
group_variable_values[0] = 0.0
group_variable_values[1] = -0.6
group_variable_values[2] = 1.87
group_variable_values[3] = 0.0
group_variable_values[4] = 0.0
group_variable_values[5] = 0.0
arm_group.set_joint_value_target(group_variable_values)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(2)
'''
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/odom', '/target', rospy.Time(0))
        print(trans)
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue


pose_target = geometry_msgs.msg.Pose()
q = tf.transformations.quaternion_from_euler(0,1.57,0.0)
#orientation_list =[0,0,0,1]#[0.0,0.70688,0.0,0.70733]
#(roll, pitch, yaw) = tf.euler_from_quaternion (orientation_list) 
#print (roll, pitch, yaw)
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]

pose_target.position.x = trans[0]
pose_target.position.y = trans[1]
pose_target.position.z = trans[2] 

pose_target.position.x = trans[0] - 0.1
arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
gripper_group.set_named_target("gripper_open") 
plan_gripper = gripper_group.go(wait=True)
rospy.sleep(2)

pose_target.position.x = trans[0] - 0.02
arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(2)
gripper_group.set_named_target("gripper_close") 
plan_gripper = gripper_group.go(wait=True)
rospy.sleep(2)

pose_target.position.z = trans[2] + 0.05
#pose_target.position.x = trans[0] - 0.1
arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(2)
gripper_group.set_named_target("gripper_open") 
plan_gripper = gripper_group.go(wait=True)
rospy.sleep(2)
'''
group_variable_values = arm_group.get_current_joint_values()
group_variable_values[0] = 0.0
group_variable_values[1] = -0.6
group_variable_values[2] = 1.87
group_variable_values[3] = 0.0
group_variable_values[4] = 0.0
group_variable_values[5] = 0.0
arm_group.set_joint_value_target(group_variable_values)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(2)
'''
moveit_commander.roscpp_shutdown()
