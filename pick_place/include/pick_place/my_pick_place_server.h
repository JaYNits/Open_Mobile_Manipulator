/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Perception project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#ifndef MY_PICK_PLACE_SERVER_H
#define MY_PICK_PLACE_SERVER_H

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

#include <geometric_shapes/shape_operations.h>

#include <eigen_conversions/eigen_msg.h>
#include <pick_place/PickPlace.h>
#include <pick_place/Grasp.h>
#include <tf/tf.h>

class MyPickPlace
{
public:
  explicit MyPickPlace(ros::NodeHandle nh);
  ~MyPickPlace();

  bool Routine(pick_place::PickPlace::Request &req,
                      pick_place::PickPlace::Response &res);

private:
  ros::NodeHandle nh_;

  ros::ServiceClient client, grasp_client;

  std::vector<geometry_msgs::Pose> grasp_list;
  bool left_success, right_success;

  const std::string PLANNING_GROUP = "arm";
  const std::string GRIPPER_GROUP = "gripper";


  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::MoveGroupInterface gripper_group;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;


  const robot_state::JointModelGroup *joint_model_group;
  const robot_state::JointModelGroup *gripper_joint_model_group;
  

  // Define PlanningSceneInterface object to add and remove collision objects
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
  bool success;
  //ros::Publisher world_joint_pub;
  
  //TEEEEEST
  //ros::Publisher aco_pub;
  //teeest

/*
   * Functions for gripper actuation
   * close_gripper = 0; open gripper
   *                 = 1; close gripper
   */
  //test
  void Attach_object_to_gripper(int i);
  //test
  bool OperateGripper(const bool &close_gripper);

  bool SetupCollisionObject(const std::string &object_id,
                            const geometry_msgs::Pose &object_pose,
                            moveit_msgs::CollisionObject &collision_object);

  tf::Quaternion RPYToQuaternion(float R, float P, float Y);

  //bool IsPickPoseWithinLimits(geometry_msgs::Pose &pick_pose,
  //                            geometry_msgs::Pose &act_obj_pose);
};

#endif  // MY_PICK_PLACE_SERVER_H
