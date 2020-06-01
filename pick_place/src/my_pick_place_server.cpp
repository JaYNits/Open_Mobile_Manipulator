/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Perception project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include<pick_place/my_pick_place_server.h>

MyPickPlace::MyPickPlace(ros::NodeHandle nh)
  : nh_(nh),
    move_group(PLANNING_GROUP),
    gripper_group(GRIPPER_GROUP)
{
  grasp_client = nh.serviceClient<pick_place::Grasp>("/get_grasp");

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  gripper_joint_model_group =
    gripper_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("robot_footprint"));
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->loadRemoteControl();

  // Create text marker for displaying current state
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  text_pose.translation().x() = 0.3;
  visual_tools_ptr->publishText(text_pose, "Pick and Place",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);

  // Publish messages to rviz
  visual_tools_ptr->trigger();
  //visual_tools_ptr->prompt("Click Next");

  /*
   * Collision Objects:
   * Create an object list and populate it with custom virtual objects
   * Then insert objects in scene for collision avoidance and interaction
   */
  std::vector<moveit_msgs::CollisionObject> collision_object_list;
  //std::vector<std::string> object_ids;
  moveit_msgs::CollisionObject book_collision_object, object_collision_object;
  // Define pose for the objects (specified relative to base_footprint)
  geometry_msgs::Pose book_pose;

  book_pose.position.x = 0.4;
  book_pose.position.y = 0.0;
  book_pose.position.z = 0.0;
  book_pose.orientation.w = 0;//0.707;
  book_pose.orientation.x = 0;
  book_pose.orientation.y = 0;
  book_pose.orientation.z = 0;//0.707;


  SetupCollisionObject("book", book_pose,
                       book_collision_object);
  //SetupCollisionObject("right_dropbox", DROPBOX_MESH_PATH, right_mesh_pose,
  //                     right_dropbox_collision_object);

  collision_object_list.push_back(book_collision_object);
  //collision_object_list.push_back(right_dropbox_collision_object);


  // Add the object list to the world scene
  planning_scene_interface.addCollisionObjects(collision_object_list);

  //test
  //Attach_object_to_gripper(1);
  ros::Duration(5).sleep();
  //Attach_object_to_gripper(0);
  //test

  //start pos
  //move_group.setNamedTarget("out_of_view"); //out_of_view

  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

}

bool MyPickPlace::Routine(pick_place::PickPlace::Request &req,
                      pick_place::PickPlace::Response &res)
{

  // test
  visual_tools_ptr->deleteAllMarkers();

  // Create text marker for displaying current state
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  text_pose.translation().x() = 0.3;
  visual_tools_ptr->publishText(text_pose, "New request received",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  // poses from request
  geometry_msgs::Pose dist_pose, pick_pose, lift_pose;
  std::vector<double> pre_place_pose, place_pose, away_pose;
  dist_pose = req.dist_pose;
  pick_pose = req.pick_pose;
  lift_pose = req.lift_pose;
  //place_pose = req.place_pose;

  // Plan arm motion
  // set starting pose
  

  // set safe distance pose
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(dist_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO("Visualizing plan to safe distance from object as trajectory line");
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(dist_pose, "safe_distance_pose");
  visual_tools_ptr->publishText(text_pose, "Safe Distance From Object", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);

  //Close In movement
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pick_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(pick_pose, "pick_pose");
  visual_tools_ptr->publishText(text_pose, "Grasping Object Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);


  //Close Gripper
  OperateGripper(true);
  ros::Duration(2.0).sleep();

  //Lift movement
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(lift_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(lift_pose, "lift_pose");
  visual_tools_ptr->publishText(text_pose, "Lift Object Off the Ground", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);


  // Go to Pre Place Joint state pos

  moveit::core::RobotStatePtr arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      pre_place_pose);
  pre_place_pose[0] = 1.5;
  pre_place_pose[1] = 0.0;
  pre_place_pose[2] = 1.5;
  pre_place_pose[3] = 0.0;
  pre_place_pose[4] = 0.0;
  pre_place_pose[5] = 0.0;
  move_group.setJointValueTarget(pre_place_pose);
  ros::Duration(1.5).sleep();
  //bool success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");
  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(pre_place_pose, "pre_place_pose");
  visual_tools_ptr->publishText(text_pose, "Pre Place Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);


  // Go to Place Joint state pos
  
  arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      place_pose);
  place_pose[0] = 1.5;
  place_pose[1] = 0.8;
  place_pose[2] = 1.5;
  place_pose[3] = 0.0;
  place_pose[4] = -0.8;
  place_pose[5] = 0.0;
  move_group.setJointValueTarget(place_pose);
  ros::Duration(1.5).sleep();
  //bool success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(place_pose, "place_pose");
  visual_tools_ptr->publishText(text_pose, "Place Object to Ground", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);

  //Open Gripper
  
  OperateGripper(false);
  ros::Duration(5.0).sleep();

  // Move away from Object

  arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      away_pose);
  away_pose[0] = 1.5;
  away_pose[1] = 0.0;
  away_pose[2] = 1.5;
  away_pose[3] = 0.0;
  away_pose[4] = 0.0;
  away_pose[5] = 0.0;
  move_group.setJointValueTarget(away_pose);
  ros::Duration(1.5).sleep();
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(away_pose, "away_pose");
  visual_tools_ptr->publishText(text_pose, "Move Away From Object", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);

  //Go to Start position
  
  move_group.setNamedTarget("start");
  ros::Duration(1.5).sleep();
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");
  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(start_pose, "start_pose");
  visual_tools_ptr->publishText(text_pose, "Go to Start location", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  res.success = true;
  
}


bool MyPickPlace::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    gripper_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.02;
    gripper_joint_positions[1] = 0.02;
  }
  else
  {
    gripper_joint_positions[0] = 0.0;
    gripper_joint_positions[1] = 0.0;
  }

  gripper_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = gripper_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  return success;
}

bool MyPickPlace::SetupCollisionObject(const std::string &object_id,
    const geometry_msgs::Pose &object_pose,
    moveit_msgs::CollisionObject &collision_object)
{
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = object_id;
  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = 0.2;
  collision_object.primitives[0].dimensions[1] = 0.05;
  collision_object.primitives[0].dimensions[2] = 0.35;
  /* Define the pose of the table. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position = object_pose.position;
  collision_object.primitive_poses[0].orientation = object_pose.orientation;

  //collision_object.mesh_poses[0].position = object_pose.position;
  //collision_object.mesh_poses[0].orientation = object_pose.orientation;

}

void MyPickPlace::Attach_object_to_gripper(int i){

  // TEEEEEEEEEEEEEEEEEEEEEST
  
  ros::Publisher object_pub = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object",10);
ros::Duration(0.5).sleep();
moveit_msgs::CollisionObject object;
 geometry_msgs::Pose object_pose;
object_pose.position.x = 0.08;
object_pose.position.y = 0;
object_pose.position.z = 0.08;
object_pose.orientation.w = 0;
object_pose.orientation.x = 0;
object_pose.orientation.y = 0;
object_pose.orientation.z = 0;
shape_msgs::SolidPrimitive shape;
shape.type = shape.BOX;
shape.dimensions.push_back(0.17); //x dimension
shape.dimensions.push_back(0.04); //y dimension
shape.dimensions.push_back(0.04); //z dimension
object.header.frame_id = "/link_5";
object.header.stamp = ros::Time::now();
object.primitives.push_back(shape);             //shape is of type shape_msgs::SolidPrimitive
object.primitive_poses.push_back(object_pose);  //object_pose is of type geometry_msgs::Pose
object.operation = object.ADD;
object.id = "object1";
object_pub.publish(object);

//Sleep to make sure the object is received within the MoveGroup node
ros::Duration(0.5).sleep();

//Attach the object
ros::Publisher aco_pub = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",10);
ros::Duration(0.5).sleep();
moveit_msgs::AttachedCollisionObject aco;
aco.object.id = object.id;
aco.link_name = "link_5";
if(i){
aco.touch_links.push_back(aco.link_name);

  aco.object.operation = moveit_msgs::CollisionObject::ADD;}
else
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
aco_pub.publish(aco);

//Sleep again
ros::Duration(0.5).sleep();
/*
//Check if the object is now attached using the /get_planning_scene service
ros::ServiceClient client_get_scene = nodehandle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
moveit_msgs::GetPlanningScene scene_srv;
scene_srv.request.components.components = scene_srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS;
 if (!client_get_scene.call(scene_srv)){
    ROS_WARN("Failed to call service /get_planning_scene");
} else {
    ROS_INFO_STREAM("Number of attached bodies according to /get_planning_scene: " << scene_srv.response.scene.robot_state.attached_collision_objects.size());
}

//Check if the object is now attached by getting the current state of the robot via the MoveGroup interface
robot_state::RobotStatePtr RS = move_group.getCurrentState();
std::vector<const robot_state::AttachedBody*> bodies;
RS->getAttachedBodies(bodies);
ROS_INFO_STREAM("Number of attached bodies according to MoveGroup interface: " << bodies.size());
*/

  // TEEEEEEEEEEEEEEEEEEEEEEST

}

tf::Quaternion MyPickPlace::RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}

MyPickPlace::~MyPickPlace(){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_pick_place_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(8);
  spinner.start();
  MyPickPlace my_pick_place(nh);
  ros::ServiceServer service = nh.advertiseService("pick_place_routine", &MyPickPlace::Routine, &my_pick_place);
  ros::waitForShutdown();
  return 0;
}
