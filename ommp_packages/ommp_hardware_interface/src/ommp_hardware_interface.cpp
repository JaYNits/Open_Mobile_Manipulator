#include <cstdlib>
#include <iostream>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ommp_hardware_interface/ommp_hardware_interface.h>
#include <sstream>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

namespace ommp_hardware_interface {
OmmpHardwareInterface::OmmpHardwareInterface(ros::NodeHandle &nh) : nh_(nh) {
  init();
  controller_manager_.reset(
      new controller_manager::ControllerManager(this, nh_));
  nh_.param("/ommp/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ =
      nh_.createTimer(update_freq, &OmmpHardwareInterface::update, this);
  wheel_vel_pub = nh_.advertise<std_msgs::Float32MultiArray>("set_vel", 10);
  wheel_enc_sub = nh_.subscribe("/encoder_ticks", 10,
                                &OmmpHardwareInterface::enc_ticks_CB, this);
}

OmmpHardwareInterface::~OmmpHardwareInterface() {}

void OmmpHardwareInterface::init() {
  // Get joint names
  nh_.getParam("/ommp/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize Controller
  for (int i = 0; i < num_joints_; ++i) {
    /* ommpcpp::Joint joint = luxo.getJoint(joint_names_[i]); */

    // Create joint state interface
    JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                      &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Limits
    JointLimits limits;
    SoftJointLimits softLimits;
    getJointLimits(joint_names_[i], nh_, limits);

    // Create position joint interface
    JointHandle jointPositionHandle(jointStateHandle,
                                    &joint_position_command_[i]);
    PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits,
                                                    softLimits);
    positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
    position_joint_interface_.registerHandle(jointPositionHandle);

    // Create velocity joint interface
    JointHandle jointVelocityHandle(jointStateHandle,
                                    &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create effort joint interface
    // JointHandle jointEffortHandle(jointStateHandle,
    // &joint_effort_command_[i]);
    // effort_joint_interface_.registerHandle(jointEffortHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  // registerInterface(&effort_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);
}

void OmmpHardwareInterface::update(const ros::TimerEvent &e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read(elapsed_time_);
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);

  // Clear Array
  vel_setpoint_cmd_array.data.clear();
  // Store Prev encoder ticks
  for (int i = 0; i < 4; i++) {
    prev_enc[i] = curr_enc[i];
  }
}

void OmmpHardwareInterface::read(ros::Duration elapsed_time) {
  /* for (int i = 0; i < num_joints_; i++) {
      joint_position_[i] = ommp.getJoint(joint_names_[i]).read();
  } */

  // To read From the Real Feedback
  ///*
  this -> enc_feedback_to_joint_pos_vel(elapsed_time);
  joint_velocity_[0] = joint_feedback_vel[0];
  joint_position_[0] = joint_feedback_pos[0];
  joint_velocity_[1] = joint_feedback_vel[1];
  joint_position_[1] = joint_feedback_pos[1];
  joint_velocity_[3] = joint_feedback_vel[2];
  joint_position_[3] = joint_feedback_pos[2];
  joint_velocity_[4] = joint_feedback_vel[3];
  joint_position_[4] = joint_feedback_pos[3];
  //*/

  // For Dummy passthrough Vel without the use of topics
  /*
  for (int i = 0; i < 5; i++) {
    if(i==2) continue; // Skip the Kinect joint at pos 2
    joint_velocity_[i] = joint_velocity_command_[i];
    joint_position_[i] += joint_velocity_command_[i] * elapsed_time_.toSec();
  }
  //*/

  // Assume Perfect Execution for this joints dummy pass through
  joint_position_[2] = joint_position_command_[2];
  for (int i = 5; i < num_joints_; i++) {
    joint_position_[i] = joint_position_command_[i];
  }
}

void OmmpHardwareInterface::write(ros::Duration elapsed_time) {
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
  velocityJointSoftLimitsInterface.enforceLimits(elapsed_time);

  // To Publish to the Arduino Vel Setpoint Callback
  ///*
  this -> cmd_to_setpoint();
  // Transform From joint_velocity_command to setpoint
  this->vel_setpoint_cmd_array.data.push_back(vel_set_cmd[0]);
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[1]);
  wheel_vel_pub.publish(vel_setpoint_cmd_array);
  //*/
  /*
  std::cout << "setting velocity joint 0 to " <<
  joint_velocity_command_[0] << "\n";
  std::cout << "setting velocity joint 1 to " <<
  joint_velocity_command_[1] << "\n";
  std::cout << "setting velocity joint 3 to " <<
  joint_velocity_command_[3] << "\n";
  std::cout << "setting velocity joint 4 to " <<
  joint_velocity_command_[4] << "\n";
  //*/
  /*  
  for(int i=0; i<num_joints_ ; i++) {
          std::cout << "setting position joint " << i << " to " <<
  joint_position_command_[i] << "\n";
  }
  /*
  for(int i = num_joints_ - 4; i<num_joints_; i++) {
          std::cout << "setting velocity joint " << i << " to " <<
  joint_velocity_command_[i] << "\n";
  }*/
  /* for (int i = 0; i < num_joints_; i++) {
      ommp.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
  } */
}
void OmmpHardwareInterface::enc_feedback_to_joint_pos_vel(ros::Duration elapsed_time) {
  for (int i = 0; i < 4; i++) {
    joint_feedback_vel[i] = float((curr_enc[i] - prev_enc[i])) / elapsed_time.toSec();
    joint_feedback_vel[i] *= ENC_TO_VEL_FACTOR;
    joint_feedback_pos[i] = curr_enc[i];
    joint_feedback_pos[i] *= ENC_TO_POS_FACTOR;
  }
}

void OmmpHardwareInterface::cmd_to_setpoint() {
  // On the same side they recieve the same command
  vel_set_cmd[0] =
      joint_velocity_command_[0] * CMD_TO_SET_FACTOR; // back left -> = left
  vel_set_cmd[1] =
      joint_velocity_command_[1] * CMD_TO_SET_FACTOR; // back right -> = right
}

void OmmpHardwareInterface::enc_ticks_CB(
    const std_msgs::Int64MultiArray::ConstPtr &enc_msg) {
  curr_enc[0] = enc_msg->data[2];
  curr_enc[1] = enc_msg->data[3];
  curr_enc[2] = enc_msg->data[0];
  curr_enc[3] = enc_msg->data[1];
}

} // namespace ommp_hardware_interface