
#ifndef ROS_CONTROL__ommp_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ommp_HARDWARE_INTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ommp_hardware_interface/ommp_hardware.h>
#include <ros/ros.h>

// Publish Subscribe Includes
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

namespace ommp_hardware_interface {
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

// Tranform Factors From encoder to joint_state Universe
static const double ENC_TO_VEL_FACTOR = 0.000679;
static const double ENC_TO_POS_FACTOR = 0.000679;
//cmd msg 0.25 -> 4.5454 -> x = 0.055
static const double CMD_TO_SET_FACTOR = 0.055;
class OmmpHardwareInterface : public ommp_hardware_interface::OmmpHardware {
public:
  OmmpHardwareInterface(ros::NodeHandle &nh);
  ~OmmpHardwareInterface();
  void init();
  void update(const ros::TimerEvent &e);
  void read(ros::Duration elapsed_time);
  void write(ros::Duration elapsed_time);

  // Subscriber wheel encoder Callback
  void enc_ticks_CB(const std_msgs::Int64MultiArray::ConstPtr &enc_msg);
  void cmd_to_setpoint();
  void enc_feedback_to_joint_pos_vel(ros::Duration elapsed_time);

protected:
  // omppcpp::ommp ommp;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  VelocityJointInterface velocityJointInterface;
  VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;

  // Publish and Subscriber to from Arduino
  ros::Publisher wheel_vel_pub;
  ros::Subscriber wheel_enc_sub;
  std_msgs::Float32MultiArray vel_setpoint_cmd_array;
  // To hold the encoder ticks and the previous encoder ticks
  // Used to calculate speed
  int curr_enc[4] = {0, 0, 0, 0};
  int prev_enc[4] = {0, 0, 0, 0};
  double joint_feedback_vel[4] = {0., 0., 0., 0.};
  double joint_feedback_pos[4] = {0., 0., 0., 0.};
  double vel_set_cmd[2] = {0., 0.};
};

} // namespace ommp_hardware_interface

#endif