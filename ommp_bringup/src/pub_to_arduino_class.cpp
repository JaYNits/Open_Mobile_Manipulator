#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <math.h>

#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
class arduinoHandler {
public:
  arduinoHandler() {
    js_sub = nh.subscribe("/joint_states", 10, &arduinoHandler::jsCB, this);
    arm_pub = nh.advertise<std_msgs::Int16MultiArray>("servo_cmd", 10);
  }
  int transform(float x, float y) { return int((x + 1.57) * 450.0 / 3.14 + y); }
  int transform_gripper(float x) {
    // joint_state goes from 0 to 0.02 we map 0 to upper and 0.02 to lower
    float open = 230.0;
    float close = 380.0;
    float scaling_value = (close - open) / 0.02;
    int gripper_ard_cmd = int(open + scaling_value * x);
    return gripper_ard_cmd;
  }
  void jsCB(const sensor_msgs::JointState::ConstPtr &joint_state) {
    std_msgs::Int16MultiArray arm_array;

    arm_array.data.clear();
    float curr_0 = joint_state->position[1];
    float curr_1 = joint_state->position[2];
    float curr_2 = joint_state->position[3];
    float curr_3 = joint_state->position[4];
    float curr_4 = joint_state->position[5];
    float curr_5 = joint_state->position[6];
    float curr_6 = joint_state->position[7];
    bool cond_0 = fabs(prev_0 - curr_0) < 0.001;
    bool cond_1 = fabs(prev_1 - curr_1) < 0.001;
    bool cond_2 = fabs(prev_2 - curr_2) < 0.001;
    bool cond_3 = fabs(prev_3 - curr_3) < 0.001;
    bool cond_4 = fabs(prev_4 - curr_4) < 0.001;
    bool cond_5 = fabs(prev_5 - curr_5) < 0.001;
    bool cond_6 = fabs(prev_6 - curr_6) < 0.001;
    if (cond_0 && cond_1 && cond_2 && cond_3 && cond_4 && cond_5 && cond_6) {
      return;
    }

    int servo_0 = this->transform(curr_0, -35);
    int servo_1 = this->transform(curr_1, -40);
    int helper = (450 - servo_1 - 15);
    // ROS_INFO("helper is %d ", helper);
    int servo_2 = helper + 1.1 * (450 - this->transform(curr_2, 0));
    int servo_3 = this->transform(curr_3, 0);
    int servo_4 = this->transform(curr_4, -35);
    int servo_5 = 450 - this->transform(curr_5, +15);
    int servo_6 = this->transform_gripper(curr_6);

    if (fabs(servo_2 - helper) > 85) {
      if (servo_2 > helper) {
        servo_2 = helper + 85;
      } else {
        servo_2 = helper - 85;
      }
    }
    if (servo_0 > 440) {
      servo_0 = 440;
    }
    if (servo_0 < 0) {
      servo_0 = 0;
    }
    if (servo_1 > 440) {
      servo_1 = 440;
    }
    if (servo_1 < 40) {
      servo_1 = 40;
    }
    if (servo_2 > 400) {
      servo_2 = 400;
    }
    if (servo_2 < 0) {
      servo_2 = 0;
    }
    if (servo_3 > 440) {
      servo_3 = 440;
    }
    if (servo_3 < 0) {
      servo_3 = 0;
    }
    if (servo_4 > 440) {
      servo_4 = 440;
    }
    if (servo_4 < 0) {
      servo_4 = 0;
    }
    if (servo_5 > 440) {
      servo_5 = 440;
    }
    if (servo_5 < 0) {
      servo_5 = 0;
    }
    if (servo_6 > 380) {
      servo_6 = 380;
    }
    if (servo_6 < 230) {
      servo_6 = 230;
    }

    arm_array.data.push_back(servo_0);
    arm_array.data.push_back(servo_1);
    arm_array.data.push_back(servo_2);
    arm_array.data.push_back(servo_3);
    arm_array.data.push_back(servo_4);
    arm_array.data.push_back(servo_5);
    arm_array.data.push_back(servo_6);
    arm_pub.publish(arm_array);
    prev_0 = curr_0;
    prev_1 = curr_1;
    prev_2 = curr_2;
    prev_3 = curr_3;
    prev_4 = curr_4;
    prev_5 = curr_5;
    prev_6 = curr_6;
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber js_sub;
  ros::Publisher arm_pub;
  float prev_0 = 0, prev_1 = 0, prev_2 = 0, prev_3 = 0, prev_4 = 0, prev_5 = 0,
        prev_6 = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "driver_to_arduino");
  arduinoHandler handler;
  ros::spin();
  return 0;
}
