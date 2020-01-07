#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#include <math.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"

class arduinoHandler
{
public:
	arduinoHandler()
	{
		js_sub = nh.subscribe("/joint_states",10, &arduinoHandler::jsCB,this);
		ard_pub =  nh.advertise<std_msgs::Int16MultiArray>("servo_cmd", 10);
	}
	int transform(float x, float y)
	{
   		return int((x+1.57)*450.0/3.14 + y);
	}

	void jsCB(const sensor_msgs::JointState::ConstPtr& joint_state)
	{
		std_msgs::Int16MultiArray array;
    	array.data.clear();
    	float curr_0 = joint_state->position[4];
    	float curr_1 = joint_state->position[5];
    	float curr_2 = joint_state->position[6];
    	float curr_3 = joint_state->position[7];
    	float curr_4 = joint_state->position[8];
    	float curr_5 = joint_state->position[9];
    	bool cond_0 = fabs(prev_0 - curr_0) < 0.001;
    	bool cond_1 = fabs(prev_1 - curr_1) < 0.001;
    	bool cond_2 = fabs(prev_2 - curr_2) < 0.001;
    	bool cond_3 = fabs(prev_3 - curr_3) < 0.001;
    	bool cond_4 = fabs(prev_4 - curr_4) < 0.001;
    	bool cond_5 = fabs(prev_5 - curr_5) < 0.001;
    	if(cond_0 && cond_1 && cond_2 && cond_3 && cond_4 && cond_5){return;}

    	int servo_0 = this->transform(curr_0,-35);
    	int servo_1 = this->transform(curr_1,-40);
    	int helper = (450 - servo_1 - 15);
    	//ROS_INFO("helper is %d ", helper);
    	int servo_2 = helper + 1.1*(450 - this->transform(curr_2,0));
    	int servo_3 = 450 - this->transform(curr_3,0);
    	int servo_4 = this->transform(curr_4,-35);
    	int servo_5 = 450 - this->transform(curr_5,+15);
    
    	if (fabs(servo_2 - helper) > 85){
        	if(servo_2 > helper){servo_2 = helper + 85;}
        	else {servo_2 = helper - 85;}
    	}
    	if (servo_0 > 440){servo_0 = 440;}
    	if (servo_0 < 0){servo_0 = 0;}
    	if (servo_1 > 440){servo_1 = 440;}
    	if (servo_1 < 40){servo_1 = 40;}
    	if (servo_2 > 400){servo_2 = 400;}
    	if (servo_2 < 0){servo_2 = 0;}
    	if (servo_3 > 440){servo_3 = 440;}
    	if (servo_3 < 0){servo_3 = 0;}
    	if (servo_4 > 440){servo_4 = 440;}
    	if (servo_4 < 0){servo_4 = 0;}
    	if (servo_5 > 440){servo_5 = 440;}
    	if (servo_5 < 0){servo_5 = 0;}
    	array.data.push_back(servo_0);
    	array.data.push_back(servo_1);
    	array.data.push_back(servo_2);
    	array.data.push_back(servo_3);
    	array.data.push_back(servo_4);
    	array.data.push_back(servo_5);
    	ard_pub.publish(array);
    	prev_0 = curr_0;
		prev_1 = curr_1;
    	prev_2 = curr_2;
    	prev_3 = curr_3;
    	prev_4 = curr_4;
    	prev_5 = curr_5;
	}
protected:
	ros::NodeHandle nh;
	ros::Subscriber js_sub;
	ros::Publisher ard_pub;
	float prev_0=0,prev_1=0,prev_2=0,prev_3=0,prev_4=0,prev_5=0;
};



int main(int argc, char** argv){
    ros::init(argc, argv, "driver_to_arduino");
    arduinoHandler handler;
    ros::spin();
    return 0;
}
