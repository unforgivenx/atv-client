#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

#include <sstream>
ros::Publisher speed_pub, brake_pub, steer_pub;
ros::Time begin;
float st_angle=0;
void stCallback(const std_msgs::Float32::ConstPtr& angle)
{
	st_angle=angle->data;
	std_msgs::Int16 steering_ang;
	steering_ang.data = (180-st_angle)*100/180.0; 
	steer_pub.publish(steering_ang);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ros::Duration difft = ros::Time::now() - begin;
	if (difft.toSec() > 0.00001)
	{
		std_msgs::Int16 brake;
		std_msgs::Int16 steering;
		//std_msgs::UInt16 throttle;
		std_msgs::Int16MultiArray throttle;
		int th0, th1, th2, th3;
		if (joy->buttons[0] <= 0)
		{
			th0 = 0;
			th2 = 0;
		}
		else
		{
			th0 = 1;
			th2 = 1;
		}

		if (joy->axes[2] > -0.5)
		{
			  //brake.data = (2-joy->axes[1])*100-200;
			brake.data = (joy->axes[2]+1)*50;
			//throttle.data.push_back(0);// = 0;
			//throttle.data.push_back(0);
			//throttle.data.push_back(0);
			//throttle.data.push_back(0);
			th1 = 0;
			th3 = 0;
		}
		else
		{
			brake.data = 0;
			//throttle.data = (joy->axes[1])*100;
			th1 = (joy->axes[1]+1)*64;//*((joy->axes[3]+1)/2);
			th3 = (joy->axes[1]+1)*64;//*((joy->axes[3]+1)/2);
			if (th1 > 100) th1 = 100;
			if (th3 > 100) th3 = 100;
		}
		throttle.data.push_back(th0);
		throttle.data.push_back(th1);
		throttle.data.push_back(th2);
		throttle.data.push_back(th3);
		  
		ROS_INFO("brake: %d, steering: %d, throttle: %d", brake.data, steering.data, throttle.data[0]);
		speed_pub.publish(throttle);
		brake_pub.publish(brake);
		//steering.data = (180-st_angle)*100/180.0; // for obstacle avoidance
		steering.data = (-1*joy->axes[0]+1)*50; //for joystick
		steer_pub.publish(steering);
		begin = ros::Time::now();
	}
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "teleop_joy");


  ros::NodeHandle n;


  //speed_pub = n.advertise<std_msgs::UInt16>("throttle", 1000);
speed_pub = n.advertise<std_msgs::Int16MultiArray>("throttle", 1000);
brake_pub = n.advertise<std_msgs::Int16>("brake_perc", 1000);
steer_pub = n.advertise<std_msgs::Int16>("steering_perc", 1000);
ros::Subscriber sub_st = n.subscribe<std_msgs::Float32>("steering_angle", 1000, stCallback);
ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

