/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <WProgram.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ros.h>
#include <std_msgs/UInt16.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;
int rightMotor = 6;      // LED connected to digital pin 9
int leftMotor = 5;      // LED connected to digital pin 9
int rightReverse = 4;      // LED connected to digital pin 9
int leftReverse = 8;      // LED connected to digital pin 9
int rightProx = 2;
int leftProx = 3;
volatile long r_count = 0;
volatile long l_count = 0;
volatile int count_flag = 0;

long r_count_old = 0;
long l_count_old = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long r_vel;
long l_vel;

/*void throttle_cb( const std_msgs::UInt16& cmd_msg){
  analogWrite(leftMotor, cmd_msg.data);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  analogWrite(rightMotor, cmd_msg.data);
}*/

void throttle_cb(const std_msgs::Int16MultiArray& array)
{
 
  digitalWrite(leftReverse, array.data[0]);
  digitalWrite(rightReverse, array.data[2]);
  analogWrite(leftMotor, array.data[1]);
  analogWrite(rightMotor, array.data[3]);
   
  return;
}


//ros::Subscriber<std_msgs::UInt16> sub("throttle", throttle_cb);
ros::Subscriber<std_msgs::Int16MultiArray> sub("throttle", throttle_cb);
std_msgs::UInt16 l_msg;
std_msgs::UInt16 r_msg;
std_msgs::UInt16 vl_msg;
std_msgs::UInt16 vr_msg;
ros::Publisher publ("l_count", &l_msg);
ros::Publisher pubr("r_count", &r_msg);
ros::Publisher pubvl("l_speed", &vl_msg);
ros::Publisher pubvr("r_speed", &vr_msg);

void setup(){
  pinMode(rightMotor, OUTPUT);   // sets the pin as output
  pinMode(leftMotor, OUTPUT);   // sets the pin as output
  pinMode(rightReverse, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightProx, INPUT);
  pinMode(leftProx, INPUT);
  attachInterrupt(1, r_isr, CHANGE);
  attachInterrupt(0, l_isr, CHANGE);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(publ);
  nh.advertise(pubr);
  nh.advertise(pubvl);
  nh.advertise(pubvr);
}

void loop(){
  if (count_flag == 1)
    {
      count_flag = 0;
      newtime = millis();
      l_vel = (l_count-l_count_old) * 1000 /(newtime-oldtime);
      r_vel = (r_count-r_count_old) * 1000 /(newtime-oldtime);
      l_msg.data = l_count;
      r_msg.data = r_count;
      vl_msg.data = l_vel;
      vr_msg.data = r_vel;
      publ.publish(&l_msg);
      pubr.publish(&r_msg);
      pubvl.publish(&vl_msg);
      pubvr.publish(&vr_msg);
      oldtime = newtime;
      r_count_old = r_count;
      l_count_old = l_count;
    }
  nh.spinOnce();
  delay(1);
}
 
void r_isr()
{
  r_count = r_count + 1;
  count_flag = 1;
}

void l_isr()
{
  l_count = l_count + 1;
  count_flag = 1;
}
