/* 
 * File:   Servo_node.cpp
 * Author: raffaello
 *
 * Created on 10 October 2013, 16:08
 */

#include <cstdlib>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "JoyControl.h"

#include <sensor_msgs/JointState.h>

//#include <maestro_node/Servo.h>

using namespace std;
JoyControl* space;
float angle = 0, step = 0.1;
float angle_tilt = 0;
int pan, tilt, track_left, track_right;
float servo_pan_old, servo_tilt_old;
ros::Publisher pub_servo;
sensor_msgs::JointState joint;

void decrease_angle_tilt()
{
  if (angle_tilt + step < 1.6)
  {
    angle_tilt += step;
    joint.position[2] = angle_tilt;
    pub_servo.publish(joint);
  }
  ROS_INFO("TILT decrease %f", angle_tilt);
}

void increase_angle_tilt()
{
  if (angle_tilt - step > -1.6)
  {
    angle_tilt -= step;
    joint.position[2] = angle_tilt;
    pub_servo.publish(joint);
  }
  ROS_INFO("TILT increase %f", angle_tilt);

}

void decrease_angle_shovels()
{
  if (angle + step < 1.6)
  {
    angle += step;
    joint.position[0] = angle;
    joint.position[1] = -angle;
    pub_servo.publish(joint);
  }
  ROS_INFO("decrease %f", angle);
}

void increase_angle_shovels()
{
  if (angle - step > -1.6)
  {
    angle -= step;
    joint.position[0] = angle;
    joint.position[1] = -angle;
    pub_servo.publish(joint);
  }
  ROS_INFO("increase %f", angle);

}

void full_open_shovels()
{
  angle = -1.5;
  joint.position[0] = angle;
  joint.position[1] = -angle;
  pub_servo.publish(joint);
	ROS_INFO("FULL OPEN");
}

void open_shovels()
{
  angle = -0.5;
  joint.position[0] = angle;
  joint.position[1] = -angle;
  pub_servo.publish(joint);
	ROS_INFO("OPEN ---");
}

void close_shovels()
{
  angle = 1.5;
  joint.position[0] = angle;
  joint.position[1] = -angle;
  pub_servo.publish(joint);
	ROS_INFO("CLOSE");
}

/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_teleop");
  ros::NodeHandle nh;
  
  joint.header.frame_id = "joint_states";
  joint.name.resize(3);
  joint.name[0] = "Left_shovel";
  joint.name[1] = "Right_shovel";
  joint.name[2] = "Tilt";
  joint.position.resize(3);
  joint.velocity.resize(3);
  joint.effort.resize(3);
  pub_servo = nh.advertise<sensor_msgs::JointState>("/maestro_node/command/joint_states", 1);

  space = new JoyControl(nh);
  space->addButton("increase_angle_tilt", increase_angle_tilt);
  space->addButton("decrease_angle_tilt", decrease_angle_tilt);
  space->addButton("increase_angle_shovels", increase_angle_shovels);
  space->addButton("decrease_angle_shovels", decrease_angle_shovels);
  space->addButton("full_open_shovels", full_open_shovels);
  space->addButton("open_shovels", open_shovels);
  space->addButton("close_shovels", close_shovels);

  ROS_INFO("Ready!");
  ros::spin();

  return 0;
}

