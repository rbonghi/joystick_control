/* 
 * File:   mouse3D_node.cpp
 * Author: raffaello
 *
 * Created on 10 September 2013, 16:22
 */

#include <cstdlib>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "JoyControl.h"

#include <geometry_msgs/Twist.h>
#include <serial_bridge/Enable.h>
#include <serial_bridge/Pose.h>

using namespace std;
ros::Publisher pub_enable, pub_velocity, pub_pose;
ros::Subscriber enable_sub;
bool enable = false;

std::string enable_string_convert(bool enable)
{
  return ((enable) ? "ON" : "OFF");
}

void steering(float x_step, float y_step)
{
  ROS_INFO("[linear: %f - angular: %f]", x_step, y_step);
  geometry_msgs::Twist velocity;
  velocity.linear.x = x_step;
  velocity.angular.z = y_step;
  pub_velocity.publish(velocity);
}

void enableCallback(const serial_bridge::Enable::ConstPtr& enable_msg)
{
  enable = enable_msg.get()->enable;
  ROS_INFO("Enable now: %s", enable_string_convert(enable).c_str());
  enable_sub.shutdown();
}

void enableButton()
{
  serial_bridge::Enable enable_msg;
  enable = enable ? false : true;
  enable_msg.enable = enable;
  ROS_INFO("Enable now: %s", enable_string_convert(enable_msg.enable).c_str());
  pub_enable.publish(enable_msg);
}

void resetPose()
{
  serial_bridge::Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;
  pub_pose.publish(pose);
}

/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop");
  ros::NodeHandle nh;

  pub_velocity = nh.advertise<geometry_msgs::Twist>("/robot/command/velocity", 1);
  pub_enable = nh.advertise<serial_bridge::Enable>("/robot/command/enable_motors", 1);
  pub_pose = nh.advertise<serial_bridge::Pose>("/robot/command/pose", 1);
  //enable_sub = nh.subscribe<serial_bridge::Enable>("/robot/enable_motors", 10, enableCallback);
  
  JoyControl* space = new JoyControl(nh);

  space->addAxis("steering", steering);
  //space->addButton("enable_motors", enableButton);
  space->addButton("reset_pose", resetPose);

  ROS_INFO("Ready!");
  ros::spin();

  return 0;
}

