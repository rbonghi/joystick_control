/* 
 * File:   JoyControl.cpp
 * Author: raffaello
 * 
 * Created on 09 October 2013, 11:55
 */

#include <string>
#include <map>

#include "JoyControl.h"

JoyControl::JoyControl(const ros::NodeHandle& nh) : nh_(nh)
{
  counter_map_ = 0;
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_string, 10, &JoyControl::joyCallback, this);
}

template <typename T>
std::string JoyControl::NumberToString(T Number)
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

JoyControl::JoyControl(const JoyControl& orig)
{
}

JoyControl::~JoyControl()
{
}

void JoyControl::addButton(std::string name, ButtonCommand command)
{
  //button association
  if (nh_.hasParam("/" + joy_string + "/" + name))
  {
    int number_button;
    nh_.getParam("/" + joy_string + "/" + name, number_button);
    //Add function in map
    ROS_INFO("Load button");
    button_map_.insert(std::pair<std::string, int>(name, number_button));
    func_button_map_.insert(std::pair<std::string, ButtonCommand>(name, command));
  }
  else
  {
    ROS_ERROR("Never param Button %s", name.c_str());
  }
}

void JoyControl::resetAxis(std::string name)
{
  std::map<std::string, int>::iterator it = step_axis_map_.find(name);
  if (it != step_axis_map_.end())
  {
//    ROS_INFO("Step Name: %s - number: %d", step_axis_map_.find(name)->first.c_str(), step_axis_map_.find(name)->second);
    int number = step_axis_map_.find(name)->second;
    axis_step_[number].x = 0;
    axis_step_[number].y = 0;
    axis_old_[number].x = 0;
    axis_old_[number].y = 0;
  }
  it = axis_map_.find(name);
  if (it != axis_map_.end())
  {
//    ROS_INFO("Name: %s - number: %d", axis_map_.find(name)->first.c_str(), axis_map_.find(name)->second);
    int number = axis_map_.find(name)->second;
    axis_old_[number].x = 0;
    axis_old_[number].y = 0;
  }
}

void JoyControl::addAxis(std::string name, AxisCommand command)
{
  if (nh_.hasParam("/" + joy_string + "/" + name))
  {
    double temp;
    if (nh_.hasParam("/" + joy_string + "/" + name + "/step"))
    {
      step_axis_map_.insert(std::pair<std::string, int>(name, counter_map_));
      nh_.getParam("/" + joy_string + "/" + name + "/step/x", temp);
      scale_[counter_map_].x = temp;
      nh_.getParam("/" + joy_string + "/" + name + "/step/y", temp);
      scale_[counter_map_].y = temp;
      nh_.getParam("/" + joy_string + "/" + name + "/scale/x", temp);
      step_[counter_map_].x = temp;
      nh_.getParam("/" + joy_string + "/" + name + "/scale/y", temp);
      step_[counter_map_].y = temp;
    }
    else
    {
      axis_map_.insert(std::pair<std::string, int>(name, counter_map_));
      nh_.getParam("/" + joy_string + "/" + name + "/scale/x", temp);
      scale_[counter_map_].x = temp;
      nh_.getParam("/" + joy_string + "/" + name + "/scale/y", temp);
      scale_[counter_map_].y = temp;
    }
    nh_.getParam("/" + joy_string + "/" + name + "/axis/x", temp);
    axis_number_[counter_map_].x = temp;
    nh_.getParam("/" + joy_string + "/" + name + "/axis/y", temp);
    axis_number_[counter_map_].y = temp;

    //Add function in map
    function_map_.insert(std::pair<std::string, AxisCommand>(name, command));
  }
  else
  {
    ROS_ERROR("Never param Joystick %s", name.c_str());
  }
  counter_map_++;
}

bool JoyControl::button_control(const sensor_msgs::Joy::ConstPtr& joy, int number)
{
  if (joy->buttons[number] == 1 && buttons_[number] == false)
  {
    buttons_[number] = true;
    //    ROS_INFO("button pressed: %d", number + 1);
    return true;
  }
  else if (joy->buttons[number] == 0 && buttons_[number] == true)
  {
    buttons_[number] = false;
    return false;
  }
  return false;
}

bool JoyControl::axis_control(const sensor_msgs::Joy::ConstPtr& joy, int n_axis)
{
  axis_[n_axis].x = scale_[n_axis].x * joy->axes[((int) axis_number_[n_axis].x)];
  axis_[n_axis].y = scale_[n_axis].y * joy->axes[((int) axis_number_[n_axis].y)];
  if ((axis_old_[n_axis].x != axis_[n_axis].x) || (axis_old_[n_axis].y != axis_[n_axis].y))
  {
    axis_old_[n_axis] = axis_[n_axis];
    //    ROS_INFO("Command [%f, %f]", axis_[n_axis].x, axis_[n_axis].y);
    return true;
  }
  return false;
}

void JoyControl::axis(const sensor_msgs::Joy::ConstPtr& joy, std::string name)
{
  int number = step_axis_map_.find(name)->second;
  if (axis_control(joy, number))
  {
    AxisCommand function = function_map_.find(name)->second;
    function(axis_[number].x, axis_[number].y);
  }
}

void JoyControl::stepAxis(const sensor_msgs::Joy::ConstPtr& joy, std::string name)
{
  int number = step_axis_map_.find(name)->second;
  if (axis_control(joy, number))
  {
    if ((axis_[number].x != 0) && (abs(axis_step_[number].x + axis_[number].x) <= step_[number].x))
    {
      axis_step_[number].x += axis_[number].x;
    }
    if ((axis_[number].y != 0) && (abs(axis_step_[number].y + axis_[number].y) <= step_[number].y))
    {
      axis_step_[number].y += axis_[number].y;
    }
    AxisCommand function = function_map_.find(name)->second;
    function(axis_step_[number].x, axis_step_[number].y);
  }
}

void JoyControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  for (std::map<std::string, int>::iterator list_iter = step_axis_map_.begin(); list_iter != step_axis_map_.end(); list_iter++)
  {
    //    ROS_INFO("Step axis name: %s", list_iter->first.c_str());
    stepAxis(joy, list_iter->first);
  }
  for (std::map<std::string, int>::iterator list_iter = axis_map_.begin(); list_iter != axis_map_.end(); list_iter++)
  {
    //    ROS_INFO("Axis name: %s", list_iter->first.c_str());
    axis(joy, list_iter->first);
  }
  for (std::map<std::string, int>::iterator list_iter = button_map_.begin(); list_iter != button_map_.end(); list_iter++)
  {
    if (button_control(joy, list_iter->second))
    {
      ButtonCommand function = func_button_map_.find(list_iter->first)->second;
      function();
    }
  }
}