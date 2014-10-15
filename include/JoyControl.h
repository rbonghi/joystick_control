/* 
 * File:   JoyControl.h
 * Author: raffaello
 *
 * Created on 09 October 2013, 11:55
 */

#ifndef JOYCONTROL_H
#define	JOYCONTROL_H

#include <ros/ros.h>
#include <map>

#include <sensor_msgs/Joy.h>
#include <joystick_control/Axis.h>

const std::string joy_string = "joy";
const std::string robot_string = "robot";
const std::string velocity_string = "velocity";
const std::string enable_string = "enable_motors";
const std::string pose_string = "pose";
const std::string command_string = "command";

const std::string maestro_string = "maestro_node";
const std::string servos_string = "servos";

class JoyControl {
public:
    typedef void(* AxisCommand) (float x_step, float y_step);
    typedef void(* ButtonCommand) ();
    JoyControl(const ros::NodeHandle& nh);
    JoyControl(const JoyControl& orig);
    virtual ~JoyControl();

    void resetAxis(std::string name);
    void addAxis(std::string name, AxisCommand command);
    void addButton(std::string name, ButtonCommand command);
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    bool buttons_[20];
    
    std::map<std::string, int> step_axis_map_, axis_map_, button_map_;
    std::map<std::string, AxisCommand> function_map_;
    std::map<std::string, ButtonCommand> func_button_map_;
    int counter_map_;
    joystick_control::Axis axis_step_[10];
    joystick_control::Axis axis_old_[10];
    joystick_control::Axis axis_[10];
    joystick_control::Axis scale_[10];
    joystick_control::Axis step_[10];
    joystick_control::Axis axis_number_[10];
    
    void axis(const sensor_msgs::Joy::ConstPtr& joy, std::string name);
    void stepAxis(const sensor_msgs::Joy::ConstPtr& joy, std::string name);
    bool axis_control(const sensor_msgs::Joy::ConstPtr& joy, int n_axis);
    
    template <typename T> std::string NumberToString(T Number);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool button_control(const sensor_msgs::Joy::ConstPtr& joy, int number);
};

#endif	/* JOYCONTROL_H */

