#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <std_msgs/msg/Float64.hpp>
// #include <std_msgs/msg/Float64MultiArray.hpp>
// #include <sensor_msgs/msg/JointState.hpp>
// #include <controller_manager/controller_manager.h>
// #include "wheel_controller/hardware_interface.h"
#include "mobile_base_msgs/msg/wheel_direction.hpp"
#include "wheel_controller/joint_data.h"


#define DOF 2
#define WHEEL_DIAMETER 0.15
#define WHEEL_RADIUS 0.075

namespace wheel_controller
{
class WheelController : public std::enable_shared_from_this<WheelController>
{
private:
    void jointDataInit(std::vector<int> motors_id);
    /* data */
    // hardware_interface::SwerveDriveInterface* swerve_drive_interface;
    // controller_manager::ControllerManager* wheel_cm;
    double metersToRads(const double &meters);
    double radsTometers(const double &rads);

    // ros::Subscriber wheel_cmd_sub_;
    // ros::Subscriber joint_state_sub_;
    // ros::Publisher  wheel_state_pub_;
    // ros::Publisher  swerve_joint_pub_;
    // ros::Publisher  wheel_joint_pub_;
    // std::string wheel_name_;
    bool sim_;

public:
    WheelController(bool sim, std::vector<int> motors_id);
    ~WheelController();
    void updateJointData(const std::vector<std::vector<double>>& joint_state,
                         mobile_base_msgs::msg::WheelDirection::SharedPtr wheel_state);
    void getWheelCmd(const mobile_base_msgs::msg::WheelDirection::SharedPtr msg, std::vector<double>& cmds);
    // void process(ros::Rate& loop_rate);
    std::vector<JointData*> joint_data;
    // float sample_rate;
    // std::string control_mode;
    std::shared_ptr<WheelController> SharedPtr(){return shared_from_this();};
};
}
