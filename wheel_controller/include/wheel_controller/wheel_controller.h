#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <controller_manager/controller_manager.h>

#include "wheel_controller/hardware_interface.h"
#include "wheel_controller/joint_data.h"


#define DOF 2

enum WheelState {Idle, Busy, Error, Disable};

class WheelController
{
private:
    void jointDataInit();
    /* data */
    WheelState wheel_state;
    hardware_interface::SwerveDriveInterface* swerve_drive_interface;
    controller_manager::ControllerManager* wheel_cm;

    ros::NodeHandle& nodeHandle_;

public:
    WheelController(ros::NodeHandle& nodeHandle);
    ~WheelController();
    void closeDevice();
    void process(ros::Rate& loop_rate);
    std::vector<JointData*> joint_data;
    float sample_rate;
    std::string control_mode;
};
