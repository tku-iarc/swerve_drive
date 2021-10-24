#pragma once
#include <ros/ros.h>
#include <vehicle_controller/VehicleCmd.h>
#include <vehicle_controller/VehicleState.h>
#include <vehicle_controller/Calibration.h>
#include <std_msgs/Float64MultiArray.h>
#include <wheel_controller/WheelState.h>
#include "vehicle_controller/kinematics.h"

namespace vehicle_controller
{
class VehicleController
{
public:
    VehicleController(ros::NodeHandle& nodeHandle);
    ~VehicleController();
    void process();
private:
    bool calibrationCallback(vehicle_controller::Calibration::Request &req, 
                             vehicle_controller::Calibration::Response &res);
    void vehicleCmdCallback(const vehicle_controller::VehicleCmd::ConstPtr& cmd);
    void wheelStateCallback(const wheel_controller::WheelState::ConstPtr& state);
    void initKinematicsData();
    ros::NodeHandle& nodeHandle_;
    int wheel_numbers;
    float dir_acc_max_;
    float ang_acc_max_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    ros::ServiceServer calib_server_;
    std::vector<std::string> wheels_name_;
    std::map<std::string, ros::Subscriber> wheels_sub_;
    std::map<std::string, ros::Publisher> wheels_pub_;
    KinematicsData kinematics_data_;
};
}