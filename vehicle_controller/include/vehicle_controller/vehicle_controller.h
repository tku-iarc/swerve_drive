#pragma once
#include <ros/ros.h>
#include <vehicle_controller/VehicleCmd.h>
#include <vehicle_controller/VehicleState.h>
#include <vehicle_controller/Calibration.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <wheel_controller/WheelDirection.h>
#include "vehicle_controller/kinematics.h"

namespace vehicle_controller
{
class VehicleController
{
public:
    VehicleController(ros::NodeHandle& nodeHandle);
    ~VehicleController();
    void process(ros::Rate& loop_rate);
private:
    bool calibrationCallback(vehicle_controller::Calibration::Request &req, 
                             vehicle_controller::Calibration::Response &res);
    void vehicleCmdCallback(const vehicle_controller::VehicleCmd::ConstPtr& cmd);
    void joysticMsgCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void wheelStateCallback(const wheel_controller::WheelDirection::ConstPtr& state);
    void initKinematicsData();
    void ensureCmdLimit();
    void vehicleOdometer(ros::Rate& loop_rate);
    void vehicleStatePublish();
    ros::NodeHandle& nodeHandle_;
    int wheel_numbers;
    float dir_acc_max_;
    float ang_acc_max_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber joy_sub;
    ros::Publisher state_pub_;
    ros::Publisher odom_pub_;
    ros::ServiceServer calib_server_;
    tf::TransformBroadcaster odom_broadcaster_;
    std::vector<std::string> wheels_name_;
    std::map<std::string, ros::Subscriber> wheels_sub_;
    std::map<std::string, ros::Publisher> wheels_pub_;
    KinematicsData kinematics_data_;
    VehicleKinematics kinematics;
};
}