#pragma once
#include <ros/ros.h>
#include <mobile_base_msgs/msg/VehicleCmd.hpp>
#include <mobile_base_msgs/msg/VehicleState.hpp>
#include <mobile_base_msgs/srv/Calibration.hpp>
#include <std_msgs/msg/Float64MultiArray.hpp>
#include <geometry_msgs/msg/Twist.hpp>
#include <nav_msgs/msg/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/msg/Joy.hpp>
#include "mobile_base_msgs/msg/WheelDirection.hpp"
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
    bool calibrationCallback(const std::shared_ptr<mobile_base_msgs::srv::Calibration::Request> req, 
                             std::shared_ptr<mobile_base_msgs::srv::Calibration::Response> res);
    void vehicleCmdCallback(const mobile_base_msgs::msg::VehicleCmd::SharedPtr cmd);
    void joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void wheelStateCallback(const wheel_controller::msg::WheelDirection::SharedPtr state);
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