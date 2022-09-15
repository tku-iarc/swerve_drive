//============================================================================
// Name        : vehicle_controller.h
// Author      : Andy Chien
// Version     : 1.0.0
// Created on  : 08.08.2022
// Copyright   : MIT
// Description : Controller for swerve drive mobile base
//============================================================================

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "mobile_base_msgs/msg/wheel_direction.hpp"
#include "mobile_base_msgs/msg/vehicle_state.hpp"
#include "mobile_base_msgs/srv/calibration.hpp"
#include "mobile_base_msgs/msg/vehicle_cmd.hpp"
#include "wheel_controller/wheel_controller.h"
#include "vehicle_controller/kinematics.h"

namespace vehicle_controller
{
class VehicleController : public rclcpp::Node
{
public:
    VehicleController(std::string& node_name);
    ~VehicleController();

    template <typename ParameterT>
    auto auto_declare(const std::string & name, const ParameterT & default_value)
    {
        if (!this->has_parameter(name))
            return this->declare_parameter<ParameterT>(name, default_value);
        else
            return this->get_parameter(name).get_value<ParameterT>();
    }

private:
    bool calibrationCallback(const std::shared_ptr<mobile_base_msgs::srv::Calibration::Request> req, 
                             std::shared_ptr<mobile_base_msgs::srv::Calibration::Response> res);
    void vehicleCmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);
    void joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr state);
    // void wheelStateCallback(const mobile_base_msgs::msg::WheelDirection::SharedPtr state);
    void initKinematicsData(std::vector<std::string>& wheels_name);
    void ensureCmdLimit();
    void vehicleOdometer(double cycle_time);
    void vehicleStatePublish();
    void sendCmd();

    std::string prefix_;
    bool sim_;
    bool en_joy_;
    int wheel_numbers;
    double dir_acc_max_;
    double ang_acc_max_;
    double last_odomap_update_time_secs_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<mobile_base_msgs::msg::VehicleState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr swerve_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Service<mobile_base_msgs::srv::Calibration>::SharedPtr calib_server_;

    std::vector<std::string> wheels_name_;
    std::map<std::string, std::vector<double>> joint_states_;
    std::map<std::string, wheel_controller::WheelController*> wheel_controllers_;
    std::map<std::string, mobile_base_msgs::msg::WheelDirection::SharedPtr> wheels_direction_cmd_;
    KinematicsData kinematics_data_;
    VehicleKinematics kinematics_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
};
}