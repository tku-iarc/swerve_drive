#include "vehicle_controller/vehicle_controller.h"

namespace vehicle_controller
{
VehicleController::VehicleController(ros::NodeHandle& nodeHandle)
:nodeHandle_(nodeHandle)
{
    nodeHandle_.param<float>("dir_acc_max", dir_acc_max_, 1);
    nodeHandle_.param<float>("ang_acc_max", ang_acc_max_, 1);
    cmd_sub_ = nodeHandle_.subscribe("vehicle/cmd", 1, &VehicleController::vehicleCmdCallback, this);
    state_pub_ = nodeHandle_.advertise<vehicle_controller::VehicleState>("vehicle/state", 1);
    calib_server_ = nodeHandle_.advertiseService("vehicle/calibration", &VehicleController::calibrationCallback, this);
    
    nodeHandle_.getParam("vehicle_controller/wheels_name", wheels_name_);

    for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
    {
        ros::Publisher wheel_pub = nodeHandle_.advertise<std_msgs::Float64MultiArray>(*it + "/wheel_cmd", 8);
        ros::Subscriber wheel_sub = nodeHandle_.subscribe(*it + "/wheel_state", 8, &VehicleController::wheelStateCallback, this);
        wheels_pub_.insert(std::pair<std::string, ros::Publisher>(*it, wheel_pub));
        wheels_sub_.insert(std::pair<std::string, ros::Subscriber>(*it, wheel_sub));
    }

    initKinematicsData();
}

VehicleController::~VehicleController()
{

}

void VehicleController::initKinematicsData()
{

}

bool VehicleController::calibrationCallback(vehicle_controller::Calibration::Request &req, 
                                            vehicle_controller::Calibration::Response &res)
{
    return true;
}

void VehicleController::wheelStateCallback(const wheel_controller::WheelState::ConstPtr& state)
{
    std::cout<<"Get callback success, wheel name is "<<state->wheel_name<<std::endl;
}

void VehicleController::vehicleCmdCallback(const vehicle_controller::VehicleCmd::ConstPtr& cmd)
{

}

void VehicleController::process()
{
    
}
}
