#include <cmath>
#include "vehicle_controller/vehicle_controller.h"
#define JOY_SPEED 0.3  //0~1
namespace vehicle_controller
{
VehicleController::VehicleController(ros::NodeHandle& nodeHandle)
:nodeHandle_(nodeHandle)
{
    nodeHandle_.param<float>("dir_acc_max", dir_acc_max_, 1);
    nodeHandle_.param<float>("ang_acc_max", ang_acc_max_, 1);
    cmd_sub_ = nodeHandle_.subscribe("vehicle/cmd", 1, &VehicleController::vehicleCmdCallback, this);
    joy_sub = nodeHandle_.subscribe("joy", 1, &VehicleController::joysticMsgCallback, this);
    state_pub_ = nodeHandle_.advertise<mobile_base_msgs::msg::VehicleState>("vehicle/state", 1);
    odom_pub_ = nodeHandle_.advertise<nav_msgs::Odometry>("vehicle/odom", 1);
    calib_server_ = nodeHandle_.advertiseService("vehicle/calibration", &VehicleController::calibrationCallback, this);
    
    nodeHandle_.getParam("vehicle_controller/wheel_data/wheels_name", wheels_name_);

    for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
    {
        ros::Publisher wheel_pub = nodeHandle_.advertise<mobile_base_msgs::msg::WheelDirection>(*it + "/wheel_cmd", 8);
        ros::Subscriber wheel_sub = nodeHandle_.subscribe(*it + "/wheel_state", 8, &VehicleController::wheelStateCallback, this);
        wheels_pub_.insert(std::pair<std::string, ros::Publisher>(*it, wheel_pub));
        wheels_sub_.insert(std::pair<std::string, ros::Subscriber>(*it, wheel_sub));
    }

    initKinematicsData();
    kinematics = VehicleKinematics();
}

VehicleController::~VehicleController()
{

}

void VehicleController::initKinematicsData()
{
    kinematics_data_.direction[0] = 0;
    kinematics_data_.direction[1] = 0;
    kinematics_data_.direction_cmd[0] = 0;
    kinematics_data_.direction_cmd[1] = 0;
    kinematics_data_.position[0] = 0;
    kinematics_data_.position[1] = 0;
    kinematics_data_.angular_velocity = 0;
    kinematics_data_.angular_velocity_cmd = 0;
    kinematics_data_.rotation = 0;
    for(auto it = wheels_name_.begin(); it != wheels_name_.end(); ++it)
    {
        WheelData wheel_data;
        wheel_data.wheel_name = *it;
        wheel_data.direction[0] = 0;
        wheel_data.direction[1] = 0;
        wheel_data.direction_cmd[0] = 0;
        wheel_data.direction_cmd[1] = 0;
        wheel_data.pos_on_vehicle[0] = nodeHandle_.param<double>("vehicle_controller/wheel_data/" + *it + "/pos_on_vehicle_x", 1);
        wheel_data.pos_on_vehicle[1] = nodeHandle_.param<double>("vehicle_controller/wheel_data/" + *it + "/pos_on_vehicle_y", 1);
        kinematics_data_.wheel_data_vector.push_back(wheel_data);
        kinematics_data_.wheel_data.insert(std::pair<std::string, WheelData>(*it, wheel_data));
    }
}

bool VehicleController::calibrationCallback(const std::shared_ptr<mobile_base_msgs::srv::Calibration::Request> req, 
                                            std::shared_ptr<mobile_base_msgs::srv::Calibration::Response> res)
{
    kinematics_data_.position[0] = req.pos_x;
    kinematics_data_.position[1] = req.pos_y;
    kinematics_data_.rotation = req.pos_r;
    res.success = true;
    return true;
}

void VehicleController::wheelStateCallback(const mobile_base_msgs::msg::WheelDirection::SharedPtr state)
{
    kinematics_data_.wheel_data[state->wheel_name].direction[0] = state->dir_x;
    kinematics_data_.wheel_data[state->wheel_name].direction[1] = state->dir_y;
}

void VehicleController::vehicleCmdCallback(const mobile_base_msgs::msg::VehicleCmd::SharedPtr cmd)
{
    kinematics_data_.direction_cmd[0] = cmd->vel_x;
    kinematics_data_.direction_cmd[1] = cmd->vel_y;
    kinematics_data_.angular_velocity_cmd = cmd->vel_r;
    this->ensureCmdLimit();
    kinematics.inverseKinematics(kinematics_data_);

    for(auto it=kinematics_data_.wheel_data.begin(); it!=kinematics_data_.wheel_data.end(); it++)
    {
        mobile_base_msgs::msg::WheelDirection wheel_dir;
        wheel_dir.wheel_name = it->first;
        wheel_dir.dir_x = it->second.direction_cmd[0];
        wheel_dir.dir_y = it->second.direction_cmd[1];
        wheels_pub_[it->first].publish(wheel_dir);
    }
}

void VehicleController::joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(msg->axes[5] > -0.9)
    {
        kinematics_data_.direction_cmd[0] = 0;
        kinematics_data_.direction_cmd[1] = 0;
        kinematics_data_.angular_velocity_cmd = 0;
    }
    else
    {
        kinematics_data_.direction_cmd[0] = (msg->axes[1] > 0.2) ? msg->axes[1] - 0.2 : (msg->axes[1] < -0.2) ? msg->axes[1] + 0.2 : 0;
        kinematics_data_.direction_cmd[1] = (msg->axes[0] > 0.2) ? msg->axes[0] - 0.2 : (msg->axes[0] < -0.2) ? msg->axes[0] + 0.2 : 0;
        kinematics_data_.angular_velocity_cmd = (msg->axes[3] > 0.2) ? msg->axes[3] - 0.2 : (msg->axes[3] < -0.2) ? msg->axes[3] + 0.2 : 0;
        this->ensureCmdLimit();
    }
    
    kinematics.inverseKinematics(kinematics_data_);

    for(auto it=kinematics_data_.wheel_data.begin(); it!=kinematics_data_.wheel_data.end(); it++)
    {
        mobile_base_msgs::msg::WheelDirection wheel_dir;
        wheel_dir.wheel_name = it->first;
        wheel_dir.dir_x = it->second.direction_cmd[0] * JOY_SPEED;
        wheel_dir.dir_y = it->second.direction_cmd[1] * JOY_SPEED;
        wheels_pub_[it->first].publish(wheel_dir);
    }
}

void VehicleController::ensureCmdLimit()
{
    double norm_curr, norm_cmd, dis;
    norm_curr = sqrt(pow(kinematics_data_.direction[0], 2) + pow(kinematics_data_.direction[1], 2));
    norm_cmd = sqrt(pow(kinematics_data_.direction_cmd[0], 2) + pow(kinematics_data_.direction_cmd[1], 2));
    if(norm_cmd < 0.00001 && kinematics_data_.angular_velocity_cmd < 0.00001)
        return;
    dis = fabs(norm_cmd - norm_curr);
    if(dis > dir_acc_max_)
    {
        kinematics_data_.direction_cmd[0] *= (dir_acc_max_ / dis);
        kinematics_data_.direction_cmd[1] *= (dir_acc_max_ / dis);
    }
    dis = fabs(kinematics_data_.angular_velocity_cmd - kinematics_data_.angular_velocity);
    if(dis > ang_acc_max_)
        kinematics_data_.angular_velocity_cmd *= (ang_acc_max_ / dis);
}

void VehicleController::vehicleOdometer(ros::Rate& loop_rate)
{
    kinematics.forwardKinematics(kinematics_data_);
    double cycle_time = loop_rate.expectedCycleTime().toSec();
    kinematics_data_.rotation += kinematics_data_.angular_velocity * cycle_time;
    kinematics_data_.rotation += (kinematics_data_.rotation > M_PI) ? -2 * M_PI : (kinematics_data_.rotation < -1 * M_PI) ? 2 * M_PI : 0;
    kinematics_data_.position[0] += (cos(kinematics_data_.rotation) * kinematics_data_.direction[0] * cycle_time
                                     - sin(kinematics_data_.rotation) * kinematics_data_.direction[1] * cycle_time);
    kinematics_data_.position[1] += (cos(kinematics_data_.rotation) * kinematics_data_.direction[1] * cycle_time
                                     + sin(kinematics_data_.rotation) * kinematics_data_.direction[0] * cycle_time);
}

void VehicleController::vehicleStatePublish()
{
    ros::Time curr_time = ros::Time::now();
    mobile_base_msgs::msg::VehicleState state;
    state.vel_x = kinematics_data_.direction[0];
    state.vel_y = kinematics_data_.direction[1];
    state.vel_r = kinematics_data_.angular_velocity;
    state.pos_x = kinematics_data_.position[0];
    state.pos_y = kinematics_data_.position[1];
    state.pos_r = kinematics_data_.rotation;
    state_pub_.publish(state);

    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(kinematics_data_.rotation);
    odom_trans.header.stamp = curr_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = kinematics_data_.position[0];
    odom_trans.transform.translation.y = kinematics_data_.position[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = curr_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = kinematics_data_.position[0];
    odom.pose.pose.position.y = kinematics_data_.position[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = kinematics_data_.direction[0];
    odom.twist.twist.linear.y = kinematics_data_.direction[1];
    odom.twist.twist.angular.z = kinematics_data_.angular_velocity;

    //publish the message
    odom_pub_.publish(odom);

}

void VehicleController::process(ros::Rate& loop_rate)
{
    this->vehicleOdometer(loop_rate);
    this->vehicleStatePublish();
}
}
