#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "vehicle_controller/vehicle_controller.h"
#define JOY_SPEED 0.5  //0~1
using namespace std::chrono_literals;

namespace vehicle_controller
{
VehicleController::VehicleController(std::string& node_name)
    :Node(node_name)
{
    auto_declare<bool>("sim", false);
    auto_declare<double>("dir_acc_max", NAN);
    auto_declare<double>("ang_acc_max", NAN);
    auto_declare<std::string>("prefix", std::string());
    auto_declare<std::vector<std::string>>("wheel_data.wheels_name", std::vector<std::string>());

    this->get_parameter_or("sim", sim_, false);
    this->get_parameter_or("dir_acc_max", dir_acc_max_, 1.0);
    this->get_parameter_or("ang_acc_max", ang_acc_max_, 1.0);
    this->get_parameter_or("prefix", prefix_, std::string());

    wheels_name_ = this->get_parameter("wheel_data.wheels_name").as_string_array();
    for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
    {
        auto_declare<std::vector<int>>("wheel_data." + *it + ".motors_id", std::vector<int>());
        auto_declare<std::vector<std::string>>("wheel_data." + *it + ".joints", std::vector<std::string>());
        auto_declare<std::vector<double>>("wheel_data." + *it + ".pos_on_vehicle", std::vector<double>());
    }


    state_pub_        = this->create_publisher<mobile_base_msgs::msg::VehicleState>("vehicle/state", 1);
    odom_pub_         = this->create_publisher<nav_msgs::msg::Odometry>("vehicle/odom", 1);
    swerve_cmd_pub_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("swerve_controller/commands", 1);
    wheel_cmd_pub_    = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_controller/commands", 1);
    cmd_sub_          = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(
                        &VehicleController::vehicleCmdCallback, this, std::placeholders::_1));
    joy_sub           = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(
                        &VehicleController::joysticMsgCallback, this, std::placeholders::_1));
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(
                        &VehicleController::jointStatesCallback, this, std::placeholders::_1));
    calib_server_     = this->create_service<mobile_base_msgs::srv::Calibration>("vehicle/calibration", std::bind(
                        &VehicleController::calibrationCallback, this, std::placeholders::_1, std::placeholders::_2));

    for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
    {
        auto id_param = this->get_parameter("wheel_data." + *it + ".motors_id").as_integer_array();
        std::vector<int> id(id_param.begin(), id_param.end());
        mobile_base_msgs::msg::WheelDirection::SharedPtr wheel_cmd_init;
        wheel_controller::WheelController* wheel_controller = new wheel_controller::WheelController(sim_, id);
        auto joints_name = this->get_parameter("wheel_data." + *it + ".joints").as_string_array();
        wheel_controllers_.insert(std::pair<std::string, wheel_controller::WheelController*>(*it, wheel_controller));
        wheels_direction_cmd_.insert(std::pair<std::string, mobile_base_msgs::msg::WheelDirection::SharedPtr>(*it, wheel_cmd_init));
        for(auto jn=joints_name.begin(); jn!=joints_name.end(); ++jn)
        {
            std::vector<double> vec_init(3, std::numeric_limits<double>::quiet_NaN());
            joint_states_.insert(std::pair<std::string, std::vector<double>>(prefix_ + *jn, vec_init));
        }
    }
    initKinematicsData(wheels_name_);
    kinematics = VehicleKinematics();
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(40ms, std::bind(&VehicleController::mainLoopCallback, this));
}

VehicleController::~VehicleController()
{

}

void VehicleController::initKinematicsData(std::vector<std::string>& wheels_name)
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
    for(auto it = wheels_name.begin(); it != wheels_name.end(); ++it)
    {
        WheelData wheel_data;
        wheel_data.wheel_name = *it;
        wheel_data.has_slippage = false;
        wheel_data.direction[0] = 0;
        wheel_data.direction[1] = 0;
        wheel_data.direction_cmd[0] = 0;
        wheel_data.direction_cmd[1] = 0;
        wheel_data.pos_on_vehicle[0] = this->get_parameter("wheel_data." + *it + ".pos_on_vehicle").as_double_array()[0];
        wheel_data.pos_on_vehicle[1] = this->get_parameter("wheel_data." + *it + ".pos_on_vehicle").as_double_array()[1];
        auto joints_name = this->get_parameter("wheel_data." + *it + ".joints").as_string_array();
        for(auto jn=joints_name.begin(); jn!=joints_name.end(); ++jn)
            wheel_data.joints_name.push_back(prefix_ + *jn);
        kinematics_data_.wheel_data_vector.push_back(wheel_data);
        kinematics_data_.wheel_data.insert(std::pair<std::string, WheelData>(*it, wheel_data));
    }
}

bool VehicleController::calibrationCallback(const std::shared_ptr<mobile_base_msgs::srv::Calibration::Request> req, 
                                            std::shared_ptr<mobile_base_msgs::srv::Calibration::Response> res)
{
    kinematics_data_.position[0] = req->pos_x;
    kinematics_data_.position[1] = req->pos_y;
    kinematics_data_.rotation = req->pos_r;
    res->success = true;
    return true;
}

void VehicleController::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
    for(size_t i=0; i<state->name.size(); i++)
    {
        if (joint_states_.find(state->name[i]) == joint_states_.end())
            continue;
        joint_states_[state->name[i]][0] = state->position[i];
        joint_states_[state->name[i]][1] = state->velocity[i];
        joint_states_[state->name[i]][2] = state->effort[i];
    }
    for(auto const& [name, controller] : wheel_controllers_)
    {
        std::vector<std::vector<double>> wheel_state;
        auto wheel_dir = std::make_shared<mobile_base_msgs::msg::WheelDirection>();

        for(std::string jn : kinematics_data_.wheel_data[name].joints_name)
            wheel_state.push_back(joint_states_[jn]);
        
        controller->updateJointData(wheel_state, wheel_dir);
        kinematics_data_.wheel_data[name].direction[0] = wheel_dir->dir_x;
        kinematics_data_.wheel_data[name].direction[1] = wheel_dir->dir_y;
    }
}

void VehicleController::vehicleCmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
    if(en_joy_)
        return;
    kinematics_data_.direction_cmd[0] = cmd->linear.x;
    kinematics_data_.direction_cmd[1] = cmd->linear.y;
    kinematics_data_.angular_velocity_cmd = cmd->angular.z;
    this->ensureCmdLimit();
    if(kinematics.inverseKinematics(kinematics_data_))
    {
        // for(auto it=kinematics_data_.wheel_data_vector.begin(); it!=kinematics_data_.wheel_data_vector.end(); it++)
        for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
        {
            auto wheel_dir = std::make_shared<mobile_base_msgs::msg::WheelDirection>();
            wheel_dir->wheel_name = *it;
            wheel_dir->dir_x = kinematics_data_.wheel_data[*it].direction_cmd[0];
            wheel_dir->dir_y = kinematics_data_.wheel_data[*it].direction_cmd[1];
            wheels_direction_cmd_[*it] = wheel_dir;
            // wheels_pub_[it->first]->publish(wheel_dir);
        }
        sendCmd();
    }
}

void VehicleController::joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(msg->axes[5] > -0.9)
    {
        if(en_joy_ == false)
            return;

        kinematics_data_.direction_cmd[0] = 0;
        kinematics_data_.direction_cmd[1] = 0;
        kinematics_data_.angular_velocity_cmd = 0;
        en_joy_ = false;
    }
    else
    {
        en_joy_ = true;
        double dir_x = (msg->axes[1] > 0.2) ? msg->axes[1] - 0.2 : (msg->axes[1] < -0.2) ? msg->axes[1] + 0.2 : 0;
        double dir_y = (msg->axes[0] > 0.2) ? msg->axes[0] - 0.2 : (msg->axes[0] < -0.2) ? msg->axes[0] + 0.2 : 0;
        double ang_v = (msg->axes[3] > 0.2) ? msg->axes[3] - 0.2 : (msg->axes[3] < -0.2) ? msg->axes[3] + 0.2 : 0;
        kinematics_data_.direction_cmd[0] = dir_x * JOY_SPEED;
        kinematics_data_.direction_cmd[1] = dir_y * JOY_SPEED;
        kinematics_data_.angular_velocity_cmd = ang_v * M_PI  * JOY_SPEED;
        this->ensureCmdLimit();
    }
    
    kinematics.inverseKinematics(kinematics_data_);
    for(auto it=wheels_name_.begin(); it!=wheels_name_.end(); ++it)
    {
        auto wheel_dir = std::make_shared<mobile_base_msgs::msg::WheelDirection>();
        wheel_dir->wheel_name = *it;
        wheel_dir->dir_x = kinematics_data_.wheel_data[*it].direction_cmd[0];
        wheel_dir->dir_y = kinematics_data_.wheel_data[*it].direction_cmd[1];
        wheels_direction_cmd_[*it] = wheel_dir;
    }
    sendCmd();
}

void VehicleController::ensureCmdLimit()
{
    double norm_curr, norm_cmd, dis;
    norm_curr = sqrt(pow(kinematics_data_.direction[0], 2) + pow(kinematics_data_.direction[1], 2));
    norm_cmd = sqrt(pow(kinematics_data_.direction_cmd[0], 2) + pow(kinematics_data_.direction_cmd[1], 2));
    if(norm_cmd < 0.00001 && fabs(kinematics_data_.angular_velocity_cmd) < 0.00001)
        return;
    dis = norm_cmd - norm_curr;
    if(fabs(dis) > dir_acc_max_)
    {
        if(dis > 0)
        {
            kinematics_data_.direction_cmd[0] *= ((norm_curr + dir_acc_max_) / norm_cmd);
            kinematics_data_.direction_cmd[1] *= ((norm_curr + dir_acc_max_) / norm_cmd);
        }
        else
        {
            if(norm_cmd < 0.00001)
            {
                kinematics_data_.direction_cmd[0] = kinematics_data_.direction[0] * ((norm_curr - dir_acc_max_) / norm_curr);
                kinematics_data_.direction_cmd[1] = kinematics_data_.direction[1] * ((norm_curr - dir_acc_max_) / norm_curr);
            }
            else
            {
                kinematics_data_.direction_cmd[0] *= ((norm_curr - dir_acc_max_) / norm_cmd);
                kinematics_data_.direction_cmd[1] *= ((norm_curr - dir_acc_max_) / norm_cmd);
            }
        }
    }
    dis = kinematics_data_.angular_velocity_cmd - kinematics_data_.angular_velocity;
    if(fabs(dis) > ang_acc_max_)
    {
        if(dis > 0)
            kinematics_data_.angular_velocity_cmd = kinematics_data_.angular_velocity + ang_acc_max_;
        else
            kinematics_data_.angular_velocity_cmd = kinematics_data_.angular_velocity - ang_acc_max_;
    }
}

void VehicleController::vehicleOdometer(rclcpp::Rate& /*loop_rate*/)
{
    if(kinematics.forwardKinematics(kinematics_data_))
    {    
        double cycle_time = 0.04; //loop_rate.expectedCycleTime().toSec();
        kinematics_data_.rotation += kinematics_data_.angular_velocity * cycle_time;
        kinematics_data_.rotation += (kinematics_data_.rotation > M_PI) ? -2 * M_PI : (kinematics_data_.rotation < -1 * M_PI) ? 2 * M_PI : 0;
        kinematics_data_.position[0] += (cos(kinematics_data_.rotation) * kinematics_data_.direction[0] * cycle_time
                                        - sin(kinematics_data_.rotation) * kinematics_data_.direction[1] * cycle_time);
        kinematics_data_.position[1] += (cos(kinematics_data_.rotation) * kinematics_data_.direction[1] * cycle_time
                                        + sin(kinematics_data_.rotation) * kinematics_data_.direction[0] * cycle_time);
    }
}

void VehicleController::vehicleStatePublish()
{
    rclcpp::Time curr_time = this->get_clock()->now();
    mobile_base_msgs::msg::VehicleState state;
    state.vel_x = kinematics_data_.direction[0];
    state.vel_y = kinematics_data_.direction[1];
    state.vel_r = kinematics_data_.angular_velocity;
    state.pos_x = kinematics_data_.position[0];
    state.pos_y = kinematics_data_.position[1];
    state.pos_r = kinematics_data_.rotation;
    state_pub_->publish(state);
    tf2::Quaternion q;
    q.setRPY(0, 0, kinematics_data_.rotation);
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = curr_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = prefix_ + "base_footprint";

    odom_trans.transform.translation.x = kinematics_data_.position[0];
    odom_trans.transform.translation.y = kinematics_data_.position[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    odom_broadcaster_->sendTransform(odom_trans);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = curr_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = kinematics_data_.position[0];
    odom.pose.pose.position.y = kinematics_data_.position[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    //set the velocity
    odom.child_frame_id = prefix_ + "base_footprint";
    odom.twist.twist.linear.x = kinematics_data_.direction[0];
    odom.twist.twist.linear.y = kinematics_data_.direction[1];
    odom.twist.twist.angular.z = kinematics_data_.angular_velocity;

    //publish the message
    odom_pub_->publish(odom);
}

void VehicleController::sendCmd()
{
    std_msgs::msg::Float64MultiArray swerves_cmd, wheels_cmd;
    for(auto const& name : wheels_name_)
    {
        std::vector<double> cmds;
        cmds.resize(2, std::numeric_limits<double>::quiet_NaN());
        wheel_controllers_[name]->getWheelCmd(wheels_direction_cmd_[name], cmds);
        swerves_cmd.data.push_back(cmds[0]);
        wheels_cmd.data.push_back(cmds[1]);
    }
    swerve_cmd_pub_->publish(swerves_cmd);
    wheel_cmd_pub_->publish(wheels_cmd);
}

void VehicleController::mainLoopCallback()
{
    rclcpp::Rate loop_rate(10);
    this->vehicleOdometer(loop_rate);
    this->vehicleStatePublish();
}
}
