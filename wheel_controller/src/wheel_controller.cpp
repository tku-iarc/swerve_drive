#include <cmath>
#include <cfloat>
#include "wheel_controller/wheel_controller.h"

namespace wheel_controller
{
WheelController::WheelController(bool sim, std::vector<int> motors_id)
{
    // ros::NodeHandle nh_private("~");
    // wheel_name_ = nh_private.param<std::string>("wheel_name", "wheel");
    // sample_rate = nh_private.param<int>("sample_rate", 12);
    // control_mode = nh_private.param<std::string>("control_mode", "velocity");
    // int swerve_id = nh_private.param<int>("swerve_id", 1);
    // int wheel_id = nh_private.param<int>("wheel_id", 2);
    // double swerve_gear_ratio = nh_private.param<double>("swerve_gear_ratio", 0.2193);
    // double wheel_gear_ratio = nh_private.param<double>("wheel_gear_ratio", 1.03158);

    jointDataInit(motors_id);
    sim_ = sim;
    // sim_ = nh_private.param<bool>("sim", false);
    // if(!sim_)
    // {
    //     swerve_drive_interface = new hardware_interface::SwerveDriveInterface(nodeHandle_, this->joint_data, sample_rate, control_mode);
    //     wheel_cm = new controller_manager::ControllerManager(swerve_drive_interface, nodeHandle);
    // }
    // else
    // {
    //     joint_state_sub_ = nodeHandle_.subscribe("/joint_states" , 1, &WheelController::jointStateCallback, this);
    // }
    // wheel_cmd_sub_ = nodeHandle_.subscribe("wheel_cmd", 16, &WheelController::wheelCmdCallback, this);
    // wheel_state_pub_ = nodeHandle_.advertise<mobile_base_msgs::msg::WheelDirection>("wheel_state", 1);
	// swerve_joint_pub_ = nodeHandle_.advertise<std_msgs::Float64>("swerve_controller/command", 1);
    // wheel_joint_pub_ = nodeHandle_.advertise<std_msgs::Float64>("wheel_controller/command", 1);
}

WheelController::~WheelController()
{
    // if(!sim_)
    // {
    //     delete swerve_drive_interface;
    //     delete wheel_cm;
    // }
}

void WheelController::jointDataInit(std::vector<int> motors_id)
{
    joint_data.push_back(new JointData());
    joint_data[0]->id_             = motors_id[0];
    joint_data[0]->control_mode_   = 0;
    joint_data[0]->joint_name_     = "swerve_joint";
    joint_data[0]->joint_position_ = 0;
    joint_data[0]->joint_angle_    = 0;
    joint_data[0]->min_angle_      = -1 * DBL_MAX;
    joint_data[0]->max_angle_      = DBL_MAX;
    joint_data[0]->max_velocity_   = 0.567 * 2 * M_PI * 0.9;
    joint_data[0]->velocity_       = 0;
    joint_data[0]->acceleration_   = 0;
    joint_data[0]->deceleration_   = 0;
    joint_data[0]->angle_cmd_      = 0;
    joint_data[0]->velocity_cmd_   = 0;
    joint_data[0]->effort_         = 0;
    joint_data[0]->home_offset_    = 0;
    // joint_data[0]->gear_ratio_     = swerve_gear_ratio;

    joint_data.push_back(new JointData());
    joint_data[1]->id_             = motors_id[1];
    joint_data[0]->control_mode_   = 1;
    joint_data[1]->joint_name_     = "wheel_joint";
    joint_data[1]->joint_position_ = 0;
    joint_data[1]->joint_angle_    = 0;
    joint_data[1]->min_angle_      = -1 * DBL_MAX;
    joint_data[1]->max_angle_      = DBL_MAX;
    joint_data[1]->max_velocity_   = (2.67 - 0.42)* 2 * M_PI * 0.9;
    joint_data[1]->velocity_       = 0;
    joint_data[1]->acceleration_   = 0;
    joint_data[1]->deceleration_   = 0;
    joint_data[1]->angle_cmd_      = 0;
    joint_data[1]->velocity_cmd_   = 0;
    joint_data[1]->effort_         = 0;
    joint_data[1]->home_offset_    = 0;
    // joint_data[1]->gear_ratio_     = wheel_gear_ratio;
}

void WheelController::updateJointData(const std::vector<std::vector<double>>& joint_state, 
                                      mobile_base_msgs::msg::WheelDirection::SharedPtr wheel_state)
{
    for(size_t i=0; i<joint_data.size(); i++)
    {
        joint_data[i]->joint_position_ = joint_state[i][0];
        double joint_angle = fmod(joint_state[i][0], 2 * M_PI);
        if(joint_angle > M_PI)
            joint_angle -= 2 * M_PI;
        else if (joint_angle < -1 * M_PI)
            joint_angle += 2 * M_PI;
        joint_data[i]->joint_angle_ = joint_angle;
        joint_data[i]->velocity_ = joint_state[1][i];
        joint_data[i]->acceleration_ = joint_state[i][2];
    }
    wheel_state->dir_x = cos(joint_data[0]->joint_angle_);
    wheel_state->dir_x *= radsTometers(joint_data[1]->velocity_);
    wheel_state->dir_y = sin(joint_data[0]->joint_angle_) * radsTometers(joint_data[1]->velocity_);

    // std::string token_self = wheel_name_.substr(0, wheel_name_.find("_"));
    // for(int i=0; i<msg->name.size(); i++)
    // {
    //     if(msg->name[i].find(token_self) != std::string::npos)
    //     {
    //         if(msg->name[i].find("swerve") != std::string::npos)
    //         {
    //             joint_data[0]->joint_position_ = msg->position[i];
    //             joint_data[0]->joint_angle_ = fmod(msg->position[i], 2 * M_PI);
    //             if(joint_data[0]->joint_angle_ > M_PI)
    //                 joint_data[0]->joint_angle_ -= 2 * M_PI;
    //             else if (joint_data[0]->joint_angle_ < -1 * M_PI)
    //                 joint_data[0]->joint_angle_ += 2 * M_PI;
    //             joint_data[0]->velocity_ = msg->velocity[i];
    //         }
    //         else if(msg->name[i].find("wheel") != std::string::npos)
    //         {
    //             joint_data[1]->joint_position_ = msg->position[i];
    //             joint_data[1]->joint_angle_ = fmod(msg->position[i], 2 *M_PI);
    //             if(joint_data[1]->joint_angle_ > M_PI)
    //                 joint_data[1]->joint_angle_ -= 2 * M_PI;
    //             else if (joint_data[1]->joint_angle_ < -1 * M_PI)
    //                 joint_data[1]->joint_angle_ += 2 * M_PI;
    //             joint_data[1]->velocity_ = msg->velocity[i];
    //         }
    //     }
    // }
}

void WheelController::getWheelCmd(const mobile_base_msgs::msg::WheelDirection::SharedPtr msg, std::vector<double>& cmds)
{
    double swerve_angle = atan2(msg->dir_y, msg->dir_x);
    double wheel_velocity = metersToRads(sqrt(msg->dir_x*msg->dir_x + msg->dir_y*msg->dir_y));
    double dis_angle = fabs(swerve_angle - joint_data[0]->joint_angle_);
    // double dis_angle = (fabs(swerve_angle - joint_data[0]->joint_angle_) > M_PI) ? 
    //     2 * M_PI - fabs(swerve_angle - joint_data[0]->joint_angle_) : fabs(swerve_angle - joint_data[0]->joint_angle_);
    if (dis_angle > M_PI)
        dis_angle = 2 * M_PI - dis_angle;
    if(dis_angle > M_PI / 2)
    {
        swerve_angle += (swerve_angle > 0) ? -1 * M_PI : M_PI;
        wheel_velocity *= -1;
    }
    if(sim_)
    {
        dis_angle = swerve_angle - joint_data[0]->joint_angle_;
        if(fabs(dis_angle) > M_PI)
            dis_angle += (dis_angle > 0) ? -2 * M_PI : 2 * M_PI;
        swerve_angle = joint_data[0]->joint_position_ + dis_angle;
    }
    cmds[0] = swerve_angle;
    cmds[1] = wheel_velocity;
}

double WheelController::metersToRads(const double &meters)
{
    return meters / WHEEL_RADIUS;
}

double WheelController::radsTometers(const double &rads)
{
    return rads * WHEEL_RADIUS;
}

// void WheelController::process(ros::Rate& loop_rate)
// {
//     if(sim_)
//         return;
//     ros::Time start_time = ros::Time::now();
//     ros::Duration timeout(1.0); // Timeout of 2 seconds
//     ros::Rate rate(100);
//     while(swerve_drive_interface->read() == false)
//     {
//         ros::spinOnce();
//         if(ros::Time::now() - start_time > timeout)
//         {
//             ROS_ERROR("Wait for state update timeout!!");
//             break;
//         }
//         rate.sleep();
//     }
//     wheel_cm->update(ros::Time::now(), loop_rate.expectedCycleTime());
//     swerve_drive_interface->writeVelocity(loop_rate.expectedCycleTime());
//     return;
// }
} // end of namespace wheel_controller