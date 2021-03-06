#include <cmath>
#include "wheel_controller/wheel_controller.h"

WheelController::WheelController(ros::NodeHandle& nodeHandle)
    :nodeHandle_(nodeHandle)
{
    ros::NodeHandle nh_private("~");
    wheel_name_ = nh_private.param<std::string>("wheel_name", "wheel");
    sample_rate = nh_private.param<int>("sample_rate", 12);
    control_mode = nh_private.param<std::string>("control_mode", "velocity");
    int swerve_id = nh_private.param<int>("swerve_id", 1);
    int wheel_id = nh_private.param<int>("wheel_id", 2);
    double swerve_gear_ratio = nh_private.param<double>("swerve_gear_ratio", 0.2193);
    double wheel_gear_ratio = nh_private.param<double>("wheel_gear_ratio", 1.03158);

    wheel_state = Disable;
    jointDataInit(swerve_id, wheel_id, swerve_gear_ratio, wheel_gear_ratio);
    wheel_reverse_ = nh_private.param<bool>("wheel_reverse", false);
    sim_ = nh_private.param<bool>("sim", false);
    if(!sim_)
    {
        swerve_drive_interface = new hardware_interface::SwerveDriveInterface(nodeHandle_, this->joint_data, sample_rate, control_mode);
        wheel_cm = new controller_manager::ControllerManager(swerve_drive_interface, nodeHandle);
    }
    else
    {
        joint_state_sub_ = nodeHandle_.subscribe("/joint_states" , 1, &WheelController::jointStateCallback, this);
    }
    wheel_cmd_sub_ = nodeHandle_.subscribe("wheel_cmd", 16, &WheelController::wheelCmdCallback, this);
    wheel_state_pub_ = nodeHandle_.advertise<wheel_controller::WheelDirection>("wheel_state", 1);
	swerve_joint_pub_ = nodeHandle_.advertise<std_msgs::Float64>("swerve_controller/command", 1);
    wheel_joint_pub_ = nodeHandle_.advertise<std_msgs::Float64>("wheel_controller/command", 1);
}

WheelController::~WheelController()
{
    delete swerve_drive_interface;
    delete wheel_cm;
}

void WheelController::jointDataInit(int swerve_id, int wheel_id, double swerve_gear_ratio, double wheel_gear_ratio)
{
    joint_data.push_back(new JointData());
    joint_data[0]->id_             = swerve_id;
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
    joint_data[0]->gear_ratio_     = swerve_gear_ratio;

    joint_data.push_back(new JointData());
    joint_data[1]->id_             = wheel_id;
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
    joint_data[1]->gear_ratio_     = wheel_gear_ratio;
}

void WheelController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::string token_self = wheel_name_.substr(0, wheel_name_.find("_"));
    for(int i=0; i<msg->name.size(); i++)
    {
        if(msg->name[i].find(token_self) != std::string::npos)
        {
            if(msg->name[i].find("swerve") != std::string::npos)
            {
                joint_data[0]->joint_position_ = msg->position[i];
                joint_data[0]->joint_angle_ = fmod(msg->position[i], 2 * M_PI);
                if(joint_data[0]->joint_angle_ > M_PI)
                    joint_data[0]->joint_angle_ -= 2 * M_PI;
                else if (joint_data[0]->joint_angle_ < -1 * M_PI)
                    joint_data[0]->joint_angle_ += 2 * M_PI;
                joint_data[0]->velocity_ = msg->velocity[i];
            }
            else if(msg->name[i].find("wheel") != std::string::npos)
            {
                joint_data[1]->joint_position_ = msg->position[i];
                joint_data[1]->joint_angle_ = fmod(msg->position[i], 2 *M_PI);
                if(joint_data[1]->joint_angle_ > M_PI)
                    joint_data[1]->joint_angle_ -= 2 * M_PI;
                else if (joint_data[1]->joint_angle_ < -1 * M_PI)
                    joint_data[1]->joint_angle_ += 2 * M_PI;
                joint_data[1]->velocity_ = msg->velocity[i];
            }
        }
    }
}

void WheelController::wheelCmdCallback(const wheel_controller::WheelDirection::ConstPtr& msg)
{
    double swerve_angle = atan2(msg->dir_y, msg->dir_x);
    double wheel_velocity = metersToRads(sqrt(msg->dir_x*msg->dir_x + msg->dir_y*msg->dir_y));
    double dis_angle = (fabs(swerve_angle - joint_data[0]->joint_angle_) > M_PI) ? 
        2 * M_PI - fabs(swerve_angle - joint_data[0]->joint_angle_) : fabs(swerve_angle - joint_data[0]->joint_angle_);
    if(dis_angle > M_PI / 2)
    {
        swerve_angle += (swerve_angle > 0) ? -1 * M_PI : M_PI;
        wheel_velocity *= -1;
        
    }
    
    dis_angle = swerve_angle - joint_data[0]->joint_angle_;
    if(fabs(dis_angle) > M_PI)
        dis_angle += (dis_angle > 0) ? -2 * M_PI : 2 * M_PI;
    swerve_angle = joint_data[0]->joint_position_ + dis_angle;

    std_msgs::Float64 cmd;
    cmd.data = swerve_angle;
    swerve_joint_pub_.publish(cmd);
    cmd.data = (wheel_reverse_) ? wheel_velocity * -1 : wheel_velocity;
    wheel_joint_pub_.publish(cmd);
}

double WheelController::metersToRads(const double &meters)
{
    return meters / WHEEL_RADIUS;
}

double WheelController::radsTometers(const double &rads)
{
    return rads * WHEEL_RADIUS;
}

void WheelController::statePublish()
{
    wheel_controller::WheelDirection state;
    state.wheel_name = wheel_name_;
    state.dir_x = cos(joint_data[0]->joint_angle_); //(joint_data[0]->joint_angle_ >= 0) ? cos(joint_data[0]->joint_angle_) : -1 * cos(joint_data[0]->joint_angle_);
    state.dir_x *= radsTometers(joint_data[1]->velocity_);
    state.dir_y = sin(joint_data[0]->joint_angle_) * radsTometers(joint_data[1]->velocity_);
    wheel_state_pub_.publish(state);
}

void WheelController::process(ros::Rate& loop_rate)
{
    if(sim_)
        return;
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0); // Timeout of 2 seconds
    ros::Rate rate(100);
    while(swerve_drive_interface->read() == false)
    {
        ros::spinOnce();
        if(ros::Time::now() - start_time > timeout)
        {
            ROS_ERROR("Wait for state update timeout!!");
            break;
        }
        rate.sleep();
    }
    if(wheel_reverse_)
        joint_data[0]->velocity_ *= -1;
    wheel_cm->update(ros::Time::now(), loop_rate.expectedCycleTime());
    swerve_drive_interface->writeVelocity(loop_rate.expectedCycleTime());
    return;
}
