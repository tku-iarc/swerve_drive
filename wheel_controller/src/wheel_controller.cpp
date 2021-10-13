#include <cmath>
#include "wheel_controller/wheel_controller.h"

WheelController::WheelController(ros::NodeHandle& nodeHandle)
    :nodeHandle_(nodeHandle)
{
    ros::NodeHandle nh_private("~");
    sample_rate = nh_private.param<int>("sample_rate", 12);
    control_mode = nh_private.param<std::string>("control_mode", "velocity");
    int swerve_id = nh_private.param<int>("swerve_id", 1);
    int wheel_id = nh_private.param<int>("wheel_id", 2);
    double swerve_gear_ratio = nh_private.param<double>("swerve_gear_ratio", 0.2193);
    double wheel_gear_ratio = nh_private.param<double>("wheel_gear_ratio", 1.03158);

    wheel_state = Disable;
    jointDataInit(swerve_id, wheel_id, swerve_gear_ratio, wheel_gear_ratio);
    swerve_drive_interface = new hardware_interface::SwerveDriveInterface(nodeHandle_, this->joint_data, sample_rate, control_mode);
    wheel_cm = new controller_manager::ControllerManager(swerve_drive_interface, nodeHandle);

    wheel_cmds_sub_ = nodeHandle_.subscribe("wheel_cmds", 16, &WheelController::wheelCmdsCallback, this);
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
    joint_data[0]->id_           = swerve_id;
    joint_data[0]->control_mode_ = 0;
    joint_data[0]->joint_name_   = "swerve_joint";
    joint_data[0]->joint_angle_  = 0;
    joint_data[0]->min_angle_    = -1 * DBL_MAX;
    joint_data[0]->max_angle_    = DBL_MAX;
    joint_data[0]->max_velocity_ = 0.567 * 2 * M_PI * 0.9;
    joint_data[0]->velocity_     = 0;
    joint_data[0]->acceleration_ = 0;
    joint_data[0]->deceleration_ = 0;
    joint_data[0]->angle_cmd_    = 0;
    joint_data[0]->velocity_cmd_ = 0;
    joint_data[0]->effort_       = 0;
    joint_data[0]->home_offset_  = 0;
    joint_data[0]->gear_ratio_   = swerve_gear_ratio;

    joint_data.push_back(new JointData());
    joint_data[1]->id_           = wheel_id;
    joint_data[0]->control_mode_ = 1;
    joint_data[1]->joint_name_   = "wheel_joint";
    joint_data[1]->joint_angle_  = 0;
    joint_data[1]->min_angle_    = -1 * DBL_MAX;
    joint_data[1]->max_angle_    = DBL_MAX;
    joint_data[1]->max_velocity_ = (2.67 - 0.42)* 2 * M_PI * 0.9;
    joint_data[1]->velocity_     = 0;
    joint_data[1]->acceleration_ = 0;
    joint_data[1]->deceleration_ = 0;
    joint_data[1]->angle_cmd_    = 0;
    joint_data[1]->velocity_cmd_ = 0;
    joint_data[1]->effort_       = 0;
    joint_data[1]->home_offset_  = 0;
    joint_data[1]->gear_ratio_   = wheel_gear_ratio;
}

void WheelController::wheelCmdsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    double swerve_angle = atan2(msg->data[1], msg->data[0]);
    double wheel_velocity = metersToRads(sqrt(msg->data[0]*msg->data[0] + msg->data[1]*msg->data[1]));
    double dis_angle = (fabs(swerve_angle - joint_data[0]->joint_angle_) > M_PI) ? 
        2 * M_PI - fabs(swerve_angle - joint_data[0]->joint_angle_) : fabs(swerve_angle - joint_data[0]->joint_angle_);
    if(dis_angle > M_PI / 2)
    {
        swerve_angle += (swerve_angle > 0) ? -1 * M_PI : M_PI;
        wheel_velocity *= -1;
    }
    std_msgs::Float64 cmd;
    cmd.data = swerve_angle;
    swerve_joint_pub_.publish(cmd);
    cmd.data = wheel_velocity;
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

void WheelController::process(ros::Rate& loop_rate)
{
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
    wheel_cm->update(ros::Time::now(), loop_rate.expectedCycleTime());
    swerve_drive_interface->writeVelocity(loop_rate.expectedCycleTime());
    return;
}
