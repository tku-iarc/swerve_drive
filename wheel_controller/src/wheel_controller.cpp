#include "wheel_controller/wheel_controller.h"

WheelController::WheelController(ros::NodeHandle& nodeHandle)
    :nodeHandle_(nodeHandle)
{
    ros::NodeHandle nh_private("~");
    sample_rate = nh_private.param<int>("sample_rate", 12);
    control_mode = nh_private.param<std::string>("control_mode", "velocity");
    int swerve_id = nh_private.param<int>("swerve_id", 1);
    int wheel_id = nh_private.param<int>("wheel_id", 2);

    wheel_state = Disable;
    jointDataInit(swerve_id, wheel_id);
    swerve_drive_interface = new hardware_interface::SwerveDriveInterface(nodeHandle_, this->joint_data, sample_rate, control_mode);
    wheel_cm = new controller_manager::ControllerManager(swerve_drive_interface, nodeHandle);
}

WheelController::~WheelController()
{
    delete swerve_drive_interface;
    delete wheel_cm;
}

void WheelController::jointDataInit(int swerve_id, int wheel_id)
{
    joint_data.push_back(new JointData());
    joint_data[0]->id_           = swerve_id;
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
    joint_data[0]->gear_ratio_   = 0.2193;

    joint_data.push_back(new JointData());
    joint_data[1]->id_           = wheel_id;
    joint_data[1]->joint_name_   = "wheel_joint";
    joint_data[1]->joint_angle_  = 0;
    joint_data[1]->min_angle_    = -1 * DBL_MAX;
    joint_data[1]->max_angle_    = DBL_MAX;
    joint_data[1]->max_velocity_ = 2.67 * 2 * M_PI * 0.9;
    joint_data[1]->velocity_     = 0;
    joint_data[1]->acceleration_ = 0;
    joint_data[1]->deceleration_ = 0;
    joint_data[1]->angle_cmd_    = 0;
    joint_data[1]->velocity_cmd_ = 0;
    joint_data[1]->effort_       = 0;
    joint_data[1]->home_offset_  = 0;
    joint_data[1]->gear_ratio_   = 1.03158;
}

void WheelController::process(ros::Rate& loop_rate)
{
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(2.0); // Timeout of 2 seconds
    ros::Rate rate(100);
    while(swerve_drive_interface->read() == false)
    {
        ROS_INFO("in the while");
        ros::spinOnce();
        ROS_INFO("done_spin");
        if(ros::Time::now() - start_time > timeout)
        {
            ROS_ERROR("Wait for state update timeout!!");
            break;
        }
        rate.sleep();
    }
    std::cout<<"loop_rate.expectedCycleTime() = "<<loop_rate.expectedCycleTime().toSec()*1000<<" ms"<<std::endl;
    wheel_cm->update(ros::Time::now(), loop_rate.expectedCycleTime());
    swerve_drive_interface->writeVelocity(loop_rate.expectedCycleTime());
    return;
}
