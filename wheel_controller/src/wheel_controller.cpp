#include "wheel_controller/wheel_controller.h"

WheelController::WheelController(ros::NodeHandle& nodeHandle)
    :nodeHandle_(nodeHandle)
{
    sample_rate = nodeHandle.param<int>("/blue_arm_control/sample_rate", 12);
    control_mode = nodeHandle.param<std::string>("/blue_arm_control/control_mode", "velocity");
    arm_state = Disable;
    jointDataInit();
    blue_arm_interface = new hardware_interface::BlueArmInterface(nodeHandle_, this->joint_data, sample_rate, control_mode);
    blue_arm_cm = new controller_manager::ControllerManager(blue_arm_interface, nodeHandle);
}

WheelController::~WheelController()
{
    delete blue_arm_interface;
    delete blue_arm_cm;
}

void WheelController::jointDataInit()
{
    joint_data.push_back(new JointData());
    joint_data[0]->id_           = 1;
    joint_data[0]->joint_name_   = "joint_1";
    joint_data[0]->joint_angle_  = 0;
    joint_data[0]->min_angle_    = -1 * M_PI;
    joint_data[0]->max_angle_    = M_PI;
    joint_data[0]->max_velocity_ = 1.165 * M_PI * 0.9;
    joint_data[0]->velocity_     = 0;
    joint_data[0]->acceleration_ = 0;
    joint_data[0]->deceleration_ = 0;
    joint_data[0]->angle_cmd_    = 0;
    joint_data[0]->velocity_cmd_ = 0;
    joint_data[0]->effort_       = 0;
    joint_data[0]->home_offset_  = 0;

    joint_data.push_back(new JointData());
    joint_data[1]->id_           = 2;
    joint_data[1]->joint_name_   = "joint_2";
    joint_data[1]->joint_angle_  = 0;
    joint_data[1]->min_angle_    = 0;
    joint_data[1]->max_angle_    = M_PI;
    joint_data[1]->max_velocity_ = 1.165 * M_PI * 0.9;
    joint_data[1]->velocity_     = 0;
    joint_data[1]->acceleration_ = 0;
    joint_data[1]->deceleration_ = 0;
    joint_data[1]->angle_cmd_    = 0;
    joint_data[1]->velocity_cmd_ = 0;
    joint_data[1]->effort_       = 0;
    joint_data[0]->home_offset_  = 0;

    joint_data.push_back(new JointData());
    joint_data[2]->id_           = 3;
    joint_data[2]->joint_name_   = "joint_3";
    joint_data[2]->joint_angle_  = 0;
    joint_data[2]->min_angle_    = -1 * M_PI;
    joint_data[2]->max_angle_    = M_PI;
    joint_data[2]->max_velocity_ = 0.846 * M_PI * 0.9;
    joint_data[2]->velocity_     = 0;
    joint_data[2]->acceleration_ = 0;
    joint_data[2]->deceleration_ = 0;
    joint_data[2]->angle_cmd_    = M_PI;
    joint_data[2]->velocity_cmd_ = 0;
    joint_data[2]->effort_       = 0;
    joint_data[2]->home_offset_  = M_PI;

    joint_data.push_back(new JointData());
    joint_data[3]->id_           = 4;
    joint_data[3]->joint_name_   = "joint_4";
    joint_data[3]->joint_angle_  = 0;
    joint_data[3]->min_angle_    = -1 * M_PI;
    joint_data[3]->max_angle_    = 0;
    joint_data[3]->max_velocity_ = 0.846 * M_PI * 0.9;
    joint_data[3]->velocity_     = 0;
    joint_data[3]->acceleration_ = 0;
    joint_data[3]->deceleration_ = 0;
    joint_data[3]->angle_cmd_    = -1 * M_PI;
    joint_data[3]->velocity_cmd_ = 0;
    joint_data[3]->effort_       = 0;
    joint_data[3]->home_offset_  = -1 * M_PI;

    joint_data.push_back(new JointData());
    joint_data[4]->id_           = 5;
    joint_data[4]->joint_name_   = "joint_5";
    joint_data[4]->joint_angle_  = 0;
    joint_data[4]->min_angle_    = -1 * M_PI;
    joint_data[4]->max_angle_    = M_PI;
    joint_data[4]->max_velocity_ = 0.846 * M_PI * 0.9;
    joint_data[4]->velocity_     = 0;
    joint_data[4]->acceleration_ = 0;
    joint_data[4]->deceleration_ = 0;
    joint_data[4]->angle_cmd_    = 0;
    joint_data[4]->velocity_cmd_ = 0;
    joint_data[4]->effort_       = 0;
    joint_data[4]->home_offset_  = 0;

    joint_data.push_back(new JointData());
    joint_data[5]->id_           = 6;
    joint_data[5]->joint_name_   = "joint_6";
    joint_data[5]->joint_angle_  = 0;
    joint_data[5]->min_angle_    = -1.7;
    joint_data[5]->max_angle_    = 1.7;
    joint_data[5]->max_velocity_ = 0.98 * M_PI * 0.9;
    joint_data[5]->velocity_     = 0;
    joint_data[5]->acceleration_ = 0;
    joint_data[5]->deceleration_ = 0;
    joint_data[5]->angle_cmd_    = 0;
    joint_data[5]->velocity_cmd_ = 0;
    joint_data[5]->effort_       = 0;
    joint_data[5]->home_offset_  = 0;

    joint_data.push_back(new JointData());
    joint_data[6]->id_           = 7;
    joint_data[6]->joint_name_   = "joint_7";
    joint_data[6]->joint_angle_  = 0;
    joint_data[6]->min_angle_    = -1 * M_PI;
    joint_data[6]->max_angle_    = M_PI;
    joint_data[6]->max_velocity_ = 0.98 * M_PI * 0.9;
    joint_data[6]->velocity_     = 0;
    joint_data[6]->acceleration_ = 0;
    joint_data[6]->deceleration_ = 0;
    joint_data[6]->angle_cmd_    = 0;
    joint_data[6]->velocity_cmd_ = 0;
    joint_data[6]->effort_       = 0;
    joint_data[6]->home_offset_  = 0;
}

void WheelController::closeDevice()
{
    blue_arm_interface->closeDevice();
    return;
}
void WheelController::process(ros::Rate& loop_rate)
{
    blue_arm_interface->readPosition();
    blue_arm_cm->update(ros::Time::now(), loop_rate.expectedCycleTime());
    blue_arm_interface->writeVelocity(loop_rate.expectedCycleTime());
    return;
}
