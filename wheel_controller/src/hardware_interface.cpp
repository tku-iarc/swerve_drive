#include <ros/ros.h>
#include "wheel_controller/hardware_interface.h"

namespace hardware_interface
{

SwerveDriveInterface::SwerveDriveInterface(ros::NodeHandle& nodeHandle, std::vector<JointData*>& joint_data, float sample_rate, std::string control_mode)
    :nodeHandle_(nodeHandle)
{ 
    // this->control_mode = (control_mode.compare("position") == 0) ? 0 : 1;
    this->sample_rate = sample_rate;
    jd_ptr = joint_data;
    for (int i=0; i < jd_ptr.size(); i++)
    {   
        hardware_interface::JointStateHandle state_handle(jd_ptr[i]->joint_name_, &jd_ptr[i]->joint_angle_, &jd_ptr[i]->velocity_, &jd_ptr[i]->effort_);
        jnt_state_interface.registerHandle(state_handle);
        
    }
    registerInterface(&jnt_state_interface);
    
    initJointInterface(jnt_pos_interface);
    // initJointInterface(jnt_vel_interface);
    motor_states_sub_ = nodeHandle_.subscribe("motor_states", 1, &SwerveDriveInterface::motorStatesCallback, this);
	motor_cmds_pub_ = nodeHandle_.advertise<maxon_epos2::MotorCmds>("motor_cmds", 1);
    received_states = false;
}

SwerveDriveInterface::~SwerveDriveInterface()
{
}

template <typename JointInterface>
void SwerveDriveInterface::initJointInterface(JointInterface& jnt_interface)
{
    // for (int i=0; i < jd_ptr.size(); i++)
    // {
    //     double* cmd = new double;
    //     if(jd_ptr[i]->control_mode_ == 0)
    //     {
    //         cmd = &jd_ptr[i]->angle_cmd_;
    //         hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(jd_ptr[i]->joint_name_), cmd);
    //         jnt_pos_interface.registerHandle(joint_handle);
    //         joint_limits_interface::JointLimits limits;
    //         const bool rosparam_limits_ok = getJointLimits(jd_ptr[i]->joint_name_, nodeHandle_, limits);
    //         joint_limits_interface::PositionJointSaturationHandle limit_handle(joint_handle, limits);
    //         jnt_pos_limits_interface.registerHandle(limit_handle);
    //         std::cout<<"jnt_pos_interface.registerHandle"<<std::endl;
    //     }
    //     else
    //     {
    //         cmd = &jd_ptr[i]->velocity_cmd_;
    //         hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(jd_ptr[i]->joint_name_), cmd);
    //         jnt_vel_interface.registerHandle(joint_handle);
    //         joint_limits_interface::JointLimits limits;
    //         const bool rosparam_limits_ok = getJointLimits(jd_ptr[i]->joint_name_, nodeHandle_, limits);
    //         joint_limits_interface::VelocityJointSaturationHandle limit_handle(joint_handle, limits);
    //         jnt_vel_limits_interface.registerHandle(limit_handle);
    //         std::cout<<"jnt_vel_interface.registerHandle"<<std::endl;
    //     }
    //     // jnt_interface.registerHandle(joint_handle);
    //     // if(jd_ptr[i]->control_mode_ == 0)
    //     //     jnt_pos_interface.registerHandle(joint_handle);
    //     // else
    //     //     jnt_vel_interface.registerHandle(joint_handle);
    //     // joint_limits_interface::JointLimits limits;
    //     // const bool rosparam_limits_ok = getJointLimits(jd_ptr[i]->joint_name_, nodeHandle_, limits);
    //     // if(jd_ptr[i]->control_mode_ == 0)
    //     // {
    //     //     joint_limits_interface::PositionJointSaturationHandle limit_handle(joint_handle, limits);
    //     //     jnt_pos_limits_interface.registerHandle(limit_handle);
    //     // }
    //     // else
    //     // {
    //     //     joint_limits_interface::VelocityJointSaturationHandle limit_handle(joint_handle, limits);
    //     //     jnt_vel_limits_interface.registerHandle(limit_handle);
    //     // }
    // }
    double* cmd = new double;
    cmd = &jd_ptr[0]->angle_cmd_;
    hardware_interface::JointHandle joint_1_handle(jnt_state_interface.getHandle(jd_ptr[0]->joint_name_), cmd);
    jnt_pos_interface.registerHandle(joint_1_handle);
    joint_limits_interface::JointLimits limits_1;
    const bool rosparam_limits_ok_1 = getJointLimits(jd_ptr[0]->joint_name_, nodeHandle_, limits_1);
    joint_limits_interface::PositionJointSaturationHandle limit_1_handle(joint_1_handle, limits_1);
    jnt_pos_limits_interface.registerHandle(limit_1_handle);
    std::cout<<"jnt_pos_interface.registerHandle "<<rosparam_limits_ok_1<<std::endl;
    registerInterface(&jnt_pos_interface);

    cmd = new double;
    cmd = &jd_ptr[1]->velocity_cmd_;
    hardware_interface::JointHandle joint_2_handle(jnt_state_interface.getHandle(jd_ptr[1]->joint_name_), cmd);
    jnt_vel_interface.registerHandle(joint_2_handle);
    joint_limits_interface::JointLimits limits_2;
    const bool rosparam_limits_ok_2 = getJointLimits(jd_ptr[1]->joint_name_, nodeHandle_, limits_2);
    joint_limits_interface::VelocityJointSaturationHandle limit_2_handle(joint_2_handle, limits_2);
    jnt_vel_limits_interface.registerHandle(limit_2_handle);
    registerInterface(&jnt_vel_interface);
    std::cout<<"jnt_vel_interface.registerHandle "<<rosparam_limits_ok_2<<std::endl;


}

void SwerveDriveInterface::checkCmdLimit(int cmd_indx)
{
    if(jd_ptr[cmd_indx]->velocity_cmd_ > jd_ptr[cmd_indx]->max_velocity_)
        jd_ptr[cmd_indx]->velocity_cmd_ = jd_ptr[cmd_indx]->max_velocity_;
    // if(jd_ptr[cmd_indx]->angle_cmd_ > jd_ptr[cmd_indx]->max_angle_)
    //     jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->max_angle_;
    // if(jd_ptr[cmd_indx]->angle_cmd_ < jd_ptr[cmd_indx]->min_angle_)
    //     jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->min_angle_;
    return;
}

bool SwerveDriveInterface::read()
{
    if(received_states)
    {
        received_states = false;
        return true;
    }
    return false;
}

void SwerveDriveInterface::writeVelocity(ros::Duration period)
{
    maxon_epos2::MotorCmds cmds;
    jnt_vel_limits_interface.enforceLimits(period);
    for (int i=0; i < jd_ptr.size(); i++)
    {
        checkCmdLimit(i);
        cmds.motor_ids.push_back(jd_ptr[i]->id_);
        if(i == 0)
            cmds.cmd_values.push_back(jd_ptr[i]->angle_cmd_ / jd_ptr[i]->gear_ratio_);
        else
            cmds.cmd_values.push_back(jd_ptr[i]->velocity_cmd_ / jd_ptr[i]->gear_ratio_ - jd_ptr[0]->velocity_ * 0.714286);
    }
        motor_cmds_pub_.publish(cmds);
    return;
}

void SwerveDriveInterface::motorStatesCallback(const maxon_epos2::MotorStates::ConstPtr& msg)
{
    for(int i=0; i<msg->motor_states.size(); i++)
    {
        for(int j=0; j<jd_ptr.size(); j++)
        {
            if(msg->motor_states[i].motor_id == jd_ptr[j]->id_)
            {
                
                if(j == 0)
                {
                    jd_ptr[j]->joint_angle_ = msg->motor_states[i].pos * jd_ptr[j]->gear_ratio_;
                    jd_ptr[j]->velocity_ = msg->motor_states[i].vel * jd_ptr[j]->gear_ratio_;
                }
                else
                    jd_ptr[j]->velocity_ = msg->motor_states[i].vel * jd_ptr[j]->gear_ratio_;
                jd_ptr[j]->effort_ = 0;
            }
        }
    }
    received_states = true;
    return;
}
}