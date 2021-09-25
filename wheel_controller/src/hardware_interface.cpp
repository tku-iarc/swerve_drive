#include <ros/ros.h>
#include "wheel_controller/hardware_interface.h"

namespace hardware_interface
{

SwerveDriveInterface::SwerveDriveInterface(ros::NodeHandle& nodeHandle, std::vector<JointData*>& joint_data, float sample_rate, std::string control_mode)
    :nodeHandle_(nodeHandle)
{ 
    this->control_mode = (control_mode.compare("position") == 0) ? 0 : 1;
    this->sample_rate = sample_rate;
    jd_ptr = joint_data;
    for (int i=0; i < jd_ptr.size(); i++)
    {   
        hardware_interface::JointStateHandle state_handle(jd_ptr[i]->joint_name_, &jd_ptr[i]->joint_angle_, &jd_ptr[i]->velocity_, &jd_ptr[i]->effort_);
        jnt_state_interface.registerHandle(state_handle);
        
    }
    registerInterface(&jnt_state_interface);
    
    if(this->control_mode == 0)
    {
        initJointInterface(jnt_pos_interface);
    }
    else
    {
        initJointInterface(jnt_vel_interface);
    }
}

SwerveDriveInterface::~SwerveDriveInterface()
{
    //if node is interrupted, close device
    epos_controller.closeDevice();
}

template <typename JointInterface>
void SwerveDriveInterface::initJointInterface(JointInterface& jnt_interface)
{
    for (int i=0; i < jd_ptr.size(); i++)
    {
        double* cmd = new double;
        if(control_mode == 0)
            cmd = &jd_ptr[i]->angle_cmd_;
        else
            cmd = &jd_ptr[i]->velocity_cmd_;
        hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(jd_ptr[i]->joint_name_), cmd);
        jnt_interface.registerHandle(joint_handle);
        joint_limits_interface::JointLimits limits;
        const bool rosparam_limits_ok = getJointLimits(jd_ptr[i]->joint_name_, nodeHandle_, limits);
        if(control_mode == 0)
        {
            joint_limits_interface::PositionJointSaturationHandle limit_handle(joint_handle, limits);
            jnt_pos_limits_interface.registerHandle(limit_handle);
        }
        else
        {
            joint_limits_interface::VelocityJointSaturationHandle limit_handle(joint_handle, limits);
            jnt_vel_limits_interface.registerHandle(limit_handle);
        }
    }
    registerInterface(&jnt_interface);
}

void SwerveDriveInterface::closeDevice()
{
    epos_controller.closeDevice();
    std::cout<<"Device closed!!!!!!!!!!!!"<<std::endl;
    return;
}

void SwerveDriveInterface::checkCmdLimit(int cmd_indx)
{
    if(jd_ptr[cmd_indx]->velocity_cmd_ > jd_ptr[cmd_indx]->max_velocity_)
        jd_ptr[cmd_indx]->velocity_cmd_ = jd_ptr[cmd_indx]->max_velocity_;
    if(jd_ptr[cmd_indx]->angle_cmd_ > jd_ptr[cmd_indx]->max_angle_)
        jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->max_angle_;
    if(jd_ptr[cmd_indx]->angle_cmd_ < jd_ptr[cmd_indx]->min_angle_)
        jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->min_angle_;
    return;
}

bool SwerveDriveInterface::readFake()
{
    for (int i=0; i < jd_ptr.size(); i++)
    {
        jd_ptr[i]->joint_angle_ = jd_ptr[i]->angle_cmd_;
        jd_ptr[i]->velocity_ = 0;
        jd_ptr[i]->effort_ = 0;
    }
    return true;
}

bool SwerveDriveInterface::readPosition()
{
    for (int i=0; i < jd_ptr.size(); i++)
    {
        jd_ptr[i]->velocity_ = jd_ptr[i]->velocity_cmd_;
        jd_ptr[i]->effort_ = 0;
        if(epos_controller.readPosition(jd_ptr[i]->id_, jd_ptr[i]->joint_angle_,  jd_ptr[i]->home_offset_) == false)
        {
            ROS_ERROR("Read Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

bool SwerveDriveInterface::readAll()
{
    for (int i=0; i < jd_ptr.size(); i++)
    {
        if(epos_controller.read(jd_ptr[i]->id_, jd_ptr[i]->joint_angle_, jd_ptr[i]->velocity_, jd_ptr[i]->effort_,  jd_ptr[i]->home_offset_) == false)
        {
            ROS_ERROR("Read Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

bool SwerveDriveInterface::writePosition(ros::Duration period)
{
    if(epos_controller.deviceOpenedCheck() == false)
        return false;
    jnt_pos_limits_interface.enforceLimits(period);
    for (int i=0; i < jd_ptr.size(); i++)
    {
        float move_dis = fabs(jd_ptr[i]->angle_cmd_ - jd_ptr[i]->joint_angle_);
        jd_ptr[i]->velocity_cmd_ = move_dis * sample_rate; // move_dis / (1 / sample_rate)
        checkCmdLimit(i);
        if(epos_controller.writePosition(jd_ptr[i]->id_, jd_ptr[i]->angle_cmd_, jd_ptr[i]->home_offset_) == false)
        {
            ROS_ERROR("Write Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

bool SwerveDriveInterface::writeVelocity(ros::Duration period)
{
    if(epos_controller.deviceOpenedCheck() == false)
        return false;
    jnt_vel_limits_interface.enforceLimits(period);
    for (int i=0; i < jd_ptr.size(); i++)
    {
        checkCmdLimit(i);
        if(epos_controller.writeVelocity(jd_ptr[i]->id_, jd_ptr[i]->velocity_cmd_) == false)
        {
            ROS_ERROR("Write Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

}