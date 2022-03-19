//============================================================================
// Name        : EposController.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2
//				 (subscribers, parameters, timers, etc.).
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_epos2/epos_controller.hpp"

namespace maxon_epos2 {
// EposController::EposController()
// 	:nodeHandle_()
// {
// 	//Initialize device:
// 	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) ROS_ERROR("Device initialization");
// 	//Start position mode during homing callback function:
// 	if((epos_device_.startVolicityMode())==MMC_FAILED) ROS_ERROR("Starting velocity mode failed");
// }

EposController::EposController(ros::NodeHandle& nodeHandle)
	:nodeHandle_(nodeHandle)
{
	std::vector<int> id_list;
    std::vector<int> pos_list;
    std::vector<int> vel_list;
    nodeHandle_.getParam("id_list", id_list);
	nodeHandle_.getParam("pos_list", pos_list);
    nodeHandle_.getParam("vel_list", vel_list);
	swerve_gear_ratio = nodeHandle.param<double>("swerve_gear_ratio", 0.2193);
	epos_device_.setSwerveJointGearRatio(swerve_gear_ratio);
	this->motors = id_list.size();
	for (auto it = id_list.begin(); it != id_list.end(); ++it)
	{
		this->id_list_.push_back(*it);
		cmd.insert(std::pair<unsigned short, double>(*it, 0));
		pos.insert(std::pair<unsigned short, double>(*it, 0));
		vel.insert(std::pair<unsigned short, double>(*it, 0));
		cur.insert(std::pair<unsigned short, double>(*it, 0));
	}
	//Initialize device:
	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) ROS_ERROR("Device initialization");
	if((epos_device_.startVolicityMode(vel_list))==MMC_FAILED) ROS_ERROR("Starting velocity mode failed");
	 
	for(auto it = pos_list.begin(); it != pos_list.end(); ++it)
	{
		if((epos_device_.setHomingParameter(*it, 3000))==MMC_FAILED) ROS_ERROR("setHomingParameter Error");
		if((epos_device_.homing(*it, false))==MMC_FAILED) ROS_ERROR("Homing Error");
	}
	bool homing_done = false;
	while(!homing_done)
	{
		homing_done = true;
		for(int i=0; i<pos_list.size(); i++)
		{
			readVelocity(pos_list[i]);
			double cmd = -1 * vel[pos_list[i]] * 0.1566416;
			writeVelocity(vel_list[i], cmd);
			homing_done = homing_done && (epos_device_.homingSuccess(pos_list[i]) == MMC_SUCCESS);
		}
	}

	//Start position mode during homing callback function:
	if((epos_device_.startPositionMode(pos_list))==MMC_FAILED) ROS_ERROR("Starting position mode failed");	
	for(auto it = pos_list.begin(); it != pos_list.end(); ++it)
	{
		readPosition(*it);
		setMotorCmd(*it, pos[*it]);
	}
	for(auto it = vel_list.begin(); it != vel_list.end(); ++it)
	{
		readVelocity(*it);
		setMotorCmd(*it, vel[*it]);
	}
	motor_cmds_sub_ = nodeHandle_.subscribe("motor_cmds", 16, &EposController::motorCmdsCallback, this);
	motor_states_pub_ = nodeHandle_.advertise<maxon_epos2::MotorStates>("motor_states", 1);
}

EposController::~EposController()
{
}

bool EposController::deviceOpenedCheck()
{
	return epos_device_.deviceOpenedCheck() == MMC_SUCCESS;
}

bool EposController::read(int id, double& pos, double& vel, double& eff, double offset)
{
	double contorller_pos = 0;
	if(epos_device_.getPosition(id, &contorller_pos) == MMC_FAILED)
	{
		ROS_ERROR("Get position failed");
		return false;
	}
	if(epos_device_.getVelocity(id, &vel) == MMC_FAILED)
	{
		ROS_ERROR("Get velocity failed");
		return false;
	}
	pos = contorller_pos + offset;
	eff = 0;
	return true;
}

bool EposController::readPosition(int id, double& pos, double offset)
{
	double contorller_pos = 0;
	if(epos_device_.getPosition(id, &contorller_pos) == MMC_FAILED)
	{
		ROS_ERROR("Get position failed");
		return false;
	}
	pos = contorller_pos + offset;
	return true;
}

bool EposController::readPosition(int id)
{
	if(epos_device_.getPosition(id, &pos[id]) == MMC_FAILED)
	{
		ROS_ERROR("Get position failed");
		return false;
	}
	return true;
}

bool EposController::readVelocity(int id)
{
	if(epos_device_.getVelocity(id, &vel[id]) == MMC_FAILED)
	{
		ROS_ERROR("Get velocity failed");
		return false;
	}
	return true;
}

bool EposController::writeVelocity(int id, double& cmd)
{
	if(epos_device_.setVelocityMust(id, cmd)==MMC_FAILED)
	{
		ROS_ERROR("Seting velocity failed");
		return false;
	}
	return true;
}

bool EposController::writeVelocity(int id)
{
	if(epos_device_.setVelocityMust(id, cmd[id])==MMC_FAILED)
	{
		ROS_ERROR("Seting velocity failed");
		return false;
	}
	return true;
}

bool EposController::writePosition(int id, double& cmd, double offset)
{
	double goal_cmd = cmd - offset;
	if(epos_device_.setPositionMust(id, goal_cmd)==MMC_FAILED)
	{
		ROS_ERROR("Seting position failed");
		return false;
	}
	return true;
}

bool EposController::writePosition(int id)
{
	if(fabs(cmd[id] - pos[id]) > (2 * M_PI / swerve_gear_ratio))
	{
		ROS_WARN("Ignore position command cause too big");
		return true;
	}
	if(epos_device_.setPositionMust(id, cmd[id])==MMC_FAILED)
	{
		ROS_ERROR("Seting position failed");
		return false;
	}
	return true;
}

bool EposController::writeProfilePosition(int id, double& cmd, double& vel, double offset)
{
	if(epos_device_.setPositionProfile(id, vel, 4 * vel, 4 * vel)==MMC_FAILED)
	{
		ROS_ERROR_STREAM("Seting position profile failed, vel = "<<vel);
		return false;
	}
	if(epos_device_.setPosition(id, cmd - offset)==MMC_FAILED)
	{
		ROS_ERROR("Seting position failed");
		return false;
	}
	return true;
}

bool EposController::homeResetCheck(int id)
{
	if(fabs(pos[id]) > SAFE_POS && fabs(vel[id]) < 0.0001)
	{
		epos_device_.resetHomePoseition(id);
		return true;
	}
	return false;
}

void EposController::setMotorCmd(int id, double& cmd)
{
	this->cmd[id] = cmd;
}

void EposController::motorCmdsCallback(const maxon_epos2::MotorCmds::ConstPtr& msg)
{
	for(int i=0; i < msg->motor_ids.size(); i++)
		this->cmd[msg->motor_ids[i]] = msg->cmd_values[i];
}

void EposController::motorStatesPublisher()
{
	maxon_epos2::MotorStates msg;
	for(auto it = id_list_.begin(); it != id_list_.end(); ++it)
	{
		maxon_epos2::MotorState motor_state;
		motor_state.motor_id = *it;
		motor_state.pos = this->pos[*it];
		motor_state.vel = this->vel[*it];
		motor_state.cur = this->cur[*it];
		msg.motor_states.push_back(motor_state);
	}
	motor_states_pub_.publish(msg);
}

void EposController::closeDevice(){
	  if((epos_device_.closeDevice()) == MMC_FAILED) ROS_ERROR("Device closing failed");
}

} /* namespace */
