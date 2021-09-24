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

#include "maxon_epos2/EposController.hpp"

namespace maxon_epos2 {
EposController::EposController()
{
	//Initialize device:
	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) ROS_ERROR("Device initialization");
	//Start position mode during homing callback function:
	if((epos_device_.startVolicityMode())==MMC_FAILED) ROS_ERROR("Starting velocity mode failed");
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

bool EposController::writeVelocity(int id, double& cmd)
{
	if(epos_device_.setVelocityMust(id, cmd)==MMC_FAILED)
	{
		ROS_ERROR("Seting position failed");
		return false;
	}
	return true;
}

bool EposController::writePosition(int id, double& cmd, double offset)
{
	if(epos_device_.setPositionMust(id, cmd - offset)==MMC_FAILED)
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
}

void EposController::closeDevice(){
	  if((epos_device_.closeDevice()) == MMC_FAILED) ROS_ERROR("Device closing failed");
}

} /* namespace */
