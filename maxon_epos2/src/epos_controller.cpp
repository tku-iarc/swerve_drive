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

#include <chrono>
#include "maxon_epos2/epos_controller.hpp"

using namespace std::chrono_literals;

namespace maxon_epos2 {

EposController::EposController(std::string& node_name)
	:Node(node_name)
{
	auto id_list_param = this->get_parameter("id_list").as_integer_array();
	std::vector<unsigned short> id_list(std::begin(id_list_param), std::end(id_list_param));
	id_list_param = this->get_parameter("pos_list").as_integer_array();
	pos_list_ = std::vector<unsigned short>(std::begin(id_list_param), std::end(id_list_param));
    id_list_param = this->get_parameter("vel_list").as_integer_array();
	vel_list_ = std::vector<unsigned short>(std::begin(id_list_param), std::end(id_list_param));

	this->get_parameter_or("swerve_gear_ratio", swerve_gear_ratio, 0.2193);
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
	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(), "Device initialization");
	if((epos_device_.startVolicityMode(vel_list_))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(), "Starting velocity mode failed");
	 
	for(auto it = pos_list_.begin(); it != pos_list_.end(); ++it)
	{
		if((epos_device_.setHomingParameter(*it, 3000))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(), "setHomingParameter Error");
		if((epos_device_.homing(*it, false))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(), "Homing Error");
	}
	bool homing_done = false;
	while(!homing_done)
	{
		homing_done = true;
		for(size_t i=0; i<pos_list_.size(); i++)
		{
			readVelocity(pos_list_[i]);
			double cmd = -1 * vel[pos_list_[i]] * 0.1566416;
			writeVelocity(vel_list_[i], cmd);
			homing_done = homing_done && (epos_device_.homingSuccess(pos_list_[i]) == MMC_SUCCESS);
		}
	}

	//Start position mode during homing callback function:
	if((epos_device_.startPositionMode(pos_list_))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(), "Starting position mode failed");	
	for(auto it = pos_list_.begin(); it != pos_list_.end(); ++it)
	{
		readPosition(*it);
		setMotorCmd(*it, pos[*it]);
	}
	for(auto it = vel_list_.begin(); it != vel_list_.end(); ++it)
	{
		readVelocity(*it);
		setMotorCmd(*it, vel[*it]);
	}
	motor_cmds_sub_ = this->create_subscription<mobile_base_msgs::msg::MotorCmds>(
		"motor_cmds", 16, std::bind(&EposController::motorCmdsCallback, this, std::placeholders::_1));
	motor_states_pub_ = this->create_publisher<mobile_base_msgs::msg::MotorStates>("motor_states", 1);
	timer_ = this->create_wall_timer(500ms, std::bind(&EposController::mainLoopCallback, this));
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
		RCLCPP_ERROR(this->get_logger(), "Get position failed");
		return false;
	}
	if(epos_device_.getVelocity(id, &vel) == MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Get velocity failed");
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
		RCLCPP_ERROR(this->get_logger(), "Get position failed");
		return false;
	}
	pos = contorller_pos + offset;
	return true;
}

bool EposController::readPosition(int id)
{
	if(epos_device_.getPosition(id, &pos[id]) == MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Get position failed");
		return false;
	}
	return true;
}

bool EposController::readVelocity(int id)
{
	if(epos_device_.getVelocity(id, &vel[id]) == MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Get velocity failed");
		return false;
	}
	return true;
}

bool EposController::writeVelocity(int id, double& cmd)
{
	if(epos_device_.setVelocityMust(id, cmd)==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting velocity failed");
		return false;
	}
	return true;
}

bool EposController::writeVelocity(int id)
{
	if(epos_device_.setVelocityMust(id, cmd[id])==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting velocity failed");
		return false;
	}
	return true;
}

bool EposController::writePosition(int id, double& cmd, double offset)
{
	double goal_cmd = cmd - offset;
	if(epos_device_.setPositionMust(id, goal_cmd)==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting position failed");
		return false;
	}
	return true;
}

bool EposController::writePosition(int id)
{
	if(fabs(cmd[id] - pos[id]) > (2 * M_PI / swerve_gear_ratio))
	{
		RCLCPP_WARN(this->get_logger(), "Ignore position command cause too big");
		return true;
	}
	if(epos_device_.setPositionMust(id, cmd[id])==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting position failed");
		return false;
	}
	return true;
}

bool EposController::writeProfilePosition(int id, double& cmd, double& vel, double offset)
{
	if(epos_device_.setPositionProfile(id, vel, 4 * vel, 4 * vel)==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting position profile failed, vel = %f", vel);
		return false;
	}
	if(epos_device_.setPosition(id, cmd - offset)==MMC_FAILED)
	{
		RCLCPP_ERROR(this->get_logger(), "Seting position failed");
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

void EposController::motorCmdsCallback(const mobile_base_msgs::msg::MotorCmds::SharedPtr msg)
{
	for(size_t i=0; i < msg->motor_ids.size(); i++)
		this->cmd[msg->motor_ids[i]] = msg->cmd_values[i];
}

void EposController::motorStatesPublisher()
{
	mobile_base_msgs::msg::MotorStates msg;
	for(auto it = id_list_.begin(); it != id_list_.end(); ++it)
	{
		mobile_base_msgs::msg::MotorState motor_state;
		motor_state.motor_id = *it;
		motor_state.pos = this->pos[*it];
		motor_state.vel = this->vel[*it];
		motor_state.cur = this->cur[*it];
		msg.motor_states.push_back(motor_state);
	}
	motor_states_pub_->publish(msg);
}

void EposController::closeDevice(){
	if((epos_device_.closeDevice()) == MMC_FAILED) 
	  	RCLCPP_ERROR(this->get_logger(), "Device closing failed");
	else
		RCLCPP_INFO(this->get_logger(), "Device closed");
}

void EposController::mainLoopCallback()
{
	if(this->deviceOpenedCheck() == false)
		return;
	for(auto it = pos_list_.begin(); it != pos_list_.end(); ++it)
		this->writePosition(*it);
	for(auto it = pos_list_.begin(); it != pos_list_.end(); ++it)
	{
		this->readPosition(*it);
		this->readVelocity(*it);
		if(this->homeResetCheck(*it))
		{
			this->readPosition(*it);
			this->readVelocity(*it);
		}
	}
	for(size_t i=0; i<vel_list_.size(); i++)
	{
		double cmd = this->getCmd(vel_list_[i]) - (this->getVel(pos_list_[i]) * 0.1566416);
		this->writeVelocity(vel_list_[i], cmd);
		this->setVel(vel_list_[i], cmd);  // this is because velocity given by hall sensor is not credible
	}
	this->motorStatesPublisher();
}

} /* namespace */
