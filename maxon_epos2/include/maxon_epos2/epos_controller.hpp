//============================================================================
// Name        : EposController.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#pragma once

// STD
#include <string>
#include <cmath>
// #include <memory>


// ROS
#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"

#include "maxon_epos2/epos_communication.hpp"
#include "mobile_base_msgs/msg/motor_cmds.hpp"
#include "mobile_base_msgs/msg/motor_state.hpp"
#include "mobile_base_msgs/msg/motor_states.hpp"


namespace maxon_epos2 {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class EposController : public rclcpp::Node
{
 public:
  // EposController();
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  EposController(std::string& node_name);

  /*!
   * Destructor.
   */
  virtual ~EposController();
  bool deviceOpenedCheck();
  bool read(int id, double& pos, double& vel, double& eff, double offset=0);
  bool readPosition(int id, double& pos, double offset);
  bool readPosition(int id);
  bool readVelocity(int id);
  bool writeProfilePosition(int id, double& cmd, double& vel, double offset=0);
  bool writePosition(int id, double& cmd, double offset);
  bool writePosition(int id);
  bool writeVelocity(int id, double& cmd);
  bool writeVelocity(int id);
  void setMotorCmd(int id, double& cmd);
  void motorStatesPublisher();
  void closeDevice();
  double getCmd(unsigned short id){return cmd[id];}
  double getPos(unsigned short id){return pos[id];}
  double getVel(unsigned short id){return vel[id];}
  double getCur(unsigned short id){return cur[id];}

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  // bool readParameters();
  //! Device object
  EposCommunication epos_device_;
  void motorCmdsCallback(const mobile_base_msgs::msg::MotorCmds::SharedPtr msg);
  void mainLoopCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<mobile_base_msgs::msg::MotorCmds>::SharedPtr motor_cmds_sub_;
  rclcpp::Publisher<mobile_base_msgs::msg::MotorStates>::SharedPtr motor_states_pub_;


  //! Create variable for publishing motor info
  // maxon_epos2::epos_motor_info motor;

  std::map<unsigned short, double> cmd;
  std::map<unsigned short, double> pos;
  std::map<unsigned short, double> vel;
  std::map<unsigned short, double> cur;

  std::vector<unsigned short> id_list_;
  std::vector<unsigned short> pos_list_;
  std::vector<unsigned short> vel_list_;
  int motors;
  double swerve_gear_ratio;
};

} /* namespace */
