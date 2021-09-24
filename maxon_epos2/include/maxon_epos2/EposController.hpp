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

#include "maxon_epos2/EposCommunication.hpp"

// ROS
#include <ros/ros.h>

#include <std_srvs/Trigger.h>

namespace maxon_epos2 {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class EposController
{
 public:
  EposController();
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  // EposController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~EposController();
  bool deviceOpenedCheck();
  bool read(int id, double& pos, double& vel, double& eff, double offset=0);
  bool readPosition(int id, double& pos, double offset);
  bool writeProfilePosition(int id, double& cmd, double& vel, double offset=0);
  bool writePosition(int id, double& cmd, double offset);
  bool writeVelocity(int id, double& cmd);
  void closeDevice();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  // bool readParameters();
  //! Device object
  EposCommunication epos_device_;

  //! Create variable for publishing motor info
  // maxon_epos2::epos_motor_info motor;

  unsigned short id_list[7] = {1, 2, 3, 4, 5, 6, 7};
  int motors = 7;
};

} /* namespace */
