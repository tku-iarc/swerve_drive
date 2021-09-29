//============================================================================
// Name        : EposCommunication.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the communication functions for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#pragma once

//Define some parameters:
#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 1
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 0
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif
//--

#ifndef EPOSCOMMUNICATION_H_
#define EPOSCOMMUNICATION_H_
// Include headers of Maxon EPOS library libCmdEpos.so:
#include "maxon_epos2/definitions.h"
#include "maxon_epos2/win_types.h"

#include <iostream>
//STD
#include <string>
#include <sstream>
#include <getopt.h>
#include "stdint.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <unistd.h>
#include <list>
#include <sys/types.h>
#include <sys/times.h>
#include <sys/time.h>

namespace maxon_epos2 {

/*!
 * Class containing the algorithmic part of the package.
 */
class EposCommunication
{
 public:
  /*!
   * Constructor.
   */
  EposCommunication();

  /*!
   * Destructor.
   */
  virtual ~EposCommunication();

  int 	initialization(std::vector<unsigned short> nodeIdList, int motors);
  bool 	deviceOpenedCheck();
  int 	homing();
  int 	startPositionMode(std::vector<int> id_list);
  int   startVolicityMode(std::vector<int> id_list);
  int   setHomingParameter(unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int   setPositionProfile(unsigned short p_usNodeId,
                           double profile_velocity,
										       double profile_acceleration,
										       double profile_deceleration);
  int 	setPosition(unsigned short p_usNodeI, double position_setpoint);
  int   setPositionMust(unsigned short p_usNodeId, double& position_setpoint);
  int   setVelocityMust(unsigned short p_usNodeId, double& velocity_setpoint);
  int 	getPosition(unsigned short p_usNodeI, double* pPositionIs);
  int 	getVelocity(unsigned short p_usNodeI, double* pVelocityIs);
  int 	closeDevice();

 private:
  //define HANDLE
  typedef void* HANDLE;
  typedef int BOOL;

  //Variables:
  bool deviceOpenedCheckStatus = MMC_FAILED;
  bool homingCompletedStatus = MMC_FAILED;
  void* g_pKeyHandle;
  void* g_pSubKeyHandle;
  unsigned short g_usNodeId;
  unsigned short g_usSubNodeId;
  unsigned short *g_nodeIdList;
  unsigned int pMaxFollowingError;
	unsigned int pMaxProfileVelocity;
	unsigned int pMaxAcceleration;
  int g_baudrate;
  int g_motors;
  std::string g_deviceName;
  std::string g_protocolStackName;
  std::string g_subProtocolStackName;
  std::string g_interfaceName;
  std::string g_portName;
  const std::string g_programName = "EPOS2 Controller";
  
  //Functions:
  void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode, unsigned short p_usNodeId);
  void  LogInfo(std::string message);
  void  SeparatorLine();
  void  PrintHeader();
  void  PrintSettings();
  void  SetDefaultParameters(std::vector<unsigned short> nodeIdList , int motors);
  int	  SetPositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode,
                           unsigned int profile_velocity,
										       unsigned int profile_acceleration,
										       unsigned int profile_deceleration);

  int 	SetHomingParameter(unsigned int* p_pErrorCode);
  int	  SetSensor(unsigned int* p_pErrorCode);
  int   OpenDevice(unsigned int* p_pErrorCode);
  int   OpenSubDevice(unsigned int* p_pErrorCode);
  int   CloseDevice(unsigned int* p_pErrorCode);
  int   PrepareEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  // int	  PositionMode(unsigned int* p_pErrorCode);
  int 	HomingMode(unsigned int* p_pErrorCode);
  int	  ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int   ActivatePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int   ActivateVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int 	ActivateHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
  int 	FindHome(unsigned int* p_pErrorCode);
  int 	HomingSuccess(bool* homing_success, unsigned int* p_pErrorCode);
  int	  SetPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long position_setpoint, unsigned int* p_pErrorCode);
  int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
  int   PrintAvailableInterfaces();
  int   PrintDeviceVersion();
  int	  PrintAvailableProtocols();
  int	  GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode);
  int	  GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode);
  double countsToRads(const int& counts);
  int 	radsToCounts(const double& mm);
  int   radsToRpm(const double& rads);
  double rpmToRads(const int& rpm);


}; /* Class */


#endif /* EPOSCOMMUNICATION_H_ */

} /* namespace */
