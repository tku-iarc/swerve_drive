//============================================================================
// Name        : EposCommunication.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the communication functions for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================
#include <cmath>
#include "maxon_epos2/epos_communication.hpp"

namespace maxon_epos2 {

EposCommunication::EposCommunication()
{
	g_pKeyHandle = 0; //set adress to zero
	g_pSubKeyHandle = 0;
	g_usNodeId = 1;
	g_baudrate = 0;
	swerve_gear_ratio = 1;
}

EposCommunication::~EposCommunication()
{
}

void EposCommunication::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode, unsigned short p_usNodeId = 0)
{
	std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult <<", ID = " << p_usNodeId << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

void EposCommunication::LogInfo(std::string message)
{
	std::cout << message << std::endl;
}

void EposCommunication::SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		std::cout << "-";
	}
	std::cout << std::endl;
}

void EposCommunication::PrintHeader()
{
	SeparatorLine();

	LogInfo("Initializing EPOS2 Communication Library");

	SeparatorLine();
}

void EposCommunication::PrintSettings()
{
	std::stringstream msg;

	msg << "default settings:" << std::endl;
	msg << "main node id             = " << g_usNodeId << std::endl;
	msg << "device name         = '" << g_deviceName << "'" << std::endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
	msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
	msg << "port name           = '" << g_portName << "'"<< std::endl;
	msg << "baudrate            = " << g_baudrate << std::endl;
	msg << "node id list        = " << g_nodeIdList;

	LogInfo(msg.str());

	SeparatorLine();
}

void EposCommunication::SetDefaultParameters(std::vector<unsigned short> nodeIdList, int motors)
{

	/* Options:
	 * device name: EPOS2, EPOS4, default: EPOS4
	 * protocol stack name: MAXON_RS232, CANopen, MAXON SERIAL V2, default: MAXON SERIAL V2
	 * interface name: RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default: USB
	 * port name: COM1, USB0, CAN0,... default: USB0
	 * baudrate: 115200, 1000000,... default: 1000000
	 */

	//USB
	g_usNodeId = nodeIdList[0];
	g_deviceName = "EPOS2"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_baudrate = 1000000; 
	g_subProtocolStackName = "CANopen";
	// g_usSubNodeId = 2;
	g_motors = motors;
	g_nodeIdList = (unsigned short*) std::calloc(g_motors, sizeof(unsigned short));
	for(int i = 0; i < g_motors; i++)
		g_nodeIdList[i] = nodeIdList[i];

	//get the port name:
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;
	VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), (char*)g_interfaceName.c_str(), lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode);
	g_portName = pPortNameSel;
	LogInfo("Port Name: " + g_portName);

}

int EposCommunication::SetPositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode,
										  unsigned int profile_velocity = 500,
										  unsigned int profile_acceleration = 1000,
										  unsigned int profile_deceleration = 1000)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	int vel_rpm = radsToRpm(profile_velocity);
	if(vel_rpm == 0)
		return lResult;
	if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, vel_rpm, radsToRpm(profile_acceleration), radsToRpm(profile_deceleration), p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetPositionProfile", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::SetHomingParameter(unsigned int* p_pErrorCode)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	unsigned int homing_acceleration = 20;
	unsigned int speed_switch = 50;
	unsigned int speed_index = 50;
	int home_offset = 0;
	unsigned short current_threshold = 0;
	int home_position = 0;
	if(VCS_SetHomingParameter(g_pKeyHandle, g_usNodeId, homing_acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetHomingParameter", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::SetSensor(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_SetSensorType(g_pKeyHandle, g_usNodeId, 1, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetSensorType", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(VCS_SetIncEncoderParameter(g_pKeyHandle, g_usNodeId, 256, 0, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetIncEncoderParameter", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");
	LogInfo(pInterfaceName);

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
		LogError("Opening device failed.", MMC_FAILED, *p_pErrorCode);
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int EposCommunication::OpenSubDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_subProtocolStackName.c_str());

	LogInfo("Open device...");

	g_pSubKeyHandle = VCS_OpenSubDevice(g_pKeyHandle, pDeviceName, pProtocolStackName, p_pErrorCode);

	if(g_pSubKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pSubKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
		{
			if(VCS_SetProtocolStackSettings(g_pSubKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
			{
				if(VCS_GetProtocolStackSettings(g_pSubKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
		LogError("Opening sub device failed.", MMC_FAILED, *p_pErrorCode);
	}

	delete []pDeviceName;
	delete []pProtocolStackName;

	return lResult;
}

int EposCommunication::CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	*p_pErrorCode = 0;

	if(VCS_CloseAllSubDevices(g_pSubKeyHandle, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_CloseAllSubDevices FAILED", lResult, *p_pErrorCode);
	}

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_CloseDevice FAILED", lResult, *p_pErrorCode);
	}

	if(VCS_CloseAllDevices(p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_CloseAllDevices FAILED", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::PrepareEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0; //0 is not in fault state

	if(VCS_GetFaultState(p_DeviceHandle, p_usNodeId, &oIsFault, p_pErrorCode ) == MMC_FAILED)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	LogInfo("Debug 1: FaultState: " + std::to_string(oIsFault));
	if(lResult == MMC_SUCCESS)
	{
		if(oIsFault)
		{
			std::stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult == MMC_SUCCESS)
		{

			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode) == MMC_FAILED)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}


			if(!oIsEnabled)
			{
				if(VCS_SetEnableState(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
				{
					LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
					lResult = MMC_FAILED;
				}
				else{
					VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode);
					LogInfo("SetEnableState should be 1: " + std::to_string(oIsEnabled));
				}
			}
		}
	}
	return lResult;
}

// int EposCommunication::PositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
// {
// 	int lResult = MMC_SUCCESS;

// 	lResult = ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode);

// 	if(lResult != MMC_SUCCESS)
// 	{
// 		LogError("ActivateProfilePositionMode", lResult, *p_pErrorCode);
// 	}

// 	return lResult;
// }

int EposCommunication::HomingMode(unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	lResult = ActivateHomingMode(p_DeviceHandle, p_usNodeId, p_pErrorCode);

	if(lResult != MMC_SUCCESS)
	{
		LogError("ActivateHomingMode", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		LogInfo("VCS_ActivateProfilePositionMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivatePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivatePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivatePositionMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		LogInfo("VCS_ActivatePositionMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivateVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set velocity mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateVelocityMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateVelocityMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		LogInfo("VCS_ActivateVelocityMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivateHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set homing mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::FindHome(unsigned short p_usNodeId, signed char homing_method, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	if(VCS_FindHome(p_DeviceHandle, p_usNodeId, homing_method, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::HomingSuccess(unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	// unsigned int timeout = 6000000; //timeout in ms, should be shorter after testing
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	// if(VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, timeout, p_pErrorCode) == MMC_FAILED)
	// {
	// 	LogError("VCS_WaitForHomingAttained", lResult, *p_pErrorCode);
	// 	lResult = MMC_FAILED;
	// }

	int pHomingAttained;
	int pHomingError;

	if(VCS_GetHomingState(p_DeviceHandle, p_usNodeId, &pHomingAttained, &pHomingError, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetHomingState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return pHomingAttained;
}

int EposCommunication::SetPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long position_setpoint, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	bool absolute = true;

	if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, position_setpoint, absolute, 1, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
		std::cout<<"Fuck Fail_1"<<std::endl;
	}
	else{
		// LogInfo("Movement executed.");
	}

	return lResult;
}

int EposCommunication::PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

int EposCommunication::PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pInterfaceNameSel;

	return lResult;
}

int EposCommunication::PrintDeviceVersion()
{
	int lResult = MMC_FAILED;
	unsigned short usHardwareVersion = 0;
	unsigned short usSoftwareVersion = 0;
	unsigned short usApplicationNumber = 0;
	unsigned short usApplicationVersion = 0;
	unsigned int ulErrorCode = 0;

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
	{
		printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
				g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::PrintAvailableProtocols()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pProtocolNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("protocol stack name = %s\n", pProtocolNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pProtocolNameSel;

	return lResult;
}

int EposCommunication::GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

//public functions:

int EposCommunication::initialization(std::vector<unsigned short> nodeIdList, int motors){
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;

	//Print Header:
	PrintHeader();

	//Set Default Parameters:
	SetDefaultParameters(nodeIdList, motors);

	//Print Settings:
	PrintSettings();

	//Open device:
	if((lResult = OpenDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		deviceOpenedCheckStatus = MMC_FAILED;
	}
	else {
		deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
	}
	
	//Prepare EPOS controller:
	if((lResult = PrepareEpos(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	{
		LogError("PrepareEpos", lResult, ulErrorCode);
	}
	
	if(g_motors > 1)
	{
		if((lResult = OpenSubDevice(&ulErrorCode))==MMC_FAILED)
		{
			LogError("OpenSubDevice", lResult, ulErrorCode);
			deviceOpenedCheckStatus = MMC_FAILED;
		}
		else {
			deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
		}
		for(int i = 1; i < g_motors; i++)
		{
			if((lResult = PrepareEpos(g_pSubKeyHandle, g_nodeIdList[i], &ulErrorCode))==MMC_FAILED)
			{
				LogError("PrepareSubEpos ID = ", lResult, ulErrorCode , g_nodeIdList[i]);
			}
		}
	}

	unsigned int MaxAcceleration = 10000;
	if((lResult = VCS_GetMaxFollowingError(g_pKeyHandle, g_usNodeId, &pMaxFollowingError, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxFollowingError", lResult, ulErrorCode);
	}
	if((lResult = VCS_GetMaxProfileVelocity(g_pKeyHandle, g_usNodeId, &pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxProfileVelocity", lResult, ulErrorCode);
	}
	if((lResult = VCS_SetMaxProfileVelocity(g_pKeyHandle, g_usNodeId, pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_SetMaxProfileVelocity", lResult, ulErrorCode);
	}
	if((lResult = VCS_SetMaxAcceleration(g_pKeyHandle, g_usNodeId, MaxAcceleration, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_SetMaxAcceleration", lResult, ulErrorCode);
	}
	if((lResult = VCS_GetMaxAcceleration(g_pKeyHandle, g_usNodeId, &pMaxAcceleration, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxAcceleration", lResult, ulErrorCode);
	}
	for(int i = 1; i < g_motors; i++)
	{
		if((lResult = VCS_GetMaxFollowingError(g_pSubKeyHandle, g_nodeIdList[i], &pMaxFollowingError, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxFollowingError", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_GetMaxProfileVelocity(g_pSubKeyHandle, g_nodeIdList[i], &pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxProfileVelocity", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_SetMaxAcceleration(g_pSubKeyHandle, g_nodeIdList[i], MaxAcceleration, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_SetMaxAcceleration", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_GetMaxAcceleration(g_pSubKeyHandle, g_nodeIdList[i], &pMaxAcceleration, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxAcceleration", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		std::cout<<"ID: "<<g_nodeIdList[i]<<", pMaxFollowingError: "<<pMaxFollowingError<<", pMaxProfileVelocity: "<<pMaxProfileVelocity<<", pMaxAcceleration: "<<pMaxAcceleration<<std::endl;
	}
	
	LogInfo("Initialization successful");

	return lResult;
}

int EposCommunication::setHomingParameter(unsigned short p_usNodeId, unsigned int p_Velocity)
{
	//to use set variables below first!
	p_Velocity = (p_Velocity > pMaxProfileVelocity) ? pMaxProfileVelocity : p_Velocity;
	int lResult = MMC_SUCCESS;
	unsigned int *ulErrorCode = 0;
	unsigned int homing_acceleration = pMaxAcceleration;
	unsigned int speed_switch = p_Velocity;
	unsigned int speed_index = p_Velocity;
	int home_offset = 0;
	unsigned short current_threshold = 0;
	int home_position = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
	if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, homing_acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position, ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetHomingParameter", lResult, *ulErrorCode);
	}

	return lResult;
}

int EposCommunication::homing(unsigned short p_usNodeId, bool refind)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	signed char homing_method;
	if(refind)
		homing_method = HM_HOME_SWITCH_NEGATIVE_SPEED;
	else
		homing_method = HM_HOME_SWITCH_NEGATIVE_SPEED;

	//Start homing mode:
	if((lResult = HomingMode(p_usNodeId, &ulErrorCode))==MMC_FAILED)
	{
		LogError("HomingMode", lResult, ulErrorCode);
	}

	//Find home:
	if((lResult = FindHome(p_usNodeId, homing_method, &ulErrorCode))==MMC_FAILED)
	{
		LogError("FindHome", lResult, ulErrorCode);
	}
	return lResult;
}

int EposCommunication::homingSuccess(unsigned short p_usNodeId)
{
	// int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	// if((lResult = HomingSuccess(p_usNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("HomingSuccess", lResult, ulErrorCode);
	// }
	return HomingSuccess(p_usNodeId, &ulErrorCode);
}

int EposCommunication::startPositionMode(std::vector<unsigned short> id_list)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	for (std::vector<unsigned short>::iterator it = id_list.begin() ; it != id_list.end(); ++it)
	{
		HANDLE p_DeviceHandle = (*it == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
		if((lResult = ActivatePositionMode(p_DeviceHandle, *it, &ulErrorCode))==MMC_FAILED)
		{
			LogError("ActivatePositionMode", lResult, ulErrorCode, *it);
		}
	}
	return lResult;
}

int EposCommunication::startVolicityMode(std::vector<unsigned short> id_list)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	for (std::vector<unsigned short>::iterator it = id_list.begin() ; it != id_list.end(); ++it)
	{
		HANDLE p_DeviceHandle = (*it == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
		if((lResult = ActivateVelocityMode(p_DeviceHandle, *it, &ulErrorCode))==MMC_FAILED)
		{
			LogError("ActivateVelocityMode Sub", lResult, ulErrorCode, *it);
		}
	}
	return lResult;
}

int EposCommunication::setPositionProfile(unsigned short p_usNodeId, double profile_velocity,
										  double profile_acceleration = 1000,
										  double profile_deceleration = 1000)
{
	unsigned int ulErrorCode = 0;
	int lResult = MMC_SUCCESS;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	int vel_rpm = radsToRpm(profile_velocity);
	if(vel_rpm == 0)
		return lResult;

	if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, vel_rpm, radsToRpm(profile_acceleration), radsToRpm(profile_deceleration), &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetPositionProfile", lResult, ulErrorCode, p_usNodeId);
		std::cout<< "radsToRpm(profile_velocity) = "<< radsToRpm(profile_velocity)<<std::endl;
	}

	return lResult;
}

bool EposCommunication::deviceOpenedCheck()
{
	return deviceOpenedCheckStatus;
}

int EposCommunication::setPosition(unsigned short p_usNodeId, double position_setpoint){
	//Set position, call this function in service callback:
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	//Safety check setpoint and homing:
	if(position_setpoint <= M_PI && position_setpoint >= -1 * M_PI)
	{
		bool absolute = true;

		if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, radsToCounts(position_setpoint), absolute, 1, &ulErrorCode) == MMC_FAILED)
		{
			lResult = MMC_FAILED;
			LogError("VCS_MoveToPosition", lResult, ulErrorCode, p_usNodeId);
		}
		else{
			// LogInfo("Movement executed.");
		}
	}
	return lResult;
}

int EposCommunication::setPositionMust(unsigned short p_usNodeId, double& position_setpoint)
{
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	if(VCS_SetPositionMust(p_DeviceHandle, p_usNodeId, radsToCounts(position_setpoint), &ulErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetPositionMust", lResult, ulErrorCode, p_usNodeId);
		std::cout<<"position_setpoint: "<<position_setpoint<<", to counts: "<<radsToCounts(position_setpoint)<<std::endl;
		lResult = MMC_FAILED;
	}
	else{
		// LogInfo("Movement executed.");
	}

	return lResult;
}

int EposCommunication::setVelocityMust(unsigned short p_usNodeId, double& velocity_setpoint)
{
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle; 
	// long velocity_cmd = (fabs(radsToRpm(velocity_setpoint)) > pMaxProfileVelocity) ? ((velocity_setpoint > 0) - (velocity_setpoint < 0)) * pMaxProfileVelocity : radsToRpm(velocity_setpoint);
	if(VCS_SetVelocityMust(p_DeviceHandle, p_usNodeId, radsToRpm(velocity_setpoint), &ulErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetVelocityMust", lResult, ulErrorCode, p_usNodeId);
		std::cout<<"velocity_setpoint: "<<velocity_setpoint<<", to counts: "<<radsToRpm(velocity_setpoint)<<std::endl;
		lResult = MMC_FAILED;
	}
	else{
		// LogInfo("Movement executed.");
	}

	return lResult;
}

int EposCommunication::getPosition(unsigned short p_usNodeId, double* pPositionIs)
{
	unsigned int ulErrorCode = 0;
	int pPositionIsCounts = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	// if((lResult = GetPosition(&pPositionIsCounts, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("getPosition", lResult, ulErrorCode);
	// 	return lResult;
	// }

	int lResult = MMC_SUCCESS;

	if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &pPositionIsCounts, &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetPositionIs", lResult, ulErrorCode, p_usNodeId);
	}

	*pPositionIs = countsToRads(pPositionIsCounts);

	//only for Debugging
	//LogInfo("!!! pPositionIs: " + std::to_string(*pPositionIs) + " pPositionIsCounts: " + std::to_string(pPositionIsCounts));
	return lResult;
}

int EposCommunication::getVelocity(unsigned short p_usNodeId, double* pVelocityIs)
{
	unsigned int ulErrorCode = 0;
	int pVelocityIsCounts;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
	// if((lResult = GetVelocity(&pVelocityIsCounts, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("getVelocity", lResult, ulErrorCode);
	// 	return lResult;
	// }
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(p_DeviceHandle, p_usNodeId, &pVelocityIsCounts, &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetVelocityIs", lResult, ulErrorCode, p_usNodeId);
	}
	*pVelocityIs = rpmToRads(pVelocityIsCounts);
	return lResult;
}

void EposCommunication::resetHomePoseition(unsigned short p_usNodeId)
{
	int moto_pos = 0;
	unsigned int ulErrorCode = 0;
	int lResult = MMC_SUCCESS;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
	lResult = VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &moto_pos, &ulErrorCode);
	lResult = VCS_SetPositionMust(p_DeviceHandle, p_usNodeId, moto_pos, &ulErrorCode);
	lResult = VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &ulErrorCode);
	int home_pos;
	if(moto_pos > 0)
		home_pos = moto_pos - radsToCounts(2 * M_PI / swerve_gear_ratio);
	else
		home_pos = radsToCounts(2 * M_PI / swerve_gear_ratio) - moto_pos;
	lResult = VCS_DefinePosition(p_DeviceHandle, p_usNodeId, home_pos, &ulErrorCode);
	lResult = VCS_ActivatePositionMode(p_DeviceHandle, p_usNodeId, &ulErrorCode);
	if(lResult == MMC_FAILED)
		LogError("resetHomePoseition Failed", lResult, ulErrorCode, p_usNodeId);
	return;
}

int EposCommunication::closeDevice(){
	//Close device:
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	if((lResult = CloseDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return lResult;
}

double EposCommunication::countsToRads(const int& counts){
	double mm = 2 * M_PI * (counts) / 2048. / (103275.0/3211.0);
	return mm;
}

int EposCommunication::radsToCounts(const double& mm){
	int counts = mm  * 2048 * (103275.0/3211.0) / (2 * M_PI);
	// LogInfo("counts: " + std::to_string(counts));
	return counts;
}

int EposCommunication::radsToRpm(const double& rads)
{
	int rpm;
	rpm = rads * (103275.0/3211.0) * 60 / (2 * M_PI);
	return rpm;
}

double EposCommunication::rpmToRads(const int& rpm)
{
	double rads;
	rads = (rpm) / (103275.0/3211.0) / 60. * (2 * M_PI);
	return rads;
}

	/* workflow:
	 * initialize, setPosition, closeDevice
	 */

} /* namespace */
