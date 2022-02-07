/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include "../include/SerRelayBoard.h"


//-----------------------------------------------


#define NUM_BYTE_SEND_20CHAR_LCD 50
#define NUM_BYTE_SEND_60CHAR_LCD 79
#define NUM_BYTE_SEND_RELAYBOARD_1_4 88
#define NUM_BYTE_REC_RELAYBOARD_1_4 124

#define RS422_BAUDRATE 420000
#define RS422_RX_BUFFERSIZE 1024
#define RS422_TX_BUFFERSIZE 1024

#define RS422_TIMEOUT 0.025

#define NUM_BYTE_REC_MAX 120
#define NUM_BYTE_REC_HEADER 4
#define NUM_BYTE_REC_CHECKSUM 2
#define NUM_BYTE_REC 104

#define NEO_PI 3.14159265358979323846
#define DEG2RAD(x) NEO_PI/180 * x


//#define OUTPUTERROR(...)

SerRelayBoard::SerRelayBoard()
{
	autoSendRequest = false;	//requests are being send when calling sendRequest();
	m_iFoundMotors =  m_iHomedMotors = m_iFoundExtHardware = m_iConfigured = 0;
	m_iNumBytesSend = 0;
	m_iNumBytesRec = 0;
	m_ihasRelayData = 0;
	m_ihas_LCD_DATA = 0;
	m_iHasIOBoard = 0;
	m_iHasUSBoard = 0;
	m_iHasSpeakerData = 0;
	m_iChargeState = 0;

	std::cerr << "starting serrelayboard node\n";

}

/*SerRelayBoard::SerRelayBoard(int iTypeLCD, std::string pathToConf)
{
	autoSendRequest = true;		//requests are being send when calling setWheelVel(...);
	setStartValues(iTypeLCD); //TODO: backwards compatibility
}
*/

void SerRelayBoard::readConfig(	int iTypeLCD,std::string pathToConf, std::string sNumComPort, 	int hasMotorRight, 
				int hasMotorLeft, int hasMotorRearRight, int hasMotorRearLeft, 
				int hasIOBoard, int hasUSBoard, int hasRadarBoard, int hasGyroBoard, 
				double quickfix1, double quickfix2, double quickfix3, double quickfix4, 
				DriveParam driveParamLeft, DriveParam driveParamRight,
				DriveParam driveParamRearLeft, DriveParam driveParamRearRight
			)
{
	int i, iHasBoard, iHasMotorRight, iHasMotorLeft;
	m_bComInit = false;

	//RelayBoard
	m_cSoftEMStop = 0;
	resetEMStop();
	iHasBoard = 0;
	m_iRelBoardBattVoltage = 0;
	m_iConfigRelayBoard = 0;
	m_iRelBoardKeyPad = 0xFFFF;

	m_iTypeLCD = iTypeLCD;
	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		m_iNumBytesSend = NUM_BYTE_SEND_20CHAR_LCD;
	}
	else if (m_iTypeLCD == LCD_60CHAR_TEXT)
	{
		m_iNumBytesSend = NUM_BYTE_SEND_60CHAR_LCD;
	}
	else if (m_iTypeLCD == RELAY_BOARD_1_4)
	{
		m_iNumBytesSend = NUM_BYTE_SEND_RELAYBOARD_1_4;
	}
	else if (m_iTypeLCD == RELAY_BOARD_2)
	{
		//type relayboard 2 only
		m_iNumBytesSend = 4; //4Byte Header
		m_iNumBytesRec = 4; //4Byte Header

		m_iNumBytesSend += 2; //2Byte Data Mask, Soft EMStop
		m_iNumBytesRec += 13; //11Byte ActRelayboardConfig, State, Charging Current, Charging State, Batt Voltage, Keypad, Temp

		m_iNumBytesSend += 0; //(4*4); //Send data for 4 Motors (4 Byte pro active Motor)
		m_iNumBytesRec += 0; //10 Byte pro active Motor

		if(hasIOBoard == 1)
		{
			m_iNumBytesSend += 2;
			m_iNumBytesRec += 20;
		}
		if(hasUSBoard == 1)
		{
			m_iNumBytesSend += 2;
			m_iNumBytesRec += 26;
		}

		m_iNumBytesSend += 2; //2Byte Checksum
		m_iNumBytesRec += 2; //2Byte Checksum	
	}

	m_sNumComPort = sNumComPort;

	iHasMotorRight = hasMotorRight;
	iHasMotorLeft = hasMotorLeft;
	int iHasMotorRearRight = hasMotorRearRight;
	int iHasMotorRearLeft = hasMotorRearLeft;
	if( 	(iHasMotorRight != 0) || (iHasMotorLeft != 0) && 
		(iHasMotorRearRight == 0) && (iHasMotorRearLeft == 0))
	{
		m_iConfigRelayBoard |= CONFIG_HAS_DRIVES;
	}

	if( (iHasMotorRight != 0) && (iHasMotorLeft != 0) && 
		(iHasMotorRearRight != 0) && (iHasMotorRearLeft != 0))
	{
		m_iConfigRelayBoard |= CONFIG_HAS_4_DRIVES;
	}	

	iHasBoard = hasIOBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_IOBOARD;
	}

	iHasBoard = hasUSBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_USBOARD;
	}
	
	iHasBoard = hasRadarBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_RADARBOARD1;
		m_iConfigRelayBoard |= CONFIG_HAS_RADARBOARD2;
	}

	iHasBoard = hasGyroBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_GYROBOARD;
	}

	m_iCmdRelayBoard = 0;
	m_iRelBoardStatus = 0;

	// IOBoard
	m_iIOBoardBattVoltage = 0;
	m_iIOBoardDigIn = 0;
	m_iIOBoardDigOut = 0;
	for(i = 0; i < 8; i++) { m_iIOBoardAnalogIn[i] = 0; }



	// drive parameters
	
	m_DriveParamLeft = driveParamLeft;
	m_DriveParamRight = driveParamRight;
	m_DriveParamRearLeft = driveParamRearLeft;
	m_DriveParamRearRight = driveParamRearRight;

	//
	m_iVelCmdMotRightEncS = 0;
	m_iVelCmdMotLeftEncS = 0;
	m_iPosMeasMotRightEnc = 0;
	m_iVelMeasMotRightEncS = 0;
	m_iPosMeasMotLeftEnc = 0;
	m_iVelMeasMotLeftEncS = 0;


	m_iVelCmdMotRearRightEncS = 0;
	m_iVelCmdMotRearLeftEncS = 0;
	m_iPosMeasMotRearRightEnc = 0;
	m_iVelMeasMotRearRightEncS = 0;
	m_iPosMeasMotRearLeftEnc = 0;
	m_iVelMeasMotRearLeftEncS = 0;

	quickFix[0] = quickfix1;
	quickFix[1] = quickfix2;
	quickFix[2] = quickfix3;
	quickFix[3] = quickfix4;

	// GyroBoard
	m_bGyroBoardZeroGyro = false;
	m_iGyroBoardAng = 0;
	for(i = 0; i < 3; i++) { m_iGyroBoardAcc[i]; }
	
	// RadarBoard
	for(i = 0; i < 4; i++) { m_iRadarBoardVel[i] = 0; }
	
	// USBoard
	m_iUSBoardSensorActive = 0xFFFF;

	for(i = 0; i < 16; i++) { m_iUSBoardSensorData[i] = 0; }
	for(i = 0; i < 4; i++) { m_iUSBoardAnalogData[i] = 0; }

	m_dLastPosRight = 0;
	m_dLastPosLeft = 0;

	for(i = 0; i < 60; i++) { m_cTextDisplay[i] = 0; }
}





//-----------------------------------------------
SerRelayBoard::~SerRelayBoard()
{
	m_SerIO.closeIO();
}

//-----------------------------------------------
void SerRelayBoard::readConfiguration()
{

}

//-----------------------------------------------
int SerRelayBoard::evalRxBuffer()
{
	if(m_iTypeLCD == RELAY_BOARD_2)
	{
		evalRxBufferRelayBoard2();	
		return 0;
	}
	else
	{

		int errorFlag = NO_ERROR;
		static int siNoMsgCnt = 0;

		double dDltT;
	
		int iNumByteRec = NUM_BYTE_REC;
		if(m_iTypeLCD == RELAY_BOARD_1_4)
		{
			iNumByteRec = NUM_BYTE_REC_RELAYBOARD_1_4;
		}

		const int c_iNrBytesMin = NUM_BYTE_REC_HEADER + iNumByteRec + NUM_BYTE_REC_CHECKSUM;
		const int c_iSizeBuffer = 4096;

		int i;
		int iNrBytesInQueue, iNrBytesRead, iDataStart;
		unsigned char cDat[c_iSizeBuffer];
		unsigned char cTest[4] = {0x02, 0x80, 0xD6, 0x02};

		//m_CurrTime = ros::Time::now();
		//dDltT = m_CurrTime.toSec()- m_LastTime.toSec();

		if( !m_bComInit ) return 0;

		//enough data in queue?
		iNrBytesInQueue = m_SerIO.getSizeRXQueue();

		if(iNrBytesInQueue < c_iNrBytesMin)
		{
			siNoMsgCnt++;
			if(siNoMsgCnt > 29)
			{
				siNoMsgCnt = 0;
				errorFlag = NO_MESSAGES;
			}  else errorFlag = TOO_LESS_BYTES_IN_QUEUE;


			return errorFlag;
		}
		else
		{
			siNoMsgCnt = 0;
		}

		// search most recent data from back of queue
		iNrBytesRead = m_SerIO.readBlocking((char*)&cDat[0], iNrBytesInQueue);

		//log
		if(logging == true)
		{
			log_to_file(2, cDat); //direction 1 = transmitted; 2 = recived
		}	

		for(i = (iNrBytesRead - c_iNrBytesMin); i >= 0 ; i--)
		{
			//try to find start bytes
			if((cDat[i] == cTest[0]) && (cDat[i+1] == cTest[1]) && (cDat[i+2] == cTest[2]) && (cDat[i+3] == cTest[3]))
			{
				iDataStart = i + 4;

				// checksum ok?
				if( convRecMsgToData(&cDat[iDataStart]) )
				{
					return errorFlag;
				}
				else
				{
					errorFlag = CHECKSUM_ERROR;
					return errorFlag;
				}
			}
		}

		return errorFlag;
	}
}

//----------------------------------------------
int SerRelayBoard::evalRxBufferRelayBoard2()
{
	int errorFlag = NO_ERROR;

	if( !m_bComInit ) return 0;

	bool found_header = false;
	int msg_type = 100;
	int error_cnt = 0;
	int no_data_cnt = 0;
	int BytesToRead = 0;
	int received_checksum = 0;
	int my_checksum = 0;
	const int c_iSizeBuffer = 130;//4096;
	int iNrBytesRead = 0;
	unsigned char cDat[c_iSizeBuffer];
	unsigned char cHeader[4] = {0x00, 0x00, 0x00, 0x00};

	while(found_header == false)
	{
		//std::cout << "warte auf daten " << std::endl; //JNN
		if(m_SerIO.getSizeRXQueue() >= 1)
		{
			//std::cout << "In der schleife: " << std::endl; //JNN
			//Read Header
			cHeader[3] = cHeader[2];
			cHeader[2] = cHeader[1];
			cHeader[1] = cHeader[0];
			iNrBytesRead = m_SerIO.readBlocking((char*)&cHeader[0], 1);
			//std::cout << "Header: " << (int)cHeader[0] << "----" << (int)cHeader[1] << "----" << (int)cHeader[2] << "----" << (int)cHeader[3] << "----" << std::endl; //JNN
			if((cHeader[3] == 0x08) && (cHeader[2] == 0xFE) && (cHeader[1] == 0xEF) && (cHeader[0] == 0x08))
			{
				//Update Msg
				msg_type = 1;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0x80) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Config Msg
				msg_type = 2;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0xFF) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Error Msg
				msg_type = 3;
				found_header = true;
			}
			if(++error_cnt > 20)
			{
				//No Header in first 20 Bytes -> Error
				//flush input
				return 99;
			}

		}
	}
	switch(msg_type)
	{
		case 1: BytesToRead = m_iNumBytesRec - 4;
				break;
		case 2: BytesToRead = 6;
				break;
		case 3: BytesToRead = 3;
				break;
		default: return 98;
	}
	error_cnt = 0;

	//std::cout << " daten erwartet: " << BytesToRead << std::endl; //JNN
	while(m_SerIO.getSizeRXQueue() < BytesToRead)
	{
		usleep(2000);
		//std::cout << " Queue Size: " << m_SerIO.getSizeRXQueue() << std::endl; //JNN
	}
	iNrBytesRead = m_SerIO.readBlocking((char*)&cDat[0], BytesToRead);
	//std::cout << " anz. empf. daten: " << iNrBytesRead << std::endl; //JNN

	//Calc Checksum
	//my_checksum %= 0xFF00;
	my_checksum += cHeader[3];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[2];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[1];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[0];
	for(int e = 0; e < iNrBytesRead - 2; e++)
	{
		my_checksum %= 0xFF00;
		my_checksum += cDat[e];
	}
	//received checksum
	received_checksum = (cDat[BytesToRead - 1] << 8);
	received_checksum += cDat[BytesToRead - 2];
	//std::cout << "empfangene Checksumme: " << received_checksum << std::endl; //JNN
	if(received_checksum != my_checksum)
	{
		//Wrong Checksum
		return CHECKSUM_ERROR;
	}
	if(msg_type == 1)
	{
		convRecMsgToDataRelayBoard2(&cDat[0]);
		return NO_ERROR;
	}
	else if(msg_type == 2)
	{
		m_iFoundMotors = cDat[0];
		m_iHomedMotors = cDat[1];
		m_iFoundExtHardware = cDat[2];
		m_iConfigured = cDat[3];
		return MSG_CONFIG;
	}
	else if(msg_type == 3)
	{
		ROS_INFO("ERROR evalrxBuffer: ");
		int iLastError = cDat[0];
		
		return GENERAL_SENDING_ERROR;
	}
	else
	{
		return GENERAL_SENDING_ERROR;
	}

	return 0;

}

//-----------------------------------------------
bool SerRelayBoard::init() 
{ 
	return initPltf();
};

//------------------------------------------------
bool SerRelayBoard::initPltf()
{
	int iRet;
	m_SerIO.setBaudRate(RS422_BAUDRATE);
	m_SerIO.setDeviceName( m_sNumComPort.c_str() );
	m_SerIO.setBufferSize(RS422_RX_BUFFERSIZE, RS422_TX_BUFFERSIZE);
	m_SerIO.setTimeout(RS422_TIMEOUT);
	iRet = m_SerIO.openIO();
	if (iRet != 0)
	{
		m_bComInit = false;
		return false;	
	}

	m_bComInit = true;

	if(m_iTypeLCD == RELAY_BOARD_2)
	{
		initRelayBoard2();
	}
	else
	{
		m_iCmdRelayBoard |= CMD_RESET_POS_CNT;
	}

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::initRelayBoard2()
{
	unsigned char cConfig_Data[33]; //4 Byte Header 3 Config Bytes 24 Byte Modulo 2 Byte Checksum JNN
	unsigned char cExtHardware = 0;
	int iChkSum = 0;
	int byteswritten = 0;
	int answertype = 0;
	//Header
	cConfig_Data[0] = 0x02;
	cConfig_Data[1] = 0x80;
	cConfig_Data[2] = 0xD6;
	cConfig_Data[3] = 0x02;
	//configuration Bytes
	cConfig_Data[4] = 0;	// 2 fake - motors
	cConfig_Data[5] = 0;	// do not home motors
	cConfig_Data[6] = cExtHardware;	//no ext hardware

	//---------------Modulo for all Motors - char 7 - 30--------------------------->
	for (int i = 7; i < 31; i++)
	{
		cConfig_Data[i] = 0;
	}
	
	//----------------Calc Checksum------------------------------------->
	for(int i=4;i<=30;i++)    //r=4 => Header not added to Checksum
	{
		iChkSum %= 0xFF00;
		iChkSum += cConfig_Data[i];
	}
	//----------------END Calc Checksum--------------------------------->

	//----------------Add Checksum to Data------------------------------>
	cConfig_Data[31] = iChkSum >> 8;
	cConfig_Data[32] = iChkSum;
	//------------END Add Checksum to Data------------------------------>

	byteswritten = m_SerIO.writeIO((char*)cConfig_Data,33);
	if(byteswritten != 33)
	{
		//------Log Error here -------------------
		std::cerr << "Config not sent. \n";
		return false;
	}
	else
	{
		std::cerr << "Relayboard config sent. \n";
	}

	answertype = evalRxBufferRelayBoard2();

	if(answertype == 2)
	{
		//Check received Data
		if(m_iConfigured == 1)
		{
			//Configuration Failed
			std::cerr << "RelayBoard Configuration ok. ";
			return true;
			//------Log Error here -------------------
		}
		else
		{
			std::cerr << "RelayBoard Configuration FAILED. ";
			return -1;
			//------Log Error here -------------------
		}

	}
	

}

//-----------------------------------------------
bool SerRelayBoard::reset(){
	return resetPltf();
}

//-----------------------------------------
bool SerRelayBoard::resetPltf()
{
	m_SerIO.closeIO();
	m_bComInit = false;

	init();
	return true;
}

//-----------------------------------------------
bool SerRelayBoard::shutdown()
{
	return shutdownPltf();
}


bool SerRelayBoard::shutdownPltf()
{
	m_SerIO.closeIO();
	m_bComInit = false;
	return true;
}

//-----------------------------------------------
bool SerRelayBoard::isComError()
{
	return false;	
}

//-----------------------------------------------
bool SerRelayBoard::isDriveError()
{
	return false;	
}

//-----------------------------------------------
bool SerRelayBoard::isEMStop()
{
	if( (m_iRelBoardStatus & 0x0001) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------
bool SerRelayBoard::isScannerStop()
{
	if( (m_iRelBoardStatus & 0x0002) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------
void SerRelayBoard::sendNetStartCanOpen()
{

}

//-------------------------------------------------
int SerRelayBoard::sendRequest()
{
	if(!autoSendRequest){
		int errorFlag = NO_ERROR;
		int iNrBytesWritten;

		unsigned char cMsg[m_iNumBytesSend+50];
	
		m_Mutex.lock();
	
			if(m_iTypeLCD == RELAY_BOARD_2)
			{
				convDataToSendMsgRelayBoard2(cMsg);			
			}
			else
			{
				convDataToSendMsg(cMsg);
			}

			m_SerIO.purgeTx();
			iNrBytesWritten = m_SerIO.writeIO((char*)cMsg,m_iNumBytesSend);
	
			if(iNrBytesWritten < m_iNumBytesSend) {
				errorFlag = GENERAL_SENDING_ERROR;
			}
			//log
			if(logging == true)
			{
				log_to_file(1, cMsg); //direction 1 = transmitted; 2 = recived
			}
			//m_LastTime = ros::Time::now();
		m_Mutex.unlock();
		return errorFlag;
	}
	else {
		std::cerr << "You are running the depreced mode for backward compability, that's why sendRequest is being handled by setWheelVel()\n";
		return 0;
	}
};


//-----------------------------------------------
// MotCtrlBoard
//-----------------------------------------------

//-----------------------------------------------
int SerRelayBoard::disableBrake(int iCanIdent, bool bDisabled)
{
	return 0;
}

//-----------------------------------------------
int SerRelayBoard::setWheelVel(int iCanIdent, double dVelWheel, bool bQuickStop)
{		
	//ROS_INFO("set wheel called %d ", iCanIdent);

	static bool bDataMotRight = false;
	static bool bDataMotLeft = false;
	static bool bDataMotRearRight = false;
	static bool bDataMotRearLeft = false;

	bool bVelLimited = false;
	int iNrBytesWritten;

	unsigned char cMsg[NUM_BYTE_SEND_RELAYBOARD_1_4];

	/*if(m_iTypeLCD != RELAY_BOARD_1_4)
	{
		//set rear data available for 2-drive-robot
		bDataMotRearRight = true;
		bDataMotRearLeft = true;

	}*/
	
	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		m_iVelCmdMotRightEncS = m_DriveParamRight.getSign()	*
			m_DriveParamRight.convRadSToIncrPerPeriod(dVelWheel);

/* TODO:
		if(MathSup::limit(&m_iVelCmdMotRightEncS, (int)m_DriveParamRight.getVelMax()) != 0)
		{
			LOGALERT("vel right limited");
		}
*/
		bDataMotRight = true;
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		m_iVelCmdMotLeftEncS = m_DriveParamLeft.getSign()	*
			m_DriveParamLeft.convRadSToIncrPerPeriod(dVelWheel);


/* TODO:
		if(MathSup::limit(&m_iVelCmdMotLeftEncS, (int)m_DriveParamLeft.getVelMax()) != 0)
		{
			LOGALERT("vel left limited");
		}
*/
	
		bDataMotLeft = true;
	}

	if(iCanIdent == CANNODE_MOTORREARRIGHT)
	{
		m_iVelCmdMotRearRightEncS = (int)(m_DriveParamRearRight.getSign()	*
			m_DriveParamRearRight.convRadSToIncrPerPeriod(dVelWheel));

/*		if(MathSup::limit(&m_iVelCmdMotRearRightEncS, (int)m_DriveParamRearRight.getVelMax()) != 0)
		{
			LOGALERT("vel rearright limited");
		}
*/
		bDataMotRearRight = true;
	}

	if(iCanIdent == CANNODE_MOTORREARLEFT)
	{
		m_iVelCmdMotRearLeftEncS = (int)(m_DriveParamRearLeft.getSign()	*
			m_DriveParamRearLeft.convRadSToIncrPerPeriod(dVelWheel));

/*		if(MathSup::limit(&m_iVelCmdMotRearLeftEncS, (int)m_DriveParamRearLeft.getVelMax()) != 0)
		{
			LOGALERT("vel rearleft limited");
		}
*/	
		bDataMotRearLeft = true;
	}

	/*if(bDataMotRight && bDataMotLeft && bDataMotRearLeft && bDataMotRearRight)
	{

		//ROS_INFO("sending data to relayboard ");
		bDataMotRight = bDataMotLeft = bDataMotRearRight = bDataMotRearLeft = false; 

		if(bQuickStop)
		{
			m_iCmdRelayBoard |= CMD_QUICK_STOP;
		}
		else
		{
			m_iCmdRelayBoard &= ~CMD_QUICK_STOP;
		}
	
		//data will be sent with autoupdate after calling setWheel with every drive
		/*
		convDataToSendMsg(cMsg);

		m_SerIO.purgeTx();

		// send data
		iNrBytesWritten = m_SerIO.write((char*)cMsg, m_iNumBytesSend);
		if(iNrBytesWritten < m_iNumBytesSend)
		{
			//LOGALERT("only " << iNrBytesWritten << " bytes written to socket");
		}
	

	}*/


	m_Mutex.unlock();

	return iNrBytesWritten;
}
//-----------------------------------------------
int SerRelayBoard::setWheelPosVel(int iCanIdent, double dPos, double dVel, bool bQuickStop)
{
	return 0;
}

//-----------------------------------------------
int SerRelayBoard::requestMotPosVel(int iCanIdent)
{
	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestDriveStatus()
{

}

//-----------------------------------------------
int SerRelayBoard::getWheelPosVel(	int iCanIdent, double* pdAngWheel, double* pdVelWheel)
{
	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		*pdAngWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrToRad(m_iPosMeasMotRightEnc) * quickFix[0];	
		*pdVelWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrPerPeriodToRadS(m_iVelMeasMotRightEncS) * quickFix[0];
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		*pdAngWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrToRad(m_iPosMeasMotLeftEnc) * quickFix[1];	
		*pdVelWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrPerPeriodToRadS(m_iVelMeasMotLeftEncS) * quickFix[1];
	}
	if(iCanIdent == CANNODE_MOTORREARRIGHT)
	{
		*pdAngWheel = m_DriveParamRearRight.getSign() * m_DriveParamRearRight.convIncrToRad(m_iPosMeasMotRearRightEnc) * quickFix[2];	
		*pdVelWheel = m_DriveParamRearRight.getSign() * m_DriveParamRearRight.convIncrPerPeriodToRadS(m_iVelMeasMotRearRightEncS) * quickFix[2];
	}

	if(iCanIdent == CANNODE_MOTORREARLEFT)
	{
		*pdAngWheel = m_DriveParamRearLeft.getSign() * m_DriveParamRearLeft.convIncrToRad(m_iPosMeasMotRearLeftEnc) * quickFix[3];	
		*pdVelWheel = m_DriveParamRearLeft.getSign() * m_DriveParamRearLeft.convIncrPerPeriodToRadS(m_iVelMeasMotRearLeftEncS) * quickFix[3];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
int SerRelayBoard::getWheelDltPosVel(	int iCanIdent, double* pdDltAng, double* pdVelWheel)
{
	double dCurrPos;

	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		dCurrPos = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrToRad(m_iPosMeasMotRightEnc) * quickFix[0];
		
		*pdDltAng = dCurrPos - m_dLastPosRight;
		m_dLastPosRight = dCurrPos;

		*pdVelWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrPerPeriodToRadS(m_iVelMeasMotRightEncS) * quickFix[0];
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		dCurrPos = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrToRad(m_iPosMeasMotLeftEnc) * quickFix[1];

		*pdDltAng = dCurrPos - m_dLastPosLeft;
		m_dLastPosLeft = dCurrPos;

		*pdVelWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrPerPeriodToRadS(m_iVelMeasMotLeftEncS) * quickFix[1];
	}

	if(iCanIdent == CANNODE_MOTORREARRIGHT)
	{
		dCurrPos = m_DriveParamRearRight.getSign() * m_DriveParamRearRight.convIncrToRad(m_iPosMeasMotRearRightEnc) * quickFix[2];
		
		*pdDltAng = dCurrPos - m_dLastPosRearRight;
		m_dLastPosRearRight = dCurrPos;	

		*pdVelWheel = m_DriveParamRearRight.getSign() * m_DriveParamRearRight.convIncrPerPeriodToRadS(m_iVelMeasMotRearRightEncS) * quickFix[2];
	}

	if(iCanIdent == CANNODE_MOTORREARLEFT)
	{
		dCurrPos = m_DriveParamRearLeft.getSign() * m_DriveParamRearLeft.convIncrToRad(m_iPosMeasMotRearLeftEnc) * quickFix[3];	

		*pdDltAng = dCurrPos - m_dLastPosRearLeft;
		m_dLastPosRearLeft = dCurrPos;

		*pdVelWheel = m_DriveParamRearLeft.getSign() * m_DriveParamRearLeft.convIncrPerPeriodToRadS(m_iVelMeasMotRearLeftEncS) *  quickFix[3];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::getStatus(int iCanIdent, int* piStatus, int* piTempCel)
{
	switch(iCanIdent)
	{
	case CANNODE_MOTORLEFT:
		*piStatus = m_iMotLeftStatus;
		break;
	case CANNODE_MOTORRIGHT:
		*piStatus = m_iMotRightStatus;
		break;
	case CANNODE_MOTORREARLEFT:
		*piStatus = m_iMotRearLeftStatus;
		break;
	case CANNODE_MOTORREARRIGHT:
		*piStatus = m_iMotRearRightStatus;
		break;
	default:
		std::cout<<"Canident of drive unknown: " << iCanIdent<<std::endl;

	}

	*piTempCel = 0;
}

//-----------------------------------------------
int SerRelayBoard::execHoming(int CanIdent)
{
	// NOT IMPLEMENTED !!!!
	return 0;
}

//-----------------------------------------------
// RelayBoard
//-----------------------------------------------


int SerRelayBoard::setRelayBoardDigOut(int iChannel, bool bOn)
{
	switch( iChannel)
	{
	case 0:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_CHARGE_RELAY; }
		else { m_iCmdRelayBoard &= ~CMD_SET_CHARGE_RELAY; }
		
		break;

	case 1:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY1; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY1; }

		break;

	case 2:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY2; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY2; }

		break;

	case 3:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY3; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY3; }

		break;

	case 4:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY4; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY4; }

		break;

	case 5:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY5; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY5; }

		break;

	case 6:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY6; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY6; }

		break;

	default:

		return -1;
	}
	
	return 0;
}
//-----------------------------------------------
int SerRelayBoard::getRelayBoardAnalogIn(int* piAnalogIn)
{
	piAnalogIn[0] = m_iChargeCurrent;
	piAnalogIn[1] = m_iRelBoardBattVoltage;
	piAnalogIn[2] = m_iRelBoardTempSensor;
	piAnalogIn[3] = m_iRelBoardKeyPad;
	piAnalogIn[4] = m_iRelBoardIRSensor[0];
	piAnalogIn[5] = m_iRelBoardIRSensor[1];
	piAnalogIn[6] = m_iRelBoardIRSensor[2];
	piAnalogIn[7] = m_iRelBoardIRSensor[3];

	return 0;
}

//-----------------------------------------------
int SerRelayBoard::getRelayBoardDigIn()
{
	//is transmitted in keypaddata
	//m_iRelBoardKeyPad & 
	if((~m_iRelBoardKeyPad) & 0x20)
		//first digin 31
		return 1;
	if((~m_iRelBoardKeyPad) & 0x10)
		//second digin 47
		return 2;
	else
		return 0;

}

//-----------------------------------------------
// IOBoard
//-----------------------------------------------

//-----------------------------------------------
void SerRelayBoard::requestIOBoardData()
{
}

//-----------------------------------------------
void SerRelayBoard::requestIOBoardAnalogIn()
{
}

//-----------------------------------------------
void SerRelayBoard::getIOBoardJoyValWheelMean(double* pdVelWheelLeftMean, double* pdVelWheelRightMean)
{
	*pdVelWheelLeftMean = 0;
	*pdVelWheelRightMean = 0;
}

//-----------------------------------------------
void SerRelayBoard::getIOBoardJoyValNorm(double* pdJoyXNorm, double* pdJoyYNorm)
{
	*pdJoyXNorm = 0;
	*pdJoyYNorm = 0;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardBattVoltage()
{
	return m_iIOBoardBattVoltage;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardDigIn()
{
	return m_iIOBoardDigIn;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardDigOut()
{
	return m_iIOBoardDigOut;
}

//-----------------------------------------------
int SerRelayBoard::setIOBoardDigOut(int iChannel, bool bVal)
{
	int iMask;

	iMask = (1 << iChannel);
	
	if(bVal)
	{
		m_iIOBoardDigOut |= iMask;
	}
	else
	{
		m_iIOBoardDigOut &= ~iMask;
	}
	
	return 0;	
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piAnalogIn[i] = m_iIOBoardAnalogIn[i];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::writeIOBoardLCD(int iLine, int iColumn, const std::string& sText)
{
	int i, iSize;

	iSize = sText.size();

	protocol_version = m_iTypeLCD;
	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		for(i = 0; i < 20; i++)
		{
			if(i < iSize)
			{
				m_cTextDisplay[i] = sText[i];
			}
			else
			{
				m_cTextDisplay[i] = 0;
			}
		}
	}
	else
	{
		for(i = 0; i < 60; i++)
		{
			if(i < iSize)
			{
				m_cTextDisplay[i] = sText[i];
			}
			else
			{
				m_cTextDisplay[i] = ' ';
			}
		}
	}
}

//-----------------------------------------------
//USBoard
//-----------------------------------------------

//-----------------------------------------------
int SerRelayBoard::startUS(int iChannelActive)
{
	m_iUSBoardSensorActive = iChannelActive;
	
	return 0;
}
//-----------------------------------------------
int SerRelayBoard::stopUS()
{
	m_iUSBoardSensorActive = 0x00;

	return 0;
}
//-----------------------------------------------
void SerRelayBoard::requestUSBoardData1To8()
{
}

//-----------------------------------------------
int SerRelayBoard::getUSBoardData1To8(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_iUSBoardSensorData[i];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestUSBoardData9To16()
{
}

//-----------------------------------------------
int SerRelayBoard::getUSBoardData9To16(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_iUSBoardSensorData[i + 8];
	}

	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestUSBoardAnalogIn()
{
}

//-----------------------------------------------
void SerRelayBoard::getUSBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 4; i++)
	{
		piAnalogIn[i] = m_iUSBoardAnalogData[i];
	}

	m_Mutex.unlock();
}




//-----------------------------------------------
// GyroBoard
//-----------------------------------------------

//-----------------------------------------------
void SerRelayBoard::zeroGyro(bool bZeroActive)
{
	if(bZeroActive)
	{
		m_iCmdRelayBoard |= CMD_ZERO_GYRO;
	}
	else
	{
		m_iCmdRelayBoard &= ~CMD_ZERO_GYRO;
	}
}

//-----------------------------------------------
int SerRelayBoard::getGyroBoardAng(double* pdAngRad, double dAcc[])
{
	const double cdGyroScale = 1 / 520.5;

	int i;
	double dAng;
	
	m_Mutex.lock();
	
	dAng = m_iGyroBoardAng * cdGyroScale; // gyroboard transmits gyro value in degrees
	*pdAngRad = DEG2RAD(dAng);

	for(i = 0; i < 3; i++)
	{
		dAcc[i] = m_iGyroBoardAcc[i];
	}

	m_Mutex.unlock();
	
	return 0;
}
int SerRelayBoard::getGyroBoardAngBoost(double* pdAngRad, boost::array<double, 3u>& dAcc)
{
	const double cdGyroScale = 1 / 520.5;

	int i;
	double dAng;
	
	m_Mutex.lock();
	
	dAng = m_iGyroBoardAng * cdGyroScale; // gyroboard transmits gyro value in degrees
	*pdAngRad = DEG2RAD(dAng);

	for(i = 0; i < 3; i++)
	{
		dAcc[i] = m_iGyroBoardAcc[i];
	}

	m_Mutex.unlock();
	
	return 0;
}


//-----------------------------------------------
int SerRelayBoard::getGyroBoardDltAng(double* pdAng, double dAcc[])
{
	return 0;
}

//-----------------------------------------------
// RadarBoard
//-----------------------------------------------
void SerRelayBoard::requestRadarBoardData()
{
}

//-----------------------------------------------
int SerRelayBoard::getRadarBoardData(double* pdVelMMS)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 4; i++)
	{
		pdVelMMS[i] = m_iRadarBoardVel[i];

	}

	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
// CanClientGeneric
//-----------------------------------------------
void SerRelayBoard::addGenericCANListeningId(int id)
{
}

//-----------------------------------------------
void SerRelayBoard::removeGenericCANListeningId(int id)
{
}

//-----------------------------------------------
void SerRelayBoard::getGenericCANMessages(std::vector<CANTimedMessage>& pMessages)
{
}

//-----------------------------------------------
void SerRelayBoard::sendGenericCANMessage(CanMsg& message)
{
}

//-----------------------------------------------
void SerRelayBoard::setEMStop()
{		
	if (m_cSoftEMStop & 0x01)
	{
		m_cSoftEMStop &= 0xFE;
		//LOGINFO ("Emergency Stop INAKTIVE");
	}
	else if ((m_cSoftEMStop == 0) || (m_cSoftEMStop & 0x02))
	{
		m_cSoftEMStop |= 0x01;
		//LOGINFO("Emergency Stop AKTIVE");
	}
}

//-----------------------------------------------
void SerRelayBoard::resetEMStop()
{	
	m_cSoftEMStop |= 0x02;
}

//-----------------------------------------------
void SerRelayBoard::enable_logging()
{
	logging = true;
}
//-----------------------------------------------
void SerRelayBoard::disable_logging()
{
	logging = false;
}

void SerRelayBoard::log_to_file(int direction, unsigned char cMsg[])
{
	FILE * pFile;
	//Open Logfile
	pFile = fopen ("neo_relayboard_RX_TX_log.log","a");
	//Write Data to Logfile
	if(direction == 1)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<NUM_BYTE_SEND_RELAYBOARD_1_4; i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	if(direction == 2)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<NUM_BYTE_REC_RELAYBOARD_1_4; i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	//Close Logfile
	fclose (pFile);
}

//-----------------------------------------------
void SerRelayBoard::convDataToSendMsg(unsigned char cMsg[])
{
	int i;
	static int j = 0;
	int iCnt = 0;
	int iChkSum = 0;

	if (m_cSoftEMStop & 0x02)
	{
		if (j == 1)
		{
			m_cSoftEMStop &= 0xFD;
			j = 0;
		}
		else if (j == 0)
		{
			j = 1;
		}
	}

	cMsg[iCnt++] = CMD_RELAISBOARD_GET_DATA;

	cMsg[iCnt++] = m_iConfigRelayBoard >> 8;
	cMsg[iCnt++] = m_iConfigRelayBoard;

	cMsg[iCnt++] = m_iCmdRelayBoard >> 8;
	cMsg[iCnt++] = m_iCmdRelayBoard;

	cMsg[iCnt++] = m_iIOBoardDigOut >> 8;
	cMsg[iCnt++] = m_iIOBoardDigOut;

	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 24;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 16;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 8;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS;

	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 24;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 16;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 8;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS;

	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 24;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 16;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 8;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS;

		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 24;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 16;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 8;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS;
	}


	cMsg[iCnt++] = m_iUSBoardSensorActive >> 8;
	cMsg[iCnt++] = m_iUSBoardSensorActive;

	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		for(i = 0; i < 20; i++)
		{
			cMsg[iCnt++] = m_cTextDisplay[i];
		}

		// fill remaining msg with 0's
		do
		{
			cMsg[iCnt++] = 0;
		}
		while(iCnt < (m_iNumBytesSend - 2));
	}
	else
	{
		for(i = 0; i < 60; i++)
		{
			cMsg[iCnt++] = m_cTextDisplay[i];
		}
	}

	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{

		cMsg[iCnt++] = m_cSoftEMStop;
	}

	// calc checksum
	for(i = 0; i < (m_iNumBytesSend - 2); i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}

	cMsg[m_iNumBytesSend - 2] = iChkSum >> 8;
	cMsg[m_iNumBytesSend - 1] = iChkSum;

	// reset flags
	m_iCmdRelayBoard &= ~CMD_RESET_POS_CNT;
}

//-----------------------------------------------
bool SerRelayBoard::convRecMsgToData(unsigned char cMsg[])
{

	int iNumByteRec = NUM_BYTE_REC;

	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		iNumByteRec = NUM_BYTE_REC_RELAYBOARD_1_4;
	}

	const int c_iStartCheckSum = iNumByteRec;

	int i;
	unsigned int iTxCheckSum;
	unsigned int iCheckSum;

	m_Mutex.lock();

	// test checksum
	iTxCheckSum = (cMsg[c_iStartCheckSum + 1] << 8) | cMsg[c_iStartCheckSum];

	iCheckSum = 0;
	for(i = 0; i < c_iStartCheckSum; i++)
	{
		iCheckSum %= 0xFF00;
		iCheckSum += cMsg[i];
	}

	if(iCheckSum != iTxCheckSum)
	{
		return false;
	}

	// convert data
	int iCnt = 0;

	// RelayBoard

	m_iRelBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	m_iChargeCurrent = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];

	iCnt += 2;

	m_iRelBoardBattVoltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	m_iRelBoardKeyPad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	for(i = 0; i < 4; i++)
	{
		m_iRelBoardIRSensor[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iRelBoardTempSensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// IOBoard

	m_iIOBoardDigIn = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	for(i = 0; i < 8; i++)
	{
		m_iIOBoardAnalogIn[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iIOBoardStatus =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Motion control boards

	m_iPosMeasMotRightEnc =	cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);

	m_cDebugRight[0] = cMsg[iCnt];
	m_cDebugRight[1] = cMsg[iCnt + 1];
	m_cDebugRight[2] = cMsg[iCnt + 2];
	m_cDebugRight[3] = cMsg[iCnt + 3];

	iCnt += 4;

	m_iVelMeasMotRightEncS =cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);
	iCnt += 4;

	m_iPosMeasMotLeftEnc =	cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);

	m_cDebugLeft[0] = cMsg[iCnt];
	m_cDebugLeft[1] = cMsg[iCnt + 1];
	m_cDebugLeft[2] = cMsg[iCnt + 2];
	m_cDebugLeft[3] = cMsg[iCnt + 3];

	iCnt += 4;

	m_iVelMeasMotLeftEncS = cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);
	iCnt += 4;

	m_iMotRightStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	m_iMotLeftStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;


	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		m_iPosMeasMotRearRightEnc =	cMsg[iCnt] |
									(cMsg[iCnt + 1] << 8) |
									(cMsg[iCnt + 2] << 16) |
									(cMsg[iCnt + 3] << 24);
		
		m_cDebugRearRight[0] = cMsg[iCnt];
		m_cDebugRearRight[1] = cMsg[iCnt + 1];
		m_cDebugRearRight[2] = cMsg[iCnt + 2];
		m_cDebugRearRight[3] = cMsg[iCnt + 3];

		iCnt += 4;

		m_iVelMeasMotRearRightEncS =cMsg[iCnt] |
									(cMsg[iCnt + 1] << 8) |
									(cMsg[iCnt + 2] << 16) |
									(cMsg[iCnt + 3] << 24);
	
		iCnt += 4;

		m_iPosMeasMotRearLeftEnc =	cMsg[iCnt] |
									(cMsg[iCnt + 1] << 8) |
									(cMsg[iCnt + 2] << 16) |
									(cMsg[iCnt + 3] << 24);
		
		m_cDebugRearLeft[0] = cMsg[iCnt];
		m_cDebugRearLeft[1] = cMsg[iCnt + 1];
		m_cDebugRearLeft[2] = cMsg[iCnt + 2];
		m_cDebugRearLeft[3] = cMsg[iCnt + 3];

		iCnt += 4;

		m_iVelMeasMotRearLeftEncS = cMsg[iCnt] |
									(cMsg[iCnt + 1] << 8) |
									(cMsg[iCnt + 2] << 16) |
									(cMsg[iCnt + 3] << 24);		
		iCnt += 4;

		m_iMotRearRightStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;	

		m_iMotRearLeftStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	// GyroBoard

	m_iGyroBoardAng =	(cMsg[iCnt] << 24) |
						(cMsg[iCnt + 1] << 16) |
						(cMsg[iCnt + 2] << 8) |
						cMsg[iCnt + 3];
	iCnt += 4;

	for(i = 0; i < 3; i++)
	{
		m_iGyroBoardAcc[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iGyroBoardStatus =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// RadarBoard

	for(i = 0; i < 3; i++)
	{
		m_iRadarBoardVel[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iRadarBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// USBoard

	for(i = 0; i < 16; i++)
	{
		m_iUSBoardSensorData[i] = (cMsg[iCnt++]);
	}

	for(i = 0; i < 4; i++)
	{
		m_iUSBoardAnalogData[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iUSBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	m_Mutex.unlock();

	if( iCnt >= NUM_BYTE_REC_MAX )
	{
		ROS_INFO("msg size too small");
	}

	return true;
}

//---------------------------------------------------------
bool SerRelayBoard::convRecMsgToDataRelayBoard2(unsigned char cMsg[])
{
	int data_in_message = 0;

	m_Mutex.lock();

	// convert data
	int iCnt = 0;

	//Has Data
	data_in_message = cMsg[iCnt];
	iCnt++;
	//Relayboard Status
	m_iRelBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Charging Current
	m_iChargeCurrent = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Charging Current: " << m_iChargeCurrent << std::endl; //JNN
	iCnt += 2;
	//Charging State
	m_iChargeState = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Charging State: " << m_iChargeState << std::endl; //JNN
	iCnt += 2;
	//Battery Voltage
	m_iRelBoardBattVoltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Battery Voltage: " << m_iRelBoardBattVoltage << std::endl; //JNN
	iCnt += 2;
	//Keypad
	m_iRelBoardKeyPad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Temp Sensor
	m_iRelBoardTempSensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Relayboard Temp: " << m_iRelBoardTempSensor << std::endl; //JNN
	iCnt += 2;

	// IOBoard
	if(m_iHasIOBoard)
	{
		m_iIOBoardDigIn = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;

		for(int i = 0; i < 8; i++)
		{
			m_iIOBoardAnalogIn[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}

		m_iIOBoardStatus =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}
	// USBoard
	if(m_iHasUSBoard)
	{
		for(int i = 0; i < 16; i++)
		{
			m_iUSBoardSensorData[i] = (cMsg[iCnt++]);
			//std::cout << "USBoard Sensor Data " << i << ": " << m_iUSBoardSensorData[i] << std::endl; //JNN
		}
		for(int i = 0; i < 4; i++)
		{
			m_iUSBoardAnalogData[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
			//std::cout << "USBoard Analog Data " << i << ": " << m_iUSBoardAnalogData[i] << std::endl; //JNN
		}

		m_iUSBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
		//std::cout << "USBoard Status: " << m_iUSBoardStatus << std::endl; //JNN
	}

	m_Mutex.unlock();


	return true;
}

//------------------------------------------------------------
void SerRelayBoard::convDataToSendMsgRelayBoard2(unsigned char cMsg[])
{
	int iCnt = 0;
	int iChkSum = 0;

	int has_data = 0;
	int has_motor_data8 = 0;
	int has_motor_data4 = 0;

	//First Bit not in use yet
	has_data = (has_motor_data8 << 6) + (has_motor_data4 << 5) + (m_ihasRelayData << 4) + (m_ihas_LCD_DATA << 3) + (m_iHasIOBoard << 2) +(m_iHasUSBoard << 1) + (m_iHasSpeakerData);

	//Data in Message:
	//Header
	cMsg[iCnt++] = 0x02;
	cMsg[iCnt++] = 0xD6;
	cMsg[iCnt++] = 0x80;
	cMsg[iCnt++] = 0x02;
	//has_data
	//soft_em
	//Motor9-6
	//Motor5-2
	//Relaystates
	//LCD Data
	//IO Data
	//US Data
	//Speaker Data
	//Checksum
	cMsg[iCnt++] = has_data;
	cMsg[iCnt++] = m_cSoftEMStop; //SoftEM

	//Relaystates
	if(m_ihasRelayData)
	{
		cMsg[iCnt++] = m_iCmdRelayBoard >> 8;
		cMsg[iCnt++] = m_iCmdRelayBoard;
	}
	//LCD Data
	if(m_ihas_LCD_DATA)
	{
		for(int u = 0; u < 20; u++)
		{
			cMsg[iCnt++] = m_cTextDisplay[u];
		}
	}
	//IO Data
	if(m_iHasIOBoard)
	{
		cMsg[iCnt++] = m_iIOBoardDigOut >> 8;
		cMsg[iCnt++] = m_iIOBoardDigOut;
	}
	//US Data
	if(m_iHasUSBoard)
	{
		cMsg[iCnt++] = m_iUSBoardSensorActive >> 8;
		cMsg[iCnt++] = m_iUSBoardSensorActive;
	}
	//Speaker Data
	if(m_iHasSpeakerData)
	{
		cMsg[iCnt++] = 0 >> 8;
		cMsg[iCnt++] = 0;
	}
	// calc checksum
	for(int i = 4; i < (m_iNumBytesSend - 2); i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}

	cMsg[iCnt++] = iChkSum >> 8;
	cMsg[iCnt++] = iChkSum;
	m_iNumBytesSend = iCnt;
}

