/****************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology
 *
 * Maintainer: Denis ��togl, email: denis.stogl@kit.edu
 *
 * Date of update: 2014-2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * Date of creation: June 2010
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// general includes
#include <unistd.h>

#include <cob_forcetorque/ForceTorqueCtrl.h>

// Headrs provided by cob-packages
#include <cob_generic_can/CanPeakSysUSB.h>
#include <cob_generic_can/SocketCan.h>

ForceTorqueCtrl::ForceTorqueCtrl()
{
  // for types and baudrates see:
  // https://github.com/ipa320/cob_robots/blob/hydro_dev/cob_hardware_config/raw3-5/config/base/CanCtrl.ini
  m_RS485Device = "/dev/ttyUSB0";
  m_ModbusBaseIdentifier = 0xA;  			//ID of the ATI sensor is always set to 10 (As specified in 8.3)
  m_ModbusBaudrate = MODBUSBAUD_125K;		///By default, the baudrate is set to 1.250.000
}

ForceTorqueCtrl::ForceTorqueCtrl(std::string device_path, int device_baudrate, int base_identifier)
{
  // for types and baudrates see:
  m_RS485Device = device_path;
  m_ModbusBaudrate = device_baudrate;
  m_ModbusBaseIdentifier = base_identifier;
}

ForceTorqueCtrl::~ForceTorqueCtrl()
{
  if (m_modbusCtrl != NULL)
  {
    //delete m_modbusCtrl;		//TODO Delete here?
  }
}

bool ForceTorqueCtrl::Init()
{
  bool ret = true;

  if (initRS485())
  {
    // This is way of testing if communication is also successful
    if (!ReadFTCalibrationData())
    {
      std::cout << "Can not read Calibration Data from FTS!" << std::endl;
      ret = false;
    }
    // Add return values and checking
    //SetActiveCalibrationMatrix(0);
    SetCalibMatrix();
  }
  else
  {
    std::cout << "RS485 initialization unsuccessful!" << std::endl;
    ret = false;
  }

  return ret;
}

bool ForceTorqueCtrl::initRS485()
{
  //bool ret = true;
  m_modbusCtrl = modbus_new_rtu(m_RS485Device.c_str(), m_ModbusBaudrate, 'r', 0, 0);

  modbus_set_slave(m_modbusCtrl, m_ModbusBaseIdentifier);

  int rc = modbus_rtu_set_serial_mode(m_modbusCtrl, MODBUS_RTU_RS485);
  printf("modbus_rtu_set_serial_mode: %d \n",rc);

  if (rc != 0)
  {
	  printf("modbus_rtu_set_serial_mode: %s \n",modbus_strerror(errno));
	  return false;
  }

  return true;
}

bool ForceTorqueCtrl::ReadFTCalibrationData()
{
#if DEBUG
  std::cout << "\n\n*********FTSerialNumber**********" << std::endl;
#endif
  uint16_t tab_reg[168];

  int rc = modbus_read_registers(m_modbusCtrl, 0x00e3, 168, tab_reg);
  if (rc == -1)
  {
	  #if DEBUG
		  std::cout << "Reading Calibration Data failed with status " << rc << std::endl;
	  #endif
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return true;
  }
  for (unsigned int i=0; i < rc; i++)
  {
	  printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
  }
  m_calibrationData.calibSerialNumber = std::string( tab_reg[0], tab_reg[4] );
#if DEBUG
  std::cout << "Serial Number is " << m_calibrationData.calibSerialNumber << std::endl;
#endif
  m_calibrationData.calibPartNumberNumber = std::string( tab_reg[4], tab_reg[20] );
#if DEBUG
  std::cout << "Part Number is " << m_calibrationData.calibPartNumberNumber << std::endl;
#endif
  m_calibrationData.calibFamilyID =  std::string( tab_reg[20], tab_reg[22] );
#if DEBUG
  std::cout << "Calib Family ID is " << m_calibrationData.calibFamilyID << std::endl;
#endif
  m_calibrationData.calibTime =   std::string( tab_reg[22], tab_reg[32] );
#if DEBUG
  std::cout << "Calib Time is " << m_calibrationData.calibTime << std::endl;
#endif
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
    	m_calibrationData.basicMatrix[i][j] = modbus_get_float(&tab_reg[32+(i*6+j)*2]);
    }
  }
  m_calibrationData.forceUnitsInt = MODBUS_GET_LOW_BYTE(tab_reg[104]);
#if DEBUG
  std::cout << "Force Units are " << m_calibrationData.forceUnitsInt << std::endl;
#endif
  m_calibrationData.torqueUnitsInt = MODBUS_GET_HIGH_BYTE(tab_reg[104]);
#if DEBUG
  std::cout << "Torque Units are " << m_calibrationData.torqueUnitsInt << std::endl;
#endif
  for (unsigned int i = 0; i < 6; i++)
  {
  	m_calibrationData.maxRating[i] = modbus_get_float(&tab_reg[105 +i*2]);
  }
  m_calibrationData.countsPerForce = MODBUS_GET_INT32_FROM_INT16(tab_reg, 117);
#if DEBUG
  std::cout << "Counts per force are " << m_calibrationData.countsPerForce << std::endl;
#endif
  m_calibrationData.countsPerTorque = MODBUS_GET_INT32_FROM_INT16(tab_reg, 119);
#if DEBUG
  std::cout << "Counts per torque " << m_calibrationData.countsPerTorque << std::endl;
#endif
  for (unsigned int i = 0; i < 6; i++)
  {
  	m_calibrationData.gageGain[i] = tab_reg[121 + i];
  }
  for (unsigned int i = 0; i < 6; i++)
  {
  	m_calibrationData.gageOffset[i] = tab_reg[127 + i];
  }
  return true;
}

bool ForceTorqueCtrl::SetStorageLock(bool lock)
{
  uint8_t lockCode;
  if (lock)
  {
	lockCode = 0x18;
  }
  else
  {
	lockCode = 0xaa;
  }

  uint8_t raw_req[] = { m_ModbusBaseIdentifier, 0x6a, lockCode };		//OpCode is 0x6a = 104, Data is 0x18 (to lock) or 0xaa (to unlock) as specified in 8.3.1.
  uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];
  int req_length = modbus_send_raw_request(m_modbusCtrl, raw_req, 3 * sizeof(uint8_t));
  int len = modbus_receive_confirmation(m_modbusCtrl, rsp);

  if (len == -1)
  {
    return false;
  }
  std::cout << "Setting storage lock was successful" << std::endl;

  return true;
}

bool ForceTorqueCtrl::ReadStatusWord()
{
#if DEBUG
  std::cout << "\n\n*********FTStatusWord**********" << std::endl;
#endif
  uint16_t tab_reg[1];

  int rc = modbus_read_registers(m_modbusCtrl, 0x001D, 1, tab_reg);
  if (rc == -1)
  {
	  #if DEBUG
		  std::cout << "Reading Status Word failed with status " << rc << std::endl;
	  #endif
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return true;
  }
  if (tab_reg[1] & ST_WATCHDOG_RESET)
  {
	  std::cout << "Watchdog reset – the analog board was reset by the watchdog timer." << std::endl;
  }
  if (tab_reg[1] & ST_EXC_VOLTAGE_HIGH)
  {
	  std::cout << "Excitation voltage too high." << std::endl;
  }
  if (tab_reg[1] & ST_EXC_VOLTAGE_LOW)
  {
	  std::cout << "Excitation voltage too low." << std::endl;
  }
  if (tab_reg[1] & ST_ART_ANALOG_GRND_OOR)
  {
	  std::cout << "Artificial analog ground out of range." << std::endl;
  }
  if (tab_reg[1] & ST_PWR_HIGH)
  {
	  std::cout << "Power supply too high." << std::endl;
  }
  if (tab_reg[1] & ST_PWR_LOW)
  {
	  std::cout << "Power supply too low." << std::endl;
  }
  if (tab_reg[1] & ST_EEPROM_ERR)
  {
	  std::cout << "Error accessing stored settings in EEPROM." << std::endl;
  }
  if (tab_reg[1] & ST_INV_CONF_DATA)
  {
	  std::cout << "Invalid configuration data." << std::endl;
  }
  if (tab_reg[1] & ST_STRAIN_GAGE_SUPPLY_HIGH)
  {
	  std::cout << "Strain gage bridge supply current too high." << std::endl;
  }
  if (tab_reg[1] & ST_STRAIN_GAGE_SUPPLY_LOW)
  {
	  std::cout << "Strain gage bridge supply current too low." << std::endl;
  }
  if (tab_reg[1] & ST_THERMISTOR_HIGH)
  {
	  std::cout << "Thermistor too high." << std::endl;
  }
  if (tab_reg[1] & ST_THERMISTOR_LOW)
  {
	  std::cout << "Thermistor too low." << std::endl;
  }
  if (tab_reg[1] & ST_DAC_READING_OOR)
  {
	  std::cout << "DAC reading out of range." << std::endl;
  }
  return true;
}


bool ForceTorqueCtrl::readDiagnosticADCVoltages(int index, short int &value)
{
  // TODO: Check for Init
#if DEBUG
  std::cout << "\n\n*******Read Diagnostic ADC Voltages on index: " << index << "********" << std::endl;
#endif
  bool ret = true;
//  CanMsg CMsg;
//  CMsg.setID(m_CanBaseIdentifier | READ_DIAGNOV);
//  CMsg.setLength(1);
//  // TODO: offset or typecast index
//  CMsg.setAt(index, 0);
//
//  ret = m_pCanCtrl->transmitMsg(CMsg, true);
//
//  if (ret)
//  {
//    CanMsg replyMsg;
//    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
//    if (ret)
//    {
//#if DEBUG
//      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
//      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
//#endif
//      if (replyMsg.getID() == (m_CanBaseIdentifier | READ_DIAGNOV))
//      {
//#if DEBUG
//        std::cout << "Reading Diagnostic ADC Voltage succeed!" << std::endl;
//        std::cout << "ADC Voltage of diagnostic value " << index << " : " << replyMsg.getAt(0) << " "
//                  << replyMsg.getAt(1) << std::endl;
//#endif
//
//        ibBuf.bytes[0] = replyMsg.getAt(1);
//        ibBuf.bytes[1] = replyMsg.getAt(0);
//        value = ibBuf.value;
//      }
//      else
//        std::cout << "Error: Received wrong opcode!" << std::endl;
//    }
//    else
//      std::cout << "Error: Receiving Message failed!" << std::endl;
//  }
//  else
//  {
//    std::cout << "ForceTorqueCtrl::readDiagnosticADCVoltages(byte index): Can not transmit message!" << std::endl;
//  }

  return ret;
}

bool ForceTorqueCtrl::SetActiveCalibrationMatrix(int num)
{
#if DEBUG
  std::cout << "\n\n*******Setting Active Calibration Matrix Num to: " << num << "********" << std::endl;
#endif
  bool ret = true;
//  CanMsg CMsg;
//  CMsg.setID(m_CanBaseIdentifier | SET_CALIB);
//  CMsg.setLength(1);
//  CMsg.setAt(num, 0);
//
//  ret = m_pCanCtrl->transmitMsg(CMsg, true);
//
//  if (ret)
//  {
//    CanMsg replyMsg;
//    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
//    if (ret)
//    {
//#if DEBUG
//      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
//      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
//#endif
//      if (replyMsg.getID() == (m_CanBaseIdentifier | SET_CALIB))
//      {
//#if DEBUG
//        std::cout << "Setting Calibration Matrix succeed!" << std::endl;
//        std::cout << "Calibration Matrix: " << replyMsg.getAt(0) << " is Activ!" << std::endl;
//#endif
//      }
//      else
//      {
//#if DEBUG
//        std::cout << "Error: Received wrong opcode!" << std::endl;
//#endif
//        ret = false;
//      }
//    }
//    else
//    {
//      std::cout << "Error: Receiving Message failed!" << std::endl;
//      ret = false;
//    }
//  }
//  else
//  {
//    std::cout << "ForceTorqueCtrl::SetActiveCalibrationMatrix(int num): Can not transmit message!" << std::endl;
//    ret = false;
//  }

  return ret;
}

bool ForceTorqueCtrl::SetBaudRate(int value, bool setVolatile)
{
#if DEBUG
  std::cout << "\n\n*******Setting Baud Rate value to: " << value << "********" << std::endl;
#endif
  uint16_t baudrateIndex = 0;
  if (value == MODBUSBAUD_125K)
  {
	  baudrateIndex = 0;
  }
  else if (value == MODBUSBAUD_115200)
  {
	  baudrateIndex = 2;
  }
  else if (value == MODBUSBAUD_19200)
  {
	  baudrateIndex = 1;
  }
  else //Baudrate not supported
  {
	  fprintf(stderr, "Baudrate %i is not supported \n", value);
	  return false;
  }

  //If Baudrate should be set non-volatile, the 0x001E flag has to be set first (According to 8.3.2 of the sensors user manual)
  if(!setVolatile)
  {
	  uint16_t tab_reg[1] = {1};

	  int rc = modbus_write_registers(m_modbusCtrl, 0x001E, 1, tab_reg);
	  if (rc == -1)
	  {
		  #if DEBUG
			  std::cout << "Setting non-volatile baud rate flag failed with status " << rc << std::endl;
		  #endif
	      fprintf(stderr, "%s\n", modbus_strerror(errno));
	      return false;
	  }
  }

  uint16_t tab_reg[1] = {baudrateIndex};

  int rc = modbus_write_registers(m_modbusCtrl, 0x001F, 1, tab_reg);
  if (rc == -1)
  {
	  #if DEBUG
		  std::cout << "Setting baudrate failed with status " << rc << std::endl;
	  #endif
      fprintf(stderr, "%s\n", modbus_strerror(errno));
      return false;
  }
  m_ModbusBaudrate = value;
  //Reset connection here
  Reset();

  return true;
}

bool ForceTorqueCtrl::Reset()
{
  std::cout << "\n\n******* Reseting the RS485 Interface ********" << std::endl;
  if (!Close())
  {
	  return false;
  }
  if (!Init())
  {
	  return false;
  }
  return true;
}

bool ForceTorqueCtrl::Close()
{
  modbus_close(m_modbusCtrl);
  modbus_free(m_modbusCtrl);

  return true;
}

bool ForceTorqueCtrl::SetBaseIdentifier(int identifier)
{
#if DEBUG
  std::cout << "\n\n*******Setting Base Identifier value to HEX : " << std::hex << identifier << " ********"
            << std::endl;
#endif
  bool ret = true;
//  CanMsg CMsg;
//  CMsg.setID(m_CanBaseIdentifier | SET_BASEID);
//  CMsg.setLength(1);
//  CMsg.setAt(identifier, 0);
//
//  ret = m_pCanCtrl->transmitMsg(CMsg, true);
//
//  if (ret)
//  {
//    CanMsg replyMsg;
//    ret = m_pCanCtrl->receiveMsgRetry(&replyMsg, 10);
//    if (ret)
//    {
//#if DEBUG
//      std::cout << "reply ID: \t" << std::hex << replyMsg.getID() << std::endl;
//      std::cout << "reply Length: \t" << replyMsg.getLength() << std::endl;
//#endif
//      if (replyMsg.getID() == (m_CanBaseIdentifier | SET_BASEID))
//      {
//#if DEBUG
//        std::cout << "Setting Base Identifier succeed!" << std::endl;
//        std::cout << "Send Base Identifier value: " << std::hex << CMsg.getAt(0) << "!" << std::endl;
//#endif
//        ret = true;
//      }
//      else
//      {
//        std::cout << "Error: Received wrong opcode!" << std::endl;
//        ret = false;
//      }
//    }
//    else
//    {
//      std::cout << "Error: Receiving Message failed!" << std::endl;
//      ret = false;
//    }
//  }
//  else
//  {
//    std::cout << "ForceTorqueCtrl::SetBaseIdentifier(int identifier): Can not transmit message!" << std::endl;
//    ret = false;
//  }

  return ret;
}

void ForceTorqueCtrl::ReadCalibrationMatrix()
{
  Eigen::VectorXf vCoef(6);

  // Read Fx coefficients
  //ReadMatrix(0, vCoef);
  m_v3FXGain = vCoef;

  // Read Fy coefficients
  //ReadMatrix(1, vCoef);
  m_v3FYGain = vCoef;

  // Read Fz coefficients
  //ReadMatrix(2, vCoef);
  m_v3FZGain = vCoef;

  // Read Tx coefficients
  //ReadMatrix(3, vCoef);
  m_v3TXGain = vCoef;

  // Read Ty coefficients
  //ReadMatrix(4, vCoef);
  m_v3TYGain = vCoef;

  // Read Tz coefficients
  //ReadMatrix(5, vCoef);
  m_v3TZGain = vCoef;
  SetCalibMatrix();
}


bool ForceTorqueCtrl::ReadFirmwareVersion()
{
#if DEBUG
  std::cout << "\n\n*******Reading Firmware Version*******" << std::endl;
#endif
  bool ret = true;
// TODO Find equivalent over modbus?
  return ret;
}

bool ForceTorqueCtrl::ReadSGData(int statusCode, double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz)
{
  int sg0 = 0, sg1 = 0, sg2 = 0, sg3 = 0, sg4 = 0, sg5 = 0;
  if (!isStreaming)
  {
	  StartStreaming();
  }

  bool ret = true;
  m_read_mutex.lock();
  sg0 = m_buffer.gageData[0];
  sg1 = m_buffer.gageData[1];
  sg2 = m_buffer.gageData[2];
  sg3 = m_buffer.gageData[3];
  sg4 = m_buffer.gageData[4];
  sg5 = m_buffer.gageData[5];
  m_read_mutex.unlock();

  StrainGaugeToForce(sg0, sg1, sg2, sg3, sg4, sg5);
  Fx = m_vForceData(0);
  Fy = m_vForceData(1);
  Fz = m_vForceData(2);
  Tx = m_vForceData(3);
  Ty = m_vForceData(4);
  Tz = m_vForceData(5);

  return true;
}

bool ForceTorqueCtrl::StartStreaming()
{
	if (!isStreaming)
	{
		uint8_t raw_req[] = { m_ModbusBaseIdentifier, 0x46, 0x55 };		//OpCode is 0x46 = 70, Data is 0x55 as specified in 8.3.1.
		uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];
		int req_length = modbus_send_raw_request(m_modbusCtrl, raw_req, 3 * sizeof(uint8_t));
		int len = modbus_receive_confirmation(m_modbusCtrl, rsp);

		if (len == -1)
		{
			return false;
		}
		else if (rsp[0] == 1)
		{
			//Confirmed
			std::cout << "Start streaming" << std::endl;

			/* Open File Descriptor */
			m_rs485 = open( m_RS485Device.c_str(), O_RDWR| O_NONBLOCK | O_NDELAY );
			/* Error Handling */
			if ( m_rs485 < 0 )
			{
				std::cout << "Error " << errno << " opening " << m_RS485Device << ": " << strerror (errno) << std::endl;
			}
			isStreaming = true;
		}
		m_readThread = new std::thread(&ForceTorqueCtrl::ReadDataLoop, this);
	}
	return true;
}

bool ForceTorqueCtrl::ReadData()
{
	if (isStreaming)
	{
		/* Allocate memory for read buffer */
		uint8_t buf [13];
		memset (&buf, 0, sizeof buf);

		/* *** READ *** */
		int n = read( m_rs485, &buf , sizeof buf );

		/* Error Handling */
		if (n < 0)
		{
		     std::cout << "Error reading: " << strerror(errno) << std::endl;
		}

		uint8_t check = buf[12];

		if (((check >> 7) & 0x01))	//Check status bit
		{
			//If it is set to 1, an error occured -> Stop streaming

			return false;
		}
		uint8_t checksum = check & 0x7F;


		int compareChecksum = 0;
		for (unsigned int i = 0; i < 11; i++)
		{
			compareChecksum += buf[i];
		}
		if ((compareChecksum & 0x7F) != checksum)
		{
			std::cout << "Package has invalid checksum! Ignoring data..." << std::endl;
			return false;
		}
		m_read_mutex.lock();
		m_buffer.gageData[0] = buf[0] | (buf[1] << 8);
		m_buffer.gageData[1]  = buf[6] | (buf[7] << 8);
		m_buffer.gageData[2]  = buf[2] | (buf[3] << 8);
		m_buffer.gageData[3]  = buf[8] | (buf[9] << 8);
		m_buffer.gageData[4]  = buf[4] | (buf[5] << 8);
		m_buffer.gageData[5]  = buf[10] | (buf[11] << 8);
		m_buffer.timestamp = ros::Time::now();
		m_read_mutex.unlock();
	}
}

void ForceTorqueCtrl::ReadDataLoop()
{
	if (isStreaming)
	{
		if (!ReadData())
		{
			std::cout << "Error while reading. " << std::endl;
			StopStreaming();
			sleep(1000);
			ReadStatusWord();
		}
		usleep((unsigned int)(1000000/m_ModbusBaudrate));	//TODO sleep less?
	}
}


bool ForceTorqueCtrl::StopStreaming()
{
	if (isStreaming)
	{
		unsigned char cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//Send jamming sequence of 14 bytes to stop streaming
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		int n_written = write( m_rs485, cmd, sizeof(cmd) -1);
		if (m_readThread)
		{
			m_readThread->join();
		}
		isStreaming = false;
	}
}

void ForceTorqueCtrl::StrainGaugeToForce(int &sg0, int &sg1, int &sg2, int &sg3, int &sg4, int &sg5)
{
  Eigen::VectorXf v6SG(6);
  Eigen::VectorXf v6tmp(6);
  Eigen::VectorXf test(6);

  v6SG[0] = sg0;
  v6SG[1] = sg1;
  v6SG[2] = sg2;
  v6SG[3] = sg3;
  v6SG[4] = sg4;
  v6SG[5] = sg5;
  test = m_mXCalibMatrix * v6SG;
  m_vForceData = test * 0.000001;
}

void ForceTorqueCtrl::SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = sg0Off;
  tmp[1] = sg1Off;
  tmp[2] = sg2Off;
  tmp[3] = sg3Off;
  tmp[4] = sg4Off;
  tmp[5] = sg5Off;
  m_v3StrainGaigeOffset = tmp;
}
void ForceTorqueCtrl::SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = gg0;
  tmp[1] = gg1;
  tmp[2] = gg2;
  tmp[3] = gg3;
  tmp[4] = gg4;
  tmp[5] = gg5;
  m_v3GaugeGain = tmp;
  // std::cout<<"GaugeGain: \n"<<m_v3GaugeGain<<"\n\n";
}

void ForceTorqueCtrl::SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fxg0;
  tmp[1] = fxg1;
  tmp[2] = fxg2;
  tmp[3] = fxg3;
  tmp[4] = fxg4;
  tmp[5] = fxg5;
  m_v3FXGain = tmp;
  // std::cout<<"FXGain: \n"<<m_v3FXGain<<"\n\n";
}
void ForceTorqueCtrl::SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fyg0;
  tmp[1] = fyg1;
  tmp[2] = fyg2;
  tmp[3] = fyg3;
  tmp[4] = fyg4;
  tmp[5] = fyg5;
  m_v3FYGain = tmp;
  // std::cout<<"FYGain: \n"<<m_v3FYGain<<"\n\n";
}
void ForceTorqueCtrl::SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = fzg0;
  tmp[1] = fzg1;
  tmp[2] = fzg2;
  tmp[3] = fzg3;
  tmp[4] = fzg4;
  tmp[5] = fzg5;
  m_v3FZGain = tmp;
  // std::cout<<"FZGain: \n"<<m_v3FZGain<<"\n\n";
}
void ForceTorqueCtrl::SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = txg0;
  tmp[1] = txg1;
  tmp[2] = txg2;
  tmp[3] = txg3;
  tmp[4] = txg4;
  tmp[5] = txg5;
  m_v3TXGain = tmp;
  // std::cout<<"TXGain: \n"<<m_v3TXGain<<"\n\n";
}
void ForceTorqueCtrl::SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = tyg0;
  tmp[1] = tyg1;
  tmp[2] = tyg2;
  tmp[3] = tyg3;
  tmp[4] = tyg4;
  tmp[5] = tyg5;
  m_v3TYGain = tmp;
  // std::cout<<"TYGain: \n"<<m_v3TYGain<<"\n\n";
}
void ForceTorqueCtrl::SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5)
{
  Eigen::VectorXf tmp(6);
  tmp[0] = tzg0;
  tmp[1] = tzg1;
  tmp[2] = tzg2;
  tmp[3] = tzg3;
  tmp[4] = tzg4;
  tmp[5] = tzg5;
  m_v3TZGain = tmp;
  // std::cout<<"TZGain: \n"<<m_v3TZGain<<"\n\n";
}

void ForceTorqueCtrl::CalcCalibMatrix()
{
  Eigen::MatrixXf tmp(6, 6);
  tmp(0) = m_v3FXGain[0] / m_v3GaugeGain[0];
  tmp(1) = m_v3FXGain[1] / m_v3GaugeGain[1];
  tmp(2) = m_v3FXGain[2] / m_v3GaugeGain[2];
  tmp(3) = m_v3FXGain[3] / m_v3GaugeGain[3];
  tmp(4) = m_v3FXGain[4] / m_v3GaugeGain[4];
  tmp(5) = m_v3FXGain[5] / m_v3GaugeGain[5];

  tmp(6) = m_v3FYGain[0] / m_v3GaugeGain[0];
  tmp(7) = m_v3FYGain[1] / m_v3GaugeGain[1];
  tmp(8) = m_v3FYGain[2] / m_v3GaugeGain[2];
  tmp(9) = m_v3FYGain[3] / m_v3GaugeGain[3];
  tmp(10) = m_v3FYGain[4] / m_v3GaugeGain[4];
  tmp(11) = m_v3FYGain[5] / m_v3GaugeGain[5];

  tmp(12) = m_v3FZGain[0] / m_v3GaugeGain[0];
  tmp(13) = m_v3FZGain[1] / m_v3GaugeGain[1];
  tmp(14) = m_v3FZGain[2] / m_v3GaugeGain[2];
  tmp(15) = m_v3FZGain[3] / m_v3GaugeGain[3];
  tmp(16) = m_v3FZGain[4] / m_v3GaugeGain[4];
  tmp(17) = m_v3FZGain[5] / m_v3GaugeGain[5];

  tmp(18) = m_v3TXGain[0] / m_v3GaugeGain[0];
  tmp(19) = m_v3TXGain[1] / m_v3GaugeGain[1];
  tmp(20) = m_v3TXGain[2] / m_v3GaugeGain[2];
  tmp(21) = m_v3TXGain[3] / m_v3GaugeGain[3];
  tmp(22) = m_v3TXGain[4] / m_v3GaugeGain[4];
  tmp(23) = m_v3TXGain[5] / m_v3GaugeGain[5];

  tmp(24) = m_v3TYGain[0] / m_v3GaugeGain[0];
  tmp(25) = m_v3TYGain[1] / m_v3GaugeGain[1];
  tmp(26) = m_v3TYGain[2] / m_v3GaugeGain[2];
  tmp(27) = m_v3TYGain[3] / m_v3GaugeGain[3];
  tmp(28) = m_v3TYGain[4] / m_v3GaugeGain[4];
  tmp(29) = m_v3TYGain[5] / m_v3GaugeGain[5];

  tmp(30) = m_v3TZGain[0] / m_v3GaugeGain[0];
  tmp(31) = m_v3TZGain[1] / m_v3GaugeGain[1];
  tmp(32) = m_v3TZGain[2] / m_v3GaugeGain[2];
  tmp(33) = m_v3TZGain[3] / m_v3GaugeGain[3];
  tmp(34) = m_v3TZGain[4] / m_v3GaugeGain[4];
  tmp(35) = m_v3TZGain[5] / m_v3GaugeGain[5];

  m_mXCalibMatrix = tmp;
}

void ForceTorqueCtrl::SetCalibMatrix()
{
  Eigen::MatrixXf tmp(6, 6);
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
    	tmp(i,j) = m_calibrationData.basicMatrix[i][j];
    }
  }

  m_mXCalibMatrix = tmp.transpose();
}
