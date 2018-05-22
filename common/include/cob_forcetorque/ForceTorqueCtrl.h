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

#ifndef FORCETORQUECTRL_INCLUDEDEF_H
#define FORCETORQUECTRL_INCLUDEDEF_H

#include <iostream>
#include <fstream>
#include <Eigen/Core>

// Headers provided by other cob-packages
#include <cob_generic_can/CanItf.h>
#include <modbus/modbus-rtu.h>
#include <ros/time.h>
#include <mutex>
#include <thread>

#define DEBUG 0

// opCodes for the ForceTorque Can Interface
// set as Can Message ID
#define READ_SG 0x0
#define READ_MATRIX 0x2
#define READ_SERIALNR 0x5
#define SET_CALIB 0x6
#define READ_COUNTSPERU 0x7
#define READ_UNITCODE 0x8
#define READ_DIAGNOV 0X9
#define RESET 0xC
#define SET_BASEID 0xD
#define SET_BAUD 0xE
#define READ_FIRMWARE 0xF

#define ATI_CAN_BAUD_2M 0
#define ATI_CAN_BAUD_1M 1
#define ATI_CAN_BAUD_500K 3
#define ATI_CAN_BAUD_250K 7

#define MODBUSBAUD_125K		1250000
#define MODBUSBAUD_115200	115200
#define MODBUSBAUD_19200	19200

enum ForceUnit
{
	FU_POUNDS,
	FU_NEWTON,
	FU_KILOPOUND,
	FU_KILONEWTON,
	FU_K_EQV_FORCE,
	FU_G_EQV_FORCE
};

enum TorqueUnit
{
	TU_POUNDS_INCH,
	TU_PUBD_FOOT,
	TU_NEWTON_METER,
	TU_NEWTON_MILITMETER,
	TU_K_EQV_CENTIMETER,
	TU_KILONEWTONMETER
};

//Status Words
#define ST_WATCHDOG_RESET 1
#define ST_EXC_VOLTAGE_HIGH 2
#define ST_EXC_VOLTAGE_LOW 4
#define ST_ART_ANALOG_GRND_OOR 8
#define ST_PWR_HIGH 16
#define ST_PWR_LOW 32
//Bit 6 not used
#define ST_EEPROM_ERR 128
#define ST_INV_CONF_DATA 256
#define ST_STRAIN_GAGE_SUPPLY_HIGH 512
#define ST_STRAIN_GAGE_SUPPLY_LOW 1024
#define ST_THERMISTOR_HIGH 2048
#define ST_THERMISTOR_LOW 4096
#define ST_DAC_READING_OOR 8192


struct CalibrationData
{
	  std::string calibSerialNumber;
	  std::string calibPartNumberNumber;
	  std::string calibFamilyID;
	  std::string calibTime;
	  float basicMatrix[6][6];
	  uint8_t forceUnitsInt;
	  uint8_t torqueUnitsInt;
	  float maxRating[6];
	  int32_t countsPerForce;
	  int32_t countsPerTorque;
	  uint16_t gageGain[6];
	  uint16_t gageOffset[6];
};

struct GageVector
{
	uint16_t gageData[6];
	ros::Time timestamp;
};

class ForceTorqueCtrl
{
public:
  ForceTorqueCtrl();
  ForceTorqueCtrl(std::string device_path, int device_baudrate, int base_identifier);
  ~ForceTorqueCtrl();

  bool Init();
  bool ReadFTCalibrationData();
  bool ReadStatusWord();
  bool readDiagnosticADCVoltages(int index, short int& value);
  bool SetActiveCalibrationMatrix(int num);
  bool SetBaudRate(int value, bool setVolatile = true);
  bool SetBaseIdentifier(int identifier);
  bool Reset();
  bool Close();
  bool StartStreaming();
  bool StopStreaming();
  bool ReadData();
  void ReadDataLoop();
  bool ReadSGData(int statusCode, double& Fx, double& Fy, double& Fz, double& Tx, double& Ty, double& Tz);
  bool ReadFirmwareVersion();
  void ReadCalibrationMatrix();

  void SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off);
  void SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5);
  void SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5);
  void SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5);
  void SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5);
  void SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5);
  void SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5);
  void SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5);

  void SetCalibMatrix();
  void CalcCalibMatrix();
  void StrainGaugeToForce(int& sg0, int& sg1, int& sg2, int& sg3, int& sg4, int& sg5);

protected:
  bool initRS485();
  bool SetStorageLock(bool lock);

  bool m_bWatchdogErr;

private:


  modbus_t* m_modbusCtrl;
  int m_rs485;

  std::string m_RS485Device;
  uint8_t m_ModbusBaseIdentifier;
  int m_ModbusBaudrate;

  bool isStreaming = false;

  CalibrationData m_calibrationData;
  unsigned int d_len;
  Eigen::VectorXf m_v3StrainGaigeOffset;
  Eigen::VectorXf m_v3GaugeGain;
  Eigen::VectorXf m_v3FXGain;
  Eigen::VectorXf m_v3FYGain;
  Eigen::VectorXf m_v3FZGain;
  Eigen::VectorXf m_v3TXGain;
  Eigen::VectorXf m_v3TYGain;
  Eigen::VectorXf m_v3TZGain;
  Eigen::MatrixXf m_mXCalibMatrix;
  Eigen::MatrixXf m_vForceData;

  GageVector m_buffer;
  std::mutex m_read_mutex;
  std::thread* m_readThread;

  // the Parameter indicates the Axis row to Read
  // Fx = 0 | Fy = 1 | Fz = 2 | Tx = 3 | Ty = 4 | Tz = 5
  //void ReadMatrix(int axis, Eigen::VectorXf& vec);

  union
  {
    char bytes[2];
    short int value;
  } ibBuf;

  union
  {
    char bytes[4];
    int value;
  } intbBuf;

  union
  {
    char bytes[4];
    float value;
  } fbBuf;

  std::ofstream out;
};

#endif
