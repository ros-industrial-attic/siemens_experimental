/*********************************************************************************************//**
* @file cp1616_io_device.h
*
* Main cp1616_io_device node
*
* Copyright {2015} {Frantisek Durovsky}
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at

*      http://www.apache.org/licenses/LICENSE-2.0

*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.

* *********************************************************************************************/
#ifndef SIEMENS_CP1616_IO_DEVICE_H
#define SIEMENS_CP1616_IO_DEVICE_H

//CP 1616 Annotations
#define ANNOT_NAME       "StarterKit"      // device type (String 25) 
#define ANNOT_ORDERID    "6GK1 161-6AA00"  // Order Id    (String 20) 
#define ANNOT_HW_REV     0                 // HwRevision  (short)     
#define ANNOT_SW_PREFIX 'V'                // SwRevisionPrefix (char) 
#define ANNOT_SW_REV_1   2                 // SwRevision1 (short)     
#define ANNOT_SW_REV_2   6                 // SwRevision2 (short)     
#define ANNOT_SW_REV_3   0                 // SwRevision3 (short)     

//Values for PNIO_device_open
#define VENDOR_ID    0x002a
#define DEVICE_ID    0x0003
#define INSTANCE_ID  0x0001
#define MAX_ALARM    16
#define USER_HANDLE  0x1234               // User id for alarm handling in case of multiuser scenario
#define NUMOF_BYTES_PER_SLOT 256          // Maximum data length for one slot

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

//IO Base headers
#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

namespace siemens_cp1616
{
/**
 * \brief Struct to keep STEP7 module information parsed from yaml. config file
 */
struct DeviceModuleData
{
  std::string label;
  std::string type;
  int size;
  int slot;
  int subslot;
  PNIO_UINT32 modId;
  PNIO_UINT32 subslotId;
  PNIO_UINT32 api;
  PNIO_UINT16 maxSubslots;
  int modState;
  int subState;
  int dir;
  std::string topic;
};

/**
 * \brief Struct used for sending diagnostic alarms
 */
struct DiagAlarmData
{
  unsigned int chanNum;
  unsigned int chanProp;
  unsigned int chanErrType;
};

/**
 * \brief Struct used for sending diagnostic alarms
 */
struct DiagAlarmDataExt
{
  unsigned int chanNum;
  unsigned int chanProp;
  unsigned int chanErrType;
  unsigned int extChannelErrType;
  unsigned int extChannelAddValue;
};

/**
 * \brief Overloading extraction operator for yaml parsing
 */
void operator >> (const YAML::Node &node, DeviceModuleData &module);

/**
 * \brief This class defines ROS-Profinet IO Device implementation for communication processor Siemens CP1616
 */

class Cp1616IODevice
{
public:
  /**
   * \brief Function to create IO Device instance 
   */
  static Cp1616IODevice* createDeviceInstance(std::string filepath);

  /**
   * \brief Public instance accesssor
   */
  static Cp1616IODevice* getDeviceInstance();
  
  /**
   * \brief Destructs an IOController object
   */
  ~Cp1616IODevice();

  /**
   * \brief Initializes and starts IO Device
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int init();

  /**
   * \brief Closes and uninitializes IO Device
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int uinit();

  /**
   * \brief Updates Output (from IO Controller perspective) data - read CP input data
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int updateCyclicOutputData();

  /**
   * \brief Updates Input (from IO Controller perspective) data - write CP output data
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int updateCyclicInputData();
      
  /**
   * \brief Sends diagnostic alarm message to the IO Controller
   *
   * \param channel_error_type Type of error - alarm type
   * \param alarm_slot diagnostic tag/alarm handle
   * 
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int sendDiagnosticAlarm(PNIO_UINT32 channel_error_type, PNIO_UINT16 alarm_slot);
  
  /**
   * \brief Reset diagnostic alarm 
   *
   * \param alarm_slot diagnostic tag/alarm handle
   * 
   * \return error_code (see pnioerrx.h for detailed description)
   */
  int resetDiagnosticAlarm(PNIO_UINT16 alarm_slot);
    
  /**
   * \brief Container which holds STEP7 configuration params parsed from yaml file
   */
  std::vector<DeviceModuleData> modules_;
 
  /**
   * \brief Input module data
   */
  std::vector<std::vector<PNIO_UINT8> > input_data_;
  std::vector<PNIO_UINT32> input_data_length_;
  std::vector<PNIO_IOXS>   input_data_iocs_;
  std::vector<PNIO_IOXS>   input_data_iops_;
        
  /**
   * \brief Output module data
   */
  std::vector<std::vector<PNIO_UINT8> > output_data_;
  std::vector<PNIO_UINT32> output_data_length_;
  std::vector<PNIO_IOXS>   output_data_iocs_;
  std::vector<PNIO_IOXS>   output_data_iops_;
 
  void setArInfoIndFlag(int value);
  void setPrmEndIndFlag(int value);
  void setIndataIndFlag(int value);
  void setOfflineIndFlag(int value);
  void setCpSessionKey(PNIO_UINT16 value);
  void setCpArNumber(PNIO_UINT16 value);
    
  int getArInfoIndFlag();
  int getPrmEndIndFlag();
  int getIndataIndFlag();
  int getOfflineIndFlag();
  int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub);
  PNIO_UINT16 getCpSessionKey();
  PNIO_UINT16 getCpArNumber();  
  
private:  
  Cp1616IODevice(std::string filepath);
  static Cp1616IODevice *device_instance_;
  
  int parseConfigFile(std::string filepath);
  void configureDeviceData();
  int addApi();
  int removeApi();
  int addModSubMod();
  int removeModSubMod();
  int doAfterIndataIndCbf();
  int doAfterPrmEndIndCbf();
  int startOperation();
  int stopOperation();
  
  std::vector<DiagAlarmDataExt> alarms_;
  
  PNIO_UINT32 cp_handle_;
  PNIO_UINT32 cp_id_;
                  
  int ar_info_ind_flag_;
  int prm_end_ind_flag_;
  int indata_ind_flag_;
  int offline_ind_flag_;
  PNIO_UINT16 cp_session_key_;
  PNIO_UINT16 cp_ar_number_;

  std::vector<int>idx_table_;
    
  static const int MAX_NUMBER_OF_SUBSLOTS = 1;
  static const int WAIT_FOR_CALLBACK_PERIOD = 100000;
  static const int MAX_PRM_END_COUNT = 500;
  static const int MAX_INDATA_IND_COUNT = 500;
  static const int MAX_OFFLINE_IND_COUNT = 100; 
   
  static const int CH_ERR_INVAL_LINKUP = 0x0100;
  static const int CH_ERR_INVAL_LINKDOWN = 0x0101;
  static const int CH_ERR_NO_REDUND_PS = 0x0200;
  static const int CH_ERR_NO_CPLUG = 0x0201;
  static const int CH_ERR_CPLUG_ERROR = 0x0202;
  static const int CH_ERR_OVERLOAD = 0x0004;
  static const int CH_ERR_DATA_TRANS_IMP = 0x8000;
  static const int CH_ERR_PS_FAIL = 0x0011;
  static const int CH_ERR_MANSPEC = 0x0101;
  static const int CH_ERR_GROUND_FAULT = 0x0014;  
  
}; //cp1616_io_device class
}  //siemens_cp1616


#endif //SIEMENS_CP1616_IO_DEVICE_H
