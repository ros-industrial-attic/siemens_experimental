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
#ifndef CP1616_IO_DEVICE_H
#define CP1616_IO_DEVICE_H

//CP 1616 Annotations
#define ANNOT_NAME       "StarterKit"      // device type (String 25) 
#define ANNOT_ORDERID    "6GK1 161-6AA00"  // Order Id    (String 20) 
#define ANNOT_HW_REV     0                 // HwRevision  (short)     
#define ANNOT_SW_PREFIX  'V'               // SwRevisionPrefix (char) 
#define ANNOT_SW_REV_1   2                 // SwRevision1 (short)     
#define ANNOT_SW_REV_2   6                 // SwRevision2 (short)     
#define ANNOT_SW_REV_3   0                 // SwRevision3 (short)     

//Known Diagnose alarms for this device
#define CH_ERR_INVAL_LINKUP     0x0100
#define CH_ERR_INVAL_LINKDOWN   0x0101
#define CH_ERR_NO_REDUND_PS     0x0200
#define CH_ERR_NO_CPLUG         0x0201
#define CH_ERR_CPLUG_ERROR      0x0202

//Values for PNIO_device_open
#define NUMOF_SLOTS             2	   // DAP module + 1 slot
#define NUMOF_SUBSLOTS          2         // Number of subslots 
#define NUMOF_BYTES_PER_SUBSLOT 256       // Maximum data length as configured in the sample 
#define VENDOR_ID    0x002a
#define DEVICE_ID    0x0003
#define INSTANCE_ID  0x0001

#define DEVICE_DATA_ENTRIES     2          // The total number of members of DEVICE_DATA structure  

#include <ros/ros.h>

//IO Base headers
#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

namespace cp1616
{

/**
 * \brief Structure to keep STEP7 project data required for plugging modules and submodules
 * */

struct DeviceData
{
  unsigned int slot;
  unsigned int subslot;
  PNIO_UINT32 modId;
  PNIO_UINT32 subslotId;
  PNIO_UINT32 api;
  PNIO_UINT16 maxSubslots;
  int modState;
  int subState;
  int dir;
};

/**
 * \brief This class defines ROS-Profinet IO Device implementation for communication processor Siemens CP1616
 */

class Cp1616IODevice
{
public:
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
   * \brief Initializes Device data structures
   */
          void configureDeviceData();

  /**
   * \brief Adds associated Api
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int addApi();

  /**
   * \brief Removes associated Api
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int removeApi();

  /**
   * \brief Adds Modules and submodules (to mirror STEP7 configuration)
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int addModSubMod();

  /**
   * \brief Removes existing Modules and submodules
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int removeModSubMod();

  /**
   * \brief Starts IO device operation
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int startOperation();

  /**
   * \brief Stops IO device operation
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int stopOperation();

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
   * \brief This function is called after the PNIO_cbf_prm_end_ind callback has been
   * called, it calls PNIO_initiate_data_write(), PNIO_initiate_data_read for the first
   * time and PNIO_set_appl_state_ready
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int doAfterPrmEndIndCbf();

  /**
   * \brief This function is called after the PNIO_cbf_ar_indata_ind callback has been
   * called, it calls PNIO_initiate_data_write() and PNIO_initiate_data_read()
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int doAfterIndataIndCbf();

  /**
   * \brief This function returns the index in the IO data array for the given submodule.
   *
   * \return index for given submodule
   */
          int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub);

  /**
   * \brief Data encapsulation for ar_info_ind_flag_
   *
   * \param value
   */
          void setArInfoIndFlag(int value);

  /**
   * \brief Data encapsulation for ar_info_ind_flag_
   *
   * \return ar_info_ind_flag_
   */
          int  getArInfoIndFlag();

  /**
   * \brief Data encapsulation for prm_end_ind_flag_
   *
   * \param value
   */
          void setPrmEndIndFlag(int value);

  /**
   * \brief Data encapsulation for ar_info_ind_flag_
   *
   * \return prm_end_ind_flag_
   */
         int  getPrmEndIndFlag();

  /**
   * \brief Data encapsulation for indata_ind_flag_
   *
   * \param value
   */
          void setIndataIndFlag(int value);

  /**
   * \brief Data encapsulation for indata_ind_flag_
   *
   * \return indata_ind_flag
   */
          int getIndataIndFlag();

  /**
   * \brief Data encapsulation for offline_ind_flag_
   *
   * \param value
   */
          void setOfflineIndFlag(int value);

  /**
   * \brief Data encapsulation for offline_ind_flag_
   *
   * \return offline_ind_flag_
   */
          int  getOfflineIndFlag();

  /**
   * \brief Data encapsulation for cp_session_key_
   *
   * \param value
   */
          void setCpSessionKey(PNIO_UINT16 value);

  /**
   * \brief Data encapsulation for cp_session_key_
   *
   * \return cp_session_key_
   */
          PNIO_UINT16 getCpSessionKey();

  /**
   * \brief Data encapsulation for cp_ar_number_
   *
   * \param value
   */
          void setCpArNumber(PNIO_UINT16 value);

   /**
    * \brief Data encapsulation for cp_ar_number_
    *
    * \return cp_ar_number_
    */
          PNIO_UINT16 getCpArNumber();

  /**
   * \brief pointer to device_data table that holds STEP7 configuration
   */
          DeviceData *p_device_data_;

  /**
  * \brief Table of input controller data - IO Device output data
  */
          PNIO_UINT8   in_data_              [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];

  /**
   * \brief Table of input controller data lengths
   */
          PNIO_UINT32  in_data_len_          [NUMOF_SLOTS][NUMOF_SUBSLOTS];

  /**
   * \brief Table of Input controller consumer statuses
   */
          PNIO_IOXS    in_data_iocs_         [NUMOF_SLOTS][NUMOF_SUBSLOTS];

  /**
   * \brief Table of Input controller provider statuses
   */
          PNIO_IOXS    in_data_iops_         [NUMOF_SLOTS][NUMOF_SUBSLOTS];

  /**
   * \brief Table of Output controller data - IO Device input data
   */
          PNIO_UINT8   out_data_             [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];

  /**
   * \brief Table of Output controller data lengths
   */
          PNIO_UINT32  out_data_len_         [NUMOF_SLOTS][NUMOF_SUBSLOTS];

  /**
   * \brief Table of Output controller consumer statuses
   */
          PNIO_IOXS    out_data_iocs_        [NUMOF_SLOTS][NUMOF_SUBSLOTS];

  /**
   * \brief Table of Output controller provider statuses
   */
          PNIO_IOXS    out_data_iops_        [NUMOF_SLOTS][NUMOF_SUBSLOTS];

private:

  /**
   * \brief CP handle obtained by init() function
   */
          PNIO_UINT32 cp_handle_;
  
  /**
   * \brief Private constructor
   */
  Cp1616IODevice();

  /**
   * \brief Static instance pointer
   */
         static Cp1616IODevice *device_instance_;

  /**
   * \brief CP ID according to STEP7 project configuration
   */
         PNIO_UINT32 cp_id_;

   /**
    * \brief number of slots according to STEP7 project configuration
    */
          int cp_number_of_slots_;

  /**
   * \brief callback ar_info_ind_flag_
   */
          int ar_info_ind_flag_;

  /**
   * \brief callback prm_end_ind_flag_
   */
          int prm_end_ind_flag_;

  /**
   * \brief callback indata_ind_flag_
   */
          int indata_ind_flag_;

  /**
   * \brief callback ind_flag_
   */
          int offline_ind_flag_;

  /**
   * \brief session key
   */
          PNIO_UINT16 cp_session_key_;

  /**
   * \brief callback application-relation number
   */
          PNIO_UINT16 cp_ar_number_;

  /**
   * \brief table that holds indexes of IO data array for given submodules
   */
          int idx_table_[DEVICE_DATA_ENTRIES];
  
  /**
   * \brief const used by waiting for callbacks loops
   */
          static const int WAIT_FOR_CALLBACK_PERIOD = 100000;
	  
  /**
   * \brief max counter value for prm_end_ind() callback
   */
          static const int MAX_PRM_END_COUNT = 500;

  /**
   * \brief max counter value for prm_indata_ind() callback
   */
          static const int MAX_INDATA_IND_COUNT = 500;
  
  /**
   * \brief max counter value for ar_offline_ind() callback
   */  
          static const int MAX_OFFLINE_IND_COUNT = 100; 
	 
}; //cp1616_io_device class
}  //cp1616


#endif //CP1616_IO_DEVICE_H
