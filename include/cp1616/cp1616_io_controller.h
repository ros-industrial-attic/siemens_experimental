/*********************************************************************************************//**
* @file cp1616_io_controller.h
* 
* cp1616_io_controller header 
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
#ifndef CP1616_IO_CONTROLLER_H
#define CP1616_IO_CONTROLLER_H

#include <ros/ros.h>

//IO base headers
#include "pniousrx.h"
#include "pnioerrx.h"

#define NUM_OF_INPUT_MODULES    4       //Fixed definitions for development purposes
#define NUM_OF_OUTPUT_MODULES   4

#define MAX_NUM_OF_INIT_ATTEMPTS 1000   //Number of attempts to initialize communication

void* cp1616_object;   //global variable for handling Callbacks

//Abstract IO Controller class
class IOController
{
public:
  virtual int addInputModule(unsigned int input_size, unsigned int input_start_address) = 0;
  virtual int addOutputModule(unsigned int output_size, unsigned int output_start_address) = 0;
  virtual int init() = 0;
  virtual int uinit() = 0;

private:

};

//Singleton patern
class Cp1616CallbackHandler
{
public:
    static Cp1616CallbackHandler* getInstance();
    ~Cp1616CallbackHandler();

private:
    static bool instance_flag_;
    static Cp1616CallbackHandler *instance_;
    Cp1616CallbackHandler();
};

/**
 * \brief This class defines ROS-Profinet IO Controller implementation for communication processor Siemens CP1616

  */
class Cp1616IOController: public IOController
{
public:
  /**
   * \brief Constructs an IOController object
   */
          Cp1616IOController();

  /**
   * \brief Destructs an IOController object
   */
          ~Cp1616IOController();

  /**
   * \brief Initializes and starts IOController
   *
   * \return 0 if succeded
   */
          int init();

  /**
   * \brief Closes and uninitializes IOController
   *
   * \return 0 if succeded
   */
          int uinit();

  /**
   * \brief Initializes input module variables
   *
   * \param input_size  input data length according to STEP7 setup
   * \param input_start_address I address according to STEP7 setup
   *
   * \return 0 if input module added successfully
   */
          int addInputModule(unsigned int input_size, unsigned int input_start_address);
  /**
   * \brief Initializes output module variables
   *
   * \param output_size output data length according to STEP7 setup
   * \param output_start_address Q address according to STEP7 setup
   *
   * \return 0 if input module added successfully
   */

          int addOutputModule(unsigned int output_size, unsigned int output_start_address);

  /**
   * \brief Reads recieved data from IO Base layer
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          PNIO_UINT32 updateCyclicInputData();

  /**
   * \brief Writes data to IO Base library layer for transmission
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          PNIO_UINT32 updateCyclicOutputData();

  /**
   * \brief Changes IO Controller mode
   *
   * \param requested_mode OFFLINE/OPERATE/CLEAR
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          PNIO_UINT32 changeAndWaitForPnioMode(PNIO_MODE_TYPE requested_mode);

  /**
   * \brief
   *
   * \param module output module index according to addOutputModule order
   * \param data_index index of out_data_
   * \param value requested value
   */
          void setOutData(unsigned int module, unsigned int data_index, PNIO_UINT8 value);

  /**
   * \brief CP ready communication flag
   */
          int cp_ready_;

  /** Debugging functions  */
  void printOutputData(unsigned int module);
  void printInputData(unsigned int module);

private:

  /**
   * \brief IO Controller local mode has changed
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          static void callbackForModeChangeIndication(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief Signals connection status to IO device
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          static void callbackForDeviceActivation(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief Alarm Indication
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          static void callbackForAlarmIndication(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          static void callbackForDsReadConf(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          static void callbackForDsWriteConf(PNIO_CBE_PRM *pCbfPrm);

  /**
   * brief flag used by CallbackForModeChangeIndication
   */
          int sem_mod_change_;

  /**
   * brief CP ID according to TIA portal setup (default = 1)
   */
          PNIO_UINT32 cp_id_;

  /**
   * brief CP Handle obtained by PNIO_controller_open function
   */
          PNIO_UINT32 cp_handle_;

  /**
   * brief CP current mode (see PGH_IO-Base_76.pdf )
   */
          /*volatile*/ PNIO_MODE_TYPE cp_current_mode_;

  /**
   * brief CP local state obtained by PNIO_data_write/PNIO_data_read functions
   */
          /*volatile*/ PNIO_IOXS      cp_local_state_;

  /**
   * brief counter of active input modules
   */
          PNIO_UINT32 device_input_count_;

  /**
   * brief array of input module data lenghts
   */
          PNIO_UINT32 *device_input_length_;

  /**
   * brief array of device input states
   */
          PNIO_IOXS /*volatile* volatile*/ *device_input_state_;

  /**
   * brief array of device input addresses
   */
          PNIO_ADDR *device_input_address_;


  /**
   * brief counter of active output modules
   */
          PNIO_UINT32 device_output_count_;

  /**
   * brief array of output module data lenghts
   */
          PNIO_UINT32 *device_output_length_;

  /**
   * brief array of device output states
   */
          PNIO_IOXS /*volatile* volatile*/ *device_output_state_;

  /**
   * brief array of device output addresses
   */
          PNIO_ADDR *device_output_address_;

  /**
   * brief array of input module data
   */
          PNIO_UINT8 *in_module_data_;

  /**
   * brief array of arrays of input module data
   */
          PNIO_UINT8 **in_data_;

  /**
   * brief total size of input data
   */
          unsigned int total_input_size_;

  /**
   * brief array of output module data
   */
          PNIO_UINT8 *out_module_data_;

  /**
   * brief array of arrays of output module data
   */
          PNIO_UINT8 **out_data_;

  /**
   * brief total size of output data
   */
          unsigned int total_output_size_;

};

#endif //CP1616_IO_CONTROLLER_H
