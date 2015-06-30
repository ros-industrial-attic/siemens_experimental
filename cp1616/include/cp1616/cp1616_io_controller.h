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

/**
 * \brief This class defines ROS-Profinet IO Controller implementation for communication processor Siemens CP1616
 */

class Cp1616IOController
{
public:

  /**
   * \brief Public instance accesssor
   */
          static Cp1616IOController* getControllerInstance();

  /**
   * \brief Destructs an IOController object
   */
          ~Cp1616IOController();

  /**
   * \brief Initializes and starts IO Controller
   *
   * \return error_code if succeded (see pnioerrx.h for detailed description)
   */
          int init();

  /**
   * \brief Closes and uninitializes IO Controller
   *
   * \return error_code if succeded (see pnioerrx.h for detailed description)
   */
          int uinit();

   /**
    * \brief Initialize Controller data structures
    */
          void configureControllerData();

   /**
   * \brief Adds input module to Controller data structre
   *
   * \param input_size  input data length according to STEP7 setup
   * \param input_start_address I address according to STEP7 setup
   *
   * \return 0 if input module added successfully
   */
          int addInputModule(unsigned int input_size, unsigned int input_start_address);
  /**
   * \brief Adds output module to Controller data structure
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
          int updateCyclicInputData();

  /**
   * \brief Writes data to IO Base library layer for transmission
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int updateCyclicOutputData();

  /**
   * \brief Changes IO Controller mode
   *
   * \param requested_mode OFFLINE/OPERATE/CLEAR
   *
   * \return error_code (see pnioerrx.h for detailed description)
   */
          int changeAndWaitForPnioMode(PNIO_MODE_TYPE requested_mode);

  /**
   * \brief Sets IO Controller OutData
   *
   * \param module output module index according to addOutputModule order
   * \param data_index index of out_data_
   * \param value requested value
   */
          void setOutData(unsigned int module, unsigned int data_index, PNIO_UINT8 value);

  /**
   * \brief Sets IO Controller cp_ready_ variable
   *
   * \param cp_ready_value
   */
          void setCpReady(int cp_ready_value);

  /**
   * \brief Returns IO Controller cp_ready_ state (used in callbackForAlarmIndication)
   *
   * \return cp_ready_ flag
   */
          int getCpReady();

  /**
   * \brief Sets IO Controller cp_current_mode_ flag (used in callbackForModeChangeIndication)
   *
   * \param mode - current mode
   */
          void setCpCurrentModeFlag(PNIO_MODE_TYPE mode);

  /**
   * \brief Returns IO Controller cp_current_mode_ flag
   *
   * \return cp_current_mode_
   */
          PNIO_MODE_TYPE getCpCurrentModeFlag();

  /**
   * \brief Sets sem_mode_change_ flag (used in callbackForModeChangeIndication)
   *
   * \param value
   */
          void setSemModChange(int mod_change);

  /** Debugging functions  */
  void printOutputData(unsigned int module);
  void printInputData(unsigned int module);

private:

  /**
   * \brief Constructor
   */
         Cp1616IOController();

  /**
   * \brief Static instance pointer
   */
         static Cp1616IOController *controller_instance_;

  /**
   * \brief CP ID according to TIA portal setup (default = 1)
   */
          PNIO_UINT32 cp_id_;

  /**
   * brief CP Handle obtained by PNIO_controller_open function
   */
          PNIO_UINT32 cp_handle_;

  /**
   * \brief CP ready communication flag
   */
          int cp_ready_;

  /**
   * \brief CP current mode (see PGH_IO-Base_76.pdf )
   */
          /*volatile*/ PNIO_MODE_TYPE cp_current_mode_;

   /**
   * \brief CP local state obtained by PNIO_data_write/PNIO_data_read functions
   */
          /*volatile*/ PNIO_IOXS cp_local_state_;

   /**
    * \brief flag used by CallbackForModeChangeIndication
    */
          int sem_mod_change_;

  /**
   * \brief counter of active input modules
   */
          PNIO_UINT32 device_input_count_;

  /**
   * \brief array of input module data lenghts
   */
          PNIO_UINT32 *device_input_length_;

  /**
   * \brief array of device input states
   */
          PNIO_IOXS /*volatile* volatile*/ *device_input_state_;

  /**
   * \brief array of device input addresses
   */
          PNIO_ADDR *device_input_address_;


  /**
   * \brief counter of active output modules
   */
          PNIO_UINT32 device_output_count_;

  /**
   * \brief array of output module data lenghts
   */
          PNIO_UINT32 *device_output_length_;

  /**
   * \brief array of device output states
   */
          PNIO_IOXS /*volatile* volatile*/ *device_output_state_;

  /**
   * \brief array of device output addresses
   */
          PNIO_ADDR *device_output_address_;

  /**
   * \brief array of input module data
   */
          PNIO_UINT8 *in_module_data_;

  /**
   * \brief array of arrays of input module data
   */
          PNIO_UINT8 **in_data_;

  /**
   * \brief total size of input data
   */
          unsigned int total_input_size_;

  /**
   * \brief array of output module data
   */
          PNIO_UINT8 *out_module_data_;

  /**
   * \brief array of arrays of output module data
   */
          PNIO_UINT8 **out_data_;

  /**
   * \brief total size of output data
   */
          unsigned int total_output_size_;

};

#endif //CP1616_IO_CONTROLLER_H
