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
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

//IO base headers
#include "pniousrx.h"
#include "pnioerrx.h"

namespace cp1616
{

/**
 * \brief Struct to keep STEP7 module information parsed from yaml. config file
 */
  
struct ControllerModuleData
{
  std::string id;
  std::string type;
  int size;
  int starting_address;
  std::string topic;
};

/**
 * \brief Overloading extraction operator for yaml parsing
 */
void operator >> (const YAML::Node &node, ControllerModuleData &module);

/**
 * \brief This class defines ROS-Profinet IO Controller implementation for communication processor Siemens CP1616
 */

class Cp1616IOController
{
public:

  /**
   * \brief Public instance accesssors
   */
  static Cp1616IOController* getControllerInstance();
  static Cp1616IOController* getControllerInstance(std::string filepath);  

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
  int changePnioMode(PNIO_MODE_TYPE requested_mode);

  /**
   * \brief 2D array of input module data
   */
  std::vector<std::vector<PNIO_UINT8> > input_module_data_;  
  
  /**
   * \brief 2D array of output module data
   */
  std::vector<std::vector<PNIO_UINT8> > output_module_data_;
  
  /** Data encapsulation **/
  void setSemModChange(int mod_change);
  void setCpReady(int cp_ready_value);
  int getCpReady();

  void setCpCurrentModeFlag(PNIO_MODE_TYPE mode);
  PNIO_MODE_TYPE getCpCurrentModeFlag();
  
  /** Debugging functions  */
  void printOutputData(unsigned int module);
  void printInputData(unsigned int module);

private:

  Cp1616IOController(std::string filepath);
  static Cp1616IOController *controller_instance_;

  PNIO_UINT32 parseConfigFile(std::string filepath);
  
  PNIO_UINT32 cp_handle_;
  PNIO_UINT32 cp_id_;
 
  unsigned int num_of_input_modules_;
  unsigned int num_of_output_modules_;
  
  std::vector<ControllerModuleData> input_modules_;
  std::vector<ControllerModuleData> output_modules_;
   
  int cp_ready_;
  int sem_mod_change_;
  volatile PNIO_MODE_TYPE cp_current_mode_;
  volatile PNIO_IOXS cp_local_state_;
   
  PNIO_UINT32 input_module_count_;
  unsigned int input_module_total_data_size_;
  std::vector<PNIO_UINT32> input_module_data_length_; 
  std::vector<PNIO_IOXS>   input_module_state_;    //PNIO_IOXS volatile* volatile device_input_state_;
  std::vector<PNIO_ADDR>   input_module_address_;
             
  PNIO_UINT32 output_module_count_;
  unsigned int output_module_total_data_size_;
  std::vector<PNIO_UINT32> output_module_data_length_;
  std::vector<PNIO_IOXS>   output_module_state_;   //PNIO_IOXS volatile* volatile device_output_state_;
  std::vector<PNIO_ADDR>   output_module_address_;
   
  static const int WAIT_FOR_CALLBACKS_PERIOD = 100000;
  static const int MAX_NUM_OF_INIT_ATTEMPTS = 1000;
  static const int INIT_DATA_VALUE = 0;
  	  
}; //cp1616_io_controller class

} //cp1616

#endif //CP1616_IO_CONTROLLER_H
