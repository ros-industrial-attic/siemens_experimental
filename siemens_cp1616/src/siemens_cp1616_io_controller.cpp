/*********************************************************************************************//**
* @file siemens_cp1616_io_controller.cpp
* 
* siemens_cp1616_io_controller class
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
#ifndef SIEMENS_CP1616_IO_CONTROLLER_CPP
#define SIEMENS_CP1616_IO_CONTROLLER_CPP

#include <siemens_cp1616/siemens_cp1616_io_controller.h>
#include <siemens_cp1616/siemens_cp1616_io_controller_callbacks.h>

namespace siemens_cp1616
{
//Define and initialize controller instance_ to zero value
Cp1616IOController *Cp1616IOController::controller_instance_ = 0;

const std::string INPUT = "input";
const std::string OUTPUT = "output";

Cp1616IOController* Cp1616IOController::createControllerInstance(std::string filepath)
{
  if( !controller_instance_ )
  {
    controller_instance_ = new Cp1616IOController(filepath);
  }
  return controller_instance_;
}

Cp1616IOController* Cp1616IOController::getControllerInstance()
{
  return controller_instance_;
}

Cp1616IOController::Cp1616IOController(std::string filepath) :
  cp_ready_(0),
  sem_mod_change_(0),
  cp_id_(1),
  cp_handle_(0),
  cp_current_mode_(PNIO_MODE_OFFLINE),
  cp_local_state_(PNIO_S_GOOD),
  input_module_count_(0),
  output_module_count_(0),
  input_module_total_data_size_(0),
  output_module_total_data_size_(0),
  num_of_input_modules_(0),
  num_of_output_modules_(0)
{
  try
  {
    //Parse data from yaml config file
    parseConfigFile(filepath);
  }
  catch(int e)
  {
    ROS_ERROR("Not able to parse the yaml file");
    throw;
  }   
  
  //Allocate memory for Input module variables
  input_data_length_.resize(num_of_input_modules_);
  input_data_state_.resize(num_of_input_modules_);
  input_data_address_.resize(num_of_input_modules_);

  //Allocate memory for Output module variables
  output_data_length_.resize(num_of_output_modules_);  
  output_data_state_.resize(num_of_output_modules_);
  output_data_address_.resize(num_of_output_modules_);
  
}

Cp1616IOController::~Cp1616IOController()
{
  
}

void Cp1616IOController::configureControllerData()
{
  //Allocate memory registered input modules
  if(input_module_count_ > 0)      //if any input module available
  {
    std::vector<PNIO_UINT8> temp;  //temporary row
    unsigned int i,j;
    
    for(i = 0; i < input_module_count_; i++)  //allocate memory for 2D array of input data (variable row size)
    {
      for(j = 0; j < input_data_length_[input_module_count_]; j++)
      temp.push_back(0);//fill temp row with zeros according to input_data_length_ values

      input_data_.push_back(temp);  //add row to input_data_
      temp.clear();                 //clear temp row
    }
     

    //Print InData array information
    ROS_INFO_STREAM("Input Data Array: [Total size: " << input_module_total_data_size_ << " bytes]");
    for(i = 0; i < input_module_count_; i++)
    {
      if((input_data_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("I"<< input_data_address_[i].u.Addr
          << "-" << input_data_address_[i].u.Addr + input_data_length_[i] - 1
          << " " << input_modules_[i].topic  << " [" << input_modules_[i].size << " byte]");
      }
      else
      {
        ROS_INFO_STREAM("I"<< input_data_address_[i].u.Addr
          << "-" << input_data_address_[i].u.Addr + input_data_length_[i] - 1
          << " " << input_modules_[i].topic  << " [" << input_modules_[i].size << " bytes]");
      }
    }
  }

  //Allocate memory for registered output modules
  if(output_module_count_ > 0)     //if any output module available
  {
    std::vector<PNIO_UINT8> temp;  //temporary row
    unsigned int i,j;
   
    for(i = 0; i < output_module_count_; i++)  //allocate memory for 2D array of input data (variable row size)
    {
      for(j = 0; j < output_data_length_[output_module_count_]; j++)
      temp.push_back(0);    //fill temp row with zeros according to input_data_length_ values

      output_data_.push_back(temp);  //add row to input_data_
      temp.clear();                  //clear temp row
    }

    //Print OutData array information
    ROS_INFO_STREAM("Output Data Array: [Total size: " << output_module_total_data_size_ << " byte]");
    for(i = 0; i < output_module_count_; i++)
    {
      if((output_data_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("Q"<< output_data_address_[i].u.Addr
          << "-" << output_data_address_[i].u.Addr + output_data_length_[i] - 1
          << " " << output_modules_[i].topic  << " [" << output_modules_[i].size << " bytes]");
      }
      else
      {
         ROS_INFO_STREAM("Q" << output_data_address_[i].u.Addr
           << "-" << output_data_address_[i].u.Addr + output_data_length_[i] - 1
           << " " << output_modules_[i].topic  << " [" << output_modules_[i].size << " bytes]");
      }
    }
  }
}

int Cp1616IOController::addInputModule(unsigned int input_size, unsigned int input_start_address)
{
  if(cp_current_mode_ == PNIO_MODE_OPERATE)
  {
    ROS_ERROR("Not able to add Input module in Operate state!");
    return PNIO_ERR_SEQUENCE;
  }
  else if(input_module_count_ >= num_of_input_modules_)
  {
    ROS_ERROR("Not able to add antoher input module. Max count reached!");
    return PNIO_ERR_SEQUENCE;
  }
  else
  {
    //Set variables required for PNIO_data_read function
    input_data_length_[input_module_count_] = input_size;                      //number of transferred bytes
    input_data_state_[input_module_count_]  = PNIO_S_BAD;                      //initial Input State
    input_data_address_[input_module_count_].AddrType = PNIO_ADDR_LOG;         //Address type
    input_data_address_[input_module_count_].IODataType = PNIO_IO_IN;          //Data type
    input_data_address_[input_module_count_].u.Addr = input_start_address;     //Module memory address

    
    ROS_DEBUG("Input module: Size: %d I: %d - %d",
      input_data_length_[input_module_count_],
      input_data_address_[input_module_count_].u.Addr,
      input_data_address_[input_module_count_].u.Addr + input_data_length_[input_module_count_] - 1);
    
    input_module_count_++;                         //increment input_module_count_
    input_module_total_data_size_ += input_size;   //save total input data size

    return PNIO_OK;
  }
}

int Cp1616IOController::addOutputModule(unsigned int output_size, unsigned int output_start_address)
{
    //Set variables required for PNIO_data_write function
    output_data_length_[output_module_count_] = output_size;                   //number of transferred bytes
    output_data_state_[output_module_count_]  = PNIO_S_BAD;                    //initial Input State
    output_data_address_[output_module_count_].AddrType = PNIO_ADDR_LOG;       //Address type
    output_data_address_[output_module_count_].IODataType = PNIO_IO_OUT;       //Data type
    output_data_address_[output_module_count_].u.Addr = output_start_address;  //Module memory address

  
    ROS_DEBUG("Output module: Size: %d Q: %d - %d",
      output_data_length_[output_module_count_],
      output_data_address_[output_module_count_].u.Addr,
      output_data_address_[output_module_count_].u.Addr + output_data_length_[output_module_count_] - 1);
    
    output_module_count_++;                              //increment output_module_count_
    output_module_total_data_size_ += output_size;       //save total output data size

    return PNIO_OK;
}

int Cp1616IOController::init()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Add input modules as defined in yaml file
  for(unsigned int i = 0; i < num_of_input_modules_; i++)
    addInputModule(input_modules_[i].size, input_modules_[i].starting_address);
  
  //Add output modules as defined in yaml file
  for(unsigned int i = 0; i < num_of_output_modules_; i++)
    addOutputModule(output_modules_[i].size, output_modules_[i].starting_address);
    
  //Configure Controller Data
  configureControllerData();

  //Open CP and obtain handle
  error_code = PNIO_controller_open(
              cp_id_,
              PNIO_CEP_MODE_CTRL,
              &pnio_controller_callbacks::dsReadConf,
              &pnio_controller_callbacks::dsWriteConf,
              &pnio_controller_callbacks::alarmIndication,
              &cp_handle_);

  //Check errors
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to open PNIO_controller: Error: 0x%x", (int)error_code);
    return (int)error_code;
  }
  else
    ROS_INFO("Openning CP1616 in IO_controller mode: done");

  //register modeChangeIndication callback for mode changes confirmation
  error_code = PNIO_register_cbf(
                cp_handle_,
                PNIO_CBE_MODE_IND,
                &pnio_controller_callbacks::modeChangeIndication);

  //Check errors
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Error in PNIO_register_cbf: callbackForModeChangeIndication: 0x%x", (int)error_code);
    PNIO_close(cp_handle_);
    return (int)error_code;
  }

  //register deviceActivation callback for device activation confirmation
  error_code = PNIO_register_cbf(
                cp_handle_,
                PNIO_CBE_DEV_ACT_CONF,
                &pnio_controller_callbacks::deviceActivation);

  if(error_code != PNIO_OK)
  {
    ROS_ERROR_STREAM("Error in PNIO_register_cbf: callbackForDeviceActivation: 0x%x" << (int)error_code);
    PNIO_close(cp_handle_);
    return (int)error_code;
  }

  //Change CP mode to Operate
  error_code = changePnioMode(PNIO_MODE_OPERATE);
  if(error_code != PNIO_OK)
  {
      PNIO_close(cp_handle_);
      return (int)error_code;
  }

  //Start communication
  unsigned int i = 0;

  while(!cp_ready_ ) //Wait for cp_ready flag to be set by alarmIndication callback
  {
    i++;
    usleep(WAIT_FOR_CALLBACKS_PERIOD);
    if(i == MAX_NUM_OF_INIT_ATTEMPTS)
    {
      ROS_ERROR("Not able to start communication, Uninitializing...");
      return PNIO_ERR_NO_CONNECTION;
    }    
  }
  ROS_INFO("IO Controller ready, communication started");
  return (int)error_code;  //if everything OK return PNIO_OK
}

int Cp1616IOController::uinit()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Change CP mode to OFFLINE
  error_code = changePnioMode(PNIO_MODE_OFFLINE);
  if(error_code != PNIO_OK)
  {
      PNIO_close(cp_handle_);
      return (int)error_code;
  }

  error_code = PNIO_close(cp_handle_);

  if(error_code != PNIO_OK)
    ROS_ERROR_STREAM("Not able to uninitialize IO_Controller: Error 0x%x" << (int)error_code);
  else
    ROS_INFO("Uninitializing IO_Controller: done");

  return (int)error_code;  //if everything OK return PNIO_OK
}

int Cp1616IOController::changePnioMode(PNIO_MODE_TYPE requested_mode)
{
  PNIO_UINT32 error_code;
  PNIO_UINT32 valid_cp_handle = cp_handle_;

  //set required mode
  error_code = PNIO_set_mode(cp_handle_, requested_mode);

  if(error_code != PNIO_OK)
  {
    ROS_ERROR_STREAM("Not able to change IO_Controller mode: Error 0x%x" << (int)error_code);
    PNIO_close(cp_handle_);     //Close PNIO_Controller
    return (int)error_code;
  }

  if(cp_handle_ == valid_cp_handle)  //check if cp_handle_ still valid
  {
    //wait for a callback_for_mode_change_indication
    while(!sem_mod_change_){
    usleep(WAIT_FOR_CALLBACKS_PERIOD);
    }

    setSemModChange(0);
  }

  //check if the current mode is correct
  if(cp_current_mode_ != requested_mode)
  {
    ROS_ERROR("Not able to set required mode: ERROR another mode recieved");
    PNIO_close(cp_handle_);
  }
  else
    ROS_INFO("Changing IO_controller mode: done");

  return (int)error_code;
}

int Cp1616IOController::updateCyclicInputData()
{
  PNIO_UINT32 error_code;
  PNIO_UINT32 bytes_read;

  unsigned int i;

  for(i = 0; i < input_module_count_; i++)
  {
    error_code = PNIO_data_read(
                    cp_handle_,                             // handle
                    &input_data_address_[i],                // pointer to device input address
                    input_data_length_[i],                  // length in bytes of input
                    &bytes_read,                            // number of bytes read
                    &input_data_[i][0],                     // pointer to input data row
                    cp_local_state_,                        // local status
                    (PNIO_IOXS*)&(input_data_state_[i]));   // remote status

    if(error_code != PNIO_OK)
      ROS_DEBUG("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x", (int)error_code);      
  }

  return (int)error_code;
}

int Cp1616IOController::updateCyclicOutputData()
{
  PNIO_UINT32 error_code;
  unsigned int i;

  for(i = 0; i < output_module_count_; i++)
  {
    error_code = PNIO_data_write(
                    cp_handle_,                             // handle
                    &output_data_address_[i],               // pointer to device output address
                    output_data_length_[i],                 // length in bytes of output
                    &output_data_[i][0],                    // pointer to output data row
                    cp_local_state_,                        // local status
                    (PNIO_IOXS*)&(output_data_state_[i]));  // remote status

    if(error_code != PNIO_OK)
        ROS_DEBUG("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x", (int)error_code);     
  }

  return (int)error_code;
}

void Cp1616IOController::setCpReady(int cp_ready_value)
{
  cp_ready_ = cp_ready_value;
}

int Cp1616IOController::getCpReady()
{
  return (cp_ready_);
}

void Cp1616IOController::setCpCurrentModeFlag(PNIO_MODE_TYPE mode)
{
  if((mode == PNIO_MODE_OFFLINE) || (mode == PNIO_MODE_CLEAR) || (mode == PNIO_MODE_OPERATE))
    cp_current_mode_ = mode;
  else
    ROS_WARN("Not able to set cp_current_mode_ to %d state", (int)mode);
}

PNIO_MODE_TYPE Cp1616IOController::getCpCurrentModeFlag()
{
  return(cp_current_mode_);
}

void Cp1616IOController::setSemModChange(int mod_change)
{
  sem_mod_change_ = mod_change;
}

int Cp1616IOController::parseConfigFile(std::string filepath)
{
  PNIO_UINT32 error_code = PNIO_OK;  
   
  try
  {
    YAML::Node config = YAML::LoadFile(filepath);
    ControllerModuleData temp_module;
    
    for(std::size_t i = 0; i < config.size(); i++)
    {  
      temp_module.label            = config[i]["label"].as<std::string>();
      temp_module.type             = config[i]["type"].as<std::string>();
      temp_module.size             = config[i]["size"].as<int>();
      temp_module.starting_address = config[i]["starting_address"].as<int>();
      temp_module.topic            = config[i]["topic"].as<std::string>();  
  
      ROS_DEBUG_STREAM("Label[" << i << "]: " << config[i]["label"].as<std::string>());
      ROS_DEBUG_STREAM("Type["  << i << "]: " << config[i]["type"].as<std::string>());
      ROS_DEBUG_STREAM("Size["  << i << "]: " << config[i]["size"].as<int>());
      ROS_DEBUG_STREAM("Add["   << i << "]: " << config[i]["starting_address"].as<int>());
      ROS_DEBUG_STREAM("Topic[" << i << "]: " << config[i]["topic"].as<std::string>());
      
      if(temp_module.type == INPUT)
      {
        num_of_input_modules_++;
        input_modules_.push_back(temp_module);
      }
  
      if(temp_module.type == OUTPUT)\
      {
        num_of_output_modules_++;
        output_modules_.push_back(temp_module);
      }
    }
    if((!num_of_input_modules_ ) && (!num_of_output_modules_ ))
    {
      ROS_WARN("No module found! Check configuration!");
      error_code = PNIO_ERR_NO_CONFIG;
    }
    else
    {
      ROS_INFO_STREAM("CP1616: Number of input modules: " << num_of_input_modules_);
      ROS_INFO_STREAM("CP1616: Number of output modules: " << num_of_output_modules_);
    }
  }
  catch(YAML::ParserException &e)
  {
    ROS_ERROR("Error reading yaml config file");
    error_code = PNIO_ERR_NO_CONFIG;
  }   
  
  return(error_code);
}

} //siemens_cp1616

#endif //SIEMENS_CP1616_IO_CONTROLLER_CPP
