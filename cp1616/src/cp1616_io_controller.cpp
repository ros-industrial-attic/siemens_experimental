/*********************************************************************************************//**
* @file cp1616_io_controller.cpp
* 
* cp1616_io_controller class
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
#ifndef CP1616_IO_CONTROLLER_CPP
#define CP1616_IO_CONTROLLER_CPP

#include <cp1616/cp1616_io_controller.h>
#include <cp1616/cp1616_io_controller_callbacks.h>

namespace cp1616
{
//Define and initialize controller instance_ to zero value
Cp1616IOController *Cp1616IOController::controller_instance_ = 0;

Cp1616IOController* Cp1616IOController::getControllerInstance()
{
  if( !controller_instance_ )
  {
    controller_instance_ = new Cp1616IOController();
  }
  return controller_instance_;
}

Cp1616IOController::Cp1616IOController() :
  cp_ready_(0),
  sem_mod_change_(0),
  cp_id_(1),
  cp_handle_(0),
  cp_current_mode_(PNIO_MODE_OFFLINE),
  cp_local_state_(PNIO_S_GOOD),
  input_module_count_(0),
  output_module_count_(0),
  input_module_total_data_size_(0),
  output_module_total_data_size_(0)
{
  //Allocate memory for Input module variables
  input_module_data_length_.resize(NUM_OF_INPUT_MODULES);
  input_module_state_.resize(NUM_OF_INPUT_MODULES);
  input_module_address_.resize(NUM_OF_INPUT_MODULES);

  //Allocate memory for Output module variables
  output_module_data_length_.resize(NUM_OF_OUTPUT_MODULES);  
  output_module_state_.resize(NUM_OF_OUTPUT_MODULES);
  output_module_address_.resize(NUM_OF_OUTPUT_MODULES);
}

Cp1616IOController::~Cp1616IOController()
{
  
}

void Cp1616IOController::configureControllerData()
{
  //Allocate memory registered input modules
  if(input_module_count_ > 0)    //if any input module available
  {
    std::vector<PNIO_UINT8> temp;  //a temporary row
    unsigned int i,j;
    
    for(i = 0; i < input_module_count_; i++)  //allocate memory for 2D array of input data (variable row size)
    {
      for(j = 0; j < input_module_data_length_[input_module_count_]; j++)
      temp.push_back(0);//fill temp row with zeros according to input_module_data_length_ values

      input_module_data_.push_back(temp);  //add row to input_module_data_
      temp.clear();                        //clear temp row
    }
     

    //Print InData array information
    ROS_INFO_STREAM("Input Data Array: [Total size: " << input_module_total_data_size_ << " bytes]");
    for(i = 0; i < input_module_count_; i++)
    {
      if((input_module_data_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("I: "<< input_module_address_[i].u.Addr
          << " - " << input_module_address_[i].u.Addr + input_module_data_length_[i] - 1
          << ": InData  "  << "[" << i << "]" << "[0]");
      }
      else
      {
        ROS_INFO_STREAM("I: "<< input_module_address_[i].u.Addr
          << " - " << input_module_address_[i].u.Addr + input_module_data_length_[i] - 1
          << ": InData  "  << "[" << i << "]" << "[0] ... "
          << "[" << i << "]" << "[" << input_module_data_length_[i]-1 << "]");
      }
    }
  }

  //Allocate memory for registered output modules
  if(output_module_count_ > 0)    //if any output module available
  {
    std::vector<PNIO_UINT8> temp;  //a temporary row
    unsigned int i,j;
   
    for(i = 0; i < output_module_count_; i++)  //allocate memory for 2D array of input data (variable row size)
    {
      for(j = 0; j < output_module_data_length_[output_module_count_]; j++)
      temp.push_back(0);    //fill temp row with zeros according to input_module_data_length_ values

      output_module_data_.push_back(temp);  //add row to input_module_data_
      temp.clear();                        //clear temp row
    }

    //Print OutData array information
    ROS_INFO_STREAM("Output Data Array: [Total size: " << output_module_total_data_size_ << " bytes]");
    for(i = 0; i < output_module_count_; i++)
    {
      if((output_module_data_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("I: "<< output_module_address_[i].u.Addr
          << " - " << output_module_address_[i].u.Addr + output_module_data_length_[i] - 1
          << ": OutData "  << "[" << i << "]" << "[0]");
      }
      else
      {
         ROS_INFO_STREAM("I: " << output_module_address_[i].u.Addr
           << " - " << output_module_address_[i].u.Addr + output_module_data_length_[i] - 1
           << ": OutData "  << "[" << i << "]" << "[0] ... "
           << "[" << i << "]" << "[" << output_module_data_length_[i]-1 << "]");
      }
    }
  }
}

int Cp1616IOController::addInputModule(unsigned int input_size, unsigned int input_start_address)
{
  if(cp_current_mode_ == PNIO_MODE_OPERATE)
  {
    ROS_ERROR_STREAM("Not able to add Input module in Operate state!");
    return PNIO_ERR_SEQUENCE;
  }
  else if(input_module_count_ >= NUM_OF_INPUT_MODULES)
  {
    ROS_ERROR_STREAM("Not able to add antoher input module. Max count reached!");
    return PNIO_ERR_SEQUENCE;
  }
  else
  {
    //Set variables required for PNIO_data_read function
    input_module_data_length_[input_module_count_] = input_size;                 //number of transferred bytes
    input_module_state_[input_module_count_]  = PNIO_S_BAD;                      //initial Input State
    input_module_address_[input_module_count_].AddrType = PNIO_ADDR_LOG;         //Address type
    input_module_address_[input_module_count_].IODataType = PNIO_IO_IN;          //Data type
    input_module_address_[input_module_count_].u.Addr = input_start_address;     //Module memory address

    
    ROS_DEBUG("Input module: Size: %d I: %d - %d",
      input_module_data_length_[input_module_count_],
      input_module_address_[input_module_count_].u.Addr,
      input_module_address_[input_module_count_].u.Addr + input_module_data_length_[input_module_count_] - 1);
    
    input_module_count_++;                         //increment deviceInputCount
    input_module_total_data_size_ += input_size;   //save overall Input Size

    return PNIO_OK;
  }
}

int Cp1616IOController::addOutputModule(unsigned int output_size, unsigned int output_start_address)
{
  if(cp_current_mode_ == PNIO_MODE_OPERATE)
  {
    ROS_ERROR_STREAM("Error: not able to add Output module in Operate state!");
    return PNIO_ERR_SEQUENCE;
  }
  else if(output_module_count_ >= NUM_OF_INPUT_MODULES)
  {
    ROS_ERROR_STREAM("Error: Not able to add antoher module. Max count reached!");
    return PNIO_ERR_SEQUENCE;
  }
  else
  {
    //Set variables required for PNIO_data_write function
    output_module_data_length_[output_module_count_] = output_size;              //number of transferred bytes
    output_module_state_[output_module_count_]  = PNIO_S_BAD;                    //initial Input State
    output_module_address_[output_module_count_].AddrType = PNIO_ADDR_LOG;       //Address type
    output_module_address_[output_module_count_].IODataType = PNIO_IO_OUT;       //Data type
    output_module_address_[output_module_count_].u.Addr = output_start_address;  //Module memory address

  
    ROS_DEBUG("Output module: Size: %d Q: %d - %d",
      output_module_data_length_[output_module_count_],
      output_module_address_[output_module_count_].u.Addr,
      output_module_address_[output_module_count_].u.Addr + output_module_data_length_[output_module_count_] - 1);
    
    output_module_count_++;                              //increment deviceOutputCount
    output_module_total_data_size_ += output_size;       //save overall Output Size

    return PNIO_OK;
  }
}

int Cp1616IOController::init()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Configure Controller Data
  configureControllerData();

  //Open PNIO_Controller
  
  //Connect to CP and obtain handle
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
    ROS_INFO_STREAM("Openning CP1616 in IO_controller mode: done");

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

  while(cp_ready_ == 0) //Wait for cp_ready flag to be set by alarmIndication callback
  {
    i++;
    usleep(WAIT_FOR_CALLBACKS_PERIOD);
    if(i == MAX_NUM_OF_INIT_ATTEMPTS)
    {
      ROS_ERROR_STREAM("Not able to start communication, Uninitializing...");
      return PNIO_ERR_NO_CONNECTION;
    }
  }
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
    ROS_INFO_STREAM("Uninitializing IO_Controller: done");

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

  if(cp_handle_ == valid_cp_handle)  //check if CpHandle still valid
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
    ROS_ERROR_STREAM("Not able to set required mode: ERROR another mode recieved");
    PNIO_close(cp_handle_);
  }
  else
    ROS_INFO_STREAM("Changing IO_controller mode: done");

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
                    cp_handle_,                               // handle
                    &input_module_address_[i],                // pointer to device input address
                    input_module_data_length_[i],             // length in bytes of input
                    &bytes_read,                              // number of bytes read
                    &input_module_data_[i][0],                // pointer to input data row
                    cp_local_state_,                          // local status
                    (PNIO_IOXS*)&(input_module_state_[i]));   // remote status

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
                    cp_handle_,                               // handle
                    &output_module_address_[i],               // pointer to device output address
                    output_module_data_length_[i],            // length in bytes of output
                    &output_module_data_[i][0],               // pointer to output data row
                    cp_local_state_,                          // local status
                    (PNIO_IOXS*)&(output_module_state_[i]));  // remote status

    if(error_code != PNIO_OK)
        ROS_DEBUG("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x", (int)error_code);
    
  }

  return (int)error_code;
}

void Cp1616IOController::printOutputData(unsigned int module)
{
    std::cout << "Output m." << module << std::dec
              << " [Q: " << output_module_address_[module].u.Addr
              << " - "  << output_module_address_[module].u.Addr + output_module_data_length_[module]-1
              << "]: ";

    for(int i = 0; i < output_module_data_length_[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)output_module_data_[module][i] << " ";

    std::cout << std::endl;
}

void Cp1616IOController::printInputData(unsigned int module)
{
    std::cout << "Input  m." << module << std::dec
               << " [I: " << input_module_address_[module].u.Addr
               << " - "  << input_module_address_[module].u.Addr + input_module_data_length_[module]-1
               << "]: ";

    for(int i = 0; i < input_module_data_length_[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)input_module_data_[module][i] << " ";

    std::cout << std::endl;
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

} //cp1616

#endif //CP1616_IO_CONTROLLER_CPP
