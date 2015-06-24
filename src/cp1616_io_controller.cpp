/*********************************************************************************************//**
* @file cp1616.cpp
* 
* Main cp1616 node
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

#include <cp1616/cp1616_io_controller.h>

bool Cp1616CallbackHandler::instance_flag_ = false;
Cp1616CallbackHandler* Cp1616CallbackHandler::instance_ = NULL;

Cp1616CallbackHandler* Cp1616CallbackHandler::getInstance()
{
    if(! instance_flag_)
    {
        instance_ = new Cp1616CallbackHandler();
        instance_flag_ = true;
        return instance_;
    }
    else
    {
        return instance_;
    }
}

Cp1616CallbackHandler::Cp1616CallbackHandler()
{

}


Cp1616IOController::Cp1616IOController() :
  cp_ready_(0),
  sem_mod_change_(0),
  cp_id_(1),
  cp_handle_(0),
  cp_current_mode_(PNIO_MODE_OFFLINE),
  cp_local_state_(PNIO_S_GOOD),
  device_input_count_(0),
  device_output_count_(0),
  total_input_size_(0),
  total_output_size_(0)
{
  //Allocate memory for Input module
  device_input_length_  = new PNIO_UINT32 [NUM_OF_INPUT_MODULES * sizeof(PNIO_UINT32)];
  device_input_state_   = new PNIO_IOXS /*volatile*/ [NUM_OF_INPUT_MODULES * sizeof(PNIO_IOXS)];
  device_input_address_ = new PNIO_ADDR [NUM_OF_INPUT_MODULES * sizeof(PNIO_ADDR)];

  //Allocate memory for Output module
  device_output_length_  = new PNIO_UINT32 [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_UINT32)];
  device_output_state_   = new PNIO_IOXS /*volatile*/ [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_IOXS)];
  device_output_address_ = new PNIO_ADDR [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_ADDR)];
}






Cp1616IOController::~Cp1616IOController()
{
  if(device_input_count_ > 0)
  {
    delete in_module_data_;
    delete in_data_;
  }

  if(device_output_count_ > 0)
  {
    delete out_module_data_;
    delete out_data_;
  }

  delete device_input_length_;
  delete device_input_state_;
  delete device_input_address_;

  delete device_output_length_;
  delete device_output_state_;
  delete device_output_address_;
}






int Cp1616IOController::addInputModule(unsigned int input_size, unsigned int input_start_address)
{
  if(cp_current_mode_ == PNIO_MODE_OPERATE)
  {
    ROS_ERROR_STREAM("Not able to add Input module in Operate state!");
    return -1;
  }
  else if(device_input_count_ >= NUM_OF_INPUT_MODULES)
  {
    ROS_ERROR_STREAM("Not able to add antoher input module. Max count reached!");
    return -1;
  }
  else
  {
    //Set variables required for PNIO_data_read function
    device_input_length_[device_input_count_] = input_size;                      //number of transferred bytes
    device_input_state_[device_input_count_]  = PNIO_S_BAD;                      //initial Input State
    device_input_address_[device_input_count_].AddrType = PNIO_ADDR_LOG;         //Address type
    device_input_address_[device_input_count_].IODataType = PNIO_IO_IN;          //Data type
    device_input_address_[device_input_count_].u.Addr = input_start_address;     //Module memory address

    #if DEBUG
      ROS_INFO("Input module: Size: %d I: %d - %d",
        device_input_length_[device_input_count_],
        device_input_address_[device_input_count_].u.Addr,
        device_input_address_[device_input_count_].u.Addr + device_input_length_[device_input_count_] - 1);
    #endif
    device_input_count_++;             //increment deviceInputCount
    total_input_size_ += input_size;   //save overall Input Size

    return 0;
  }
}






int Cp1616IOController::addOutputModule(unsigned int output_size, unsigned int output_start_address)
{
  if(cp_current_mode_ == PNIO_MODE_OPERATE)
  {
    ROS_ERROR_STREAM("Error: not able to add Output module in Operate state!");
    return -1;
  }
  else if(device_output_count_ >= NUM_OF_INPUT_MODULES)
  {
    ROS_ERROR_STREAM("Error: Not able to add antoher module. Max count reached!");
    return -1;
  }
  else
  {
    //Set variables required for PNIO_data_write function
    device_output_length_[device_output_count_] = output_size;                   //number of transferred bytes
    device_output_state_[device_output_count_]  = PNIO_S_BAD;                    //initial Input State
    device_output_address_[device_output_count_].AddrType = PNIO_ADDR_LOG;       //Address type
    device_output_address_[device_output_count_].IODataType = PNIO_IO_OUT;       //Data type
    device_output_address_[device_output_count_].u.Addr = output_start_address;  //Module memory address

    #if DEBUG
      ROS_INFO("Output module: Size: %d Q: %d - %d",
        device_output_length_[device_output_count_],
        device_output_address_[device_output_count_].u.Addr,
        device_output_address_[device_output_count_].u.Addr + device_output_length_[device_output_count_] - 1);
    #endif

    device_output_count_++;                  //increment deviceOutputCount
    total_output_size_ += output_size;       //save overall Output Size

    return 0;
  }
}






int Cp1616IOController::init()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //-------------------------------------------------------------------------
  //Prepare 2D array for registered input modules
  //-------------------------------------------------------------------------
  if(device_input_count_ > 0)    //if any input module available
  {
    //Memory allocation
    in_module_data_ = new PNIO_UINT8 [total_input_size_ * sizeof(PNIO_UINT8)];
    in_data_        = new PNIO_UINT8* [device_input_count_ * sizeof(PNIO_UINT8)];

    //Set initial InpData values to zero
    memset(in_module_data_, 0, total_input_size_);

    unsigned int i, j;

    //Assign array pointers
    for(i = 0, j = 0; j < device_input_count_; )
    {
      in_data_[j] = &(in_module_data_[i]);
      i += device_input_length_[j++];
    }

    //Print InData array information
    ROS_INFO_STREAM("Input Data Array: [Total size: " << total_input_size_ << " bytes]");
    for(i = 0; i < device_input_count_; i++)
    {
      if((device_input_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("I: "<< device_input_address_[i].u.Addr
          << " - " << device_input_address_[i].u.Addr + device_input_length_[i] - 1
          << ": InData  "  << "[" << i << "]" << "[0]");
      }
      else
      {
        ROS_INFO_STREAM("I: "<< device_input_address_[i].u.Addr
          << " - " << device_input_address_[i].u.Addr + device_input_length_[i] - 1
          << ": InData  "  << "[" << i << "]" << "[0] ... "
          << "[" << i << "]" << "[" << device_input_length_[i]-1 << "]");
      }
    }
  }

  //-------------------------------------------------------------------------
  //Prepare 2D array for for registered output modules
  //-------------------------------------------------------------------------
  if(device_output_count_ > 0)    //if any output module available
  {
    //Memory allocation
    out_module_data_ = new PNIO_UINT8 [total_output_size_ * sizeof(PNIO_UINT8)];
    out_data_        = new PNIO_UINT8* [device_output_count_ * sizeof(PNIO_UINT8)];

    //Set initial InpData values to zero
    memset(out_module_data_, 0, total_output_size_);

    unsigned int i, j;

    //Assign array pointers
    for(i = 0, j = 0; j < device_output_count_; )
    {
      out_data_[j] = &(out_module_data_[i]);
      i += device_output_length_[j++];
    }

    //Print OutData array information
    ROS_INFO_STREAM("Output Data Array: [Total size: " << total_output_size_ << " bytes]");
    for(i = 0; i < device_output_count_; i++)
    {
      if((device_output_length_[i]-1) == 0)
      {
        ROS_INFO_STREAM("I: "<< device_output_address_[i].u.Addr
          << " - " << device_output_address_[i].u.Addr + device_output_length_[i] - 1
          << ": OutData "  << "[" << i << "]" << "[0]");
      }
      else
      {
         ROS_INFO_STREAM("I: " << device_output_address_[i].u.Addr
           << " - " << device_output_address_[i].u.Addr + device_output_length_[i] - 1
           << ": OutData "  << "[" << i << "]" << "[0] ... "
           << "[" << i << "]" << "[" << device_output_length_[i]-1 << "]");
      }
    }
  }

  //-------------------------------------------------------------------------
  //Open PNIO_Controller
  //-------------------------------------------------------------------------

  //Connect to CP and obtain handle
  error_code = PNIO_controller_open(
              cp_id_,
              PNIO_CEP_MODE_CTRL,
              &Cp1616IOController::callbackForDsReadConf,
              &Cp1616IOController::callbackForDsWriteConf,
              &Cp1616IOController::callbackForAlarmIndication,
              &cp_handle_);

  //Check errors
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to open PNIO_controller: Error: 0x%x", (int)error_code);
    return error_code;
  }
  else
    ROS_INFO_STREAM("Openning CP1616 in IO_controller mode: done");

  //---------------------------------------------------------------------------
  //register PNIO_CBE_MODE_IND callback for mode changes confirmation
  //---------------------------------------------------------------------------
  error_code = PNIO_register_cbf(
                cp_handle_,
                PNIO_CBE_MODE_IND,
                &Cp1616IOController::callbackForModeChangeIndication);

  //Check errors
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Error in PNIO_register_cbf: callbackForModeChangeIndication: 0x", (int)error_code);
    PNIO_close(cp_handle_);
    return error_code;
  }

  //---------------------------------------------------------------------------
  //register the callback PNIO_CBE_DEV_ACT for device activation confirmation
  //---------------------------------------------------------------------------
  error_code = PNIO_register_cbf(
                cp_handle_,
                PNIO_CBE_DEV_ACT_CONF,
                &Cp1616IOController::callbackForDeviceActivation);

  if(error_code != PNIO_OK)
  {
    ROS_ERROR_STREAM("Error in PNIO_register_cbf: callbackForDeviceActivation: 0x" << (int)error_code);
    PNIO_close(cp_handle_);
    return error_code;
  }

  //---------------------------------------------------------------------------
  //Change CP mode to Operate
  //---------------------------------------------------------------------------
  error_code = changeAndWaitForPnioMode(PNIO_MODE_OPERATE);
  if(error_code != PNIO_OK)
  {
      PNIO_close(cp_handle_);
      return error_code;
  }

  //---------------------------------------------------------------------------
  //Start communication
  //---------------------------------------------------------------------------
  unsigned i = 0;

  while(cp_ready_ == 0) //Wait for cp_ready flag to be set by callbackForAlarmIndication
  {
    i++;
    usleep(100000);
    if(i == MAX_NUM_OF_INIT_ATTEMPTS)
    {
      ROS_ERROR_STREAM("Not able to start communication, Uninitializing...");
      return -1;
    }
  }

  //if everything ok
  return 0;
}






int Cp1616IOController::uinit()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Change CP mode to OFFLINE
  error_code = changeAndWaitForPnioMode(PNIO_MODE_OFFLINE);
  if(error_code != PNIO_OK)
  {
      PNIO_close(cp_handle_);
      return(error_code);
  }

  error_code = PNIO_close(cp_handle_);

  if(error_code != PNIO_OK)
  {
    ROS_ERROR_STREAM("Not able to uninitialize IO_Controller: Error 0x%x" << (int)error_code);
    return error_code;
  }
  else
  {
    ROS_INFO_STREAM("Uninitializing IO_Controller: done");
    return 0;
  }
}






PNIO_UINT32 Cp1616IOController::changeAndWaitForPnioMode(PNIO_MODE_TYPE requested_mode)
{
  PNIO_UINT32 error_code;
  PNIO_UINT32 valid_cp_handle = cp_handle_;

  //set required mode
  error_code = PNIO_set_mode(cp_handle_, requested_mode);

  if(error_code != PNIO_OK)
  {
    ROS_ERROR_STREAM("Not able to change IO_Controller mode: Error 0x%x" << (int)error_code);
    PNIO_close(cp_handle_);
    return error_code;
  }

  if(cp_handle_ == valid_cp_handle)  //check if CpHandle still valid
  {
    //wait for a callback_for_mode_change_indication
    while(!sem_mod_change_){
    usleep(100000);
    }

    sem_mod_change_ = 0;
  }

  //check if the current mode is correct
  if(cp_current_mode_ != requested_mode)
  {
    ROS_ERROR_STREAM("Not able to set required mode: ERROR another mode recieved");
    PNIO_close(cp_handle_);
    return error_code;
  }
  else
  {
      ROS_INFO_STREAM("Changing IO_controller mode: done");
    return 0;
  }
}






PNIO_UINT32 Cp1616IOController::updateCyclicInputData()
{
  PNIO_UINT32 error_code;
  PNIO_UINT32 bytes_read;

  unsigned int i;

  for(i = 0; i < device_input_count_; i++)
  {
    error_code = PNIO_data_read(
                    cp_handle_,                               // handle
                    &device_input_address_[i],                // pointer to device input address
                    device_input_length_[i],                  // length in bytes of input
                    &bytes_read,                              // number of bytes read
                    in_data_[i],                               // pointer to input data
                    cp_local_state_,                           // local status
                    (PNIO_IOXS*)&(device_input_state_[i]));    // remote status

    #if DEBUG
     if(error_code != PNIO_OK)
       ROS_ERROR_STREAM("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)error_code);
    #endif
  }

  return error_code;
}






PNIO_UINT32 Cp1616IOController::updateCyclicOutputData()
{
  PNIO_UINT32 error_code;
  unsigned int i;

  for(i = 0; i < device_output_count_; i++)
  {
    error_code = PNIO_data_write(
                    cp_handle_,                               // handle
                    &device_output_address_[i],               // pointer to device output address
                    device_output_length_[i],                 // length in bytes of output
                    out_data_[i],                             // pointer to output data
                    cp_local_state_,                          // local status
                    (PNIO_IOXS*)&(device_output_state_[i]));  // remote status

    #if DEBUG
      if(error_code != PNIO_OK)
        ROS_ERROR_STREAM("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)error_code);
    #endif
  }

  return error_code;
}






void Cp1616IOController::printOutputData(unsigned int module)
{
    std::cout << "Output m." << module << std::dec
              << " [Q: " << device_output_address_[module].u.Addr
              << " - "  << device_output_address_[module].u.Addr + device_output_length_[module]-1
              << "]: ";

    for(int i = 0; i < device_output_length_[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)out_data_[module][i] << " ";

    std::cout << std::endl;
}






void
Cp1616IOController::printInputData(unsigned int module)
{
    std::cout << "Input  m." << module << std::dec
               << " [I: " << device_input_address_[module].u.Addr
               << " - "  << device_input_address_[module].u.Addr + device_input_length_[module]-1
               << "]: ";

    for(int i = 0; i < device_input_length_[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)in_data_[module][i] << " ";

    std::cout << std::endl;
}






void
Cp1616IOController::setOutData(unsigned int module, unsigned int data_index, PNIO_UINT8 value)
{
    //Check if index argumnets correspond with OutData Array
    if(module >= device_output_count_)
        std::cout << "#Error: setOutData: Non exisitng item in OutData array, check module='"<< module <<"'!" << std::endl;
    else if (data_index >= device_output_length_[module])
        std::cout << "#Error: setOutData: Non existing item in OutData array, check data_index='" << data_index << "'!" << std::endl;
    else
        out_data_[module][data_index] = value;
}






void Cp1616IOController::callbackForDsReadConf(PNIO_CBE_PRM *pCbfPrm)
{
  ROS_INFO_STREAM("CallbackForDsReadConf occured");
}






void Cp1616IOController::callbackForDsWriteConf(PNIO_CBE_PRM* pCbfPrm)
{
  ROS_INFO_STREAM("CallbackForDsWriteConf occured");
}






void Cp1616IOController::callbackForModeChangeIndication(PNIO_CBE_PRM *pCbfPrm)
{
  //Create CallbackHandler object to access cp1616_io_controller variables from static member function
  Cp1616IOController* CallbackHandler = (Cp1616IOController*)cp1616_object;

  if(pCbfPrm->CbeType == PNIO_CBE_MODE_IND) /* Check callback type */
  {
    switch (pCbfPrm->ModeInd.Mode)
    {
      case PNIO_MODE_OFFLINE:
        ROS_INFO_STREAM("IO_controller mode change request-> OFFLINE: ");
        CallbackHandler->cp_current_mode_ = PNIO_MODE_OFFLINE;
        break;
      case PNIO_MODE_CLEAR:
        ROS_INFO_STREAM("IO_controller mode change request-> CLEAR: ");
        CallbackHandler->cp_current_mode_ = PNIO_MODE_CLEAR;
        break;
      case PNIO_MODE_OPERATE:
        ROS_INFO_STREAM("IO_controller mode change request-> OPERATE: ");
        CallbackHandler->cp_current_mode_ = PNIO_MODE_OPERATE;
        break;
      default:
        ROS_INFO_STREAM("Not able to change IO_controller mode: Wrong mode selected!");
        break;
    };

    //send notification
    CallbackHandler->sem_mod_change_ = 1;
  }
}






void Cp1616IOController::callbackForAlarmIndication(PNIO_CBE_PRM *pCbfPrm)
{
  //Create CallbackHandler object to access cp1616_io_controller variables
  Cp1616IOController* CallbackHandler = (Cp1616IOController*)cp1616_object;

  if(pCbfPrm->CbeType==PNIO_CBE_ALARM_IND) /* Check callback type */
  {
    switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType)
    {
      case PNIO_ALARM_DIAGNOSTIC:
        ROS_WARN_STREAM("PNIO_ALARM_DIAGNOSTIC");
        break;
      case PNIO_ALARM_PROCESS:
        ROS_WARN_STREAM("PNIO_ALARM_DIAGNOSTIC");
        break;
      case PNIO_ALARM_PULL:
        ROS_WARN_STREAM("PNIO_ALARM_PULL");
        break;
      case PNIO_ALARM_PLUG:
        ROS_WARN_STREAM("PNIO_ALARM_PLUG");
        break;
      case PNIO_ALARM_STATUS:
        ROS_WARN_STREAM("PNIO_ALARM_STATUS");
        break;
      case PNIO_ALARM_UPDATE:
        ROS_WARN_STREAM("PNIO_ALARM_UPDATE");
        break;
      case PNIO_ALARM_REDUNDANCY:
        ROS_WARN_STREAM("PNIO_ALARM_REDUNDANCY");
        break;
      case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
        ROS_WARN_STREAM("PNIO_ALARM_CONTROLLED_BY_SUPERVISOR");
         break;
      case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
        ROS_WARN_STREAM("PNIO_ALARM_RELEASED_BY_SUPERVISOR");
        break;
      case PNIO_ALARM_PLUG_WRONG:
        ROS_WARN_STREAM("PNIO_ALARM_PLUG_WRONG");
        break;
      case PNIO_ALARM_RETURN_OF_SUBMODULE:
        ROS_WARN_STREAM("PNIO_ALARM_RETURN_OF_SUBMODULE");
        break;
      case PNIO_ALARM_DEV_FAILURE:
        #if DEBUG
          ROS_WARN_STREAM("PNIO_ALARM_DEV_FAILURE");
        #endif
        break;
      case PNIO_ALARM_DEV_RETURN:
        #ifdef DEBUG
          ROS_WARN_STREAM("PNIO_ALARM_DEV_RETURN");
        #endif
        CallbackHandler->cp_ready_ = 1;	//Set CpReady flag to notify application that CP is ready for communication
        break;
      default:
        ROS_WARN_STREAM("callback_for_alarm_indication called with wrong type");
        break;
    };
  }
}





void Cp1616IOController::callbackForDeviceActivation(PNIO_CBE_PRM *pCbfPrm)
{
    switch( pCbfPrm->DevActConf.Mode)
    {
        case PNIO_DA_TRUE:
          ROS_INFO("IO_Controller - address %x was activated with result %x\n",
            (int)pCbfPrm->DevActConf.pAddr->u.Addr,
            (int)pCbfPrm->DevActConf.Result);
          break;
        case PNIO_DA_FALSE:
          ROS_INFO("IO_Controller - address %x was deactivated with result %x\n",
            (int)pCbfPrm->DevActConf.pAddr->u.Addr,
            (int)pCbfPrm->DevActConf.Result);
          break;
    };
}






int main(int argc, char **argv)
{
  ros::init(argc,argv, "ros_profinet_experimental");
  ros::NodeHandle nh;

  Cp1616IOController cp1616;
  cp1616_object = (void *) &cp1616;  //assign global variable to handle callbacks

  Cp1616CallbackHandler *handler;
  //Singleton: how to initialize it to point to &cp1616?


  //Add IO modules
  cp1616.addOutputModule(4,4116);
  cp1616.addOutputModule(4,4120);
  cp1616.addOutputModule(4,4124);
  cp1616.addOutputModule(16,4128);

  cp1616.addInputModule(4,4132);
  cp1616.addInputModule(4,4136);
  cp1616.addInputModule(4,4140);
  cp1616.addInputModule(16,4144);

  //Initialize CP
  cp1616.init();

  if(cp1616.cp_ready_ != 0)
  {
    PNIO_UINT8 do_value = 1;
    PNIO_UINT32 error_code = PNIO_OK;

    //Shift values
    for(int i = 0; i < 20; i++)
    {
      for(int j = 0; j < 7; j++)
      {
        //Shift Output data
        do_value = do_value << 1;
        cp1616.setOutData(0,0,do_value);

        //Print OutData, InData
        cp1616.printOutputData(0);
        cp1616.printOutputData(1);
        cp1616.printOutputData(2);
        cp1616.printOutputData(3);
        cp1616.printInputData(0);
        cp1616.printInputData(1);
        cp1616.printInputData(2);
        cp1616.printInputData(3);

        error_code = cp1616.updateCyclicOutputData();
        if(error_code != PNIO_OK)
        {
          ROS_INFO("Not able to update output data: Error 0x%x\n", (int)error_code);
        }
        usleep(1000000);
      }
      for(int j = 0; j < 7; j++)
      {
        //Shift Output data
        do_value = do_value >> 1;
        cp1616.setOutData(0,0,do_value);

        //Print OutData, InData
        cp1616.printOutputData(0);
        cp1616.printOutputData(1);
        cp1616.printOutputData(2);
        cp1616.printOutputData(3);
        cp1616.printInputData(0);
        cp1616.printInputData(1);
        cp1616.printInputData(2);
        cp1616.printInputData(3);

        error_code = cp1616.updateCyclicOutputData();
        if(error_code != PNIO_OK)
        {
          ROS_INFO("Not able to update input data: Error 0x%x\n", (int)error_code);
        }
        usleep(1000000);
      }
    }

 }

    cp1616.uinit();

    return(EXIT_SUCCESS);
}
