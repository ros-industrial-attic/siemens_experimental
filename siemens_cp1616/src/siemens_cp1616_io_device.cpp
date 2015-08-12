/*********************************************************************************************//**
* @file siemens_cp1616_io_device.cpp
*
* cp1616_io_device class
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
#ifndef SIEMENS_CP1616_IO_DEVICE_CPP
#define SIEMENS_CP1616_IO_DEVICE_CPP

#include <siemens_cp1616/siemens_cp1616_io_device.h>
#include <siemens_cp1616/siemens_cp1616_io_device_callbacks.h>

namespace siemens_cp1616
{
//Define and initialize device_instance_ to zero value;
Cp1616IODevice *Cp1616IODevice::device_instance_ = 0;

Cp1616IODevice* Cp1616IODevice::createDeviceInstance(std::string filepath)
{
  if( !device_instance_ )
  {
    device_instance_ = new Cp1616IODevice(filepath);
  }
  return device_instance_;
}

Cp1616IODevice* Cp1616IODevice::getDeviceInstance()
{
  return device_instance_;
}

Cp1616IODevice::Cp1616IODevice(std::string filepath):
    cp_id_(1),
    cp_handle_(0),
    cp_session_key_(0),
    cp_ar_number_(0),
    ar_info_ind_flag_(0),
    prm_end_ind_flag_(0),
    indata_ind_flag_(0),
    offline_ind_flag_(0)    
{
  //Parse data from yaml config file
  PNIO_UINT32 error_code = PNIO_OK;
  error_code = parseConfigFile(filepath);
    
  if(error_code == PNIO_OK)  //if parsing successful
  {        
   //Resize vectors according to yaml config 
   idx_table_.resize(modules_.size());
   alarms_.resize(MAX_ALARM);
   
   //Resize data containers (+1 due to DAP module)
   input_data_.resize(NUMOF_BYTES_PER_SLOT, std::vector<PNIO_UINT8>(modules_.size() + 1));
   input_data_length_.resize(modules_.size() + 1);
   input_data_iocs_.resize(modules_.size() + 1);
   input_data_iops_.resize(modules_.size() + 1);
   
   output_data_.resize(NUMOF_BYTES_PER_SLOT, std::vector<PNIO_UINT8>(modules_.size() + 1));
   output_data_length_.resize(modules_.size() + 1);
   output_data_iocs_.resize(modules_.size() + 1);
   output_data_iops_.resize(modules_.size() + 1);   
  }
  else
    exit(error_code); //If yaml config not loaded successfully exit the application
}

Cp1616IODevice::~Cp1616IODevice()
{

}

void operator >> (const YAML::Node &node, DeviceModuleData &module)
{
  node["label"] >> module.label;
  node["type"] >> module.type;
  node["size"] >> module.size;
  node["slot"] >> module.slot;
  node["modId"] >> module.modId;
  node["subslotId"] >> module.subslotId;
  node["topic"] >> module.topic;
}

int Cp1616IODevice::init()
{
  //Prepare Device Data
  configureDeviceData();
  
  PNIO_UINT32 error_code = PNIO_OK;
  PNIO_UINT32 ui_max_ar = 1;              //maximum application relationship supported
  cp_handle_ = 0;

  PNIO_ANNOTATION struct_pnio_annotation = {
    ANNOT_NAME,
    ANNOT_ORDERID,
    ANNOT_HW_REV,
    ANNOT_SW_PREFIX,
    ANNOT_SW_REV_1,
    ANNOT_SW_REV_2,
    ANNOT_SW_REV_3};

  PNIO_CFB_FUNCTIONS struct_cb_functions;

  //Initialize the callback structure -set the callback function pointers
  memset(&struct_cb_functions, 0, sizeof(PNIO_CFB_FUNCTIONS));
  struct_cb_functions.size                  = sizeof(PNIO_CFB_FUNCTIONS);
  struct_cb_functions.cbf_data_write        = pnio_device_callbacks::dataWrite;
  struct_cb_functions.cbf_data_read         = pnio_device_callbacks::dataRead;
  struct_cb_functions.cbf_rec_read          = pnio_device_callbacks::recordRead;
  struct_cb_functions.cbf_rec_write         = pnio_device_callbacks::recordWrite;
  struct_cb_functions.cbf_alarm_done        = pnio_device_callbacks::requestDone;
  struct_cb_functions.cbf_check_ind         = pnio_device_callbacks::checkIndication;
  struct_cb_functions.cbf_ar_check_ind      = pnio_device_callbacks::arCheckIndication;
  struct_cb_functions.cbf_ar_info_ind       = pnio_device_callbacks::arInfoIndication;
  struct_cb_functions.cbf_ar_indata_ind     = pnio_device_callbacks::arIndataIndication;
  struct_cb_functions.cbf_ar_abort_ind      = pnio_device_callbacks::arAbortIndication;
  struct_cb_functions.cbf_ar_offline_ind    = pnio_device_callbacks::arOfflineIndication;
  struct_cb_functions.cbf_apdu_status_ind   = pnio_device_callbacks::apduStatusIndication;
  struct_cb_functions.cbf_prm_end_ind       = pnio_device_callbacks::prmEndIndication;
  struct_cb_functions.cbf_cp_stop_req       = pnio_device_callbacks::cpStopRequest;
  struct_cb_functions.cbf_device_stopped    = pnio_device_callbacks::deviceStopped;
  struct_cb_functions.cbf_start_led_flash   = NULL;
  struct_cb_functions.cbf_stop_led_flash    = NULL;

  error_code = PNIO_device_open(
                      cp_id_,                        //Communication Module index
                      PNIO_CEP_MODE_CTRL,            //permission to change operation mode
                      VENDOR_ID,                     //vendor ID
                      DEVICE_ID,                     //device ID
                      INSTANCE_ID,                   //instance ID
                      ui_max_ar,                     //max AR count
                      &struct_pnio_annotation,       //annotation
                      &struct_cb_functions,          //callback functions information
                      &cp_handle_);                  //device handle

  ROS_DEBUG("Device handle: 0x%x", cp_handle_);

  //Check errors
  if(error_code != PNIO_OK)
    ROS_ERROR("Not able to open PNIO_device: Error: 0x%x", (int)error_code);
   else
    ROS_INFO("Openning CP1616 in IO_device mode: done");
  
  //Add Api
  error_code = addApi();
  if(error_code != PNIO_OK)
    return (int)error_code;
  
  //AddModSubMod
  error_code = addModSubMod();
  if(error_code != PNIO_OK)  
    return (int)error_code;   
   
  //Start operation
  error_code = startOperation();
  if(error_code != PNIO_OK)
    return (int)error_code;
    
  return error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::uinit()
{
  PNIO_UINT32 error_code = PNIO_OK;

  error_code = stopOperation();
  if(error_code != PNIO_OK)
    return (int)error_code;
  
  error_code = removeModSubMod();
  if(error_code != PNIO_OK)
    return (int)error_code;
  
  error_code = removeApi(); 
  if(error_code != PNIO_OK)
    return (int)error_code;
    
  error_code = PNIO_device_close(cp_handle_);

  if(error_code != PNIO_OK)
  {  
    ROS_ERROR("Not able to uninitialize CP: Error 0x%x", (int) error_code);
    return (int)error_code;
  }   
  else
  { 
    ROS_INFO("Closing PNIO_device: done ");
    return (int)error_code;  //if everything ok, return PNIO_OK
  }
}

int Cp1616IODevice::addApi()
{
  int i,j;
  int highest_slots_number;
  int highest_subslot_number = 0;

  PNIO_UINT32 api;
  PNIO_UINT32 error_code = PNIO_OK;

  //for each slot
  for(i = j = 0; i < modules_.size(); i++)
  {
    //read api from configuration data
    api = modules_[i].api;

    //look if api added at a prior position
    for(j = 0; j < i; j++)
    {
      if(api == modules_[j].api){
      //api was added
      break;
    }
  }

  if(i == j)   //not added, add a new api
  {
    // calculate highest slot and subslot number for this api
    highest_slots_number   = modules_[j].slot;
    highest_subslot_number = modules_[j].subslot;

    //check if the api exists in the slots ahead,
    //if yes, then update highest slot/subslot number accordingly

    for(j = i+1; j <  modules_.size(); j++)
    {
      if(api == modules_[j].api)
      {
        if(modules_[j].slot > highest_slots_number) highest_slots_number = modules_[j].slot;
        if(modules_[j].subslot > highest_subslot_number) highest_subslot_number = modules_[j].subslot;
      }
    }

    error_code = PNIO_api_add(
      cp_handle_,
      api,
      (PNIO_UINT16) highest_slots_number,
      (PNIO_UINT16) highest_subslot_number);

    if(error_code != PNIO_OK)
    {
      ROS_ERROR("Not able to add Api profile: Error 0x%x Check module configuration", (int) error_code);
      return (int)error_code;  //leave for-loop immediately
    }
  }
 }
  ROS_DEBUG("Adding Api profile: done");
  return (int)error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::removeApi()
{
  int i,j;
  PNIO_UINT32 api;
  PNIO_UINT32 error_code = PNIO_OK;

  //for each slot
  for(i = j = 0; i < modules_.size() && error_code == PNIO_OK; i++)
  {
    //read api from configuration data
    api = modules_[i].api;

    //look if the api has been added at a prior position in our g_device_data structure
    for(j = 0; j < i; j++)
    {
      if(api == modules_[j].api) break; // api added at a prior position, hence it has already been removed
    }

    if(i == j) //api not removed yet
    {
      error_code = PNIO_api_remove(
                   cp_handle_,
                   api);

      if(error_code != PNIO_OK)
      {
        ROS_ERROR("Not able to remove Api profile: Error: 0x%x", (int)error_code);
        return (int)error_code;  //leave for-loop immediately
      }
      else
        ROS_INFO("Removing Api profile: done");
    }
  }
  return (int)error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::addModSubMod()
{
  PNIO_UINT32 error_code = PNIO_OK;
  PNIO_DEV_ADDR addr;     //location (module/submodule)
  int slot = 0;
  int i;

  addr.AddrType = PNIO_ADDR_GEO;    //must be PNIO_ADDR_GEO

  //Add module 0
  
  addr.u.Geo.Slot    = modules_[0].slot;    //plug module 0
  addr.u.Geo.Subslot = modules_[0].subslot; //get the corresponding sub-slot

  error_code = PNIO_mod_plug(
               cp_handle_,   		     //device handle
               modules_[0].api,             //api number
               &addr,                       //location(slot, subslot)
               modules_[0].modId);          //module 0 identifier


  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to add module 0: Error 0x%x", (int)error_code);
    modules_[0].modState = 0;
  }
  else
  {
    ROS_DEBUG("Plugging Module 0: done");
    modules_[0].modState = 1;
  }

  if(!modules_[0].modState)
  {
    ROS_ERROR("ERROR: Failure in plugging module 0 -> no other module / submodule will be plugged...");
    return (int)error_code;
  }

  //Add submodule corresponding to module 0
  error_code = PNIO_sub_plug (
                cp_handle_,                //device handle
                modules_[0].api,           //api number
                &addr,                     //location (slot, subslot)
                modules_[0].subslotId);    //submodule 0 identifier


  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to add submodule 0 to module 0: Error 0x%x", (int) error_code);
    modules_[0].subState = 0;
  }
  else
  {
    ROS_DEBUG("Pluging submodule 0 to module 0: done ");
    modules_[0].subState = 1;
  }

  if(!modules_[0].subState)
  {
    ROS_ERROR_STREAM("ERROR: Failure in plugging the submodule corresponding to module 0"
                      << "-> no other module / submodule will be plugged...");

    return (int)error_code;  //if everything ok, return PNIO_OK
  }

  //Add all modules
  if(modules_.size() > 1)
  {

    for(i = 1; i < modules_.size();)
    {
      addr.u.Geo.Slot    = modules_[i].slot;    //plug module at correct slot
      addr.u.Geo.Subslot = modules_[i].subslot; //get the corresponding sub-slot



      error_code = PNIO_mod_plug(
             cp_handle_,           //device handle
             modules_[i].api,      //api number
             &addr,                //location(slot, subslot)
             modules_[i].modId);   //module identifier

      if(error_code != PNIO_OK)
      {
        ROS_ERROR("Not able to plug module: Error 0x%x", (int) error_code);
        modules_[i].modState = 0;
        return (int)error_code;
      }
      else
      {
        ROS_DEBUG("Plugging module %d: done", i);
        modules_[i].modState = 1;
      }

      if(error_code == PNIO_OK)
      {
        //advance in the p_device_data_ structure jumping over all the submodule entries
        //to reach the next module entry in the structure
        i += modules_[i].maxSubslots;
      }

      else
      {
        //go to the next entry in p_device_data table
        i++;
      }

    } //end for

    //Add all submodules
    for(i = 1; i < modules_.size(); i++)
    {
      if(modules_[i].maxSubslots > 0)
      {
        //beginning of a new slot
        slot = i;   //index of corresponding slot for a given subslot

        modules_[slot].subState = 1;
      }

      if(modules_[slot].modState)
      {
        //add submodule only if the module is added
        addr.u.Geo.Slot     = modules_[i].slot;
        addr.u.Geo.Subslot  = modules_[i].subslot;

        error_code = PNIO_sub_plug (
             cp_handle_,              //device handle
             modules_[i].api,         //api number
             &addr,                   //location(slot, subslot)
             modules_[i].subslotId);  //submodule identifier

        if(error_code != PNIO_OK)
        {
          ROS_ERROR("Not able to plug submodule: Error 0x%x", (int) error_code);
          modules_[i].subState = 0;
          modules_[slot].subState = 0;
          return (int)error_code;
        }
        else
        {
          ROS_DEBUG("Plugging submodule to module %d: done", i);
          modules_[i].subState = 1;
        }
      }
    }  //end for
  }

  //if not all the modules/submodules are plugged correctly, print warning
  for(i = 0; i < modules_.size(); i++)
  {
    if(!modules_[i].subState)
      {
        ROS_WARN("Not all modules or submodules were plugged correctly!!");
        break;
      }
    }
  return (int)error_code; //if everything ok, return PNIO_OK
}

int Cp1616IODevice::removeModSubMod()
{
  int i;
  PNIO_DEV_ADDR addr;    //location module/submodule
  PNIO_UINT32 error_code = PNIO_OK;

  //Remove modules/submodules in reverse order
  for(i = modules_.size() -1; i >= 0 && error_code == PNIO_OK; i--)
  {
    if(modules_[i].subState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;         //must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = modules_[i].slot;      //slot number
      addr.u.Geo.Subslot = modules_[i].subslot;

      //Remove submodules
      error_code = PNIO_sub_pull(
                  cp_handle_,
                  modules_[i].api, &addr);

      if(error_code != PNIO_OK)
        ROS_ERROR("Not able to remove submodule Error 0x%x", (int) error_code);
      else
      {
        ROS_INFO("Removing submodule from module %d: done", i);
        modules_[i].subState = 0;
        return (int)error_code;
      }

      //Notify the controller that the device state is NOT-OK every time after removing a submodule
      error_code = PNIO_set_dev_state(cp_handle_, PNIO_DEVSTAT_STATION_PROBLEM);
    }

    if(error_code == PNIO_OK && modules_[i].modState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;            //must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = modules_[i].slot;         //slot number
      addr.u.Geo.Subslot = 1;                        //doesn't matter

      //Remove modules
      error_code = PNIO_mod_pull(cp_handle_, modules_[i].api, &addr);

      if(error_code != PNIO_OK)
        ROS_ERROR("Not able to remove module: Error 0x%x", (int) error_code);
      else
      {
        ROS_INFO("Removing module %d: done", i);
        modules_[i].subState = 0;
        return (int)error_code;
      }

      //Notify the controller that the device state is NOT-OK every time after removing a module
      error_code = PNIO_set_dev_state(cp_handle_, PNIO_DEVSTAT_STATION_PROBLEM);
    }
  }
  return (int)error_code;  
}

int Cp1616IODevice::startOperation()
{
  PNIO_UINT32 error_code = PNIO_OK;

  error_code = PNIO_device_start(cp_handle_);
  if (error_code != PNIO_OK)
    ROS_ERROR("Not able to start IO Device operation: Error 0x%x", (int)error_code);
  else
      ROS_INFO("Starting operation: done");

  if(error_code == PNIO_OK)
  {
    error_code = PNIO_set_dev_state(cp_handle_, PNIO_DEVSTAT_OK);
    if(error_code != PNIO_OK)
      ROS_ERROR("Not able to set PNIO device state: Error 0x%x", (int)error_code);
    else
      ROS_INFO("Setting device state to PNIO_DEVSTAT_OK: done");
  }

  //Waiting for initialization callbacks
  unsigned int i = 0;

  ROS_INFO("Waiting for callbacks...");

  while(i != MAX_PRM_END_COUNT)
  {
    //prmEndInd() callback already called?
    if(prm_end_ind_flag_ == 1)
    {
      this->doAfterPrmEndIndCbf();
      break;
    }

    usleep(WAIT_FOR_CALLBACK_PERIOD);
    i++;
  }

  if(i < MAX_PRM_END_COUNT) //if PRM_END_IND_FLAG == 1
  {
    i = 0;
    while(i != MAX_INDATA_IND_COUNT)
    {
      //indataInd() callback already called?
      if(indata_ind_flag_ == 1)
      {
        this->doAfterIndataIndCbf();
        break;
      }

      usleep(WAIT_FOR_CALLBACK_PERIOD);
      i++;
    }

    if(i == MAX_INDATA_IND_COUNT)
    {
      ROS_ERROR("PNIOCbfIndataInd callback not recieved within defined period");
      error_code = PNIO_ERR_ABORT;
    }
  }
  else
  {
    ROS_ERROR("PNIOCbfPrmEndInd callback not recieved within defined period");
    error_code = PNIO_ERR_ABORT;
  }

  if(error_code == PNIO_OK)
    ROS_INFO("IO Device ready, communication started");
    

  return (int)error_code;
}

int Cp1616IODevice::stopOperation()
{
  PNIO_UINT32 error_code = PNIO_OK;
  error_code = PNIO_device_stop(cp_handle_);
  if (error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to stop IO Device: Error 0x%x", (int)error_code);
    return (int)error_code;
  }
  else
    ROS_INFO("Stopping device operation: done");

  //wait for OFFLINE_IND_flag to be set by ar_offline_ind() callback
  unsigned int i = 0;
  while(i != MAX_OFFLINE_IND_COUNT)
  {
    if(getOfflineIndFlag() != 0)
      break;
            
    usleep(WAIT_FOR_CALLBACK_PERIOD);
    i++;
  }
  
  return (int)error_code;
}

int Cp1616IODevice::GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub)
{
  unsigned int i,j;

  for(i = 0; i < modules_.size(); i++)      //Look for module index
  {
    if((int)mod == idx_table_[i])
    break;
  }

  if(i == modules_.size())                  //no module means also no submodule
    return -1;

  for(j = 0; j < modules_[i].maxSubslots; j++)
  {
    if(modules_[i+j].subslot == (int)sub)   //find submodule index
      return j;
  }

  return -1;
}

void Cp1616IODevice::configureDeviceData()
{
  unsigned int i = 0;
  unsigned int begin_new_slot = 0;
  unsigned int idx = 0;

  for(i = 0; i < modules_.size(); i++)
  {
    ROS_INFO("Module: slot %x sub %x mod_id %x sub_id %x",
    modules_[i].slot,
    modules_[i].subslot,
    (unsigned int) modules_[i].modId,
    (unsigned int) modules_[i].subslotId);
  }

  //fill idxTbl with -1
  for(unsigned int i = 0; i < modules_.size(); i++)
    idx_table_[i] = -1;
    
  idx_table_[idx++] = modules_[0].slot;

  //browsing through the device data structure
  for(i = 0; i < modules_.size(); i++)
  {
    if(modules_[i].slot == modules_[begin_new_slot].slot)
    {
      modules_[begin_new_slot].maxSubslots++;
      modules_[i].modId = modules_[begin_new_slot].modId;
    }
    else
    {
      begin_new_slot = i;		                 //index corresponding to the beginning of the new slots
      modules_[begin_new_slot].maxSubslots = 1;         //every new module/slot has min one sub-slot
      idx_table_[idx++] = modules_[i].slot;             //store the entry of the new slot in idxTbl
    }
  }
}

int Cp1616IODevice::updateCyclicOutputData()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Read data
  error_code = PNIO_initiate_data_read(cp_handle_);
  if(error_code != PNIO_OK)
      ROS_ERROR("Not able to initiate data read: Error 0x%x", (int)error_code);

  return (int)error_code;
}

int Cp1616IODevice::updateCyclicInputData()
{
  PNIO_UINT32 error_code = PNIO_OK;

  //Write data
  error_code = PNIO_initiate_data_write(cp_handle_);
  if(error_code != PNIO_OK)
    ROS_ERROR("Not able to initiate data write: Error 0x%x", (int)error_code);

  return (int)error_code;
}


int Cp1616IODevice::doAfterPrmEndIndCbf()
{
  PNIO_UINT32 error_code = PNIO_OK;
  PNIO_APPL_READY_LIST_TYPE ready_list_type;

  /** Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user
   * program can initialize the  incoming data (from the perspective of the IO controller)
   * for the functional submodules and set the local status to "GOOD". 
   */

  error_code = PNIO_initiate_data_write(cp_handle_);
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to initiate data write: Error 0x%x", (int)error_code);
    return (int)error_code;
  }
  /** We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program
   * can set the local status for functional submodules of all the outgoing data
   * (from the perspective of the IO controller) to GOOD. 
   */

  error_code = PNIO_initiate_data_read(cp_handle_);
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to initiate data read: Error: 0x%x", (int)error_code);
    return (int)error_code;
  }
  /** Here we need to call PNIO_set_appl_state_ready so that
   * the IO Base Device user program registers a list of the non-functional submodules
   * and the extent of readiness to get into a data exchange at the IO controller
   */

   memset(&ready_list_type, 0, sizeof(ready_list_type));
   ready_list_type.ap_list.Flink = NULL;
   ready_list_type.ap_list.Blink = NULL;

   error_code = PNIO_set_appl_state_ready(
         cp_handle_,
         cp_ar_number_,
         cp_session_key_,
         &ready_list_type);

   if(error_code != PNIO_OK)
       ROS_ERROR("Not able to set application state ready: Error 0x%x", (int)error_code);

   return (int)error_code;
}

int Cp1616IODevice::doAfterIndataIndCbf()
{
  PNIO_UINT32 error_code = PNIO_OK;

  /** Here we need to call "PNIO_initiate_data_write" so that the Device user
   * program can initialize the  incoming data (from the perspective of the IO controller)
   * for the functional submodules and set the local status to "GOOD". 
   */

  error_code = PNIO_initiate_data_write(cp_handle_);
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to initiate data write: Error 0x%x", (int)error_code);
    return (int)error_code;
  }

  /** We also have to call "PNIO_initiate_data_read" so that the Device user program
   * can set the local status for functional submodules of all the outgoing data
   * (from the perspective of the IO controller) to GOOD. 
   */

   error_code = PNIO_initiate_data_read(cp_handle_);
   if(error_code != PNIO_OK)
   {
     ROS_ERROR("Not able to initiate data read: Error 0x%x", (int)error_code);
     return (int)error_code;
   }
   
   return(int)error_code; //if everything ok return PNIO_OK
}

int Cp1616IODevice::parseConfigFile(std::string filepath)
{
  PNIO_UINT32 error_code = PNIO_OK;
  std::ifstream fin(filepath.c_str());
  
  if(fin.is_open())
  {
    YAML::Parser parser(fin);
    YAML::Node doc;
   
    try
    {
      parser.GetNextDocument(doc);
      for(unsigned i = 0; i < doc.size(); i++)
      {
        DeviceModuleData temp_module;
        doc[i] >> temp_module; 
     
        //Fixed params
        temp_module.subslot = 1;
        temp_module.api = 0;
        temp_module.maxSubslots = 0;
        temp_module.modState = 0;
        temp_module.subState = 0;
        temp_module.dir = 0;
 
        if(temp_module.size > NUMOF_BYTES_PER_SLOT)
          ROS_ERROR("CP1616 Configuration: Max data length: %d exceeded", NUMOF_BYTES_PER_SLOT);
        else
          modules_.push_back(temp_module);	
      }
      
      if(!modules_.size())
        ROS_ERROR("CP1616 Configuration: No module found! Check configuration!");
      else
        ROS_INFO_STREAM("CP1616 Configuration: Number of modules: " << modules_.size() );
    }
    catch(YAML::ParserException &e)
    {
      ROS_ERROR("Error reading yaml config file");
      error_code = PNIO_ERR_NO_CONFIG;
    }
  }
  else
  {
    ROS_ERROR("Error openning yaml config file");
  }
  return (int)error_code;
}

int Cp1616IODevice::sendDiagnosticAlarm(PNIO_UINT16 slot_num)
{
  PNIO_UINT32 error_code = PNIO_OK;
  PNIO_UINT16 ui_diag_alarm_property;   //channel property to be passed to the PNIO_diag_channel_add function
  DiagAlarmDataExt diag_data;           //diagnosis data
  
  PNIO_DEV_ADDR addr;                   //Address info of the module
  
  addr.AddrType      = PNIO_ADDR_GEO;   // must be PNIO_ADDR_GEO 
  addr.u.Geo.Slot    = slot_num;        // slot index
  addr.u.Geo.Subslot = 1;               // subslot index always 1 
  
  ROS_WARN("Sending diagnostic alarm to IO Controller");
  
  // Build channel properties             
  ui_diag_alarm_property = PNIO_build_channel_properties(
    cp_handle_,                            // Device handle 
    PNIO_DIAG_CHANPROP_TYPE_SUBMOD,        // for our dev. always this  
    PNIO_DIAG_CHAN_SPEC_ERROR_APPEARS,     // incoming alarm (from IO Controller perspective)
    PNIO_DIAG_CHAN_DIRECTION_MANUFACTURE); // 0: = specific 

  //Add extended diagnostic channel
  error_code = PNIO_diag_ext_channel_add(
    cp_handle_,                            // Device handle 
    modules_[slot_num].api,                // Api number 
    &addr,                                 // Address  
    slot_num,                              // subslot number  
    ui_diag_alarm_property,                // channel properties 
    CH_ERR_GROUND_FAULT,                   // error type 
    CH_ERR_GROUND_FAULT,                   // ext channel error type
    0x00000001,                            // ext channel add value
    slot_num);                             // diag tag/ alarm handle 
  
  if(error_code != PNIO_OK) 
  {
    ROS_ERROR("Error in adding a diagnostic channel: Error 0x%x", (int)error_code);
    return (int)error_code;
  }

   //Change device state
   error_code = PNIO_set_dev_state (cp_handle_, PNIO_DEVSTAT_STATION_PROBLEM);
   if(error_code != PNIO_OK) 
   {
     ROS_ERROR("Error in changing the device state\n");
     return (int)error_code;
   }
  
  //Prepare alarm data
  diag_data.chanProp           = ui_diag_alarm_property;
  diag_data.chanNum            = 0x8000;
  diag_data.chanErrType        = (unsigned short)slot_num;
  diag_data.extChannelErrType  = 0x8000;
  diag_data.extChannelAddValue = 0x00000001;
  
  //Copy the alarm properties to reset the alarm 
  alarms_[slot_num].chanProp	        = diag_data.chanProp;
  alarms_[slot_num].chanNum	        = diag_data.chanNum ;
  alarms_[slot_num].chanErrType        = diag_data.chanErrType;
  alarms_[slot_num].extChannelAddValue = diag_data.extChannelAddValue;
  alarms_[slot_num].extChannelAddValue = diag_data.extChannelErrType;
  
  //Send diag_data to reset the alarm 
  error_code = PNIO_diag_alarm_send(
    cp_handle_,                        // Device Handle 
    modules_[addr.u.Geo.Slot].api,     // Api number 
    cp_ar_number_,                     // received in arInfoInd 
    cp_session_key_,                   // received in arInfoInd 
    PNIO_STATE_ALARM_APPEARS,
    &addr,                             // location (slot, subslot) 
    (PNIO_UINT8 *)&diag_data,          // Alarm Data 
    sizeof(DiagAlarmData),             // size of the structure sent 
    0x8002,                            // means, alarmData contains diagnosis information 
    USER_HANDLE);                      // user handle, to identify user in multi-user scenario 

  if(error_code != PNIO_OK) 
  {
    ROS_ERROR("Not able to send diagnostic alarm: Error 0x%x", (int)error_code);
    return (int)error_code;
  }
  else
  {  
    ROS_INFO("Diagnostic alarm sent to the IO Controller");
    return (int)error_code;
  }
}

int Cp1616IODevice::resetDiagnosticAlarm(PNIO_UINT16 slot_num)
{
  PNIO_UINT32 error_code = PNIO_OK;
    
  PNIO_DEV_ADDR addr;                   // Address info of the module 
  addr.AddrType      = PNIO_ADDR_GEO;   // must be PNIO_ADDR_GEO 
  addr.u.Geo.Slot    = slot_num;        // slot index 
  addr.u.Geo.Subslot = 1;               // subslot index always 1  

  if(!slot_num) 
    ROS_INFO("Resetting all diagnostic and/or maintenance alarm(s)...");   
  else 
    ROS_INFO("Resetting the diagnostic alarm...");

  //Storing diagnosis data in a sub-slot and remove diag channel or extdiag channel 
  if(!slot_num)
  {
    error_code = PNIO_diag_channel_remove(
    cp_handle_,                       // Device handle 
    modules_[slot_num].api,    // Api number 
    &addr,                            // Address 
    slot_num);                        // alarm slot 
  }
  else
  {
    error_code = PNIO_diag_ext_channel_remove(
     cp_handle_,                      // Device handle 
     modules_[slot_num].api,   // Api number 
     &addr,                           // Address 
     slot_num);                       // diag tag/ alarm handle 
  }

  if(error_code != PNIO_OK) 
  {
    ROS_ERROR("Not able to remove diagnostic channel: Error 0x%x", (int)error_code);
    return (int)error_code;
  }

  // do this only if all modules / submodules are plugged 
  for(unsigned int i = 0; i < modules_.size(); i++) 
  {
    if(!modules_[i].subState) 
      break;
  }
  
  if(!slot_num) 
  {
    error_code = PNIO_diag_alarm_send(
      cp_handle_,                        // Device Handle 
      modules_[slot_num].api,     // Api number 
      cp_ar_number_,                     // received in PNIO_cbf_ar_info_ind 
      cp_session_key_,                   // received in PNIO_cbf_ar_info_ind 
      PNIO_STATE_ALARM_DISAPPEARS,
      &addr,
      NULL,                              // Alarm Data 
      0,                                 // size of the structure sent 
      0x8001,                            // means that alarms_ contains diagnosis information 
      USER_HANDLE);                      // user handle, to identify user in multi-user scenario      
  }  
  else
  {
    error_code = PNIO_diag_alarm_send(
      cp_handle_,                        // Device Handle 
      modules_[slot_num].api,            // Api number 
      cp_ar_number_,                     // received in PNIO_cbf_ar_info_ind 
      cp_session_key_,                   // received in PNIO_cbf_ar_info_ind 
      PNIO_STATE_ALARM_DISAPPEARS,
      &addr,
      (PNIO_UINT8 *)& alarms_[slot_num], // Alarm Data 
      0,                                 // size of the structure sent 
      0x8001,                            // means that alarms_ contains diagnosis information 
      USER_HANDLE);                      // user handle, to identify user in multi-user scenario 
      
    //reset_diag_data(diag_tag);
  }
  
  if(error_code != PNIO_OK) 
  {
    ROS_ERROR("Not able to reset diagnostic alarm: Error 0x%x", error_code);
    return (int)error_code;
  }
  
   //Change device state
   error_code = PNIO_set_dev_state (cp_handle_, PNIO_DEVSTAT_OK);
   if(error_code != PNIO_OK) 
   {
     ROS_ERROR("Error in changing the device state\n");
     return (int)error_code;
   }
  
  return (int)error_code;
}

int Cp1616IODevice::sendProcessAlarm(PNIO_UINT16 slot_num)  
{
    PNIO_UINT32 error_code = PNIO_OK;
    char alarm_data[50]; 
        
    PNIO_DEV_ADDR   addr; 
    
    addr.AddrType       = PNIO_ADDR_GEO;
    addr.u.Geo.Slot     = slot_num;
    addr.u.Geo.Subslot  = 1; 

    error_code = PNIO_process_alarm_send(
        cp_handle_,                     // Device handle 
        modules_[slot_num].api,         // Api number 
        cp_ar_number_,                  // received in PNIO_cbf_ar_info_ind 
        cp_session_key_,                // received in PNIO_cbf_ar_info_ind 
        &addr,                          // location (slot, subslot) 
        (PNIO_UINT8*)alarm_data,        // alarm data 
        sizeof (alarm_data),            // alarm data length 
        0x004d,                         // 0...0x7fff: user struct. is manufac. specific 
        USER_HANDLE);                   // user handle, to identify user in a multi-user scenario 

    if(error_code != PNIO_OK) 
        ROS_ERROR("Not able to send process alarm: Error 0x%x", error_code);
    
    return (int)error_code;
}

void Cp1616IODevice::setArInfoIndFlag(int value)
{
  ar_info_ind_flag_ = value;
}

int Cp1616IODevice::getArInfoIndFlag()
{
  return ar_info_ind_flag_;
}

void Cp1616IODevice::setPrmEndIndFlag(int value)
{
  prm_end_ind_flag_ = value;
}

int Cp1616IODevice::getPrmEndIndFlag()
{
  return prm_end_ind_flag_;
}

void Cp1616IODevice::setIndataIndFlag(int value)
{
  indata_ind_flag_ = value;
}

int Cp1616IODevice::getIndataIndFlag()
{
  return indata_ind_flag_;
}

void Cp1616IODevice::setOfflineIndFlag(int value)
{
  offline_ind_flag_ = value;
}

int Cp1616IODevice::getOfflineIndFlag()
{
  return offline_ind_flag_;
}

void Cp1616IODevice::setCpSessionKey(PNIO_UINT16 value)
{
  cp_session_key_ = value;
}

PNIO_UINT16 Cp1616IODevice::getCpSessionKey()
{
  return cp_session_key_;
}

void Cp1616IODevice::setCpArNumber(PNIO_UINT16 value)
{
  cp_ar_number_ = value;
}

PNIO_UINT16 Cp1616IODevice::getCpArNumber()
{
  return cp_ar_number_;
}

} //siemens_cp1616

#endif //SIEMENS_CP1616_IO_DEVICE_CPP
