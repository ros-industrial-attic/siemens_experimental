/*********************************************************************************************//**
* @file cp1616_io_device.cpp
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
#ifndef CP1616_IO_DEVICE_CPP
#define CP1616_IO_DEVICE_CPP

#include <cp1616/cp1616_io_device.h>
#include <cp1616/cp1616_io_device_callbacks.h>

namespace cp1616
{
//Define and initialize device_instance_ to zero value;
Cp1616IODevice *Cp1616IODevice::device_instance_ = 0;

Cp1616IODevice* Cp1616IODevice::getDeviceInstance()
{
  if( !device_instance_ )
  {
    device_instance_ = new Cp1616IODevice();
  }
  return device_instance_;
}

Cp1616IODevice::Cp1616IODevice():
    cp_id_(1),
    cp_handle_(0),
    cp_session_key_(0),
    cp_ar_number_(0),
    ar_info_ind_flag_(0),
    prm_end_ind_flag_(0),
    indata_ind_flag_(0),
    offline_ind_flag_(0),
    p_device_data_(NULL)
{

    p_device_data_ = new DeviceData [DEVICE_DATA_ENTRIES * sizeof(DeviceData)];

    //Initialize device_data_ according to STEP7 project setup
    p_device_data_[0].slot =  1;
    p_device_data_[0].subslot = 1;
    p_device_data_[0].modId = 0x19;
    p_device_data_[0].subslotId = 0x010001;
    p_device_data_[0].api = 0;
    p_device_data_[0].maxSubslots = 0;
    p_device_data_[0].modState = 0;
    p_device_data_[0].subState = 0;
    p_device_data_[0].dir = 0;

    p_device_data_[1].slot =  2;
    p_device_data_[1].subslot = 1;
    p_device_data_[1].modId = 0x23;
    p_device_data_[1].subslotId = 0x0001;
    p_device_data_[1].api = 0;
    p_device_data_[1].maxSubslots = 0;
    p_device_data_[1].modState = 0;
    p_device_data_[1].subState = 0;
    p_device_data_[1].dir = 0;

}

Cp1616IODevice::~Cp1616IODevice()
{
  delete p_device_data_;
}

int Cp1616IODevice::init()
{
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

  //Initialize the callback structure
  //Set the callback function pointers
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
                      cp_id_,                      //Communication Module index
                      PNIO_CEP_MODE_CTRL,            //permission to change operation mode
                      VENDOR_ID,                     //vendor ID
                      DEVICE_ID,                     //device ID
                      INSTANCE_ID,                   //instance ID
                      ui_max_ar,                     //max AR count
                      &struct_pnio_annotation,       //annotation
                      &struct_cb_functions,          //callback functions information
                      &cp_handle_);                  //device handle

  ROS_INFO("Device handle: 0x%x", cp_handle_);

  //Check errors
  if(error_code != PNIO_OK)
    ROS_ERROR("Not able to open PNIO_device: Error: 0x%x", (int)error_code);
   else
    ROS_INFO_STREAM("Openning CP1616 in IO_device mode: done");

  return error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::uinit()
{
  PNIO_UINT32 error_code = PNIO_OK;

  error_code = PNIO_device_close(cp_handle_);

  if(error_code != PNIO_OK)
    ROS_ERROR("Not able to uninitialize: Error 0x%x", (int) error_code);

  ROS_INFO_STREAM("Closing PNIO_device: done ");
  return (int)error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::addApi()
{
  int i,j;
  int highest_slots_number;
  int highest_subslot_number = 0;

  PNIO_UINT32 api;
  PNIO_UINT32 error_code = PNIO_OK;

  //for each slot
  for(i = j = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    //read api from configuration data
    api = p_device_data_[i].api;

    //look if api added at a prior position
    for(j = 0; j < i; j++)
    {
      if(api == p_device_data_[j].api){
      //api was added
      break;
    }
  }

  if(i == j)   // not added, add a new api
  {
    // calculate highest slot and subslot number for this api
    highest_slots_number   = p_device_data_[j].slot;
    highest_subslot_number = p_device_data_[j].subslot;

    //check if the api exists in the slots ahead,
    //if yes, then update highest slot/subslot number accordingly

    for(j = i+1; j <  DEVICE_DATA_ENTRIES; j++)
    {
      if(api == p_device_data_[j].api)
      {
        if(p_device_data_[j].slot > highest_slots_number) highest_slots_number = p_device_data_[j].slot;
        if(p_device_data_[j].subslot > highest_subslot_number) highest_subslot_number = p_device_data_[j].subslot;
      }
    }

    error_code = PNIO_api_add(
      cp_handle_,
      api,
      (PNIO_UINT16) highest_slots_number,
      (PNIO_UINT16) highest_subslot_number);

    if(error_code != PNIO_OK)
    {
      ROS_ERROR("Not able to add Api profile: Error 0x%x", (int) error_code);
      return (int)error_code;  //leave for-loop immediately
    }
  }
 }
  ROS_INFO_STREAM("Adding Api profile: done");
  return (int)error_code;  //if everything ok, return PNIO_OK
}

int Cp1616IODevice::removeApi()
{
  int i,j;
  PNIO_UINT32 api;
  PNIO_UINT32 error_code = PNIO_OK;

  //for each slot
  for(i = j = 0; i < DEVICE_DATA_ENTRIES && error_code == PNIO_OK; i++)
  {
    //read api from configuration data
    api = p_device_data_[i].api;

    //look if the api has been added at a prior position in our g_device_data structure
    for(j = 0; j < i; j++)
    {
      if(api == p_device_data_[j].api) break; // api added at a prior position, hence it has already been removed
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
        ROS_INFO_STREAM("Removing Api profile: done");
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
  
  addr.u.Geo.Slot    = p_device_data_[0].slot;    //plug module 0
  addr.u.Geo.Subslot = p_device_data_[0].subslot; //get the corresponding sub-slot

  error_code = PNIO_mod_plug(
               cp_handle_,   		              //device handle
               p_device_data_[0].api,             //api number
               &addr,                             //location(slot, subslot)
               p_device_data_[0].modId);          //module 0 identifier


  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to add module 0: Error 0x%x", (int)error_code);
    p_device_data_[0].modState = 0;
  }
  else
  {
    ROS_INFO_STREAM("Plugging Module 0: done");
    p_device_data_[0].modState = 1;
  }

  if(!p_device_data_[0].modState)
  {
    ROS_ERROR_STREAM("ERROR: Failure in plugging module 0 -> no other module / submodule will be plugged...");
    return (int)error_code;
  }

  //Add submodule corresponding to module 0
  error_code = PNIO_sub_plug (
                cp_handle_,                      //device handle
                p_device_data_[0].api,           // api number
                &addr,                           // location (slot, subslot)
                p_device_data_[0].subslotId);    // submodule 0 identifier


  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to add submodule 0 to module 0: Error 0x%x", (int) error_code);
    p_device_data_[0].subState = 0;
  }
  else
  {
    ROS_INFO_STREAM("Pluging submodule 0 to module 0: done ");
    p_device_data_[0].subState = 1;
  }

  if(!p_device_data_[0].subState)
  {
    ROS_ERROR_STREAM("ERROR: Failure in plugging the submodule corresponding to module 0"
                      << "-> no other module / submodule will be plugged...");

    return (int)error_code;  //if everything ok, return PNIO_OK
  }

  //Add all modules
  if(NUMOF_SLOTS > 1)
  {

    for(i = 1; i < DEVICE_DATA_ENTRIES;)
    {
      addr.u.Geo.Slot    = p_device_data_[i].slot;	     //plug module at correct slot
      addr.u.Geo.Subslot = p_device_data_[i].subslot;    //get the corresponding sub-slot



      error_code = PNIO_mod_plug(
             cp_handle_,                 //device handle
             p_device_data_[i].api,      //api number
             &addr,                      //location(slot, subslot)
             p_device_data_[i].modId);   //module identifier

      if(error_code != PNIO_OK)
      {
        ROS_ERROR("Not able to plug module: Error 0x%x", (int) error_code);
        p_device_data_[i].modState = 0;
        return (int)error_code;
      }
      else
      {
        ROS_INFO("Plugging module %d: done", i);
        p_device_data_[i].modState = 1;
      }

      if(error_code == PNIO_OK)
      {
        //advance in the g_device_data structure jumping over all the submodule entries
        //to reach the next module entry in the structure
        i += p_device_data_[i].maxSubslots;
      }

      else
      {
        //go to the next entry in p_device_data table
        i++;
      }

    } //end for

    //Add all submodules
    for(i = 1; i < DEVICE_DATA_ENTRIES; i++)
    {
      if(p_device_data_[i].maxSubslots > 0)
      {
        //beginning of a new slot
        slot = i;   //index of corresponding slot for a given subslot

        p_device_data_[slot].subState = 1;
      }

      if(p_device_data_[slot].modState)
      {
        //add submodule only if the module is added
        addr.u.Geo.Slot     = p_device_data_[i].slot;
        addr.u.Geo.Subslot  = p_device_data_[i].subslot;

        error_code = PNIO_sub_plug (
             cp_handle_,                    //device handle
             p_device_data_[i].api,         //api number
             &addr,                         //location(slot, subslot)
             p_device_data_[i].subslotId);  //submodule identifier

        if(error_code != PNIO_OK)
        {
          ROS_ERROR("Not able to plug submodule: Error 0x%x", (int) error_code);
          p_device_data_[i].subState = 0;
          p_device_data_[slot].subState = 0;
          return (int)error_code;
        }
        else
        {
          ROS_INFO("Plugging submodule to module %d: done", i);
          p_device_data_[i].subState = 1;
        }
      }
    }  //end for
  }

  //if not all the modules/submodules are plugged correctly, print warning
  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    if(p_device_data_[i].subState == 0)
      {
        ROS_WARN_STREAM("Not all modules or submodules were plugged correctly!!");
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
  for(i = DEVICE_DATA_ENTRIES -1; i >= 0 && error_code == PNIO_OK; i--)
  {
    if(p_device_data_[i].subState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;          //must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = p_device_data_[i].slot;  //slot number
      addr.u.Geo.Subslot = p_device_data_[i].subslot;

      //Remove submodules
      error_code = PNIO_sub_pull(
                  cp_handle_,
                  p_device_data_[i].api, &addr);

      if(error_code != PNIO_OK)
        ROS_ERROR("Not able to remove submodule Error 0x%x", (int) error_code);
      else
      {
        ROS_INFO("Removing submodule from module %d: done", i);
        p_device_data_[i].subState = 0;
        return (int)error_code;
      }

      //Notify the controller that the device state is NOT-OK every time after removing a submodule
      error_code = PNIO_set_dev_state(cp_handle_, PNIO_DEVSTAT_STATION_PROBLEM);
    }

    if(error_code == PNIO_OK && p_device_data_[i].modState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;                  //must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = p_device_data_[i].slot;         //slot number
      addr.u.Geo.Subslot = 1;                              //doesn't matter

      //Remove modules
      error_code = PNIO_mod_pull(cp_handle_, p_device_data_[i].api, &addr);

      if(error_code != PNIO_OK)
        ROS_ERROR("Not able to remove module: Error 0x%x", (int) error_code);
      else
      {
        ROS_INFO("Removing module %d: done", i);
        p_device_data_[i].subState = 0;
        return (int)error_code;
      }

      //Notify the controller that the device state is NOT-OK every time after removing a module
      error_code = PNIO_set_dev_state(cp_handle_, PNIO_DEVSTAT_STATION_PROBLEM);
    }
  }
}

int Cp1616IODevice::startOperation()
{
  PNIO_UINT32 error_code = PNIO_OK;

  error_code = PNIO_device_start(cp_handle_);
  if (error_code != PNIO_OK)
    ROS_ERROR("Not able to start IO Device operation: Error 0x%x", (int)error_code);
  else
      ROS_INFO_STREAM("Starting operation: done");

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

  ROS_INFO_STREAM("Waiting for callbacks...");

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
      error_code = -1;
    }
  }
  else
  {
    ROS_ERROR("PNIOCbfPrmEndInd callback not recieved within defined period");
    error_code = -1;
  }

  if(error_code == PNIO_OK)
    ROS_INFO_STREAM("All necessary callbacks called....");

  return (int)error_code;
}

int Cp1616IODevice::stopOperation()
{
  PNIO_UINT32 error_code = PNIO_OK;
  error_code = PNIO_device_stop(cp_handle_);
  if (error_code != PNIO_OK)
    ROS_ERROR("Not able to stop IO Device: Error 0x%x", (int)error_code);
  else
      ROS_INFO_STREAM("Stopping device operation: done");

    //wait for OFFLINE_IND_flag to be set by ar_offline_ind() callback
    unsigned int i = 0;
    while(i != MAX_OFFLINE_IND_COUNT)
    {
      if(getOfflineIndFlag() != 0)
        break;
            
      usleep(WAIT_FOR_CALLBACK_PERIOD);
      i++;
    }
}

int Cp1616IODevice::GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub)
{
  int i,j;

  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)      //Look for module index
  {
    if((int)mod == idx_table_[i])
    break;
  }

  if(i == DEVICE_DATA_ENTRIES)                  //no module means also no submodule
    return -1;

  for(j = 0; j < p_device_data_[i].maxSubslots; j++)
  {
    if(p_device_data_[i+j].subslot == (int)sub)  //find submodule index
      return j;
  }

  return -1;
}

void Cp1616IODevice::configureDeviceData()
{
  unsigned int i = 0;
  unsigned int begin_new_slot = 0;
  unsigned int idx = 0;

  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    ROS_INFO("Module: slot %x sub %x mod_id %x sub_id %x",
    p_device_data_[i].slot,
    p_device_data_[i].subslot,
    (unsigned int) p_device_data_[i].modId,
    (unsigned int) p_device_data_[i].subslotId);
  }

  //fill idxTbl with -1
  memset(idx_table_, -1, DEVICE_DATA_ENTRIES * sizeof(int));
  idx_table_[idx++] = p_device_data_[0].slot;

  //browsing through the device data structure
  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    if(p_device_data_[i].slot == p_device_data_[begin_new_slot].slot)
    {
      p_device_data_[begin_new_slot].maxSubslots++;
      p_device_data_[i].modId = p_device_data_[begin_new_slot].modId;
    }
    else
    {
      begin_new_slot = i;		                       //index corresponding to the beginning of the new slots
      p_device_data_[begin_new_slot].maxSubslots = 1;         //every new module/slot has min one sub-slot
      idx_table_[idx++] = p_device_data_[i].slot;             //store the entry of the new slot in idxTbl
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
   * for the functional submodules and set the local status to "GOOD". For all
   * non-functional submodules, the local status should be set to "BAD"
   * We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback
   */

  error_code = PNIO_initiate_data_write(cp_handle_);
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to initiate data write: Error 0x%x", (int)error_code);
    return (int)error_code;
  }
  /** We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program
   * can set the local status for functional submodules of all the outgoing data
   * (from the perspective of the IO controller) to GOOD. For all non-functional submodules,
   * the local status has to be set to "BAD".
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
   * for the functional submodules and set the local status to "GOOD". For all
   * non-functional submodules, the local status should be set to "BAD"
   * We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback
   */

  error_code = PNIO_initiate_data_write(cp_handle_);
  if(error_code != PNIO_OK)
  {
    ROS_ERROR("Not able to initiate data write: Error 0x%x", (int)error_code);
    return (int)error_code;
  }

  /** We also have to call "PNIO_initiate_data_read" so that the Device user program
   * can set the local status for functional submodules of all the outgoing data
   * (from the perspective of the IO controller) to GOOD. For all non-functional submodules,
   * the local status has to be set to "BAD".
   */

   error_code = PNIO_initiate_data_read(cp_handle_);
   if(error_code != PNIO_OK)
   {
     ROS_ERROR("Not able to initiate data read: Error 0x%x", (int)error_code);
     return (int)error_code;
   }
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

} //cp1616

#endif
