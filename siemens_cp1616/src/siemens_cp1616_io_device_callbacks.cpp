/*********************************************************************************************//**
* @file siemens_cp1616_io_device_callbacks.cpp
*
* Callbacks required by IO Base library
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
#ifndef SIEMENS_CP1616_IO_DEVICE_CALLBACKS_CPP
#define SIEMENS_CP1616_IO_DEVICE_CALLBACKS_CPP

#include <siemens_cp1616/siemens_cp1616_io_device.h>
#include <siemens_cp1616/siemens_cp1616_io_device_callbacks.h>

namespace siemens_cp1616
{
namespace pnio_device_callbacks
{
  PNIO_IOXS dataRead(
    PNIO_UINT32 dev_handle,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32 buffer_length,
    PNIO_UINT8 *p_buffer,
    PNIO_IOXS iops)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    unsigned int i;

    PNIO_UINT32 slot_num     = p_addr->u.Geo.Slot;
    PNIO_UINT32 subslot_num  = p_addr->u.Geo.Subslot;

    ROS_DEBUG("dataRead(..., len=%u, Iops=%u) for devHandle 0x%x, slot %u, subslot %u",
      buffer_length, iops, dev_handle, slot_num, subslot_num);

    callback_handler->output_data_length_.at(slot_num) = buffer_length;  //save data length (only for debugging)
    callback_handler->output_data_iops_.at(slot_num) = iops;             //provider status (of remote IO controller)

    if(!buffer_length)
    {
      ROS_ERROR(" BufLen = 0, nothing to read...");
      callback_handler->output_data_iocs_.at(slot_num) = PNIO_S_GOOD;
     }
    else if(buffer_length <= (PNIO_UINT32)NUMOF_BYTES_PER_SLOT)
    {
      //Copy the stack to the application buffer
      memcpy (&callback_handler->output_data_.at(slot_num).at(0), p_buffer, buffer_length); 
      callback_handler->output_data_iocs_[slot_num] = PNIO_S_GOOD;                    // assume everything is ok      
    }
    else
    {
      ROS_ERROR("!!! dataRead: Buflen=%lu > allowed size (%u)!!! Abort reading...",
       (unsigned long)buffer_length, NUMOF_BYTES_PER_SLOT);
      callback_handler->output_data_iocs_[slot_num] = PNIO_S_BAD; // set local status to bad
    }

  return(callback_handler->output_data_iocs_[slot_num]);         //consumer state (of local IO device)
}

  PNIO_IOXS dataWrite(
    PNIO_UINT32 dev_handle,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32 buffer_length,
    PNIO_UINT8 *p_buffer,
    PNIO_IOXS iocs)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    unsigned int i;
 
    PNIO_UINT32 slot_num    = p_addr->u.Geo.Slot;
    PNIO_UINT32 subslot_num = p_addr->u.Geo.Subslot;

    ROS_DEBUG("dataWrite(len = %u, Iocs = %u devHandle = %u, slot = %u, subslot = %u",
      buffer_length, iocs, dev_handle, slot_num, subslot_num);
  
    callback_handler->input_data_length_.at(slot_num) = buffer_length;   //save data length (only for debugging)
    callback_handler->input_data_iocs_.at(slot_num) = iocs;              //consumer status (of remote IO Controller)

    if(!buffer_length)
    {
      callback_handler->input_data_iops_.at(slot_num) = PNIO_S_GOOD;
    }
    else if (buffer_length <= (PNIO_UINT32)NUMOF_BYTES_PER_SLOT)
    {
      //copy the application data to the stack
      memcpy(p_buffer, &callback_handler->input_data_.at(slot_num).at(0), buffer_length); 
      callback_handler->input_data_iops_.at(slot_num) = PNIO_S_GOOD;        //assume everything is ok
    }
    else
    {
      ROS_ERROR("!!! dataWrite: Buflen=%lu > allowed size (%u)!!! Abort writing..",
        (unsigned long)buffer_length, NUMOF_BYTES_PER_SLOT);

      callback_handler->input_data_iops_.at(slot_num) = PNIO_S_BAD; // set local status to bad 
    }

    return(callback_handler->input_data_iops_.at(slot_num));       //return local provider status
}

  void recordWrite(
    PNIO_UINT32 dev_handle,
    PNIO_UINT32 api,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 sessi,
    PNIO_UINT32 sequence_number,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32 record_index,
    PNIO_UINT32 *p_buffer_length,
    PNIO_UINT8 *p_buffer,
    PNIO_ERR_STAT *p_pnio_state)
  {

    PNIO_UINT8  write_rec_dummy_data[50];
    PNIO_UINT32 i;
    PNIO_UINT32 error_code = PNIO_OK;

    ROS_DEBUG("recordWrite Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu",
      (unsigned long)api,
      (unsigned long)p_addr->u.Geo.Slot,
      (unsigned long)p_addr->u.Geo.Subslot,
      (unsigned long)record_index,
      (unsigned long)*p_buffer_length,
      (unsigned long)sequence_number);
 
    //check data size (accepted_data < provided_data)
    if(*p_buffer_length > sizeof(write_rec_dummy_data))
    {
      *p_buffer_length = sizeof(write_rec_dummy_data);
      ROS_WARN("Can not transmit all the data, buffer too small");
    }

    //copy the record data into a buffer for further use
    memcpy(write_rec_dummy_data,  //destination pointer for record data
           p_buffer,              //source pointer for record data
          *p_buffer_length);      //length of the accepted data
    
    if(error_code == PNIO_OK)
    {
      memset(p_pnio_state, 0, sizeof(*p_pnio_state));
      return;
    }
    else  //if an eror occured, it must be specified according to IEC 61158-6
    {
      *p_buffer_length = 0;

      p_pnio_state->ErrCode   = 0xdf; //IODWrites with ErrorDecode = PNIORW
      p_pnio_state->ErrDecode = 0x80; //PNIORW
      p_pnio_state->ErrCode1  = 9;    //example: Error Class 10 = application, ErrorNr 9 = "feature not supported"
      p_pnio_state->ErrCode2  = 0;    //not used in this case
      p_pnio_state->AddValue1 = 0;    //not used in this case
      p_pnio_state->AddValue2  =0;    //not used in this case

      return;
    }
  }

  void recordRead(
    PNIO_UINT32 dev_handle,
    PNIO_UINT32 api,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 sessi,
    PNIO_UINT32 sequence_number,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32 record_index,
    PNIO_UINT32 *p_buffer_length,
    PNIO_UINT8 *p_buffer,
    PNIO_ERR_STAT *p_pnio_state)
  {

    PNIO_UINT32 i;
    PNIO_UINT32 error_code = PNIO_OK;

    //fill dummy buffer
    PNIO_UINT8 read_rec_dummy_data[] = {"Test Record"};

    if(*p_buffer_length > sizeof(read_rec_dummy_data))
      *p_buffer_length = sizeof(read_rec_dummy_data);

    ROS_DEBUG("recordRead Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu",
      (unsigned long)api,
      (unsigned long)p_addr->u.Geo.Slot,
      (unsigned long)p_addr->u.Geo.Subslot,
      (unsigned long)record_index,
      (unsigned long)*p_buffer_length,
      (unsigned long)sequence_number);

    //copy the data to specified buffer
    if(*p_buffer_length < sizeof(read_rec_dummy_data))
      ROS_WARN(" Can not transmit all the data, buffer too small");

    memcpy(p_buffer,              //destination pointer for write data
           read_rec_dummy_data,   //source pointer for write data
           *p_buffer_length);     //length of transmitted data

    if(error_code == PNIO_OK)
    {
      memset(p_pnio_state, 0, sizeof(*p_pnio_state));
      return;
    }
    else
    {
      *p_buffer_length=0;
      p_pnio_state->ErrCode   = 0xde; //IODReadRes with ErrorDecode = PNIORW
      p_pnio_state->ErrDecode = 0x80; //PNIORW
      p_pnio_state->ErrCode1  = 9;    //example: Error Class 10 = application, ErrorNr 9 = "feature not supported"
      p_pnio_state->ErrCode2  = 0;    //not used in this case
      p_pnio_state->AddValue1 = 0;    //not used in this case
      p_pnio_state->AddValue2 = 0;    //not used in this case
      return;
    }
  }

  void checkIndication(
    PNIO_UINT32 dev_handle,
    PNIO_UINT32 api,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 session_key,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32 *p_mod_ident,
    PNIO_UINT16 *p_mod_state,
    PNIO_UINT32 *p_sub_ident,
    PNIO_UINT16 *p_sub_state)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    unsigned int idx;

    ROS_INFO("Wrong configuration: slot=%u, subslot=%u, ModId=0x%x, State(%u), SubId=%u, State (%u)",
      p_addr->u.Geo.Slot, p_addr->u.Geo.Subslot, *p_mod_ident, *p_mod_state, *p_sub_ident, *p_sub_state);

    // get the index int of our configuration 
    idx = callback_handler->GetSubmodNum(p_addr->u.Geo.Slot, p_addr->u.Geo.Subslot);

    /* Check the configuration sent by device against the configuration_data structure.
    If there is any mismatch, return error. */

    if((idx != -1) && ((unsigned int)callback_handler->modules_[idx].subslot == p_addr->u.Geo.Subslot)
                   && (callback_handler->modules_[idx].modId == *p_mod_ident)
                   && (callback_handler->modules_[idx].subslotId == *p_sub_ident))
    {
      *p_mod_state = PNIO_MOD_STATE_PROPER_MODULE;
      *p_sub_state = PNIO_SUB_STATE_IDENT_OK;
    } 
    else
    {
      ROS_WARN_STREAM("The configuration of CP1616 modules is inconsistent to downloaded HWCONFIG"
                << " please check your configuration!");
      *p_mod_state = PNIO_MOD_STATE_WRONG_MODULE;
      *p_sub_state = PNIO_SUB_STATE_IDENT_WRONG;
    }
}

  void arCheckIndication(
          PNIO_UINT32 dev_handle,
          PNIO_UINT32 host_ip,
          PNIO_UINT16 ar_type,
          PNIO_UUID_TYPE ar_uuid,
          PNIO_UINT32 ar_properties,
          PNIO_UUID_TYPE cmi_obj_uuid,
          PNIO_UINT16 cmi_station_name_length,
          PNIO_UINT8 *p_cmi_station_name,
          PNIO_AR_TYPE *p_ar)
  {
    union
    {
      unsigned long l;
      unsigned char c[4];
    } lc;

    char stname[256];
    int  len = cmi_station_name_length < 256 ? cmi_station_name_length : 255;
    lc.l = host_ip;
    strncpy(stname, (const char *)p_cmi_station_name, len);  //copy StationName to stname
    stname[len] = '\0';
    ROS_INFO("IO Controller found: %s IP %d.%d.%d.%d", stname,lc.c[0], lc.c[1], lc.c[2], lc.c[3]);
}

  void arInfoIndication(
          PNIO_UINT32 dev_handle,
          PNIO_UINT16 ar_number,
          PNIO_UINT16 session_key,
          PNIO_AR_TYPE *p_ar)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    int i,j;

    callback_handler->setCpArNumber(ar_number);        //Store the AR number
    callback_handler->setCpSessionKey(session_key);    //Store the session key

    ROS_INFO("New AR from IO controller established");
     
    // set local provider status preset values for all input/output slots
    // set local consumer status for all output slots
    for(i = 0; i < callback_handler->modules_.size(); i++)
    {
      //set local provider state = GOOD for input data
      if(i == 0) 
      {
        if(callback_handler->modules_[i].modState == 1) // plugged 
        {
          callback_handler->input_data_iops_.at(i) = PNIO_S_GOOD;
          callback_handler->output_data_iocs_.at(i) = PNIO_S_GOOD;
        }
        else
        {
          callback_handler->input_data_iops_.at(i) = PNIO_S_BAD;
          callback_handler->output_data_iocs_.at(i) = PNIO_S_BAD;
        }
      }
      else
      {
        if((callback_handler->modules_[i].modState == 1)
            && (callback_handler->modules_[i].subState == 1)) // plugged 
        {
          callback_handler->input_data_iops_.at(i) = PNIO_S_GOOD;
          callback_handler->output_data_iocs_.at(i) = PNIO_S_GOOD;
        }
        else
        {
          callback_handler->input_data_iops_.at(i) = PNIO_S_BAD;
          callback_handler->output_data_iocs_.at(i) = PNIO_S_BAD;
        }
      }
    }
  

  callback_handler->setArInfoIndFlag(1);
}

  void arIndataIndication(
          PNIO_UINT32     dev_handle,
          PNIO_UINT16     ar_number,
          PNIO_UINT16     session_key)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    callback_handler->setIndataIndFlag(1);
  }

  void arAbortIndication(
    PNIO_UINT32 dev_handle,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 session_key,
    PNIO_AR_REASON reason_code)
  {
    // AR abort after ArInData-indication
    ROS_INFO("arAbortIndication: ArNumber = %x, Reason = %x", ar_number, reason_code);
  }

  void arOfflineIndication(
    PNIO_UINT32 dev_handle,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 session_key,
    PNIO_AR_REASON reason_code)
  {
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    ROS_INFO("arOfflineIndication, ArNumber = %x, Reason = %x", ar_number, reason_code);
    callback_handler->setOfflineIndFlag(1);
  }

  void prmEndIndication(
    PNIO_UINT32 dev_handle,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 session_key,
    PNIO_UINT32 api,
    PNIO_UINT16 slot_number,
    PNIO_UINT16 subslot_number)
  {
 
    //Create callback_handler object to access cp1616_io_device variables
    Cp1616IODevice *callback_handler = Cp1616IODevice::getDeviceInstance();

    unsigned int i = 0;

    // Wait (MAX_COUNT x 0.1s) for PNIO_cbf_ar_info_ind() Callbacks
    while (!callback_handler->getArInfoIndFlag())
    {
      if (i == MAX_AR_INFO_COUNT)
      {
        ROS_ERROR("No arInfoInd event recieved");
        return;
      }
      i++;
      usleep(100000);
    }
    ROS_INFO("End of CP parametrization");

    callback_handler->setPrmEndIndFlag(1);
  }

  void cpStopRequest(
    PNIO_UINT32 dev_handle)
  {
    ROS_INFO("cpStopReq called");
  }

  void deviceStopped(
    PNIO_UINT32 dev_handle,
    PNIO_UINT32 reserved)
  {
    ROS_INFO("IO Device stopped");
  }

  void requestDone(
    PNIO_UINT32 dev_handle,
    PNIO_UINT32 user_handle,
    PNIO_UINT32 status,
    PNIO_ERR_STAT *p_pnio_state)
  {
    ROS_INFO("IO Controller confirmed alarm: UserHndl: 0x%x", user_handle, status);
  }

  void apduStatusIndication(
    PNIO_UINT32 dev_handle,
    PNIO_UINT16 ar_number,
    PNIO_UINT16 session_key,
    PNIO_APDU_STATUS_IND apdu_status)
  {
    ROS_INFO("apduStatusInd not supported");
  }
  
} //pnio_device_callbacks
} //siemens_cp1616

#endif //SIEMENS_CP1616_IO_DEVICE_CALLBACKS_CPP
