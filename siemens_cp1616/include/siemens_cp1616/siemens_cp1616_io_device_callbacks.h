/*********************************************************************************************//**
* @file cp1616_io_controller_callbacks.h
*
* Callbacks declarations - required by existing IO Base library callback interface
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
#ifndef SIEMENS_CP1616_IO_DEVICE_CALLBACKS_H
#define SIEMENS_CP1616_IO_DEVICE_CALLBACKS_H

namespace siemens_cp1616
{
namespace pnio_device_callbacks
{
  PNIO_IOXS dataWrite(
    PNIO_UINT32    dev_handle,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32    buffer_length,
    PNIO_UINT8    *p_buffer,
    PNIO_IOXS      io_consumer_status);

 PNIO_IOXS dataRead(
    PNIO_UINT32     dev_handle,
    PNIO_DEV_ADDR  *p_addr,
    PNIO_UINT32     buffer_length,
    PNIO_UINT8     *p_buffer,
    PNIO_IOXS       io_provider_status);

 void recordRead(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     api,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     sessi,
    PNIO_UINT32     sequence_number,
    PNIO_DEV_ADDR  *p_addr,
    PNIO_UINT32     record_index,
    PNIO_UINT32    *p_buffer_length,
    PNIO_UINT8     *p_buffer,
    PNIO_ERR_STAT  *p_pnio_state);

  void recordWrite(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     api,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     sessi,
    PNIO_UINT32     sequence_number,
    PNIO_DEV_ADDR  *p_addr,
    PNIO_UINT32     record_index,
    PNIO_UINT32    *p_buffer_length,
    PNIO_UINT8     *p_buffer,
    PNIO_ERR_STAT  *p_pnio_state);

  void checkIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     api,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_DEV_ADDR  *p_addr,
    PNIO_UINT32    *p_mod_ident,
    PNIO_UINT16    *p_mod_state,
    PNIO_UINT32    *p_sub_ident,
    PNIO_UINT16    *p_sub_state);
 
  void arCheckIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     host_ip,
    PNIO_UINT16     ar_type,
    PNIO_UUID_TYPE  ar_uuid,
    PNIO_UINT32     ar_properties,
    PNIO_UUID_TYPE  cmi_obj_uuid,
    PNIO_UINT16     cmi_station_name_length,
    PNIO_UINT8     *p_cmi_station_name,
    PNIO_AR_TYPE   *p_ar);
  
  void arInfoIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_TYPE   *p_ar);

  void arIndataIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key);

  void arAbortIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_REASON  reason_code);
  
  void arOfflineIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_REASON  reason_code);
 
  void prmEndIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_UINT32     api,
    PNIO_UINT16     slot_number,
    PNIO_UINT16     subslot_number);

  void cpStopRequest(
    PNIO_UINT32 dev_handle);

  void deviceStopped(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     reserved);
  
  void requestDone(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     user_handle,
    PNIO_UINT32     status,
    PNIO_ERR_STAT  *p_pnio_state);
  
  void apduStatusIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_APDU_STATUS_IND apdu_status);
  
  static const int MAX_AR_INFO_COUNT = 500;
  
} //pnio_device_callbacks
} //siemens_cp1616

#endif //SIEMENS_CP1616_IO_DEVICE_CALLBACKS_H
