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
#ifndef CP1616_IO_DEVICE_CALLBACKS_H
#define CP1616_IO_DEVICE_CALLBACKS_H

namespace cp1616
{
namespace pnio_device_callbacks
{
  
  /**
   * \brief Passes the input data from the application to the stack.
   *
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  *p_addr             geographical address
   * \param [in]  buffer_length       length of the submodule input data
   * \param [out] *p_buffer           pointer to data buffer to write to
   * \param [in]  io_consumer_status  remote (io controller) consumer status
   */
  PNIO_IOXS dataWrite(
    PNIO_UINT32    dev_handle,
    PNIO_DEV_ADDR *p_addr,
    PNIO_UINT32    buffer_length,
    PNIO_UINT8    *p_buffer,
    PNIO_IOXS      io_consumer_status);

  /**
   * \brief Passes the output data from stack to the application
   *
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  *p_addr             geographical address
   * \param [in]  buffer_length       length of the submodule input data
   * \param [in]  *p_buffer           pointer to data buffer to read from
   * \param [in]  io_provider_status  remote (io controller) provider status
   */
  PNIO_IOXS dataRead(
    PNIO_UINT32     dev_handle,
    PNIO_DEV_ADDR  *p_addr,
    PNIO_UINT32     buffer_length,
    PNIO_UINT8     *p_buffer,
    PNIO_IOXS       io_provider_status);

  /**
   * \brief Passes the output data from stack to the application
   *
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  api                 qpi number
   * \param [in]  ar_number           application relation number
   * \param [in]  sessi               session-key
   * \param [in]  sequence_number     sequence number
   * \param [in]  *p_addr             geographical address
   * \param [in]  record_index        record index
   * \param [in]  *p_buffer_length    pointer to buffer length
   * \param [in]  buffer_length       length of the submodule input data
   * \param [in]  *p_buffer           pointer to data buffer to read from
   * \param [in]  *p_pnio_state       4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, ErrCode2), see IEC61158-6
   */
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

   /**
   * \brief This callback is called to notify that a write record request has been received from the PNIO controller
   *
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  api                 api number
   * \param [in]  ar_number           application relation number
   * \param [in]  sessi               session-key
   * \param [in]  sequence_number     sequence number
   * \param [in]  *p_addr             geographical address
   * \param [in]  record_index        record index
   * \param [in/out] *p_buffer_length pointer to buffer length
   * \param [in]  buffer_length       length of the submodule input data
   * \param [in]  *p_buffer           pointer to data buffer to read from
   * \param [in]  *p_pnio_state       4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, ErrCode2), see IEC61158-6
   */
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

  /**
   * \brief This callback is called for each and every submodule of the IO Base Device interface for which the
   * configuration does not match that of the IO controller. The user program is consequently given the option
   * of changing the submodule layout or marking the submodule as compatible or incorrect. In case of agreement
   * of the configurations, this callback is not called.
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  api                 api number
   * \param [in]  ar_number           application relation number
   * \param [in]  sessi               session-key
   * \param [in]  *p_addr             geographical address
   * \param [out] *p_mod_ident        pointer to the module identifier
   * \param [out] *p_mod_state        pointer to the module state
   * \param [out] *p_sub_ident        pointer to submodule identifier
   * \param [out] *p_sub_state        pointer to submodule state
   */
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
 
  /**
   * \brief This callback is called by the IO Base Device interface as soon as an IO controller establishes
   * a connection with the IO Base Device user programm and transmits its expected configuration for the IO
   * Base Device user program. As a result of this callback, application-relation global parameters are
   * transferred to the IO Base user program for inspection. In case of errors in the application-relation
   * layout, the IO Base Device user program can terminate the application-retional global parameters
   *
   * \param [in]  dev_handle                device handle
   * \param [in]  api                       api number
   * \param [in]  host_ip                   IP address of the host IO Controller
   * \param [in]  ar_type                   application-relation type
   * \param [in]  ar_uuid                   application-relation UUID
   * \param [in]  ar_properties             application-relation properties
   * \param [in]  cmi_obj_uuid              UUID of application-relation initiator e.g IO controller
   * \param [in]  cmi_station_name_length   length of param station-name
   * \param [in]  *p_cmi_station_name       station name
   * \param [in]  *p_ar                     pointer to application-relation structure PNIO_AR_TYPE
   */
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
  
  /**
   * \brief This callback is called by the IO Base Device interface as soon as the application-relation for
   * the IO controller is laid out. Consequently, the IO-Base-Device user program is informed about the
   * modules and submodules that will be operated in this application relation.. In this callback function
   * user can initialize IO data in submodule to initial value 0 and can also set local consumer and provider
   * status
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param [in]  *p_ar               pointer to application-relation structure PNIO_AR_TYPE
   */
  void arInfoIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_TYPE   *p_ar);

  /**
   * \brief This callback is called by the IO Base Device interface as soon as an IO controller has transmitted
   * the IO data for the first time. It signals the beginning of cyclical data exchange.
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param [in]  *p_ar               pointer to application-relation structure PNIO_AR_TYPE
   */
  void arIndataIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key);

  /**
   * \brief This callback is called by the IO Base Device interface as soon as the connection is terminated
   * after a data exchange with the IO Controller began
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param [in]  *p_ar               pointer to application-relation structure PNIO_AR_TYPE
   * \param [in]  reason_code         reason code
   */
  void arAbortIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_REASON  reason_code);
  
  /**
   * \brief  This callback is called by the IO Base Device interface as soon as the connection is terminated
   * before a data exchange with the IO Controller began
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param [in]  *p_ar               pointer to application-relation structure PNIO_AR_TYPE
   * \param [in]  reason_code         reason code
   */
  void arOfflineIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_AR_REASON  reason_code);
 
  /**
   * \brief This callback is called by the IO Base Device interface, as soon as an IO controller signals
   * the end of the parametrizing phase.
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param [in]  api                 associated API
   * \param [in]  slot_number         slot number
   * \param [in]  subslot_number      subslot number
   */
  void prmEndIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_UINT32     api,
    PNIO_UINT16     slot_number,
    PNIO_UINT16     subslot_number);

  /**
   * \brief This callback is called by the IO Base Device interface, as soon as an IO controller recieves
   * stop request from PNIO_device_stop function.
   *
   * \param [in]  dev_handle          device handle
   */
  void cpStopRequest(
    PNIO_UINT32 dev_handle);

  /**
   * \brief  This callback is called by the IO Base Device interface after the device is stopped.
   *
   * \param [in]  dev_handle          device handle
   */
  void deviceStopped(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     reserved);
  
  /**
   * \brief This callback is called by the IO Base Device interface, as soon as an alarm is sent
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  user_handle         user definned handle
   * \param       status
   * \param [out] *p_pnio_state       pointer to submodule state
   */
  void requestDone(
    PNIO_UINT32     dev_handle,
    PNIO_UINT32     user_handle,
    PNIO_UINT32     status,
    PNIO_ERR_STAT  *p_pnio_state);
  
  /**
   * \brief This callback is called after the state of the Controller has changed
   *
   * \param [in]  dev_handle          device handle
   * \param [in]  ar_number           application-relation number
   * \param [in]  session_key         session key
   * \param       apdu_status
   */
  void apduStatusIndication(
    PNIO_UINT32     dev_handle,
    PNIO_UINT16     ar_number,
    PNIO_UINT16     session_key,
    PNIO_APDU_STATUS_IND apdu_status);
 
  /**
   * \brief max counter value for prm_ar_info_ind() callback
   */
  static const int MAX_AR_INFO_COUNT = 500;
  
} //pnio_device_callbacks
} //cp1616

#endif //CP1616_IO_DEVICE_CALLBACKS_H
