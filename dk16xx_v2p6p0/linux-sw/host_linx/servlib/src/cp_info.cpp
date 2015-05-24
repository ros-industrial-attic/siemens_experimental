/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : cp_info.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : SERV_CP_info_... functions
*****************************************************************************
* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  *
* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  *
* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      *
* Schutzrechtsvermerke sowie nur vollstaedig und unveraendert weitergegeben *
* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         *
* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      *
* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       *
* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN *
* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*
* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN *
* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND *
* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   *
* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  *
* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   *
*                                                                           *
* This Software is Freeware. It is distributed for free. You may copy,      *
* modify and use it for free as well as distribute it to others. You may    *
* only distribute it as a whole, unmodified and with all trademarks and     *
* copyright notices. You may not distribute it commercially (e.g. as a      *
* Share-/Freeware-Distributor) without the express written permission of    *
* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  *
* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND *
* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          *
* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         *
* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               *
* If you happen to find any faults in it please tell us.                    *
*****************************************************************************/

#include "os.h"
#include "cp16xx.h"
#include "dpr_msg.h"
#include "sysa_fwlda_ifhost.h"
#include "pnioerrx.h"
#include "servusrx.h"
#include "tracemcrm.h"

extern CTrmWrapper ti;

typedef struct {
  bool                   used;
  PNIO_UINT32            cp_idx; /* cp index 1,2,... */
  DPR_DRV_HANDLE         con_fd; /* control descriptor */
  PNIO_UINT32            handle; /* handle that got SERV_CP_info application instance by SERV_CP_info_open */

  DPR_DRV_HANDLE         mgt_chnl; /* handle of DPR-RAM MGT channel */
  bool                   mgt_chnl_bounded; /* flag is the DPR-RAM MGT channel is bounded to application, to allow receive data */

  DPR_THREAD_HANDLE      th_reader;  /* thread handle, that receive data from firmware */
  DPR_UINT16             stop_thread; /* stopped receiver thread */

  DPR_UINT32             user_id; /* will be delivered from cp1616 driver */

  DPR_CHAR              * user_pool_ptr;
  DPR_UINT32              user_pool_length; /* length of memory pool to receive data from firmware */
  DPR_UINT32              send_pool_length; /* maximum data length that can be send to firmware */

  DPR_SEMAPHORE           sem_cnf_received; /* synch_rqb_cnf received */
  bool                    sem_cnf_received_initialized; /* is the sem_cnf_received initialized */
  FWLDA_DPR_RQB           synch_rqb_cnf;    /* synch_rqb_cnf will be saved here */

  SERV_CP_INFO_CBF        user_event_cbf[SERV_CP_INFO_LED_STATE+1];
} cpinfo_instance;

#define CPINFO_MAX_USER 4

cpinfo_instance cpinfo_usr[CPINFO_MAX_USER];

DPR_MUTEX ServMutexCpInfoUser;

static class StaticInitCpInfo {
public:
  StaticInitCpInfo(void) {
    DPR_MUTEX_CREATE_UNLOCKED(ServMutexCpInfoUser);
    memset(cpinfo_usr, 0, sizeof(cpinfo_usr));
  };

} static_init_cpinfo;

/*----------------------------------------------------------------------------
 PURPOSE  : create new control instance
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
cpinfo_instance * cpinfo_new_instance(void)
{
  cpinfo_instance *i_ret = 0;
  DPR_MUTEX_LOCK(ServMutexCpInfoUser);
  for(int i = 0; i < CPINFO_MAX_USER; i++)
  {
    if(cpinfo_usr[i].used == false){

      memset(&cpinfo_usr[i], 0, sizeof(cpinfo_usr[i]));

      cpinfo_usr[i].used = true;
      cpinfo_usr[i].handle = 0xABCDEF00 + i; /* create one unique handle */

      i_ret = &cpinfo_usr[i];
      break;
    }
  }
  DPR_MUTEX_UNLOCK(ServMutexCpInfoUser);

  return i_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : find existing control instance belong to handle
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
cpinfo_instance * cpinfo_get_instance(PNIO_UINT32 handle)
{
  cpinfo_instance *i_ret = 0;
  DPR_MUTEX_LOCK(ServMutexCpInfoUser);
  for(int i = 0; i < CPINFO_MAX_USER; i++)
  {
    if(cpinfo_usr[i].used &&
        cpinfo_usr[i].handle == handle){
      i_ret = &cpinfo_usr[i];
      break;
    }
  }
  DPR_MUTEX_UNLOCK(ServMutexCpInfoUser);

  return i_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : free one control instance
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
void cpinfo_free_instance(cpinfo_instance *inst)
{
  DPR_MUTEX_LOCK(ServMutexCpInfoUser);

  for(int i = 0; i < CPINFO_MAX_USER; i++)
  {
    if(inst == &cpinfo_usr[i]){ /* check is i a valid pointer, avoid memory overwriting */
      inst->used = false;
      break;
    }
  }
  DPR_MUTEX_UNLOCK(ServMutexCpInfoUser);
}

/*----------------------------------------------------------------------------
 PURPOSE  :
------------------------------------------------------------------------------
 COMMENTS : calls user registered callback functions
----------------------------------------------------------------------------*/
void cpinfo_call_usr_cbf(FWLDA_DPR_RQB *pRqb)
{
  SERV_CP_INFO_PRM info_prm;
  SERV_CP_INFO_CBF user_cbf = 0;

  memset(&info_prm, 0, sizeof(info_prm));

  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) != FWLDA_OP_CPINFO_NOTIFICATION){
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: unknown opcode %d", LE_TO_CPU(pRqb->fwlda_rqb.opcode));
    return;
  }

  cpinfo_instance * cpinfo_i = cpinfo_get_instance(CPU_TO_LE(pRqb->dpr_hdr.userid));
  if(!cpinfo_i){
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: unknown dpr_hdr.userid %d", CPU_TO_LE(pRqb->dpr_hdr.userid));
    return;
  }

  if(LE_TO_CPU(pRqb->fwlda_rqb.rqb_len) < sizeof(pRqb->fwlda_rqb) + sizeof(FWLDA_CPINFO_REPORT)){
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong fwlda_rqb.rqb_len %d (expected >=%d)",
       LE_TO_CPU(pRqb->fwlda_rqb.rqb_len), sizeof(pRqb->fwlda_rqb) + sizeof(FWLDA_CPINFO_REPORT));
    return;
  }

  FWLDA_CPINFO_REPORT *pInfoRep = (FWLDA_CPINFO_REPORT*)((PNIO_UINT8*)pRqb + sizeof(*pRqb));

  TRM_OUT01(&ti, GR_CHNL, LV_INFO, "cpinfo_call_usr_cbf: event_type %d", LE_TO_CPU(pInfoRep->event_type));

  switch(LE_TO_CPU(pInfoRep->event_type)){
  case SERV_CP_INFO_STOP_REQ:
    if(LE_TO_CPU(pInfoRep->blk_len) < sizeof(*pInfoRep)){
      TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong pInfoRep->blk_len %d (expected %d)",
         LE_TO_CPU(pInfoRep->blk_len), sizeof(*pInfoRep));
      return;
    }

    user_cbf = cpinfo_i->user_event_cbf[SERV_CP_INFO_STOP_REQ];
    break;

  case SERV_CP_INFO_PDEV_DATA:

    {
      user_cbf = cpinfo_i->user_event_cbf[SERV_CP_INFO_PDEV_DATA];
#ifdef PNIO_ALARM_OLD_STRUC
      if(LE_TO_CPU(pInfoRep->blk_len) < sizeof(*pInfoRep) + sizeof(info_prm.u.PdevData) -
                                     sizeof(info_prm.u.PdevData.AlarmAinfo.UserAlarmData))
      {

          TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong pInfoRep->blk_len %d (expected >=%d)",
                        LE_TO_CPU(pInfoRep->blk_len), sizeof(*pInfoRep) + sizeof(info_prm.u.PdevData) -
                                     sizeof(info_prm.u.PdevData.AlarmAinfo.UserAlarmData));
        return;
      }
#else
      if(LE_TO_CPU(pInfoRep->blk_len) < sizeof(*pInfoRep) + sizeof(info_prm.u.PdevData) -
                                     sizeof(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData))
      {

          TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong pInfoRep->blk_len %d (expected >=%d)",
                        LE_TO_CPU(pInfoRep->blk_len), sizeof(*pInfoRep) + sizeof(info_prm.u.PdevData) -
                                     sizeof(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData));
        return;
      }

#endif
      SERV_CIB_PDEV_DATA *pDevData = (SERV_CIB_PDEV_DATA*)((PNIO_UINT8*)pInfoRep + sizeof(*pInfoRep));

      info_prm.u.PdevData.Version = LE_TO_CPU(pDevData->Version);
      info_prm.u.PdevData.IODataType  = (PNIO_IO_TYPE)LE_TO_CPU(pDevData->IODataType); /* input, output */
      info_prm.u.PdevData.LogAddr = LE_TO_CPU(pDevData->LogAddr);
      info_prm.u.PdevData.Slot = LE_TO_CPU(pDevData->Slot);
      info_prm.u.PdevData.Subslot = LE_TO_CPU(pDevData->Subslot);
      info_prm.u.PdevData.AlarmType = (PNIO_ALARM_TYPE)LE_TO_CPU(pDevData->AlarmType);
      info_prm.u.PdevData.AlarmPriority = (PNIO_APRIO_TYPE)LE_TO_CPU(pDevData->AlarmPriority);
      info_prm.u.PdevData.AlarmSequence = LE_TO_CPU(pDevData->AlarmSequence);
      info_prm.u.PdevData.StationNr = LE_TO_CPU(pDevData->StationNr);

      /*info_prm.u.PdevData.Reserved1[2];     reserved, don't use, silentalarm */
      /*info_prm.u.PdevData.Reserved2[48];    reserved, don't use, obstart header */

      info_prm.u.PdevData.AlarmTinfo.CompatDevGeoaddr = LE_TO_CPU16(pDevData->AlarmTinfo.CompatDevGeoaddr);
      info_prm.u.PdevData.AlarmTinfo.ProfileType      = pDevData->AlarmTinfo.ProfileType;
      info_prm.u.PdevData.AlarmTinfo.AinfoType        = pDevData->AlarmTinfo.AinfoType;
      info_prm.u.PdevData.AlarmTinfo.ControllerFlags  = pDevData->AlarmTinfo.ControllerFlags;
      info_prm.u.PdevData.AlarmTinfo.DeviceFlag       = pDevData->AlarmTinfo.DeviceFlag;
      info_prm.u.PdevData.AlarmTinfo.PnioVendorIdent  = LE_TO_CPU16(pDevData->AlarmTinfo.PnioVendorIdent);
      info_prm.u.PdevData.AlarmTinfo.PnioDevIdent     = LE_TO_CPU16(pDevData->AlarmTinfo.PnioDevIdent);
      info_prm.u.PdevData.AlarmTinfo.PnioDevInstance  = LE_TO_CPU16(pDevData->AlarmTinfo.PnioDevInstance);

      info_prm.u.PdevData.Diagnostics.Channel = LE_TO_CPU16(pDevData->Diagnostics.Channel);
      info_prm.u.PdevData.Diagnostics.Properties = LE_TO_CPU16(pDevData->Diagnostics.Properties);
      info_prm.u.PdevData.Diagnostics.ErrType    = LE_TO_CPU16(pDevData->Diagnostics.ErrType);
      info_prm.u.PdevData.Diagnostics.ExtErrType = LE_TO_CPU16(pDevData->Diagnostics.ExtErrType);
      info_prm.u.PdevData.Diagnostics.ExtAdvAlHi = LE_TO_CPU16(pDevData->Diagnostics.ExtAdvAlHi);
      info_prm.u.PdevData.Diagnostics.ExtAdvAlLo = LE_TO_CPU16(pDevData->Diagnostics.ExtAdvAlLo);

      memcpy(info_prm.u.PdevData.DiagDs, pDevData->DiagDs, sizeof(info_prm.u.PdevData.DiagDs));

      info_prm.u.PdevData.AlarmAinfo.BlockType = LE_TO_CPU16(pDevData->AlarmAinfo.BlockType);
      info_prm.u.PdevData.AlarmAinfo.BlockVersion = LE_TO_CPU16(pDevData->AlarmAinfo.BlockVersion);
      info_prm.u.PdevData.AlarmAinfo.Api = LE_TO_CPU16(pDevData->AlarmAinfo.Api);
      info_prm.u.PdevData.AlarmAinfo.AlarmSpecifier = LE_TO_CPU16(pDevData->AlarmAinfo.AlarmSpecifier);
      info_prm.u.PdevData.AlarmAinfo.ModIdent = LE_TO_CPU16(pDevData->AlarmAinfo.ModIdent);
      info_prm.u.PdevData.AlarmAinfo.SubIdent = LE_TO_CPU16(pDevData->AlarmAinfo.SubIdent);

      info_prm.u.PdevData.AlarmAinfo.UserStrucIdent = LE_TO_CPU16(pDevData->AlarmAinfo.UserStrucIdent);
      info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen = LE_TO_CPU16(pDevData->AlarmAinfo.UserAlarmDataLen);
#ifdef PNIO_ALARM_OLD_STRUC
      if(info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen > sizeof(info_prm.u.PdevData.AlarmAinfo.UserAlarmData)){
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong ...AlarmAinfo.UserAlarmDataLen %d (expected <= %d)",
                     info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen, sizeof(info_prm.u.PdevData.AlarmAinfo.UserAlarmData));

        info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen = sizeof(info_prm.u.PdevData.AlarmAinfo.UserAlarmData);
      }

      memcpy(info_prm.u.PdevData.AlarmAinfo.UserAlarmData, pDevData->AlarmAinfo.UserAlarmData,
               info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen);
#else
     if(info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen > sizeof(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData)){
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong ...AlarmAinfo.UserAlarmDataLen %d (expected <= %d)",
                     info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen, sizeof(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData));

        info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen = sizeof(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData);
      }

      memcpy(info_prm.u.PdevData.AlarmAinfo.UAData.UserAlarmData, pDevData->AlarmAinfo.UAData.UserAlarmData,
               info_prm.u.PdevData.AlarmAinfo.UserAlarmDataLen);
#endif
    }
    break;

  case SERV_CP_INFO_PDEV_INIT_DATA:
    {
      user_cbf = cpinfo_i->user_event_cbf[SERV_CP_INFO_PDEV_DATA];
      SERV_CIB_PDEV_INIT_DATA *pPdevInitData = (SERV_CIB_PDEV_INIT_DATA*)((PNIO_UINT8*)pInfoRep + sizeof(*pInfoRep));

      info_prm.u.PdevInitData.ItemNumber = LE_TO_CPU(pPdevInitData->ItemNumber);

      if(LE_TO_CPU(pInfoRep->blk_len) != sizeof(*pInfoRep) + sizeof(info_prm.u.PdevInitData.ItemNumber) +
           (info_prm.u.PdevInitData.ItemNumber * sizeof(SERV_CIB_PDEV_INIT_STATE)) ){

          TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong pInfoRep->blk_len %d (expected %d)",
                        LE_TO_CPU(pInfoRep->blk_len), sizeof(*pInfoRep) + sizeof(info_prm.u.PdevInitData.ItemNumber) +
                                                       (info_prm.u.PdevInitData.ItemNumber * sizeof(SERV_CIB_PDEV_INIT_STATE)) );
        return;
      }

      for(PNIO_UINT32 i = 0; i< info_prm.u.PdevInitData.ItemNumber; i++){
        info_prm.u.PdevInitData.InitState[i].Reserved1 = pPdevInitData->InitState[i].Reserved1;
        info_prm.u.PdevInitData.InitState[i].Reserved2 = pPdevInitData->InitState[i].Reserved2;
        info_prm.u.PdevInitData.InitState[i].Reserved3 = LE_TO_CPU16(pPdevInitData->InitState[i].Reserved3);
        info_prm.u.PdevInitData.InitState[i].Reserved4 = LE_TO_CPU16(pPdevInitData->InitState[i].Reserved4);

        info_prm.u.PdevInitData.InitState[i].PortState = LE_TO_CPU(pPdevInitData->InitState[i].PortState);
        info_prm.u.PdevInitData.InitState[i].IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pPdevInitData->InitState[i].IODataType);
        info_prm.u.PdevInitData.InitState[i].LogAddr = LE_TO_CPU(pPdevInitData->InitState[i].LogAddr);
        info_prm.u.PdevInitData.InitState[i].Slot = LE_TO_CPU(pPdevInitData->InitState[i].Slot);
        info_prm.u.PdevInitData.InitState[i].Subslot = LE_TO_CPU(pPdevInitData->InitState[i].Subslot);
      }
    }
    break;

  case SERV_CP_INFO_LED_STATE:
    {
      user_cbf = cpinfo_i->user_event_cbf[SERV_CP_INFO_LED_STATE];
      if(LE_TO_CPU(pInfoRep->blk_len) < sizeof(*pInfoRep) + sizeof(info_prm.u.LedInfo)){
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: wrong pInfoRep->blk_len %d (expected %d)",
           LE_TO_CPU(pInfoRep->blk_len), sizeof(*pInfoRep));
        return;
      }
      SERV_CIB_LED *pLedInfo = (SERV_CIB_LED*)((PNIO_UINT8*)pInfoRep + sizeof(*pInfoRep));

      info_prm.u.LedInfo.LedState = (SERV_CIB_LED_STATE)LE_TO_CPU(pLedInfo->LedState);
      info_prm.u.LedInfo.LedType  = (SERV_CIB_LED_TYPE)LE_TO_CPU(pLedInfo->LedType);
    }
    break;

  default:
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "cpinfo_call_usr_cbf: unknown pInfoRep->event_type %d", LE_TO_CPU(pInfoRep->event_type));
      return;
    break;
  }

  info_prm.CpIndex = cpinfo_i->cp_idx;
  info_prm.Handle  = cpinfo_i->handle;
  info_prm.InfoType = (SERV_CP_INFO_TYPE)LE_TO_CPU(pInfoRep->event_type);

  if(user_cbf){
    TRM_OUT01(&ti, GR_CHNL, LV_INFO, "befor call user SERV_CP_INFO_CBF function, event_type %d", LE_TO_CPU(pInfoRep->event_type));
    user_cbf(&info_prm);
    TRM_OUT00(&ti, GR_CHNL, LV_INFO, "after call user SERV_CP_INFO_CBF function");
  }
  else{
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "no user SERV_CP_INFO_CBF function registered, event_type %d", LE_TO_CPU(pInfoRep->event_type));
  }
}

#if 0
#define DPR_SEGMCHNL_STATE_NO_TRANSF      0x0
#define DPR_SEGMCHNL_STATE_TRANSF_STARTED 0x1

typedef struct {
  DPR_CHAR       used;
  DPR_DRV_HANDLE chnlhnd;
  DPR_CHAR       state;
  DPR_CHAR     * segm_msg;
  DPR_UINT32     segm_msg_len;
  DPR_CHAR     * segm_msg_curr;

} DPR_CHNL_SEGM_DATA;

DPR_CHNL_SEGM_DATA segmd = {0,0,0,0,0,0};

/*----------------------------------------------------------------------------
 PURPOSE  : builds information block sended from fw by segmentated data transfer
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
int DPR_DRV_READ_SEGM(DPR_DRV_HANDLE chnlhnd, DPR_CHAR *msg,
                        DPR_CHAR **segm_msg /* out */, DPR_UINT32 *segm_msg_len /* out */, DPR_CHAR *segm_id /* out */)
{
  DPR_MSG_HDR *msg_hdr = (DPR_MSG_HDR*)msg;
  DPR_CHNL_SEGM_DATA *chnl_data = &segmd; /* get_chnl_segm_data(chnlhnd); */

  *segm_msg = 0;
  *segm_msg_len = 0;

  if(msg_hdr->segm_id == DPR_MSG_HDR_SEGM_BEGINN)
  {
    *segm_id = DPR_MSG_HDR_SEGM_BEGINN;

    if(chnl_data->state != DPR_SEGMCHNL_STATE_NO_TRANSF){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "second start off segm. transfer %d", 0);
      return 1;
    }

    chnl_data->segm_msg_len = msg_hdr->segm_whole_len * 256;
    chnl_data->segm_msg = (DPR_CHAR*)malloc(chnl_data->segm_msg_len);
    if(!chnl_data->segm_msg){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "start of segm. transfer, can't allocate %d byte", chnl_data->segm_msg_len);
      return 2;
    }

    chnl_data->segm_msg_curr = chnl_data->segm_msg;

    memcpy(chnl_data->segm_msg_curr, msg, msg_hdr->userdatalength + sizeof(DPR_MSG_HDR));

    chnl_data->state = DPR_SEGMCHNL_STATE_TRANSF_STARTED;

    chnl_data->segm_msg_curr += msg_hdr->userdatalength + sizeof(DPR_MSG_HDR);

    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "start of segm. transfer, segm_msg_len256=%d userdatalen=%d",
                              chnl_data->segm_msg_len, msg_hdr->userdatalength);

  }
  else if(msg_hdr->segm_id == DPR_MSG_HDR_SEGM_END)
  {
    *segm_id = DPR_MSG_HDR_SEGM_END;

    if(chnl_data->state != DPR_SEGMCHNL_STATE_TRANSF_STARTED){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "end of segm. transfer, but transfer was not started %d", 0);
      return 3;
    }

    if(chnl_data->segm_msg == 0 || chnl_data->segm_msg_curr == 0 || chnl_data->segm_msg_len == 0){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "end of segm. transfer, internal data corrupted %d", chnl_data->segm_msg);
      return 4;
    }

    memcpy(chnl_data->segm_msg_curr, msg + sizeof(DPR_MSG_HDR), msg_hdr->userdatalength);

    *segm_msg = chnl_data->segm_msg;
    *segm_msg_len = (DPR_UINT32)(chnl_data->segm_msg_curr - chnl_data->segm_msg) + msg_hdr->userdatalength;

    chnl_data->state = DPR_SEGMCHNL_STATE_NO_TRANSF;
    chnl_data->segm_msg_curr = 0;
    chnl_data->segm_msg_len = 0;

    TRM_OUT01(&ti, GR_CHNL, LV_INFO, "end of segm. transfer, msg_len=%d",
                              *segm_msg_len);
    /*TRM_OUTD(&ti, GR_CHNL, LV_INFO, *segm_msg, *segm_msg_len);*/

  }
  else if(msg_hdr->segm_id == DPR_MSG_HDR_SEGM_PACKET)
  {
    *segm_id = DPR_MSG_HDR_SEGM_PACKET;

    if(chnl_data->state != DPR_SEGMCHNL_STATE_TRANSF_STARTED){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "continue of segm. transfer, but transfer was not started %d", 0);
      return 5;
    }

    if(chnl_data->segm_msg == 0 || chnl_data->segm_msg_curr == 0 || chnl_data->segm_msg_len == 0){
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "continue of segm. transfer, internal data corrupted %d", chnl_data->segm_msg);
      return 6;
    }

    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "continue of segm. transfer, curr_offset=%d userdatalen=%d",
                              chnl_data->segm_msg_curr - chnl_data->segm_msg, msg_hdr->userdatalength);

    memcpy(chnl_data->segm_msg_curr, msg + sizeof(DPR_MSG_HDR), msg_hdr->userdatalength);
    chnl_data->segm_msg_curr += msg_hdr->userdatalength;
  }
  else
  {
    *segm_id = 0; // undefined, normal, not segmented packet
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "normal, not segmented packet",
                              chnl_data->segm_msg_curr - chnl_data->segm_msg, msg_hdr->userdatalength);

  }

  return 0;
}

/*----------------------------------------------------------------------------
 PURPOSE  : worker thread to receive data from firmware
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procMgtChannelReadCpInfo(void *arg)
{
  cpinfo_instance *cpinfo_i = (cpinfo_instance *)arg;

  int ret;
  DPR_CHAR *segm_pool_ptr = 0;
  DPR_UINT32 segm_pool_len = 0;
  FWLDA_DPR_RQB *pRqb;
  DPR_CHAR segm_id;

  TRM_OUT00(&ti, GR_STATE, LV_FCTCLBF, "-> procMgtChannelReadCpInfo");

  /* loop till shutdown req */
  while(!cpinfo_i->stop_thread) {
    TRM_OUT01(&ti, GR_CHNL, LV_FCTCLBF, "procMgtChannelReadCpInfo: wait for new msg, length %d",
              cpinfo_i->user_pool_length);

    ret = DPR_DRV_READ(cpinfo_i->mgt_chnl, cpinfo_i->user_pool_ptr,
                       cpinfo_i->user_pool_length);

    if(ret < 0) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadCpInfo: DPR_DRV_READ ret %d", ret);
      continue;
    } else if(!ret && DPR_DRV_ERROR(cpinfo_i->mgt_chnl)) {
      TRM_OUT00(&ti, GR_CHNL, LV_FCTCLBF,
              "procMgtChannelReadCpInfo: DPR_DRV_READ ret 0, perform deinitialization");
      continue;
    } else if((unsigned long)ret > cpinfo_i->user_pool_length) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadCpInfo: buffer is too small, need %d bytes", ret);
      continue;
    }

    ret = DPR_DRV_READ_SEGM(cpinfo_i->mgt_chnl, cpinfo_i->user_pool_ptr, &segm_pool_ptr, &segm_pool_len, &segm_id);

    if(ret != 0)
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadCpInfo: DPR_DRV_READ_SEGM failed ret=%d", ret);

    if(segm_id == DPR_MSG_HDR_SEGM_BEGINN ||
        segm_id == DPR_MSG_HDR_SEGM_PACKET){
      // is segmentated data block, go to get next packet
      continue;
    } else if(segm_id == DPR_MSG_HDR_SEGM_END){
      // is the last segmentated block, deliver data to user
      pRqb = (FWLDA_DPR_RQB *)segm_pool_ptr;
    }
    else{
      // is the normal, not segmentated block
      pRqb = (FWLDA_DPR_RQB *)cpinfo_i->user_pool_ptr;
    }

    TRM_OUT01(&ti, GR_CHNL, LV_FCTCLBF, "procMgtChannelReadCpInfo: read %d bytes", ret);

    switch(LE_TO_CPU(pRqb->fwlda_rqb.opcode))
    {
    case FWLDA_OP_CPINFO_OPEN_CNF:
    case FWLDA_OP_CPINFO_CLOSE_CNF:
    case FWLDA_OP_CPINFO_REGISTR_EVENTTYPE_CNF:

      /* save synchron rqb confirmation temporar to process later */
      cpinfo_i->synch_rqb_cnf = *pRqb;

      DPR_SEM_POST(cpinfo_i->sem_cnf_received);
      break;

    case FWLDA_OP_CPINFO_NOTIFICATION:  /* firmware inform host about registered events */
      cpinfo_call_usr_cbf(pRqb);

      if(segm_pool_ptr != cpinfo_i->user_pool_ptr){
        // pRqb is the segmentated block, that must be freed
        segm_pool_len = 0;
        free(segm_pool_ptr);
        segm_pool_ptr = 0;
      }
      break;
    default:
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "procMgtChannelReadCpInfo: unknown opcode %d received", LE_TO_CPU(pRqb->fwlda_rqb.opcode));
      if(segm_pool_ptr != cpinfo_i->user_pool_ptr){
        // pRqb is the segmentated block, that must be freed
        segm_pool_len = 0;
        free(segm_pool_ptr);
        segm_pool_ptr = 0;
      }
      break;
    }
  }

  TRM_OUT00(&ti, GR_STATE, LV_FCTCLBF, "<- procMgtChannelReadCpInfo");

  DPR_THREAD_END();
}
#endif

/*----------------------------------------------------------------------------
 PURPOSE  : worker thread to receive data from firmware
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procMgtChannelReadCpInfo(void *arg)
{
  cpinfo_instance *cpinfo_i = (cpinfo_instance *)arg;
  int ret;

  TRM_OUT00(&ti, GR_STATE, LV_FCTCLBF, "-> procMgtChannelReadCpInfo");

  /* loop till shutdown req */
  while(!cpinfo_i->stop_thread) {
    TRM_OUT01(&ti, GR_CHNL, LV_FCTCLBF, "procMgtChannelReadCpInfo: wait for new msg, length %d",
              cpinfo_i->user_pool_length);

    ret = DPR_DRV_READ(cpinfo_i->mgt_chnl, cpinfo_i->user_pool_ptr,
                       cpinfo_i->user_pool_length);

    if(ret < 0) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadCpInfo: DPR_DRV_READ ret %d", ret);
      continue;
    } else if(!ret && DPR_DRV_ERROR(cpinfo_i->mgt_chnl)) {
      TRM_OUT00(&ti, GR_CHNL, LV_FCTCLBF,
              "procMgtChannelReadCpInfo: DPR_DRV_READ ret 0, perform deinitialization");
      continue;
    } else if((unsigned long)ret > cpinfo_i->user_pool_length) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadCpInfo: buffer is too small, need %d bytes", ret);
      continue;
    }

    TRM_OUT01(&ti, GR_CHNL, LV_FCTCLBF, "procMgtChannelReadCpInfo: read %d bytes", ret);

    FWLDA_DPR_RQB *pRqb = (FWLDA_DPR_RQB *)cpinfo_i->user_pool_ptr;

    switch(LE_TO_CPU(pRqb->fwlda_rqb.opcode))
    {
    case FWLDA_OP_CPINFO_OPEN_CNF:
    case FWLDA_OP_CPINFO_CLOSE_CNF:
    case FWLDA_OP_CPINFO_REGISTR_EVENTTYPE_CNF:

      /* save synchron rqb confirmation temporar to process later */
      cpinfo_i->synch_rqb_cnf = *pRqb;

      DPR_SEM_POST(cpinfo_i->sem_cnf_received);
      break;

    case FWLDA_OP_CPINFO_NOTIFICATION:  /* firmware inform host about registered events */
      cpinfo_call_usr_cbf(pRqb);
      break;
    default:
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "procMgtChannelReadCpInfo: unknown opcode %d received", LE_TO_CPU(pRqb->fwlda_rqb.opcode));
      break;
    }
  }

  TRM_OUT00(&ti, GR_STATE, LV_FCTCLBF, "<- procMgtChannelReadCpInfo");

  DPR_THREAD_END();
}


/*----------------------------------------------------------------------------
 PURPOSE  : synchronous firmware request and responce
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
DPR_UINT32 cpinfo_send_receive_rqb(cpinfo_instance *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE opcode,
                                     DPR_UINT32 add_data_len, DPR_CHAR * add_data)
{
  size_t ret;
  DPR_UINT32 rqbLen = sizeof(FWLDA_DPR_RQB) + add_data_len;
  FWLDA_DPR_RQB *pRqb = 0;
  int sem_ret;
  DPR_UINT32 fct_ret = PNIO_OK;

  if(rqbLen > channel->send_pool_length ) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "cpinfo_send_receive_rqb, rqbLen(%lu) > channel->send_pool_length (%lu)",
                                      rqbLen, channel->send_pool_length);
    return PNIO_ERR_NO_RESOURCE;
  }

  pRqb = (FWLDA_DPR_RQB*)DPR_VMALLOC(rqbLen);
  if(!pRqb) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "cpinfo_send_receive_rqb, alloc memory for FWLDA_DPR_RQB failed");
    return PNIO_ERR_NO_RESOURCE;
  }

  pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
  pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;         /* Fix: AP01394131 */
  pRqb->dpr_hdr.userid = CPU_TO_LE(channel->handle);
  pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

  pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(opcode);
  pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + add_data_len);

  if(add_data_len) {
    memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), add_data, add_data_len);
  }

  ret = DPR_DRV_WRITE(channel->mgt_chnl, pRqb, rqbLen);

  DPR_VFREE(pRqb);
  pRqb = 0;

  if(ret != rqbLen) {
    TRM_OUT03(&ti, GR_CHNL, LV_ERR,
              "Send failed, written %u bytes from %d, error '%s'",
              ret, rqbLen, DPR_STRERROR());
    return PNIO_ERR_NO_FW_COMMUNICATION;
  } else {
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "cpinfo_send_receive_rqb, sended rqb to fw opcode=%d, len=%d",
               opcode, rqbLen);
  }

  /* wait for confirmation from firmware */
  sem_ret = DPR_SEM_WAIT_TIME(channel->sem_cnf_received, tm_ms );

  if(sem_ret == DPR_SEM_RET_TIMEOUT){
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "cpinfo_send_receive_rqb, timeout after %lu msec", tm_ms);
    return PNIO_ERR_NO_FW_COMMUNICATION;
  }

  pRqb = &channel->synch_rqb_cnf;

  if( (pRqb->dpr_hdr.subsystem != FWLDA_SUBSYS_INDEX) || (LE_TO_CPU(pRqb->dpr_hdr.userid) != channel->handle) ) {  /* Fix: AP01394131 */
    TRM_OUT00(&ti, GR_CHNL, LV_ERR, "cpinfo_send_receive_rqb, wrong dpr channel identification");
    return PNIO_ERR_INTERNAL;
  }
  /* check is the rqb a confirmation */
  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) != (opcode + 1)) {
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "cpinfo_send_receive_rqb, wrong opcode %d (expected %d)", pRqb->fwlda_rqb.opcode, opcode + 1);
    return PNIO_ERR_WRONG_RQB_LEN;
  }

  if(LE_TO_CPU(pRqb->fwlda_rqb.response) != PNIO_OK){
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "cpinfo_send_receive_rqb, confirmation with error %d", LE_TO_CPU(pRqb->fwlda_rqb.response));
  }

  switch(LE_TO_CPU(pRqb->fwlda_rqb.response)){
    case SUBSYS_FWLDA_OK:
      fct_ret = PNIO_OK;
      break;
    case SUBSYS_FWLDA_ERR_RESOURCE:
      fct_ret = PNIO_ERR_NO_RESOURCE;
      break;
    case SUBSYS_FWLDA_ERR_WRONG_HND:
      fct_ret = PNIO_ERR_WRONG_HND;
      break;
    case SUBSYS_FWLDA_ERR_ALREADY_DONE:
      fct_ret = PNIO_ERR_ALREADY_DONE;
      break;
    case SUBSYS_FWLDA_ERR_OPEN_NOT_ALLOWED:
      fct_ret = PNIO_ERR_CONFIG_IN_UPDATE;
      break;
    case SUBSYS_FWLDA_ERR_ERROR_PARAM:
      fct_ret = PNIO_ERR_WRONG_RQB_LEN;
      break;
    case SUBSYS_FWLDA_ERR_SEQ:
    case SUBSYS_FWLDA_ERR_INTERNAL:  /* examine firmware trace to find error reason */
    default:
      fct_ret = PNIO_ERR_INTERNAL;
      break;
  }

  return fct_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : deinitalize firmware communication
------------------------------------------------------------------------------
 COMMENTS : stop worker thread and deinitialize the driver communication
----------------------------------------------------------------------------*/
PNIO_UINT32 cpinfo_close_instance(cpinfo_instance * cpinfo_i)
{
  PNIO_UINT32 fct_ret = PNIO_OK;
  struct t_register_app app;
  struct t_read_pool  usr_pool;

  if(!cpinfo_i){
    return PNIO_ERR_WRONG_HND;
  }

  fct_ret = cpinfo_send_receive_rqb(cpinfo_i, 5000, FWLDA_OP_CPINFO_CLOSE_RQB, 0, 0);
  if(fct_ret != PNIO_OK){
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "cpinfo_send_receive_rqb(CPINFO_CLOSE) failed, error 0x%x", fct_ret);
    //return fct_ret;
  }

  cpinfo_i->stop_thread = 1;

  if(cpinfo_i->mgt_chnl_bounded){
    usr_pool.user_id = cpinfo_i->user_id;
    if(DPR_DRV_IOCTL(cpinfo_i->mgt_chnl, CP16XX_IOC_UNBIND,
                     &usr_pool, sizeof(usr_pool), 0) < 0) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl unbind failed, error '%s'", DPR_STRERROR());
      fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
    }
    else{
      TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_deinit_mgt_communication CP16XX_IOC_UNBIND ok");
    }
  }

  if(cpinfo_i->th_reader) {
    TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "Stop thread, handle %x", cpinfo_i->th_reader);

    DPR_THREAD_JOIN(cpinfo_i->th_reader);
    cpinfo_i->th_reader = 0;
  }

  if(cpinfo_i->user_pool_ptr) DPR_VFREE(cpinfo_i->user_pool_ptr);
  if(cpinfo_i->sem_cnf_received_initialized) DPR_SEM_DESTROY(cpinfo_i->sem_cnf_received);
  if(cpinfo_i->mgt_chnl) DPR_DRV_CLOSE(cpinfo_i->mgt_chnl);
  if(cpinfo_i->user_id){

    TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "ioctl unregister application, user_id 0x%x",
        cpinfo_i->user_id);
    app.user_id = cpinfo_i->user_id;

    if(DPR_DRV_IOCTL(cpinfo_i->con_fd, CP16XX_IOC_CAPP,
        &app, sizeof(app), 0) < 0) {
         TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl unregister application failed, error '%s'",
            DPR_STRERROR());
    }
  }

  if(cpinfo_i->con_fd) DPR_DRV_CLOSE(cpinfo_i->con_fd);
  if(cpinfo_i) cpinfo_free_instance(cpinfo_i);

  return fct_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : initalize firmware communication
------------------------------------------------------------------------------
 COMMENTS : initialize the driver communication and start worker thread
----------------------------------------------------------------------------*/
PNIO_UINT32 cpinfo_open(PNIO_UINT32 cp_idx, PNIO_UINT32 * handle, PNIO_UINT32 reserved)
{
  PNIO_UINT32 fct_ret = PNIO_OK;
  char fd_path[64];
  struct t_register_app app;
  struct t_read_pool  usr_pool;
  cpinfo_instance * cpinfo_i = 0;

  cpinfo_i = cpinfo_new_instance();
  if(!cpinfo_i){
    fct_ret = PNIO_ERR_MAX_REACHED;
    goto cpinfo_open_cleanup;
  }

  if(cp_idx < 1) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "first cp index is 1");
    fct_ret = PNIO_ERR_PRM_CP_ID;
    goto cpinfo_open_cleanup;
  }

  cpinfo_i->cp_idx = cp_idx;
  snprintf(fd_path, sizeof(fd_path)-1, DPR_CONTROL_INTERFACE, (unsigned int)(cpinfo_i->cp_idx - 1));

  if(!(cpinfo_i->con_fd = DPR_DRV_OPEN(fd_path))) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "open <%s> failed", fd_path);
    fct_ret = PNIO_ERR_PRM_CP_ID;
    goto cpinfo_open_cleanup;
  }

  app.flags = OAPP_APP_NORMAL;

  /*  Register an application to the driver. */
  if(DPR_DRV_IOCTL(cpinfo_i->con_fd, CP16XX_IOC_OAPP, &app, sizeof(app), sizeof(app))) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl register application failed, error '%s'",
              strerror(errno));

    if(EMFILE == errno)
      fct_ret = PNIO_ERR_MAX_REACHED;
    else
      fct_ret = PNIO_ERR_CREATE_INSTANCE;

    goto cpinfo_open_cleanup;
  }
  else{
    cpinfo_i->user_id = app.user_id;
  }

  /* connect to DPR-MGT Channel */
  TRM_OUT02(&ti, GR_INIT, LV_INFO, "initiate dpram mgt channel communication cp_idx=%d user_id=%d", cpinfo_i->cp_idx, cpinfo_i->user_id);

  snprintf(fd_path, sizeof(fd_path)-1, DPR_MGT_INTERFACE, (unsigned int)(cpinfo_i->cp_idx - 1));
  cpinfo_i->mgt_chnl = DPR_DRV_OPEN(fd_path);
  if(!cpinfo_i->mgt_chnl) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "opening '%s' failed", fd_path);
    fct_ret = PNIO_ERR_CREATE_INSTANCE;
    goto cpinfo_open_cleanup;
  }

  if(DPR_SEM_CREATE(cpinfo_i->sem_cnf_received)) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR,
            "DPR_SEM_CREATE(cpinfo_i->sem_cnf_received) failed");
    fct_ret = PNIO_ERR_INTERNAL;
    goto cpinfo_open_cleanup;
  }

  cpinfo_i->sem_cnf_received_initialized = true;

  usr_pool.user_id = cpinfo_i->user_id;
  if(DPR_DRV_IOCTL(cpinfo_i->mgt_chnl, CP16XX_IOC_BIND,
                   &usr_pool, sizeof(usr_pool), sizeof(usr_pool)) < 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ioctl bind '%s' failed, error '%s'",
              fd_path, DPR_STRERROR());
    fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
    goto cpinfo_open_cleanup;
  }
  else{
    cpinfo_i->mgt_chnl_bounded = true;
  }

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, "ioctl bind successfuly, pool read len=%lu pool write len=%lu",
                                     usr_pool.read_length, usr_pool.write_length);

  if(usr_pool.write_length == 0 ||
      usr_pool.read_length == 0){
    TRM_OUT00(&ti, GR_INIT, LV_WARN, "ioctl bind successfuly, but dpr-pools still not initialized");
    fct_ret = PNIO_ERR_NO_FW_COMMUNICATION;
    goto cpinfo_open_cleanup;
  }

  cpinfo_i->send_pool_length = usr_pool.write_length;
  cpinfo_i->user_pool_length = usr_pool.read_length;
  cpinfo_i->user_pool_ptr = (DPR_CHAR*)DPR_VMALLOC(cpinfo_i->user_pool_length);
  if(!cpinfo_i->user_pool_ptr) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "alloc memory pool for '%s' failed", fd_path);
    fct_ret = PNIO_ERR_OS_RES;
    cpinfo_i->user_pool_length = 0;
    goto cpinfo_open_cleanup;
  }

  cpinfo_i->stop_thread = 0;
  /* thread for processing the callback messages */
  if(!DPR_THREAD_CREATE(&cpinfo_i->th_reader, "", procMgtChannelReadCpInfo, cpinfo_i)) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "Create thread for read from '%s' failed", fd_path);
    fct_ret = PNIO_ERR_CREATE_INSTANCE;
    goto cpinfo_open_cleanup;
  }

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, "Create thread for '%s' ok, handle %x",
            fd_path, cpinfo_i->th_reader);

  FWLDA_CPINFO_OPEN open_prm;

  memset(&open_prm, 0, sizeof(open_prm));
  open_prm.blk_len = CPU_TO_LE(sizeof(open_prm));
  open_prm.host_version = CPU_TO_LE(PNIO_API_VERSION);

  fct_ret = cpinfo_send_receive_rqb(cpinfo_i, 5000, FWLDA_OP_CPINFO_OPEN_RQB, sizeof(open_prm), (PNIO_UINT8*)&open_prm);
  if(fct_ret != PNIO_OK){
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "cpinfo_send_receive_rqb(CPINFO_OPEN) failed, error 0x%x", fct_ret);
    /* fct_ret is already seted by cpinfo_send_receive_rqb */
    goto cpinfo_open_cleanup;
  }

  /* create applicatin handle */
  *handle = cpinfo_i->handle;

cpinfo_open_cleanup:

  if(fct_ret != PNIO_OK && cpinfo_i != 0){
    (void)cpinfo_close_instance(cpinfo_i);
  }

  return fct_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : user close funtion
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
PNIO_UINT32 cpinfo_close(PNIO_UINT32 handle)
{
  PNIO_UINT32 fct_ret = PNIO_OK;

  cpinfo_instance * cpinfo_i = 0;

  cpinfo_i = cpinfo_get_instance(handle);
  if(!cpinfo_i){
    fct_ret = PNIO_ERR_WRONG_HND;
    goto cpinfo_close;
  }

  fct_ret = cpinfo_close_instance(cpinfo_i);

cpinfo_close:
  return fct_ret;
}

/*----------------------------------------------------------------------------
 PURPOSE  : user function to register event callbacks
------------------------------------------------------------------------------
 COMMENTS :
----------------------------------------------------------------------------*/
PNIO_UINT32 cpinfo_reg_cbf(PNIO_UINT32 handle, SERV_CP_INFO_TYPE info_type, SERV_CP_INFO_CBF cbf,
                                           PNIO_UINT32 reserved)
{
  PNIO_UINT32 fct_ret = PNIO_OK;
  FWLDA_CPINFO_REGISTER reg_event;

  cpinfo_instance * cpinfo_i = 0;

  cpinfo_i = cpinfo_get_instance(handle);
  if(!cpinfo_i){
    fct_ret = PNIO_ERR_WRONG_HND;
    goto cpinfo_regcbf_cleanup;
  }

  memset(&reg_event, 0, sizeof(reg_event));
  reg_event.blk_len = CPU_TO_LE(sizeof(reg_event));
  reg_event.event_type = CPU_TO_LE(info_type);
  reg_event.reg_unreg = (cbf == 0) ? CPU_TO_LE(0) : CPU_TO_LE(1);

  switch(info_type){
  case SERV_CP_INFO_STOP_REQ:
    cpinfo_i->user_event_cbf[SERV_CP_INFO_STOP_REQ] = cbf;
    break;
  case SERV_CP_INFO_PDEV_DATA:
    cpinfo_i->user_event_cbf[SERV_CP_INFO_PDEV_DATA] = cbf;
    break;
  case SERV_CP_INFO_LED_STATE:
    cpinfo_i->user_event_cbf[SERV_CP_INFO_LED_STATE] = cbf;
    break;
  default:
    fct_ret = PNIO_ERR_PRM_TYPE;
    goto cpinfo_regcbf_cleanup;
    break;
  }

  fct_ret = cpinfo_send_receive_rqb(cpinfo_i, 5000, FWLDA_OP_CPINFO_REGISTR_EVENTTYPE_RQB,
                                      sizeof(reg_event), (DPR_CHAR*)&reg_event);
  if(fct_ret != PNIO_OK){
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "cpinfo_send_receive_rqb(CPINFO_REGISTR_EVENTTYPE) failed, error 0x%x", fct_ret);

    /* fct_ret is already seted by cpinfo_send_receive_rqb */

    switch(info_type){
    case SERV_CP_INFO_STOP_REQ:
      cpinfo_i->user_event_cbf[SERV_CP_INFO_STOP_REQ] = 0;
      break;
    case SERV_CP_INFO_PDEV_DATA:
      cpinfo_i->user_event_cbf[SERV_CP_INFO_PDEV_DATA] = 0;
      break;
    case SERV_CP_INFO_LED_STATE:
      cpinfo_i->user_event_cbf[SERV_CP_INFO_LED_STATE] = 0;
      break;
    default:
      break;
    }

    goto cpinfo_regcbf_cleanup;
  }


cpinfo_regcbf_cleanup:

  return fct_ret;
}


