/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : iodu_dev.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : device specific iodata update
*****************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UBERLASSEN WIRD, KOENNEN DIE AUTOREN  */
/* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*/
/* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN */
/* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND */
/* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   */
/* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  */
/* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   */
/*                                                                           */
/* This Software is Freeware. It is distributed for free. You may copy,      */
/* modify and use it for free as well as distribute it to others. You may    */
/* only distribute it as a whole, unmodified and with all trademarks and     */
/* copyright notices. You may not distribute it commercially (e.g. as a      */
/* Share-/Freeware-Distributor) without the express written permission of    */
/* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  */
/* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND */
/* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          */
/* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         */
/* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               */
/* If you happen to find any faults in it please tell us.                    */
/*                                                                           */
/*****************************************************************************/

#include <stdlib.h>     /* malloc */
#include <string.h>     /* memset */
#include <stdio.h>      /* printf */

#include "pnioerrx.h"
#include "pniobase.h"
#include "iodataupdate.h"
#include "iodu_com.h"

#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE
#if IODU_ERTEC_TYPE == IODU_ERTEC400
#include "lsa_vers.h"

#if SYS_LSA_VERSION_NUM >= 4010000
#include "lsa_cfg.h"
#include "edd_inc.h"
#include "eddi_cns_e400.h"
#define CnsBlockRead EDDI_CnsBlockRead
#define CnsBlockFreeRead EDDI_CnsBlockFreeRead
#define CnsBlockWrite EDDI_CnsBlockWrite
#define CnsBlockFreeWrite EDDI_CnsBlockFreeWrite
#else
#include "cns_e400.h"
#endif  /* SYS_LSA_VERSION_NUM >= 4010000 */

#endif  /* IODU_ERTEC_TYPE == IODU_ERTEC400 */

#if IODU_ERTEC_TYPE == IODU_ERTEC200
 #include "edd_inc.h"
#endif
#if IODU_ERTEC_TYPE == IODU_SOFTEDD
 #include "pnioDevInit.h"
#endif

#if IODU_ERTEC_TYPE == IODU_ERTEC400


/**************************************************************************
* F u n c t i o n:  IODU_dev_write_subslot400
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_write_subslot400(IODU_Item *pIODUItem,
    KRAMIOTLB_Item *item, IODU_LOCK_TYPE eddl_type)
{
    PNIO_IOXS IProviderstat = PNIO_S_BAD;
    PNIO_IOXS IConsumerstat = PNIO_S_BAD;
    PNIO_DEV_ADDR lAdrInfo;
    PNIO_UINT32       Handle;
    IODATA_adr_info  *cp_info;
    PNIO_UINT8       *ErtecReg;
    PNIO_UINT8       *IODataBase;
    PNIO_CBF_DATA_WRITE WriteDataCbf;

    Handle = pIODUItem->UsrHandle;
    cp_info = &pIODUItem->adr_info;
    ErtecReg = cp_info->EREG_base;
    WriteDataCbf = pIODUItem->dataWrite;

    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC && cp_info->DMA_base)
      IODataBase = cp_info->DMA_base;
    else
      IODataBase = cp_info->KRAM_base;

    lAdrInfo.AddrType = PNIO_ADDR_GEO;
    lAdrInfo.IODataType = PNIO_IO_IN;
    lAdrInfo.u.Geo.Slot = SWAP_D(item->slot_nr);
    lAdrInfo.u.Geo.Subslot = SWAP_D(item->subslot_nr);


    if(eddl_type == WITH_LOCK) {
        /* Lock the data first */
        /* Wait for semaphore, locking functions do not support multi entry */
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif
        /* Get consumer status , but lock it first */
        CnsBlockRead(ErtecReg, SWAP_D(item->iocs_offset),
            SWAP_D(item->iocs_length), CNS_MAX_POLL_TIMEOUT);
    }

    IConsumerstat = (PNIO_IOXS)IODU_GET_IOXS(IODataBase + SWAP_D(item->iocs_offset),
        SWAP_D(item->iocs_length));
    /* get transfer and data status value from Frame APDU status*/
    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC)
      if (IConsumerstat==PNIO_S_GOOD) {
        IConsumerstat = IODU_GET_CONSUMER_QUALITY(
           IODataBase + SWAP_D(item->consumer_frame_apdu_stat_offset));
      }

    if(eddl_type == WITH_LOCK) {
        CnsBlockFreeRead(ErtecReg, CNS_MAX_POLL_TIMEOUT);

        /* Pointer to data area = IO_Kram_Base + data_offset */
        CnsBlockWrite(ErtecReg, SWAP_D(item->data_offset),
            SWAP_D(item->data_length) + SWAP_D(item->iops_length), CNS_MAX_POLL_TIMEOUT);
    }

    /* Call user call back */
    IProviderstat = WriteDataCbf(Handle, &lAdrInfo, SWAP_D(item->data_length),
        IODataBase + SWAP_D(item->data_offset), IConsumerstat);

    /* Write the provider status */
    IODU_SET_IOXS(IODataBase + SWAP_D(item->data_offset) + SWAP_D(item->data_length),
        SWAP_D(item->iops_length), IProviderstat, PNIO_IOXS_DETECT_BY_DEVICE);

    /* printf("write_subslot: slot %u sslot %u p_stat %x stat %x\n",
            SWAP_D(item->slot_nr), SWAP_D(item->subslot_nr), IODataBase + SWAP_D(item->data_offset) + SWAP_D(item->data_length),
             ((unsigned char*)(IODataBase + SWAP_D(item->data_offset) + SWAP_D(item->data_length)))[0]); */
    if(eddl_type == WITH_LOCK) {
        /* UnLock the data */
        CnsBlockFreeWrite(ErtecReg, CNS_MAX_POLL_TIMEOUT);
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
    }


    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_dev_read_subslot400
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_read_subslot400(IODU_Item *pIODUItem,
    KRAMIOTLB_Item *item, IODU_LOCK_TYPE eddl_type)
{
    PNIO_IOXS IProviderstat = PNIO_S_BAD;
    PNIO_IOXS IConsumerstat = PNIO_S_BAD;
    PNIO_DEV_ADDR    lAdrInfo;
    PNIO_UINT32      Handle;
    IODATA_adr_info *cp_info;
    PNIO_UINT8      *ErtecReg;
    PNIO_UINT8      *IODataBase;
    PNIO_CBF_DATA_READ ReadDataCbf;

    Handle = pIODUItem->UsrHandle;
    cp_info = &pIODUItem->adr_info;
    ErtecReg = cp_info->EREG_base;
    ReadDataCbf = pIODUItem->dataRead;

    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC && cp_info->DMA_base)
      IODataBase = cp_info->DMA_base;
    else
      IODataBase = cp_info->KRAM_base;

    lAdrInfo.AddrType = PNIO_ADDR_GEO;
    lAdrInfo.IODataType = PNIO_IO_OUT;
    lAdrInfo.u.Geo.Slot = SWAP_D(item->slot_nr);
    lAdrInfo.u.Geo.Subslot = SWAP_D(item->subslot_nr);


    if(eddl_type == WITH_LOCK) {
        /* Lock the data first */
        /* Wait for semaphore, locking functions do not support multi entry */
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

        /* Pointer to data area = IO_Kram_Base + data_offset */
        CnsBlockRead(ErtecReg, SWAP_D(item->data_offset),
            SWAP_D(item->data_length), CNS_MAX_POLL_TIMEOUT);
    }

    /* Get remote provider status */
    IProviderstat = (PNIO_IOXS) IODU_GET_IOXS(IODataBase +
        SWAP_D(item->data_offset) + SWAP_D(item->data_length),
        SWAP_D(item->iops_length));
    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC)
      if (IProviderstat == PNIO_S_GOOD) {
        IProviderstat = IODU_GET_CONSUMER_QUALITY(
          IODataBase + SWAP_D(item->consumer_frame_apdu_stat_offset));
      }

    /* Call user call back */
    IConsumerstat = ReadDataCbf(Handle, &lAdrInfo, SWAP_D(item->data_length),
        IODataBase + SWAP_D(item->data_offset), IProviderstat);

    if(eddl_type == WITH_LOCK) {
        /* UnLock the data */
        CnsBlockFreeRead(ErtecReg, CNS_MAX_POLL_TIMEOUT);

        /* Set the consumer status , but we need to lock consumer status area */
        CnsBlockWrite(ErtecReg, SWAP_D(item->iocs_offset), SWAP_D(item->iocs_length),
            CNS_MAX_POLL_TIMEOUT);
    }

    IODU_SET_IOXS(IODataBase + SWAP_D(item->iocs_offset),
        SWAP_D(item->iocs_length), IConsumerstat, PNIO_IOXS_DETECT_BY_DEVICE);

    if(eddl_type == WITH_LOCK) {
      /* Unlock write */
      CnsBlockFreeWrite(ErtecReg, CNS_MAX_POLL_TIMEOUT);
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
    }


    return PNIO_OK;
}
#endif /* IODU_ERTEC_TYPE == IODU_ERTEC400 */


/**************************************************************************
* F u n c t i o n:  IODU_dev_read_subslotSOFT
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
#if IODU_ERTEC_TYPE == IODU_SOFTEDD

PNIO_UINT32 IODU_dev_unlock_provider_consumerSOFTEDD(IODU_Item *pIODUItem)
{
  PNIO_UINT32 buf_res;
  PNIO_UINT32 ret = PNIO_OK;
  PNIO_UINT8  is_unlock_necessary = (  pIODUItem->lockedData != 0 || pIODUItem->lockedCS != 0) ? 1 : 0;

  ASSERT(pnioDevInitParams->devAppBufferUnlock != NULL);

  /* unlock, only if previosly locked */
  if(pIODUItem->lockedData)
  {
    buf_res = pnioDevInitParams->devAppBufferUnlock(pIODUItem->DeviceProviderID);
    if(buf_res != PNIO_OK){
      /* printf("edd_ProviderBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
      ret = PNIO_ERR_NO_CONNECTION;
    }
  }

  /* unlock, only if previosly locked */
  if(pIODUItem->lockedCS)
  {
    buf_res = pnioDevInitParams->devAppBufferUnlock(pIODUItem->DeviceConsumerID);
    if(buf_res != PNIO_OK){
      /* printf("edd_ConsumerBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
       ret = PNIO_ERR_NO_CONNECTION;
    }
  }

  if(is_unlock_necessary == 1){
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
  }

  return ret;
}

PNIO_UINT32 IODU_dev_lock_provider_consumerSOFTEDD(IODU_Item *pIODUItem)
{
  PNIO_UINT32 buf_res;
  PNIO_UINT32 ret = PNIO_OK;

  ASSERT(pnioDevInitParams->devAppBufferLock != NULL);

  /* check, is the lock necessary ? */
  if(pIODUItem->asynINDevCount || pIODUItem->asynOUTDevCount)
  {
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

    pIODUItem->lockedData = 0;
    pIODUItem->lockedCS = 0;

    buf_res = pnioDevInitParams->devAppBufferLock(pIODUItem->DeviceProviderID, &pIODUItem->lockedData);
    if(buf_res != PNIO_OK){
      /* printf("pnioDevInitParams->devAppBufferLock() ret=0x%x\n", buf_res); */

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

      return PNIO_ERR_NO_CONNECTION;
    }

    buf_res = pnioDevInitParams->devAppBufferLock(pIODUItem->DeviceConsumerID, &pIODUItem->lockedCS);
    if(buf_res != PNIO_OK){
      /* printf("pnioDevInitParams->devAppBufferLock() ret=0x%x\n", buf_res); */
      /* unlock Provider */

      (void)pnioDevInitParams->devAppBufferUnlock(pIODUItem->DeviceProviderID);

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

      return PNIO_ERR_NO_CONNECTION;
    }

  }

  return ret;
}

#endif /* IODU_ERTEC_TYPE == IODU_SOFTEDD */

/**************************************************************************
* F u n c t i o n:  IODU_dev_read_subslot200
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
#if IODU_ERTEC_TYPE == IODU_ERTEC200

PNIO_UINT32 IODU_dev_unlock_provider_consumer200(IODU_Item *pIODUItem)
{
  LSA_RESULT lsa_res;
  PNIO_UINT32 ret = PNIO_OK;
  PNIO_UINT8  is_unlock_necessary = ((pIODUItem->lockedData != NULL) || (pIODUItem->lockedCS != NULL)) ? 1 : 0;

  EDD_RT_PROVIDER_BUFFER_REQ_TYPE    EddProvRtBufReq;
  EDD_RT_CONSUMER_BUFFER_REQ_TYPE    EddConsRtBufReq;

  /* unlock, only if previosly locked */
  if(pIODUItem->lockedData)
  {
    EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_UNLOCK;
    EddProvRtBufReq.hDDB = (EDD_HANDLE)pIODUItem->hDDB;
    EddProvRtBufReq.ProviderID = pIODUItem->DeviceProviderID;
    EddProvRtBufReq.pBuffer = pIODUItem->lockedData;

    lsa_res = edd_ProviderBufferRequest(&EddProvRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ProviderBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
      ret = PNIO_ERR_NO_CONNECTION;
    }
  }

  /* unlock, only if previosly locked */
  if(pIODUItem->lockedCS)
  {
    EddConsRtBufReq.Service = EDD_SRV_SRT_CONSUMER_UNLOCK;
    EddConsRtBufReq.hDDB = (EDD_HANDLE)pIODUItem->hDDB;
    EddConsRtBufReq.ConsumerID = pIODUItem->DeviceConsumerID;
    EddConsRtBufReq.pBuffer = pIODUItem->lockedCS;

    lsa_res = edd_ConsumerBufferRequest(&EddConsRtBufReq);
    if(lsa_res != LSA_OK){
       /* printf("edd_ConsumerBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
       ret = PNIO_ERR_NO_CONNECTION;
    }
  }

  if(is_unlock_necessary == 1){
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
  }

  return ret;
}

PNIO_UINT32 IODU_dev_lock_provider_consumer200(IODU_Item *pIODUItem)
{
  LSA_RESULT lsa_res;
  PNIO_UINT32 ret = PNIO_OK;
  EDD_RT_PROVIDER_BUFFER_REQ_TYPE    EddProvRtBufReq;
  EDD_RT_CONSUMER_BUFFER_REQ_TYPE    EddConsRtBufReq;

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
  DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
  DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

  pIODUItem->lockedData = 0;
  pIODUItem->lockedCS = 0;

  /* check, is the lock necessary ? */
  if(pIODUItem->asynINDevCount || pIODUItem->asynOUTDevCount)
  {
    EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_LOCK_CURRENT;

    EddProvRtBufReq.hDDB = (EDD_HANDLE)pIODUItem->hDDB;
    EddProvRtBufReq.ProviderID = pIODUItem->DeviceProviderID;

    lsa_res = edd_ProviderBufferRequest(&EddProvRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ProviderBufferRequest(LOCK_CURRENT) ret=0x%x\n", lsa_res); */

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
      DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
      DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

      return PNIO_ERR_NO_CONNECTION;
    }

    pIODUItem->lockedData = EddProvRtBufReq.pBuffer;
    ASSERT(pIODUItem->lockedData);

    EddConsRtBufReq.Service = EDD_SRV_SRT_CONSUMER_LOCK;

    EddConsRtBufReq.hDDB = (EDD_HANDLE)pIODUItem->hDDB;
    EddConsRtBufReq.ConsumerID = pIODUItem->DeviceConsumerID;

    lsa_res = edd_ConsumerBufferRequest(&EddConsRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ConsumerBufferRequest(LOCK) ret=0x%x\n", lsa_res); */

      /* unlock Provider */
      EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_UNLOCK;

      EddProvRtBufReq.hDDB = (EDD_HANDLE)pIODUItem->hDDB;
      EddProvRtBufReq.ProviderID = pIODUItem->DeviceProviderID;
      EddProvRtBufReq.pBuffer = pIODUItem->lockedData;

      edd_ProviderBufferRequest(&EddProvRtBufReq);

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
      DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
      DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

      return PNIO_ERR_NO_CONNECTION;
    }

    pIODUItem->lockedCS = EddConsRtBufReq.pBuffer;
    ASSERT(pIODUItem->lockedCS);
    return ret;
  }

  /* no provider or consumer locked so unlock mutex also */
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
  DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
  DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

  return ret;
}

#endif /* IODU_ERTEC_TYPE == IODU_ERTEC200 */


#if (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD)
/**************************************************************************
* F u n c t i o n:  IODU_dev_read_subslot3BUF
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/

PNIO_UINT32 IODU_dev_read_subslot3BUF(IODU_Item *pIODUItem,
    KRAMIOTLB_Item *item, IODU_LOCK_TYPE eddl_type)
{
    PNIO_IOXS IProviderstat = PNIO_S_BAD;
    PNIO_IOXS IConsumerstat = PNIO_S_BAD;
    PNIO_DEV_ADDR    lAdrInfo;
    PNIO_UINT32      Handle;
    PNIO_CBF_DATA_READ ReadDataCbf;
    PNIO_UINT8      *IODataBase, *IOCsBase;

    Handle = pIODUItem->UsrHandle;
    ReadDataCbf = pIODUItem->dataRead;

    lAdrInfo.AddrType = PNIO_ADDR_GEO;
    lAdrInfo.IODataType = PNIO_IO_OUT;
    lAdrInfo.u.Geo.Slot = SWAP_D(item->slot_nr);
    lAdrInfo.u.Geo.Subslot = SWAP_D(item->subslot_nr);

    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUItem->adr_info.KRAM_base){
      IODataBase = IOCsBase = pIODUItem->adr_info.KRAM_base;
    }
    else {
      IODataBase = pIODUItem->lockedCS;
      IOCsBase   = pIODUItem->lockedData;
    }

    /* Get remote provider status */
    IProviderstat = (PNIO_IOXS) IODU_GET_IOXS(
       IODataBase + SWAP_D(item->data_offset) + SWAP_D(item->data_length),
                                               SWAP_D(item->iops_length));
    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC &&
                                 IProviderstat == PNIO_S_GOOD) {
        IProviderstat = IODU_GET_CONSUMER_QUALITY(
          IODataBase + SWAP_D(item->consumer_frame_apdu_stat_offset));
    }

    /* Call user call back */
    IConsumerstat = ReadDataCbf(Handle, &lAdrInfo, SWAP_D(item->data_length),
                                 IODataBase + SWAP_D(item->data_offset), IProviderstat);

    IODU_SET_IOXS(IOCsBase + SWAP_D(item->iocs_offset), SWAP_D(item->iocs_length),
                  IConsumerstat, PNIO_IOXS_DETECT_BY_DEVICE);

    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_dev_write_subslot3BUF
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_write_subslot3BUF(IODU_Item *pIODUItem,
    KRAMIOTLB_Item *item, IODU_LOCK_TYPE eddl_type)
{
    PNIO_IOXS IProviderstat = PNIO_S_BAD;
    PNIO_IOXS IConsumerstat = PNIO_S_BAD;
    PNIO_DEV_ADDR lAdrInfo;
    PNIO_UINT32       Handle;
    PNIO_UINT8      *IODataBase, *IOCsBase;


    PNIO_CBF_DATA_WRITE WriteDataCbf;

    Handle = pIODUItem->UsrHandle;
    WriteDataCbf = pIODUItem->dataWrite;

    lAdrInfo.AddrType = PNIO_ADDR_GEO;
    lAdrInfo.IODataType = PNIO_IO_IN;
    lAdrInfo.u.Geo.Slot = SWAP_D(item->slot_nr);
    lAdrInfo.u.Geo.Subslot = SWAP_D(item->subslot_nr);

    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUItem->adr_info.KRAM_base){
      IODataBase = IOCsBase = pIODUItem->adr_info.KRAM_base;
    }
    else {
      IODataBase = pIODUItem->lockedData;
      IOCsBase   = pIODUItem->lockedCS;
    }

    IConsumerstat = (PNIO_IOXS)IODU_GET_IOXS(IOCsBase +
                                             SWAP_D(item->iocs_offset), SWAP_D(item->iocs_length));

    /* get transfer and data status value from Frame APDU status*/
    if(SWAP_D(item->io_sync_type) == KRAMIOTLB_IO_SYNC){
      if (IConsumerstat==PNIO_S_GOOD) {
        IConsumerstat = IODU_GET_CONSUMER_QUALITY(
           IOCsBase + SWAP_D(item->consumer_frame_apdu_stat_offset));
      }
    }

    /* Call user call back */
    IProviderstat = WriteDataCbf(Handle, &lAdrInfo, SWAP_D(item->data_length),
        IODataBase + SWAP_D(item->data_offset), IConsumerstat);

    /* Write the provider status */
    IODU_SET_IOXS(IODataBase + SWAP_D(item->data_offset) + SWAP_D(item->data_length),
        SWAP_D(item->iops_length), IProviderstat, PNIO_IOXS_DETECT_BY_DEVICE);

    return PNIO_OK;
}

#endif /* (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD) */

/**************************************************************************
* F u n c t i o n:  IODU_dev_update_ss_tlb
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*                inout_t
*                sync_t
*                tlb
*                tlb_item_cnt
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_update_ss_tlb(IODU_Item *pIODUItem,
    KRAMIOTLB_IN_OUT_TYPE inout_t, KRAMIOTLB_IOSYNC_TYPE sync_t,
    KRAMIOTLB_Item     ** tlb, PNIO_UINT32 *tlb_item_cnt,
    KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, optional, default 0 */
    PNIO_UINT32           addition_cmp_list_len,
    KRAMIOTLB_IsItemValid p_fct_is_item_valid) /* in, optional, default 0 */
{
    KRAMIOTLB_Ret ret;
    PNIO_UINT32 tmpCount = 0;

    /* Get the number of available subslots with sync_t, inout_t */
    /*ret = KRAMIOTLB_GetDeviceItems(pIODUItem->Handle, sync_t, inout_t, &tmpCount, NULL,
            p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid,
        &pIODUItem->krtlbHeader);
    if (KRAMIOTLB_OK != ret) {
        return PNIO_ERR_INTERNAL;
    }

    if (tmpCount == 0)
        return PNIO_OK;*/

    /* Allocate memory */
    /**tlb = (KRAMIOTLB_Item **)malloc(sizeof (KRAMIOTLB_Item *) * (tmpCount));
    if(*tlb == NULL)
        return PNIO_ERR_NO_RESOURCE;

    memset(*tlb, 0, sizeof (KRAMIOTLB_Item *) * (tmpCount));*/

    tmpCount = IODU_MAX_SUBSLOT_CNT;

    /* get device subslots with sync_t, inout_t */
    ret = KRAMIOTLB_GetDeviceItems(pIODUItem->Handle, sync_t, inout_t, &tmpCount, tlb,
            p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid,
        &pIODUItem->krtlbHeader);
    if (KRAMIOTLB_OK != ret) {
        free(*tlb);
        return PNIO_ERR_INTERNAL;
    }

    if(tmpCount > IODU_MAX_SUBSLOT_CNT)
        return PNIO_ERR_NO_RESOURCE;

    *tlb_item_cnt = tmpCount;
    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_dev_open
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                cpid - in, CP id
*                IodHandle - in, handle from KRAMTLB
*                UsrHandle - in, user callback funktions will be call with this handle
*                dataRead_cbf - in, user callback function pointer for data read
*                dataWrite_cbf - in, user callback function pointer for data write
*                initStatus_cbf - in, user callback function pointer for get init status for subslots without io data
*                OpFault_cbf - in, user callback function pointer for OpFault
*                nFlags - in, DMA or KRAM access, ASYNC or SYNC
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_open(
    PNIO_UINT32 cpid,                  /* in, cp index, FW = 1 */
    PNIO_UINT32 IodHandle,             /* in, handle, which was used by KRAMIOTLB_AddItem */
    PNIO_UINT32 UsrHandle,             /* in, handel, with them IODATAUPDATE calls dataRead_cbf, .. ; FW = IodHandle*/
    PNIO_CBF_DATA_READ  dataRead_cbf,  /* in, cbf, to deliever input data to user */
    PNIO_CBF_DATA_WRITE dataWrite_cbf, /* in, cbf, to pick up output data from user */
	IODU_CBF_INIT_IOXS  initStatus_cbf, /* in, cbf, to get initial status for subslots without io data */
    void *reserved2,                   /* reserved */
    PNIO_UINT32 nFlags,                /* in, nFlags = ExtPar from PNIO_controller/device_open */
    IODATA_adr_info *adr_info,         /* in, see iodataupdate.h */
    IODU_Item **ppIODUItem)            /* out, pointer to pointer to IO-Handle,            */
                                       /*      that will be used by all IODU_... functions */
{
    /* reserved parameters */
    reserved2 = 0;

    /* check input callback function pointers. */
    if((!dataRead_cbf) || (!dataWrite_cbf))
        return PNIO_ERR_PRM_CALLBACK;

    *ppIODUItem = IODU_GetItem(cpid);
    if(!(*ppIODUItem))
        return PNIO_ERR_MAX_REACHED;

    /* makeshift - revome wenn IOD use
    #warning call IODU_set_io_validity, makeshift - revome when IOD use IODU_set_io_validity
    IODU_set_io_validity(*ppIODUItem, ITEM_VALID);*/

    KRAMIOTLB_init(adr_info->IOTLB_base, adr_info->iotlb_len,
        &(*ppIODUItem)->krtlbHeader);

    (*ppIODUItem)->Handle = IodHandle;
    (*ppIODUItem)->UsrHandle = UsrHandle;
    (*ppIODUItem)->initStatPDEV = 0; /* PDEVs are not initialized */

    /* (*ppIODUItem)->itemtyp = TYP_DEV; */
    (*ppIODUItem)->dataRead = dataRead_cbf;
    (*ppIODUItem)->dataWrite = dataWrite_cbf;
	(*ppIODUItem)->getInitPDEVStatus = initStatus_cbf;

    /* to do :- may be we need to check this param
       (ie DMA or KRAM base address is initialised ) */
    (*ppIODUItem)->nFlags = nFlags;

    (*ppIODUItem)->adr_info = *adr_info;

#if IODU_ERTEC_TYPE == IODU_ERTEC200
    /*ASSERT(SYS_TSK_GetDDB(&((*ppIODUItem)->hDDB)) == 0);*/
    /* use nic_id = 0, only first card (nic_num = 0) supported */
    (*ppIODUItem)->hDDB = (PNIO_UINT32)SysEDDCall_get_device_handle(0);
#endif

    return PNIO_OK;
}

static void IODU_set_local_status(IODU_Item *pIODUItem, KRAMIOTLB_Item *io_item, PNIO_IOXS loc_stat )
{
  PNIO_UINT32 IOStatLen, IOStatOffset;
  PNIO_UINT8 * IOStatBase = NULL;

  if(io_item->io_out_type & KRAMIOTLB_IO_IN){
    /* für daten die man liest mit PNIO_initialize_read */
    /* local status is CONSUMER status */
    IOStatOffset = SWAP_D(io_item->iocs_offset);
    IOStatLen = SWAP_D(io_item->iocs_length);
  }
  else{
    /* für daten die man schreibt mit PNIO_initialize_write */
    /* local status is PROVIDER status */
    IOStatOffset = SWAP_D(io_item->data_offset) + SWAP_D(io_item->data_length);
    IOStatLen = SWAP_D(io_item->iops_length);
  }

#if IODU_ERTEC_TYPE == IODU_ERTEC400
  if(SWAP_D(io_item->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUItem->adr_info.DMA_base != 0)
    IOStatBase = pIODUItem->adr_info.DMA_base;
  else
    IOStatBase = pIODUItem->adr_info.KRAM_base;
#else
#if IODU_ERTEC_TYPE == IODU_ERTEC200
  if(SWAP_D(io_item->io_sync_type) == KRAMIOTLB_IO_SYNC)
   IOStatBase = pIODUItem->adr_info.KRAM_base;
  else{

   if(io_item->io_out_type & KRAMIOTLB_IO_IN){
     /* for device input data */
     IOStatBase = pIODUItem->lockedData;
   }
   else{
     /* for device output data */
     IOStatBase = pIODUItem->lockedCS;
   }
  }
#else
  #if IODU_ERTEC_TYPE == IODU_SOFTEDD
   if(io_item->io_out_type & KRAMIOTLB_IO_IN){
     /* for device input data */
     IOStatBase = pIODUItem->lockedData;
   }
   else{
     /* for device output data */
     IOStatBase = pIODUItem->lockedCS;
   }
  #else
   #error "set IODU_ERTEC_TYPE to IODU_ERTEC400 or IODU_ERTEC200 or IODU_SOFTEDD"
  #endif /* IODU_ERTEC_TYPE == IODU_SOFTEDD */
#endif /* IODU_ERTEC_TYPE == IODU_ERTEC200 */
#endif /* IODU_ERTEC_TYPE == IODU_ERTEC400 */


  IODU_SET_IOXS(IOStatBase + IOStatOffset, IOStatLen,
                 loc_stat, PNIO_IOXS_DETECT_BY_DEVICE);

}

static int IODU_initialize_pdevs(IODU_Item *pIODUItem)
{
  KRAMIOTLB_Ret   ret;
  KRAMIOTLB_Item* PDEVArr[32]; /* slot 0, pdev1, pdev2, pdev3, pdev4 */
  PNIO_UINT32     PDEVCnt = 32;
  PNIO_UINT32     IOPSOffset;
  PNIO_IOXS       initIOXS = PNIO_S_GOOD;
  PNIO_DEV_ADDR   subsAddr; 
  char     *pIOPS = NULL;
  unsigned long   i;
  int             fct_ret = 0;

  /* get device pdev subslots */
  ret = KRAMIOTLB_GetDeviceItems(pIODUItem->Handle, KRAMIOTLB_IO_UNDEFSYNC,
                                  KRAMIOTLB_IO_PDEV, &PDEVCnt, PDEVArr, 0, 0, 0,
                                   &pIODUItem->krtlbHeader);
  ASSERT(KRAMIOTLB_OK == ret);
  ASSERT(PDEVCnt <= 32);
  for(i=0; i<PDEVCnt; i++)
  {
    IOPSOffset = SWAP_D(PDEVArr[i]->data_offset) + SWAP_D(PDEVArr[i]->data_length);

#if IODU_ERTEC_TYPE == IODU_ERTEC400
    if(SWAP_D(PDEVArr[i]->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUItem->adr_info.DMA_base != 0)
      pIOPS = (char *)pIODUItem->adr_info.DMA_base + IOPSOffset;
    else
      pIOPS = (char *)pIODUItem->adr_info.KRAM_base + IOPSOffset;
#else
 #if IODU_ERTEC_TYPE == IODU_ERTEC200
    if(SWAP_D(PDEVArr[i]->io_sync_type) == KRAMIOTLB_IO_SYNC)
     pIOPS = (char *)pIODUItem->adr_info.KRAM_base + IOPSOffset;
    else
     pIOPS = (char *)pIODUItem->lockedData + IOPSOffset;
 #else
    #if IODU_ERTEC_TYPE == IODU_SOFTEDD
     pIOPS = (char *)pIODUItem->lockedData + IOPSOffset;
    #else
     #error "set IODU_ERTEC_TYPE to IODU_ERTEC400 or IODU_ERTEC200 or IODU_SOFTEDD"
    #endif /* IODU_ERTEC_TYPE == IODU_SOFTEDD */
 #endif /* IODU_ERTEC_TYPE == IODU_ERTEC200 */
#endif /* IODU_ERTEC_TYPE == IODU_ERTEC400 */

    if(pIODUItem->getInitPDEVStatus)
	{
	  /* if getInitPDEVStatus registered, use it to get initial PDEV status */ 
	  subsAddr.AddrType = PNIO_ADDR_GEO;
	  subsAddr.u.Geo.Slot = SWAP_D(PDEVArr[i]->slot_nr);
	  subsAddr.u.Geo.Subslot =  SWAP_D(PDEVArr[i]->subslot_nr);
      initIOXS = pIODUItem->getInitPDEVStatus(pIODUItem->Handle, &subsAddr);
	}

    IODU_SET_IOXS(pIOPS, SWAP_D(PDEVArr[i]->iops_length),
                   initIOXS, PNIO_IOXS_DETECT_BY_DEVICE);

    /*printf("set PDEV stat slot=%u sslot=%u p_stat=%x stat=%x\n",
      SWAP_D(PDEVArr[i]->slot_nr), SWAP_D(PDEVArr[i]->subslot_nr), pIOPS, ((unsigned char*)pIOPS)[0]);*/
    /* pdevs realy initialized */
    fct_ret = 1;
  }

  return fct_ret;
}

#if IODU_ERTEC_TYPE == IODU_ERTEC400
  #define IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(iodu_hnd) PNIO_OK
  #define IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(iodu_hnd) PNIO_OK
  #define IODU_DEV_READ_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
            IODU_dev_read_subslot400(iodu_hnd, kram_item, eddl_type)
  #define IODU_DEV_WRITE_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
            IODU_dev_write_subslot400(iodu_hnd, kram_item, eddl_type)
#else
 #if IODU_ERTEC_TYPE == IODU_ERTEC200
   #define IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(iodu_hnd) \
             IODU_dev_lock_provider_consumer200(iodu_hnd)
   #define IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(iodu_hnd) \
             IODU_dev_unlock_provider_consumer200(iodu_hnd)
   #define IODU_DEV_READ_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
             IODU_dev_read_subslot3BUF(iodu_hnd, kram_item, eddl_type)
   #define IODU_DEV_WRITE_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
             IODU_dev_write_subslot3BUF(iodu_hnd, kram_item, eddl_type)
 #else
    #if IODU_ERTEC_TYPE == IODU_SOFTEDD
      #define IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(iodu_hnd) \
                IODU_dev_lock_provider_consumerSOFTEDD(iodu_hnd)
      #define IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(iodu_hnd)  \
                IODU_dev_unlock_provider_consumerSOFTEDD(iodu_hnd)
      #define IODU_DEV_READ_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
             IODU_dev_read_subslot3BUF(iodu_hnd, kram_item, eddl_type)
      #define IODU_DEV_WRITE_SUBSLOT(iodu_hnd, kram_item, eddl_type) \
             IODU_dev_write_subslot3BUF(iodu_hnd, kram_item, eddl_type)
    #else
      #error "set IODU_ERTEC_TYPE to IODU_ERTEC400 or IODU_ERTEC200 or IODU_SOFTEDD"
    #endif /* IODU_ERTEC_TYPE == IODU_SOFTEDD */
 #endif /* IODU_ERTEC_TYPE == IODU_ERTEC200 */
#endif /* IODU_ERTEC_TYPE == IODU_ERTEC400 */


/**************************************************************************
* F u n c t i o n:  IODU_intiate_data_read
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_intiate_data_read(IODU_Item *pIODUItem)
{
  return IODU_initiate_data_read_ext(pIODUItem, 0, PNIO_ACCESS_RT_WITH_LOCK);
}

/**************************************************************************
* F u n c t i o n:  PNIO_initiate_data_read_ext
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*                pAddr  - in, geographical address
*                AccessType - in, access and data type
*
* Return Value:
*
***************************************************************************/

PNIO_UINT32  IODU_initiate_data_read_ext(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType)
{
    KRAMIOTLB_Item   *pIOTLB_Item;
    PNIO_UINT32       ret, i, submodRtNumber = 0, submodIrtNumber = 0;
    char              preIrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE,
                      postIrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;

    IODU_LOCK_TYPE eddlock_t;

    if(pAddr != NULL && pAddr->AddrType != PNIO_ADDR_GEO)
      return PNIO_ERR_PRM_ADD;

    if( AccessType != PNIO_ACCESS_ALL_WITHOUT_LOCK &&
         AccessType != PNIO_ACCESS_RT_WITH_LOCK &&
          AccessType != PNIO_ACCESS_RT_WITHOUT_LOCK &&
           AccessType != PNIO_ACCESS_IRT_WITHOUT_LOCK)
      return PNIO_ERR_PRM_ACCESS_TYPE;

#if (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD)
    /* by ERTEC200, SOFTEDD is only locked access to RT data possible, because three-change-buffer-memory*/
    if( AccessType == PNIO_ACCESS_RT_WITHOUT_LOCK)
      return PNIO_ERR_PRM_ACCESS_TYPE;
#endif /* (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD) */

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    /*if (pIODUItem->io_validity == ITEM_INVALID)
      return PNIO_ERR_SEQUENCE;*/

    ret = IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
    if(ret != PNIO_OK){
      return ret;
    }

    if(!pIODUItem->initStatPDEV){
      if(IODU_initialize_pdevs(pIODUItem)){
        pIODUItem->initStatPDEV = 1; /* PDEVs are initialized */
      }
    }

    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
         AccessType == PNIO_ACCESS_RT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_IRT_WITH_LOCK
       )
        eddlock_t = WITH_LOCK;
    else
        eddlock_t = WITHOUT_LOCK;

    /* Lock the data first */
    /* Wait for semaphore, locking functions do not support multi entry */

    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
        AccessType == PNIO_ACCESS_ALL_WITHOUT_LOCK ||
         AccessType == PNIO_ACCESS_RT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_RT_WITHOUT_LOCK) {
        /* For all subslots call user data read call back function. */
        for (i = 0; i < pIODUItem->asynINDevCount; i++) {
            pIOTLB_Item = pIODUItem->asynINDevLst[i];

            if(KRAMTLB_GET_IO_VALIDITY(pIOTLB_Item) == KRAMIOTLB_NOT_VALID){
              /* subslot is not valid, because pulled or not plugged */
              continue;
            }

            if(pAddr == NULL ||
               (pAddr->u.Geo.Subslot == 0 && SWAP_D(pIOTLB_Item->slot_nr) == pAddr->u.Geo.Slot) ||
                 (pAddr->u.Geo.Subslot == SWAP_D(pIOTLB_Item->subslot_nr) &&
                    pAddr->u.Geo.Slot == SWAP_D(pIOTLB_Item->slot_nr))) {
              IODU_DEV_READ_SUBSLOT(pIODUItem, pIOTLB_Item, eddlock_t);
              submodRtNumber++;
            }
        }
    }

    /* check: IrtAccessWindow (after OpStart but before OpDone/OpFault) */

    if(pIODUItem->adr_info.IrtAccessStatus)
      preIrtAccessStatus = *pIODUItem->adr_info.IrtAccessStatus;

    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
        AccessType == PNIO_ACCESS_ALL_WITHOUT_LOCK ||
         AccessType == PNIO_ACCESS_IRT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_IRT_WITHOUT_LOCK) {
        for (i = 0; i < pIODUItem->synINDevCount; i++) {
          pIOTLB_Item = pIODUItem->synINDevLst[i];

          if(KRAMTLB_GET_IO_VALIDITY(pIOTLB_Item) == KRAMIOTLB_NOT_VALID){
            /* subslot is not valid, because pulled or not plugged */
            continue;
          }

          if(pAddr == NULL ||
             (pAddr->u.Geo.Subslot == 0 && SWAP_D(pIOTLB_Item->slot_nr) == pAddr->u.Geo.Slot) ||
               (pAddr->u.Geo.Subslot == SWAP_D(pIOTLB_Item->subslot_nr) &&
                  pAddr->u.Geo.Slot == SWAP_D(pIOTLB_Item->slot_nr))) {
            IODU_DEV_READ_SUBSLOT(pIODUItem, pIOTLB_Item, eddlock_t);
            submodIrtNumber++;
          }
        }
    }

    if(pIODUItem->adr_info.IrtAccessStatus)
      postIrtAccessStatus = *pIODUItem->adr_info.IrtAccessStatus;

    ret = IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
    if(ret != PNIO_OK){
      return ret;
    }

    if(submodRtNumber + submodIrtNumber) {
      /* if at least one submodule updated */

      if(pIODUItem->adr_info.IrtAccessStatus && submodIrtNumber){
        /* if IrtAccessStatus available */
        /* check: was Irt access consistency ? (after OpStart but before OpDone/OpFault) */
        if(preIrtAccessStatus == IRT_ACCESS_STATUS_OUTSIDE ||
            postIrtAccessStatus == IRT_ACCESS_STATUS_OUTSIDE)
          return PNIO_WARN_IRT_INCONSISTENT;
        else
          return PNIO_OK;
      }
      else
        return PNIO_OK;
    }
    else
      return PNIO_WARN_NO_SUBMODULES;
}


/**************************************************************************
* F u n c t i o n:  IODU_intiate_data_write
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_intiate_data_write(IODU_Item *pIODUItem)
{
  return IODU_initiate_data_write_ext(pIODUItem, 0, PNIO_ACCESS_RT_WITH_LOCK);
}

/**************************************************************************
* F u n c t i o n:  PNIO_initiate_data_write_ext
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*                pAddr  - in, geographical address
*                AccessType - in, access and data type
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32  IODU_initiate_data_write_ext(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType)
{
    KRAMIOTLB_Item   *pIOTLB_Item;
    PNIO_UINT32       ret, i, submodRtNumber = 0, submodIrtNumber = 0;
    char              preIrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE,
                      postIrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;

    IODU_LOCK_TYPE eddlock_t;

    if(pAddr != NULL && pAddr->AddrType != PNIO_ADDR_GEO)
      return PNIO_ERR_PRM_ADD;

    if( AccessType != PNIO_ACCESS_ALL_WITHOUT_LOCK &&
         AccessType != PNIO_ACCESS_RT_WITH_LOCK &&
          AccessType != PNIO_ACCESS_RT_WITHOUT_LOCK &&
           AccessType != PNIO_ACCESS_IRT_WITHOUT_LOCK)
      return PNIO_ERR_PRM_ACCESS_TYPE;

#if (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD)
    /* by ERTEC200, SOFTEDD is only locked access to RT data possible, because three-change-buffer-memory*/
    if( AccessType == PNIO_ACCESS_RT_WITHOUT_LOCK)
      return PNIO_ERR_PRM_ACCESS_TYPE;
#endif /* (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD) */

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    /*if (pIODUItem->io_validity == ITEM_INVALID)
      return PNIO_ERR_SEQUENCE;*/

    ret = IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
    if(ret != PNIO_OK){
      return ret;
    }

    if(!pIODUItem->initStatPDEV){
      if(IODU_initialize_pdevs(pIODUItem)){
        pIODUItem->initStatPDEV = 1; /* PDEVs are initialized */
      }
    }

    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
         AccessType == PNIO_ACCESS_RT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_IRT_WITH_LOCK
       )
        eddlock_t = WITH_LOCK;
    else
        eddlock_t = WITHOUT_LOCK;

    /* Lock the data first */
    /* Wait for semaphore, locking functions do not support multi entry */
    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
        AccessType == PNIO_ACCESS_ALL_WITHOUT_LOCK ||
         AccessType == PNIO_ACCESS_RT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_RT_WITHOUT_LOCK) {
        /* For all subslots call user data read call back function. */
        for (i = 0; i < pIODUItem->asynOUTDevCount; i++) {
            pIOTLB_Item = pIODUItem->asynOUTDevLst[i];

            if(KRAMTLB_GET_IO_VALIDITY(pIOTLB_Item) == KRAMIOTLB_NOT_VALID){
              /* subslot is not valid, because pulled or not plugged */
              continue;
            }

            if(pAddr == NULL ||
               (pAddr->u.Geo.Subslot == 0 && SWAP_D(pIOTLB_Item->slot_nr) == pAddr->u.Geo.Slot) ||
                 (pAddr->u.Geo.Subslot == SWAP_D(pIOTLB_Item->subslot_nr) &&
                    pAddr->u.Geo.Slot == SWAP_D(pIOTLB_Item->slot_nr))) {
              IODU_DEV_WRITE_SUBSLOT(pIODUItem, pIOTLB_Item, eddlock_t);
              submodRtNumber++;
            }
        }
    }

    /* check: IrtAccessWindow (after OpStart but before OpDone/OpFault) */

    if(pIODUItem->adr_info.IrtAccessStatus)
      preIrtAccessStatus = *pIODUItem->adr_info.IrtAccessStatus;

    if(AccessType == PNIO_ACCESS_ALL_WITH_LOCK ||
        AccessType == PNIO_ACCESS_ALL_WITHOUT_LOCK ||
         AccessType == PNIO_ACCESS_IRT_WITH_LOCK ||
          AccessType == PNIO_ACCESS_IRT_WITHOUT_LOCK) {
        for (i = 0; i < pIODUItem->synOUTDevCount; i++) {
          pIOTLB_Item = pIODUItem->synOUTDevLst[i];

          if(KRAMTLB_GET_IO_VALIDITY(pIOTLB_Item) == KRAMIOTLB_NOT_VALID){
            /* subslot is not valid, because pulled or not plugged */
            continue;
          }

          if(pAddr == NULL ||
             (pAddr->u.Geo.Subslot == 0 && SWAP_D(pIOTLB_Item->slot_nr) == pAddr->u.Geo.Slot) ||
               (pAddr->u.Geo.Subslot == SWAP_D(pIOTLB_Item->subslot_nr) &&
                  pAddr->u.Geo.Slot == SWAP_D(pIOTLB_Item->slot_nr))) {
            IODU_DEV_WRITE_SUBSLOT(pIODUItem, pIOTLB_Item, eddlock_t);
            submodIrtNumber++;
          }
        }
    }

    if(pIODUItem->adr_info.IrtAccessStatus)
      postIrtAccessStatus = *pIODUItem->adr_info.IrtAccessStatus;

    ret = IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
    if(ret != PNIO_OK){
      return ret;
    }

    if(submodRtNumber + submodIrtNumber) {
      /* if at least one submodule updated */
      if(pIODUItem->adr_info.IrtAccessStatus && submodIrtNumber){
        /* if IrtAccessStatus available */
        /* check: was Irt access consistency ? (after OpStart but before OpDone/OpFault) */
        if(preIrtAccessStatus == IRT_ACCESS_STATUS_OUTSIDE ||
            postIrtAccessStatus == IRT_ACCESS_STATUS_OUTSIDE)
          return PNIO_WARN_IRT_INCONSISTENT;
        else
          return PNIO_OK;
      }
      else
        return PNIO_OK;
    }
    else
      return PNIO_WARN_NO_SUBMODULES;
}

/**************************************************************************
* F u n c t i o n:  IODU_dev_close
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_dev_close(IODU_Item *pIODUItem)
{
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

  pIODUItem->asynINDevCount = 0;
  pIODUItem->asynOUTDevCount = 0;
  pIODUItem->synOUTDevCount = 0;
  pIODUItem->synINDevCount = 0;

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

  KRAMIOTLB_deinit(&pIODUItem->krtlbHeader);

  /*if (pIODUItem->asynINDevLst != NULL) {
      free(pIODUItem->asynINDevLst);
      pIODUItem->asynINDevLst = NULL;
  }

  if (pIODUItem->asynOUTDevLst != NULL) {
      free(pIODUItem->asynOUTDevLst);
      pIODUItem->asynOUTDevLst = NULL;
  }

  if (pIODUItem->synOUTDevLst != NULL) {
      free(pIODUItem->synOUTDevLst);
      pIODUItem->synOUTDevLst = NULL;
  }

  if (pIODUItem->synINDevLst != NULL) {
      free(pIODUItem->synINDevLst);
      pIODUItem->synINDevLst = NULL;
  }*/

  IODU_SetItemEmpty(pIODUItem);

  return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_update_subslot_tbl_adv
*
* D e s c r i p t i o n :
*               update internal subslot table
*                       must be called by each ar_indata, ar_aboart
*
* A r g u m e n t s:
*                Handle - in, handle of the device
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_update_subslot_tbl_adv(IODU_Item *pIODUItem,
        KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, optional, default 0 */
        PNIO_UINT32           addition_cmp_list_len, /* in, optional, default 0 */
        KRAMIOTLB_IsItemValid p_fct_is_item_valid)
{
    PNIO_UINT32 ret = PNIO_OK;
    IODATA_adr_info *cp_info;

    cp_info = &pIODUItem->adr_info;

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

    /* update all ASYNC subslots */
    ret = IODU_dev_update_ss_tlb(pIODUItem, KRAMIOTLB_IO_IN,
        KRAMIOTLB_IO_ASYNC, pIODUItem->asynINDevLst, &pIODUItem->asynINDevCount,
         p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid);
    if(ret != PNIO_OK) goto return_error;

    ret = IODU_dev_update_ss_tlb(pIODUItem, KRAMIOTLB_IO_OUT,
        KRAMIOTLB_IO_ASYNC, pIODUItem->asynOUTDevLst, &pIODUItem->asynOUTDevCount,
         p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid);
    if(ret != PNIO_OK) goto return_error;

    /* update all SYNC subslots */
    ret = IODU_dev_update_ss_tlb(pIODUItem, KRAMIOTLB_IO_IN,
        KRAMIOTLB_IO_SYNC, pIODUItem->synINDevLst, &pIODUItem->synINDevCount,
         p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid);
    if(ret != PNIO_OK) goto return_error;

    ret = IODU_dev_update_ss_tlb(pIODUItem, KRAMIOTLB_IO_OUT,
        KRAMIOTLB_IO_SYNC, pIODUItem->synOUTDevLst, &pIODUItem->synOUTDevCount,
         p_addition_cmp_list, addition_cmp_list_len, p_fct_is_item_valid);
    if(ret != PNIO_OK) goto return_error;

    /* new connection come in, initialize PDEVs IOPS again */
    pIODUItem->initStatPDEV = 0;

#if IODU_ERTEC_TYPE == IODU_ERTEC200

    if(pIODUItem->asynINDevCount){
      pIODUItem->DeviceConsumerID = pIODUItem->asynINDevLst[0]->cs_buffer.index;
      pIODUItem->DeviceProviderID = pIODUItem->asynINDevLst[0]->data_buffer.index;
    }
    else if(pIODUItem->asynOUTDevCount){
      pIODUItem->DeviceConsumerID = pIODUItem->asynOUTDevLst[0]->cs_buffer.index;
      pIODUItem->DeviceProviderID = pIODUItem->asynOUTDevLst[0]->data_buffer.index;
    }else{
      pIODUItem->DeviceConsumerID  = 0;
      pIODUItem->DeviceProviderID  = 0;
    }
#endif
#if IODU_ERTEC_TYPE == IODU_SOFTEDD

    if(pIODUItem->asynINDevCount){
      pIODUItem->DeviceConsumerID = pIODUItem->asynINDevLst[0]->cs_buffer.handle;
      pIODUItem->DeviceProviderID = pIODUItem->asynINDevLst[0]->data_buffer.handle;
    }
    else if(pIODUItem->asynOUTDevCount){
      pIODUItem->DeviceConsumerID = pIODUItem->asynOUTDevLst[0]->cs_buffer.handle;
      pIODUItem->DeviceProviderID = pIODUItem->asynOUTDevLst[0]->data_buffer.handle;
    }else{
      pIODUItem->DeviceConsumerID  = 0;
      pIODUItem->DeviceProviderID  = 0;
    }
#endif


    /* copy KRAMIOTLB_ITEMs from DPRAM memory to host memory, to improve performance by io access */
    if(pIODUItem->adr_info.host_mem_KRAMTLB){
      unsigned long hostKRamIdx = 0, i;

      KRAMIOTLB_Item *hmem_KRAM = pIODUItem->adr_info.host_mem_KRAMTLB;

      if(pIODUItem->adr_info.host_mem_KRAMTLB_max_cnt <
          (pIODUItem->asynINDevCount + pIODUItem->asynOUTDevCount +
           pIODUItem->synINDevCount + pIODUItem->synOUTDevCount) ){
        ASSERT("not enough memory to copy KRAMIOTLB_ITEMs from DPRAM memory to host" == 0);
      }

      for(i = 0; i < pIODUItem->asynINDevCount; i++, hostKRamIdx++){
       hmem_KRAM[hostKRamIdx] = *(pIODUItem->asynINDevLst[i]);
       pIODUItem->asynINDevLst[i] = &hmem_KRAM[hostKRamIdx];
      }
      for(i = 0; i < pIODUItem->asynOUTDevCount; i++, hostKRamIdx++){
       hmem_KRAM[hostKRamIdx] = *(pIODUItem->asynOUTDevLst[i]);
       pIODUItem->asynOUTDevLst[i] = &hmem_KRAM[hostKRamIdx];
      }
      for(i = 0; i < pIODUItem->synINDevCount; i++, hostKRamIdx++){
       hmem_KRAM[hostKRamIdx] = *(pIODUItem->synINDevLst[i]);
       pIODUItem->synINDevLst[i] = &hmem_KRAM[hostKRamIdx];
      }
      for(i = 0; i < pIODUItem->synOUTDevCount; i++, hostKRamIdx++){
       hmem_KRAM[hostKRamIdx] = *(pIODUItem->synOUTDevLst[i]);
       pIODUItem->synOUTDevLst[i] = &hmem_KRAM[hostKRamIdx];
      }
    }

return_error: /* don't forget unlock */

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif

    return ret;
}

PNIO_UINT32 IODU_update_subslot_tbl(IODU_Item *pIODUItem)
{
  return IODU_update_subslot_tbl_adv(pIODUItem, 0, 0, 0);
}

PNIO_UINT32 IODU_dev_read_subslot(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 BufLen,
    PNIO_UINT8 *pBuffer,
    PNIO_UINT32 *pDataLen,
    PNIO_IOXS  *pIOlocStateToRead,
    PNIO_IOXS  *pIOremStateToRead)
{
  KRAMIOTLB_Item **ss_arr, *pKRamItem = 0;
  PNIO_UINT32 ss_arr_len, copy_len, ret, i;
  PNIO_UINT8 * pData = (PNIO_UINT8 *)0x66666666;
  PNIO_UINT8 * pRemStat = (PNIO_UINT8 *)0x66666666;

  if(pAddr == NULL || pAddr->AddrType != PNIO_ADDR_GEO)
    return PNIO_ERR_PRM_ADD;
  if(pIOlocStateToRead == NULL || pIOremStateToRead == NULL)
    return PNIO_ERR_PRM_RSTATE;
  if(pBuffer == NULL)
    return PNIO_ERR_PRM_BUF;
  if(pDataLen == NULL)
    return PNIO_ERR_PRM_LEN;

  if(pAddr->IODataType == PNIO_IO_IN){
    ss_arr = pIODUItem->asynINDevLst;
    ss_arr_len = pIODUItem->asynINDevCount;
  }
  else{
    ss_arr = pIODUItem->asynOUTDevLst;
    ss_arr_len = pIODUItem->asynOUTDevCount;
  }

  /* find KRAMTLB_Item for subslot(pAddr), search first in asyn - Array */
  for(i=0; i < ss_arr_len; i++){
    if(ss_arr[i]->slot_nr == pAddr->u.Geo.Slot &&
        ss_arr[i]->subslot_nr == pAddr->u.Geo.Subslot)
    {
      pKRamItem = ss_arr[i];
      break;
    }
  }

  if(pKRamItem == 0) /* if not found, search in sync - Array*/
  {
    if(pAddr->IODataType == PNIO_IO_IN){
      ss_arr = pIODUItem->synINDevLst;
      ss_arr_len = pIODUItem->synINDevCount;
    }
    else{
      ss_arr = pIODUItem->synOUTDevLst;
      ss_arr_len = pIODUItem->synOUTDevCount;
    }

    for(i=0; i < ss_arr_len; i++){
      if(ss_arr[i]->slot_nr == pAddr->u.Geo.Slot &&
          ss_arr[i]->subslot_nr == pAddr->u.Geo.Subslot)
      {
        pKRamItem = ss_arr[i];
        break;
      }
    }
  }

  if(pKRamItem == 0)
  {
    return PNIO_ERR_UNKNOWN_ADDR;
  }

  ret = IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
  if(ret != PNIO_OK){
      return ret;
  }

  *pDataLen = SWAP_D(pKRamItem->data_length);
  copy_len = BufLen < *pDataLen ? BufLen : *pDataLen;

  #if (IODU_ERTEC_TYPE == IODU_ERTEC400)
    pData = pIODUItem->adr_info.KRAM_base + SWAP_D(pKRamItem->data_offset);
    pRemStat = pIODUItem->adr_info.KRAM_base + SWAP_D(pKRamItem->iocs_offset);
  #elif (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD)
    if(SWAP_D(pKRamItem->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUItem->adr_info.KRAM_base){
      pData = pIODUItem->adr_info.KRAM_base + SWAP_D(pKRamItem->data_offset);
      pRemStat = pIODUItem->adr_info.KRAM_base + SWAP_D(pKRamItem->iocs_offset);
    }
    else {
      if(pAddr->IODataType == PNIO_IO_OUT){
        pData   = pIODUItem->lockedData + SWAP_D(pKRamItem->data_offset);
        pRemStat = pIODUItem->lockedCS + SWAP_D(pKRamItem->iocs_offset);
      }
      else{
        /* it is twisted, because so initialised from IOD module ! */
        pData   = pIODUItem->lockedCS + SWAP_D(pKRamItem->data_offset);
        pRemStat = pIODUItem->lockedData + SWAP_D(pKRamItem->iocs_offset);
     }
    }
  #else
    return PNIO_ERR_INTERNAL; /* unknown platform */
  #endif

  /* Get remote provider status */
  if (SWAP_D(pKRamItem->io_out_type) & KRAMIOTLB_IO_OUT)
  {
    /* controller output or device input */
    /* Here we read back the locally written data */

    /* get local provider status */
    *pIOlocStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pKRamItem->data_length),
            SWAP_D(pKRamItem->iops_length));

    /* get the remote consumer status  */
    /* if( !(SWAP_D(pKRamItem->io_out_type) & KRAMIOTLB_IO_DDEX) ) no QuerVerkehr by PNIO-Devices */
    {
      *pIOremStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pRemStat, SWAP_D(pKRamItem->iocs_length));
    }
  }
  else /* controller input or device output */
  {
    /* get remote provider status */
     *pIOremStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pKRamItem->data_length),
            SWAP_D(pKRamItem->iops_length));

    /* get the local consumer status  */
    /* if( !(SWAP_D(pKRamItem->io_out_type) & KRAMIOTLB_IO_DDEX) ) no QuerVerkehr by PNIO-Devices */
    {
      *pIOlocStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pRemStat, SWAP_D(pKRamItem->iocs_length));
    }
  }

  /* read data */
  memcpy(pBuffer, pData, copy_len);

  ret = IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
  if(ret != PNIO_OK){
      return ret;
  }

  return PNIO_OK;
}

PNIO_UINT32 IODU_set_io_validity_data(IODU_Item *pIODUItem,
                                 PNIO_DEV_ADDR * pAddr,
                                 IODU_VALIDITY io_validity)
{
  PNIO_UINT32 ret = PNIO_OK;
  IODATA_adr_info *cp_info;
  KRAMIOTLB_Item **ss_arr, *pKRamItem = 0;
  PNIO_UINT32 ss_arr_len, i;
  KRAMIOTLB_VALIDITY_T kram_validity;

  cp_info = &pIODUItem->adr_info;

  kram_validity = io_validity == ITEM_VALID ? KRAMIOTLB_VALID : KRAMIOTLB_NOT_VALID;

  if(pAddr == NULL || pAddr->AddrType != PNIO_ADDR_GEO)
    return PNIO_ERR_PRM_ADD;

  if(pAddr->IODataType == PNIO_IO_IN){
    ss_arr = pIODUItem->asynINDevLst;
    ss_arr_len = pIODUItem->asynINDevCount;
  }
  else{
    ss_arr = pIODUItem->asynOUTDevLst;
    ss_arr_len = pIODUItem->asynOUTDevCount;
  }

  /* find KRAMTLB_Item for subslot(pAddr), search first in asyn - Array */
  for(i=0; i < ss_arr_len; i++){
    if(ss_arr[i]->slot_nr == pAddr->u.Geo.Slot &&
        ss_arr[i]->subslot_nr == pAddr->u.Geo.Subslot)
    {
      pKRamItem = ss_arr[i];
      break;
    }
  }

  if(pKRamItem == 0) /* if not found, search in sync - Array*/
  {
    if(pAddr->IODataType == PNIO_IO_IN){
      ss_arr = pIODUItem->synINDevLst;
      ss_arr_len = pIODUItem->synINDevCount;
    }
    else{
      ss_arr = pIODUItem->synOUTDevLst;
      ss_arr_len = pIODUItem->synOUTDevCount;
    }

    for(i=0; i < ss_arr_len; i++){
      if(ss_arr[i]->slot_nr == pAddr->u.Geo.Slot &&
          ss_arr[i]->subslot_nr == pAddr->u.Geo.Subslot)
      {
        pKRamItem = ss_arr[i];
        break;
      }
    }
  }

  if(pKRamItem == 0)
  {
    ret = PNIO_ERR_UNKNOWN_ADDR;
  }
  else
  {
#if IODU_ERTEC_TYPE == IODU_ERTEC400
  #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
  #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_LOCK(pIODUItem->MutexObj);
  #endif
#else
  ret = IODU_DEV_LOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
  if(ret != PNIO_OK){
    return ret;
  }
#endif
    KRAMTLB_SET_IO_VALIDITY(pKRamItem, kram_validity);

    if( io_validity == ITEM_INVALID){

      PNIO_IOXS IOlocStateToSet = PNIO_S_BAD;

      IODU_set_local_status(pIODUItem, pKRamItem, IOlocStateToSet);
    }

#if IODU_ERTEC_TYPE == IODU_ERTEC400
  #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
  #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
  #endif
#else
  ret = IODU_DEV_UNLOCK_PROVIDER_CONSUMER_BUF(pIODUItem);
  if(ret != PNIO_OK){
    return ret;
  }
#endif
  }

  return ret;
}

/* will be called from IOD by PNIO_sub_pull/plug
   by PNIO_sub_pull io_validity = ITEM_INVALID
     IODU: set Consumer/Provider stutus to PNIO_BAD and
           remove subslot from subslot update list
           IO-Base user don't gets callback for this subslot by PNIO_initialize_read/write
   by PNIO_sub_pull io_validity = ITEM_VALID
     IODU: put subslot in subslot update list
          IO-Base user gets callback for this subslot by PNIO_initialize_read/write */

PNIO_UINT32 IODU_set_io_validity_flag(IODU_Item *pIODUItem,
                                 PNIO_DEV_ADDR * pAddr,
                                 IODU_VALIDITY io_validity)
{
  PNIO_UINT32 ret = PNIO_OK;
  IODATA_adr_info *cp_info;
  KRAMIOTLB_Item * kram_io_item = 0;
  KRAMIOTLB_Ret kram_ret;
  KRAMIOTLB_VALIDITY_T new_validity, cur_validity;

  cp_info = &pIODUItem->adr_info;

  if(pAddr->IODataType != PNIO_IO_IN && pAddr->IODataType != PNIO_IO_OUT){
    IODU_TRC(("ERROR::IODU_set_io_validity_flag, slot:%i ss:%i io_type=%d(invalid value)\n",
               pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, pAddr->IODataType));
    return PNIO_ERR_PRM_IO_TYPE;
  }

  /* find subslot in KRAM-Tlb */
  kram_ret =  KRAMIOTLB_FindItemByGeoAddr(
          pIODUItem->Handle,       /* in, iobase instance */
          pAddr->u.Geo.Slot,
          pAddr->u.Geo.Subslot,/* in */
          (pAddr->IODataType == PNIO_IO_IN) ? KRAMIOTLB_IO_IN : KRAMIOTLB_IO_OUT,
          &kram_io_item,    /* out */
          &pIODUItem->krtlbHeader);
  if(KRAMIOTLB_OK != kram_ret){
    IODU_TRC(("IODU_set_io_validity_flag, KRAMIOTLB_FindItemByGeoAddr(hnd:%d slot:%i ss:%i io_type:%i val:%i) ret=0x%x\n",
              pIODUItem->Handle, pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, pAddr->IODataType, io_validity, kram_ret));
    return PNIO_ERR_UNKNOWN_ADDR;
  }

  cur_validity = KRAMTLB_GET_IO_VALIDITY(kram_io_item);

  new_validity = (io_validity == ITEM_VALID) ? KRAMIOTLB_VALID : KRAMIOTLB_NOT_VALID;

  if(cur_validity == new_validity)
  {
    IODU_TRC(("IODU_set_io_validity_flag, slot:%i ss:%i io_type:%i val:%i\n",
              pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, pAddr->IODataType, io_validity));
    IODU_TRC(("IODU_set_io_validity_flag, cur_validity == new_validity :%d(0-valid,1-invalid)\n", cur_validity));
    return PNIO_OK;
  }

  KRAMTLB_SET_IO_VALIDITY(kram_io_item,  new_validity);

#if (defined (CPFAM_1616x))
  /* inform Host to perform IODU_set_io_validity_data */
  {
    /* will be provided from PNIO-Device Agent */
    void PNIO_AD_send_set_validity_to_host(PNIO_UINT32 slot, PNIO_UINT32 subslot,
                                           PNIO_IO_TYPE type, PNIO_UINT32 validity);

    PNIO_AD_send_set_validity_to_host(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot,
                                     pAddr->IODataType, io_validity == ITEM_VALID ? 1 : 0);
  }
#else
  ret = IODU_set_io_validity_data(pIODUItem, pAddr, io_validity);
#endif
  return ret;
}

#endif /* KRAMIOTLB_SUPPORT_HOST_DEVICE */
