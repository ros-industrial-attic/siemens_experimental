/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : iodu_com.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : common iodata update
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

#include "iodataupdate.h"
#include "iodu_com.h"

#if IODU_ERTEC_TYPE == IODU_ERTEC400

#include "lsa_vers.h"

#if SYS_LSA_VERSION_NUM >= 4010000
#define CnsSetExcp EDDI_CnsSetExcp
#endif  /* SYS_LSA_VERSION_NUM >= 4010000 */

#include "cns_e400.h"


/* our exception handler */
void CNS_FCT_ATTR IODU_ErtecFatalError( const CNS_UINT32 Line,
    CNS_UINT8* const sFile, const CNS_UINT32 ModuleID,
    CNS_UINT8* const sErr, const CNS_UINT32 Error,
    const CNS_UINT32 DW_0, const CNS_UINT32 DW_1)
{
  fprintf(stderr, "fatal error in %s, line %d, module %d, "
                  "error text %s, values 0x%x(%d), %d, %d\n",
    sFile, Line, ModuleID, sErr, Error, Error, DW_0, DW_1);

  ASSERT("IODU_ErtecFatalError" == 0);
}

#endif /* IODU_ERTEC_TYPE == IODU_ERTEC400 */

#if IODU_ERTEC_TYPE == IODU_ERTEC200
 #include "edd_inc.h"
#endif

/*****************************************************************************/
/* Global instances                                                          */
/*****************************************************************************/
#define MAX_IODU_APP_COUNT 8
IODU_Item gIDUItems[MAX_IODU_APP_COUNT];

/**************************************************************************
* F u n c t i o n:  IODU_Init
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
void IODU_Init(void)
{
#ifndef CP_300
#if IODU_ERTEC_TYPE == IODU_ERTEC400
    CnsSetExcp(IODU_ErtecFatalError);
#endif
#endif
    memset(gIDUItems, 0, sizeof(gIDUItems));
}

/**************************************************************************
* F u n c t i o n:  IODU_GetNextItemEmptyPlace
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
IODU_Item *IODU_GetItem(PNIO_UINT32 cpid)
{
    int nIndex;
    IODU_Item *pIODUItem;

    /* Find the requested handel. */
    for (nIndex = 0; nIndex < MAX_IODU_APP_COUNT; nIndex++) {
        if (gIDUItems[nIndex].item_valid == ITEM_INVALID) {
            pIODUItem = &gIDUItems[nIndex];
            pIODUItem->cpid = cpid;
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
            if(DPR_INTERPROCESS_MUTEX_CREATE_UNLOCKED(pIODUItem->MutexObj,
                pIODUItem->cpid - 1))
                return NULL;
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
            if(DPR_MUTEX_CREATE_UNLOCKED(pIODUItem->MutexObj))
                return NULL;
#endif
            pIODUItem->item_valid = ITEM_VALID;
#if 0
            /* check duplicate Handle */
            for (iIndex = 0; iIndex < MAX_IODU_APP_COUNT; iIndex++) {
                if (gIDUItems[iIndex].Handle == IodHandle) {
                    return PNIO_ERR_ALREADY_DONE;
                }
            }
#endif
            return pIODUItem;
        }
    }

    return NULL;
}

void IODU_SetItemEmpty(IODU_Item *pIODUItem)
{
    pIODUItem->item_valid = ITEM_INVALID;
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_DESTROY(pIODUItem->MutexObj,
        pIODUItem->cpid - 1);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_DESTROY(pIODUItem->MutexObj);
#endif
    pIODUItem->cpid = 0;
    pIODUItem->Handle = 0;
}

/*-----------------------------------------------------------------------------
 * Name  : IODU_kram_read
 * Descr : This function reads input or output data for PNIO-Device or
 *         PNIO-Controller KRAMIOTLB_Item.
 * IMPORTANT NOTE: Originaly this function was intended for using by io-router
 *         only. It read data without KRAM lock. Because of correction of
 *         AP00866265 "write packed modules via cache" this function is also
 *         used by IODU_ctrl_data_read_ex() to read "diagnosis addresses" not
 *         from cache but directly from the KRAM.
 * Return: ==0: PNIO_OK (always)
 */
PNIO_UINT32 IODU_kram_read(IODU_Item *pIODUItem, KRAMIOTLB_Item  *pItem, KRAMIOTLB_ITYPE item_type,
    PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen, PNIO_UINT8 *pBuffer,
    PNIO_IOXS  *pIOlocStateToRead,
    PNIO_IOXS  *pIOlocStateToSet, /* IO-lokale State will be set, if pIOlocStateToSet != 0 */
    PNIO_IOXS  *pIOremStateToRead)
{
    IODATA_adr_info *cp_info = &pIODUItem->adr_info;
    PNIO_UINT32      copy_len;
    unsigned char   *pData;

    *pDataLen = SWAP_D(pItem->data_length);
    copy_len = BufLen < *pDataLen ? BufLen : *pDataLen;

    pData = (unsigned char *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset));

    /* read data - Copy to user buffer */
    memcpy((void *)pBuffer, pData, copy_len);

    if (SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_OUT)
    {
      /* controller output or device input */
      /* Here we read back the locally written data */

      /* get local provider status */
      *pIOlocStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pItem->data_length),
              SWAP_D(pItem->iops_length));

      /* get the remote consumer status  */
      if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
      {
        *pIOremStateToRead = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
              SWAP_D(pItem->iocs_length));
      }
    }
    else /* controller input or device output */
    {
      /* get remote provider status */
       *pIOremStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pItem->data_length),
              SWAP_D(pItem->iops_length));

      /* get the local consumer status  */
      if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
      {
        *pIOlocStateToRead = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
              SWAP_D(pItem->iocs_length));
      }

      if(pIOlocStateToSet)
      {
        if (item_type == KRAMIOTLB_CONTR)
        {
            IODU_SET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
              SWAP_D(pItem->iocs_length), *pIOlocStateToSet, PNIO_IOXS_DETECT_BY_CONTROLLER);
        }
        else
        {
            IODU_SET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
              SWAP_D(pItem->iocs_length), *pIOlocStateToSet, PNIO_IOXS_DETECT_BY_DEVICE);
        }
      }
    }

    return PNIO_OK;
} /* end of IODU_kram_read */

/*-----------------------------------------------------------------------------
 * Name  : IODU_kram_write
 * Descr : This function writes data for PNIO-Device or PNIO-Controller KRAMIOTLB_Item
 *         PNIO-Device: Write input data. PNIO-Controller: Write output data.
 *         Used by io-router. It writes without KRAM lock !!!
 * Return: alwys PNIO_OK ==0
 */
PNIO_UINT32 IODU_kram_write(IODU_Item *pIODUItem, KRAMIOTLB_Item  *pItem, KRAMIOTLB_ITYPE item_type,
                             PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
                              PNIO_IOXS IOlocStateToSet, PNIO_IOXS * pIOremStateToRead)
{

    IODATA_adr_info *cp_info = &pIODUItem->adr_info;
    unsigned char   *pData;

    /* in first stage must be one byte, therefore no locking */
    ASSERT(SWAP_D(pItem->iops_length) == 1);
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
    {
      ASSERT(SWAP_D(pItem->iocs_length) == 1);
    }

    pData = (unsigned char *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset));

    /* get remote consumer status */
    if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
      *pIOremStateToRead = PNIO_S_GOOD;
    else
      *pIOremStateToRead = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
                            SWAP_D(pItem->iocs_length));

    /* set data */
    memcpy((void *)pData, (void *)pBuffer, BufLen);

    /* set local provider status */
    if (item_type == KRAMIOTLB_CONTR)
    {
        IODU_SET_IOXS(pData + SWAP_D(pItem->data_length),
            SWAP_D(pItem->iops_length), IOlocStateToSet, PNIO_IOXS_DETECT_BY_CONTROLLER);
    }
    else
    {
        IODU_SET_IOXS(pData + SWAP_D(pItem->data_length),
            SWAP_D(pItem->iops_length), IOlocStateToSet, PNIO_IOXS_DETECT_BY_DEVICE);
    }

    return PNIO_OK;
} /* end of IODU_kram_write */


#ifdef KRAMIOTLB_SUPPORT_TARGET

#ifdef IO_ROUTER

/* Functions for IO Router in Firmware */

/**************************************************************************
* F u n c t i o n:  IODU_router_open
*
* D e s c r i p t i o n :
*                Opens Controller and Device channel for IO-Router
*
* A r g u m e n t s:
*                cpid - in, CP id
*                Handle - in, handle of the controller
*                startop_cbf - in, user callback function pointer for StartOp
*                OpFault_cbf - in, user callback function pointer for OpFault
*                nFlags - in, DMA or KRAM access, ASYNC or SYNC
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_router_open(PNIO_UINT32 cpid, PNIO_UINT32 Handle,
    PNIO_UINT32 nFlags, IODATA_adr_info *adr_info, IODU_Item **ppIODUItem)
{
    /* get next empty slot from gIDUItems */
    *ppIODUItem = IODU_GetItem(cpid);
    if(!(*ppIODUItem))
        return PNIO_ERR_MAX_REACHED;

    KRAMIOTLB_init(adr_info->IOTLB_base, adr_info->iotlb_len,
        &(*ppIODUItem)->krtlbHeader);

    (*ppIODUItem)->Handle = Handle;
    (*ppIODUItem)->UsrHandle = 0;
    (*ppIODUItem)->initStatPDEV = 0;
    (*ppIODUItem)->dataRead = NULL;
    (*ppIODUItem)->dataWrite = NULL;

    /* to do :- may be we need to check this param
    (ie DMA or KRAM base address is initialised ) */
    (*ppIODUItem)->nFlags = nFlags;

    (*ppIODUItem)->adr_info = *adr_info;

    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_router_close
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_router_close(IODU_Item *pIODUItem)
{
    KRAMIOTLB_deinit(&pIODUItem->krtlbHeader);
    IODU_SetItemEmpty(pIODUItem);

    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_router_data_write_shadow (used by controller only)
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_router_write_shadow(IODU_Item *pIODUItem, KRAMIOTLB_Item *pItem,
    PNIO_UINT32 BufLen, PNIO_UINT8 *pBuffer, PNIO_UINT8 *pShadow, PNIO_UINT8 *pStatus,
    PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState)
{
    IODATA_adr_info *cp_info;

    /* validate in params */
    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    if(!pBuffer || !BufLen || !pShadow)
        return PNIO_ERR_PRM_BUF;

    if(SWAP_D(pItem->data_length) != BufLen)
        return PNIO_ERR_VALUE_LEN;

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    cp_info = &pIODUItem->adr_info;

    if((SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC) && (cp_info->DMA_base != 0))
        return PNIO_ERR_INTERNAL; /* IRT not supported */

    /* Get remote consumer status from KRAM */
    *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
        SWAP_D(pItem->iocs_length));

    /* Copy data buffer */
    memcpy(pShadow, pBuffer, BufLen < SWAP_D(pItem->data_length) ? BufLen : SWAP_D(pItem->data_length));

    /* Write the provider status */
    pStatus[0] = IOlocState;

    return PNIO_OK;
}
#endif /* IO_ROUTER */

#endif /* KRAMIOTLB_SUPPORT_TARGET */



#if IODU_ERTEC_TYPE == IODU_ERTEC200

PNIO_UINT32 IODU_com_unlock_provider_consumer200(PNIO_UINT32 EddHnd,
                                                 PNIO_UINT16 ProviderID,
                                                 PNIO_UINT16 ConsumerID,
                                                 PNIO_UINT8 *pProviderBuffer,
                                                 PNIO_UINT8 *pConsumerBuffer)
{
  LSA_RESULT lsa_res;
  PNIO_UINT32 ret = PNIO_OK;
  PNIO_UINT8  is_unlock_necessary = (  pProviderBuffer != 0 || pConsumerBuffer != 0) ? 1 : 0;

  EDD_RT_PROVIDER_BUFFER_REQ_TYPE    EddProvRtBufReq;
  EDD_RT_CONSUMER_BUFFER_REQ_TYPE    EddConsRtBufReq;

  /* unlock, only if previosly locked */
  if(pProviderBuffer)
  {
    EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_UNLOCK;
    EddConsRtBufReq.Service = EDD_SRV_SRT_CONSUMER_UNLOCK;

    EddProvRtBufReq.hDDB = (EDD_HANDLE)EddHnd;
    EddProvRtBufReq.ProviderID = ProviderID;
    EddProvRtBufReq.pBuffer = pProviderBuffer;

    lsa_res = edd_ProviderBufferRequest(&EddProvRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ProviderBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
      ret = PNIO_ERR_NO_CONNECTION;
    }
  }

  /* unlock, only if previosly locked */
  if(pConsumerBuffer)
  {
    EddConsRtBufReq.hDDB = (EDD_HANDLE)EddHnd;
    EddConsRtBufReq.ConsumerID = ConsumerID;
    EddConsRtBufReq.pBuffer = pConsumerBuffer;

    lsa_res = edd_ConsumerBufferRequest(&EddConsRtBufReq);
    if(lsa_res != LSA_OK){
       /* printf("edd_ConsumerBufferRequest(UNLOCK) ret=0x%x\n", lsa_res); */
       ret = PNIO_ERR_NO_CONNECTION;
    }
  }

/* TODO TODO TODO TODO TODO Mutex FREIGEBEN
  if(is_unlock_necessary == 1){
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
  }*/

  return ret;
}

PNIO_UINT32 IODU_com_lock_provider_consumer200(PNIO_UINT32 EddHnd,
                                                 PNIO_UINT16 ProviderID,
                                                 PNIO_UINT16 ConsumerID,
                                                 PNIO_UINT8 **pProviderBuffer,
                                                 PNIO_UINT8 **pConsumerBuffer)
{
  LSA_RESULT lsa_res;
  PNIO_UINT32 ret = PNIO_OK;
  EDD_RT_PROVIDER_BUFFER_REQ_TYPE    EddProvRtBufReq;
  EDD_RT_CONSUMER_BUFFER_REQ_TYPE    EddConsRtBufReq;

  EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_LOCK_CURRENT;
  EddConsRtBufReq.Service = EDD_SRV_SRT_CONSUMER_LOCK;

  *pProviderBuffer = 0;
  *pConsumerBuffer = 0;

/* TODO TODO TODO TODO TODO TODO TODO   MUTEX BESETZEN
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif*/

    EddProvRtBufReq.hDDB = (EDD_HANDLE)EddHnd;
    EddProvRtBufReq.ProviderID = ProviderID;

    lsa_res = edd_ProviderBufferRequest(&EddProvRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ProviderBufferRequest(LOCK_CURRENT) ret=0x%x\n", lsa_res); */

/* TODO TODO TODO TODO TODO TODO TODO   MUTEX FREIGEBEN
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif*/

      return PNIO_ERR_NO_CONNECTION;
    }

    *pProviderBuffer = EddProvRtBufReq.pBuffer;

    EddConsRtBufReq.hDDB = (EDD_HANDLE)EddHnd;
    EddConsRtBufReq.ConsumerID = ConsumerID;

    lsa_res = edd_ConsumerBufferRequest(&EddConsRtBufReq);
    if(lsa_res != LSA_OK){
      /* printf("edd_ConsumerBufferRequest(LOCK) ret=0x%x\n", lsa_res); */

      /*unlock Provider */
      EddProvRtBufReq.Service = EDD_SRV_SRT_PROVIDER_UNLOCK;
      EddConsRtBufReq.Service = EDD_SRV_SRT_CONSUMER_UNLOCK;

      EddProvRtBufReq.hDDB = (EDD_HANDLE)EddHnd;
      EddProvRtBufReq.ProviderID = ProviderID;

      (void)edd_ProviderBufferRequest(&EddProvRtBufReq);
/* TODO TODO TODO TODO TODO TODO TODO   MUTEX FREIGEBEN
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif*/

      return PNIO_ERR_NO_CONNECTION;
    }

    *pConsumerBuffer = EddConsRtBufReq.pBuffer;

  return ret;
}

#endif
