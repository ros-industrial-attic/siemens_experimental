/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : iodu_con.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : controller specific iodata update
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

#if IODU_ERTEC_TYPE == IODU_ERTEC400

#include "lsa_vers.h"

#if SYS_LSA_VERSION_NUM >= 4010000
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

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_open
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                cpid - in, CP id
*                Handle - in, handle of the controller
*                startop_cbf - in, user callback function pointer for StartOp
*                OpFault_cbf - in, user callback function pointer for OpFault
*                nFlags - in, ExtPar: DMA or KRAM access, ASYNC or SYNC
*                nFlags - in, DMA or KRAM access, ASYNC or SYNC
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_open(PNIO_UINT32 cpid, PNIO_UINT32 Handle,
    PNIO_UINT32 nFlags, IODATA_adr_info *adr_info, IODU_Item **ppIODUItem)
{
    /* get next empty slot from gIDUItems */
    *ppIODUItem = IODU_GetItem(cpid);
    if(!(*ppIODUItem))
        return PNIO_ERR_MAX_REACHED;

    KRAMIOTLB_init(adr_info->IOTLB_base, adr_info->iotlb_len,
        &(*ppIODUItem)->krtlbHeader);

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT

    /* Create hash table */
    if(KRAMIOTLB_OK != KRAMIOTLB_CreateContrHash(Handle,
        &(*ppIODUItem)->krtlbHeader)) {

        IODU_SetItemEmpty(*ppIODUItem);
        /* return invalid parameters */
        return PNIO_ERR_NO_RESOURCE;
    }

#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT */

    (*ppIODUItem)->Handle = Handle;

    (*ppIODUItem)->nFlags = nFlags; /* save ExtPar from PNIO_controller_open */

    (*ppIODUItem)->adr_info = *adr_info;

    return PNIO_OK;
}

/* IODU_ctrl_reset_ioxs will be called in front of PNIO_controller_close
   to set all available device submodules to NOT_AVAILABLE and to set her IOXS to BAD */

PNIO_UINT32 IODU_ctrl_reset_ioxs(IODU_Item * pIODUHnd)
{
  PNIO_UINT32 ioxs_offset, ioxs_length;
  PNIO_UINT8 * pIOBase = 0;

  KRAMIOTLB_Item ** SubSlotArr = 0;
  PNIO_UINT32 i, item_cnt = 0;

  /* get number of all submodules */
  KRAMIOTLB_GetHandleItems(pIODUHnd->Handle, 0, &item_cnt, 0, &(pIODUHnd->krtlbHeader));

  // printf("get number of all submodules item_cnt:%d\n", item_cnt);

  SubSlotArr = (KRAMIOTLB_Item **)malloc(sizeof(KRAMIOTLB_Item *) * item_cnt);
  if(SubSlotArr == 0) return PNIO_ERR_NO_RESOURCE;

  /* fill subslot pointer array */
  KRAMIOTLB_GetHandleItems(pIODUHnd->Handle, 0, &item_cnt, SubSlotArr, &(pIODUHnd->krtlbHeader));

  // printf("fill subslot pointer array,  item_cnt:%d\n", item_cnt);

  for(i=0; i<item_cnt; i++)
  {
   if(SubSlotArr[i] == 0) continue;

   /* only if a submodule is available, his IOXS muss be seted to BAD */
   if(SWAP_D(SubSlotArr[i]->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) continue;

   // printf("SubSlotArr[%d]->available:0x%x\n", i, SWAP_D(SubSlotArr[i]->available));

   if(SWAP_D(SubSlotArr[i]->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUHnd->adr_info.DMA_base != 0)
   {
      pIOBase = pIODUHnd->adr_info.DMA_base;
      // printf("    pIOBase = pIODUHnd->adr_info.DMA_base,  io_sync_type:0x%x  pIOBase:0x%x\n", SWAP_D(SubSlotArr[i]->io_sync_type), pIOBase);
   }
   else
   {
      pIOBase = pIODUHnd->adr_info.KRAM_base;
      // printf("    pIOBase = pIODUHnd->adr_info.KRAM_base, io_sync_type:0x%x  pIOBase:0x%x\n", SWAP_D(SubSlotArr[i]->io_sync_type), pIOBase);
   }

   if(SWAP_D(SubSlotArr[i]->io_out_type) & KRAMIOTLB_IO_IN)
   {
     ioxs_offset = SWAP_D(SubSlotArr[i]->iocs_offset);
     ioxs_length = SWAP_D(SubSlotArr[i]->iocs_length);

     // printf("    IN:  ioxs_offset:0x%x  ioxs_length:0x%x\n", ioxs_offset, ioxs_length);
   }
   else
   {
     ioxs_offset = SWAP_D(SubSlotArr[i]->data_offset) + SWAP_D(SubSlotArr[i]->data_length);
     ioxs_length = SWAP_D(SubSlotArr[i]->iops_length);

     // printf("    OUT: ioxs_offset:0x%x  ioxs_length:0x%x\n", ioxs_offset, ioxs_length);
   }

   // printf("         dev_nr=%d s=%d ss=%d typ=0x%x(1-I,2-O,4-PDEV,8-DDEX) log_addr=0x%x(%d)\n",
   //         SubSlotArr[i]->device_ar, SubSlotArr[i]->slot_nr,
   //         SubSlotArr[i]->subslot_nr, SubSlotArr[i]->io_out_type,
   //         SubSlotArr[i]->log_addr, SubSlotArr[i]->log_addr);


   /* now set the ioxs state */
   // Nachbesserung für AP01007908: falsches Prozessabbild bei IN_DATA nach Absturz der Applikation:
   // IODU_SET_IOXS() nur aufrufen, wenn nicht ins HOST-RAM (DMA !!) gegriffen wird.

   if(SWAP_D(SubSlotArr[i]->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUHnd->adr_info.DMA_base != 0)
   {
       //pIOBase points into HostRAM !! --> do nothing
   }
   else
   {
       //pIOBase points into KRAM
        IODU_SET_IOXS( pIOBase + ioxs_offset, ioxs_length, PNIO_S_BAD, PNIO_IOXS_DETECT_BY_CONTROLLER);
   }
   // printf("    end of SubSlot\n");
  }


  free(SubSlotArr);

  return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_close
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_close(IODU_Item *pIODUItem)
{
  (void)IODU_ctrl_reset_ioxs(pIODUItem);

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT

  /* Free hash table */
  if (KRAMIOTLB_OK != KRAMIOTLB_FreeContrHash(pIODUItem->Handle,
      &pIODUItem->krtlbHeader)) {
      /* return invalid resource */
      return PNIO_ERR_NO_RESOURCE;
  }
#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT */

  KRAMIOTLB_deinit(&pIODUItem->krtlbHeader);

  IODU_SetItemEmpty(pIODUItem);

  return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_irt_data_read
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                log_addr - in, logical address
*                BufLen - in, Length of the data buffer
*                pDataLen - in, Length of the read data buffer (in bytes).
*                pBuffer - out, Pointer to the data buffer
*                IOlocState - in, Local status of the read IO data
*                pIOremState - out, Remote status of the read IO data
*
* Return Value:
*
***************************************************************************/

PNIO_UINT32 IODU_ctrl_irt_data_read(IODU_Item *pIODUItem,
    KRAMIOTLB_Item *pKramItem, PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen,
    PNIO_UINT8 *pBuffer, PNIO_IOXS *pIOlocStateToRead, PNIO_IOXS *pIOlocStateToSet, PNIO_IOXS *pIOremState)
{
  PNIO_UINT32      copy_len;
  unsigned char   *pData;
  char      preAccessStatus = IRT_ACCESS_STATUS_OUTSIDE, postAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;

  if (pIODUItem->item_valid == ITEM_INVALID)
    return PNIO_ERR_WRONG_HND;

  *pDataLen = SWAP_D(pKramItem->data_length);
  copy_len = BufLen < *pDataLen ? BufLen : *pDataLen;

  pData = (unsigned char *)(pIODUItem->adr_info.DMA_base + SWAP_D(pKramItem->data_offset));

  /* check: IrtAccessWindow (after OpStart but before OpDone/OpFault) */
  preAccessStatus = *(pIODUItem->adr_info.IrtAccessStatus);

  /* Get remote provider status */
  *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pKramItem->data_length),
      SWAP_D(pItem->iops_length));
  /* get transfer and data status value from Frame APDU status*/
  if (!((SWAP_D(pKramItem->io_out_type) & KRAMIOTLB_IO_DDEX)) &&
                                       *pIOremState == PNIO_S_GOOD) {
    *pIOremState = IODU_GET_CONSUMER_QUALITY(
       pIODUItem->adr_info.DMA_base + SWAP_D(pKramItem->consumer_frame_apdu_stat_offset));
  }

  /* read data - Copy to user buffer */
  memcpy((void *)pBuffer, pData, copy_len);

  /* Read the local status, if destination not null */
  if( pIOlocStateToRead ){
    if((SWAP_D(pKramItem->io_out_type) & KRAMIOTLB_IO_DDEX)){
      *pIOlocStateToRead = PNIO_S_GOOD; /* no local state for DDEX INPUT existent, return allready S_GOOD */
    }
    else{
      *pIOlocStateToRead = (PNIO_IOXS)IODU_GET_IOXS(pIODUItem->adr_info.DMA_base + SWAP_D(pKramItem->iocs_offset),
                                                      SWAP_D(pKramItem->iocs_length));
    }
  }

  /* Set the consumer (local) status  */
  if( pIOlocStateToSet &&
       !((SWAP_D(pKramItem->io_out_type) & KRAMIOTLB_IO_DDEX)) ) {

    if( !(SWAP_D(pKramItem->io_out_type) & KRAMIOTLB_IO_IN) ){
      /* local status can be set only for INPUT bei read */
      return PNIO_ERR_PRM_IO_TYPE;
    }

    IODU_SET_IOXS(pIODUItem->adr_info.DMA_base + SWAP_D(pKramItem->iocs_offset),
        SWAP_D(pKramItem->iocs_length), *pIOlocStateToSet, PNIO_IOXS_DETECT_BY_CONTROLLER);
  }

  postAccessStatus = *(pIODUItem->adr_info.IrtAccessStatus);

  if(preAccessStatus == IRT_ACCESS_STATUS_OUTSIDE ||
      postAccessStatus == IRT_ACCESS_STATUS_OUTSIDE)
    return PNIO_WARN_IRT_INCONSISTENT;
  else
    return PNIO_OK;
}

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_data_read
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                log_addr - in, logical address
*                BufLen - in, Length of the data buffer
*                pDataLen - in, Length of the read data buffer (in bytes).
*                pBuffer - out, Pointer to the data buffer
*                IOlocState - in, Local status of the read IO data
*                pIOremState - out, Remote status of the read IO data
*
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_data_read(IODU_Item *pIODUItem,
    PNIO_UINT32 log_addr, PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen,
    PNIO_UINT8 *pBuffer, PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState,
    PNIO_UINT8 *pOutCache, PNIO_UINT32 BegOfOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    PNIO_UINT32      copy_len;
    unsigned char   *pData;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pDataLen || (!pBuffer && BufLen))
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_IN, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    /* in first stage must be one byte, therefore no locking */
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) ) {
      ASSERT(SWAP_D(pItem->iocs_length) == 1);
    }

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pDataLen = 0;
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC &&
        pIODUItem->adr_info.DMA_base != 0)
        return IODU_ctrl_irt_data_read(pIODUItem, pItem, BufLen, pDataLen, pBuffer,
                                        0, &IOlocState, pIOremState);

   /* set data length as output but 'pItem->data_length' is max available module length.
    * For 'controller byte slice acces (CBSA)': pItem->data_length is available module length
    * from the 'read address' til module end.
    */
    *pDataLen = SWAP_D(pItem->data_length);

   /* set current used copy length:
    * 1) if BufLen (i.e.'requested read len') is less than available data: use BufLen
    * 2) if 'requested read len' > than available data: use only available data
    */
    copy_len = ( BufLen < *pDataLen ) ? BufLen : *pDataLen;

    pData = (unsigned char *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset));

   /* sizeof data less than 4 bytes (32 bit): optimized access without KRAM lock
    * 4 byte read access at aligned address of KRAM is atomic (4 byte consistency: 3 byte data + 1 iops)
    * precondition: 1 byte iops size and iops is a next byte after the io data. Note: this is not true for
    * byte slice access
    */
    if(((((unsigned long)pData)& 3)+ SWAP_D(pItem->data_length) + SWAP_D(pItem->iops_length)) <= 4) {
        unsigned char mybuf[4];
        /* copy data from KRAM -> temp buff: we 'copy' always 4 bytes from addr xxxx0 */
        *((PNIO_UINT32*)mybuf) = *((PNIO_UINT32*)((((unsigned long)pData) >> 2) << 2));
        /* copy real requested data len from temp buffer to user buffer */
        if (copy_len) {
          memcpy((void *)pBuffer, &mybuf[((unsigned long)pData)& 3], copy_len);
        }
       /* set io data provider status (iops).
        * For controller byte slice acces (CBSA) iops is not always present in the temp buffer. IOPS is present
        * only if we are reading all the data until the end of submodul + 1 byte iops (== max 3 byte data + 1 iops)
        * This is the case for standard io access but not for the byte slice access.
        */

		*pIOremState = (PNIO_IOXS)IODU_GET_IOXS(
			&mybuf[(((unsigned long)pData)& 3) + SWAP_D(pItem->data_length)],
			SWAP_D(pItem->iops_length));

        /* Set the consumer status  */
        if( !((SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX)) )
        {
         IODU_SET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
            SWAP_D(pItem->iocs_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

          /* Set the consumer status in output cache */
          if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){
 
              IODU_SET_IOXS(pOutCache + SWAP_D(pItem->iocs_offset) - BegOfOutCache,
                  SWAP_D(pItem->iocs_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
          }
        }
    } else {
        /* Wait for semaphore, locking functions do not support multi entry */
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

        /* Pointer to data area = IO_Kram_Base + data_offset */
        CnsBlockRead(cp_info->EREG_base, SWAP_D(pItem->data_offset),
        	SWAP_D(pItem->data_length) + SWAP_D(pItem->iops_length), CNS_MAX_POLL_TIMEOUT);

        /* Get remote provider status */
        *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pData + SWAP_D(pItem->data_length),
            SWAP_D(pItem->iops_length));

        /* read data - Copy to user buffer */
        memcpy((void *)pBuffer, pData, copy_len);

        /* UnLock the data */
        CnsBlockFreeRead(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);

        /* Set the consumer status  */
        if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
        {
          IODU_SET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
            SWAP_D(pItem->iocs_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

          /* Set the consumer status in output cache */
          if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){

              IODU_SET_IOXS(pOutCache + SWAP_D(pItem->iocs_offset) - BegOfOutCache,
                  SWAP_D(pItem->iocs_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
          }
        }

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
    }

    return PNIO_OK;
}

/*-----------------------------------------------------------------------------
 * Name  : IODU_ctrl_KRAM_read
 * Descr : Reads io data from KRAM. Used by iorouter watchdog.
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: PNIO_OK or PNIO_ERR_xxx
 */
PNIO_UINT32 IODU_ctrl_kram_read(IODU_Item *pIODUItem,
    PNIO_UINT32 log_addr, PNIO_IO_TYPE io_type, PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen,
    PNIO_UINT8 *pBuffer, PNIO_IOXS *pIOlocState, PNIO_IOXS *pIOremState)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    KRAMIOTLB_IN_OUT_TYPE kr_io_t;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pDataLen || (!pBuffer && BufLen))
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    switch(io_type){
      case PNIO_IO_IN:
         kr_io_t = KRAMIOTLB_IO_IN;
        break;
      case PNIO_IO_OUT:
         kr_io_t = KRAMIOTLB_IO_OUT;
        break;
      default:
          return PNIO_ERR_PRM_IO_TYPE;
        break;
    }

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        kr_io_t, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    /* in first stage must be one byte, therefore no locking */
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) ) {
      ASSERT(SWAP_D(pItem->iocs_length) == 1);
    }

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pDataLen = 0;
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC)
        return IODU_ctrl_irt_data_read(pIODUItem, pItem, BufLen, pDataLen, pBuffer,
                                        pIOlocState, 0, pIOremState);

    /* dabei wird lokalen Status NICHT gesetzt */
    return IODU_kram_read(pIODUItem, pItem, KRAMIOTLB_CONTR, BufLen, pDataLen,
                                pBuffer, pIOlocState, 0, pIOremState);
} /* end of IODU_ctrl_KRAM_read */

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_remstat_read
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                log_addr - in, logical address
*                inoutt - in, input=KRAMIOTLB_IO_IN, output=KRAMIOTLB_IO_OUT type
*                pIOremState - out, remote status of the IO data
*
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_remstat_read(IODU_Item *pIODUItem,
    PNIO_UINT32 log_addr, KRAMIOTLB_IN_OUT_TYPE inoutt, PNIO_IOXS *pIOremState)
{
    KRAMIOTLB_Item * pItem = NULL;

    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    PNIO_UINT8*      rem_stat_addres;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
              inoutt, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
       *pIOremState = PNIO_S_BAD;
       return PNIO_OK;
    }

    /* Get remote status */
    if(inoutt & KRAMIOTLB_IO_IN) {
        /* INPUT Data */
        rem_stat_addres = (cp_info->KRAM_base + SWAP_D(pItem->data_offset) +
            SWAP_D(pItem->data_length));
    } else if(inoutt & KRAMIOTLB_IO_OUT) {
        /* OUTPUT Data */
        rem_stat_addres = (cp_info->KRAM_base + SWAP_D(pItem->iocs_offset));
    } else {
        return PNIO_ERR_PRM_ACCESS_TYPE;
    }

    if( (SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
      *pIOremState = PNIO_S_GOOD;
    else{
      *pIOremState = (PNIO_IOXS) IODU_GET_IOXS(rem_stat_addres, SWAP_D(pItem->iocs_length));
      /* printf("IODU_ctrl_remstat_read rem_stat_addres=0x%x remStat=0x%x\n", rem_stat_addres, *pIOremState); */
    }

    return PNIO_OK;
}

#ifdef IODU_SUPPORT_IO_CACHE_CONTROLLER

/* to sort IODU_CONS_BND array in ascending order */
int compare_bnd(const void *p1, const void *p2)
{
    IODU_CONS_BND *i1, *i2;
    i1=(IODU_CONS_BND *)p1;
    i2=(IODU_CONS_BND *)p2;

    return (int)(i1->d_offset - i2->d_offset);
}

/* to create boundaries array for "cached" access to io data, to improve performance */
void create_bnd_array(KRAMIOTLB_Item **io_tlb, PNIO_UINT32 io_tlb_len,
    IODU_CONS_BND *bnd_arr, PNIO_UINT32 bnd_arr_size_max, PNIO_UINT32 *bnd_arr_size,
    KRAMIOTLB_IN_OUT_TYPE io_t)
{
    PNIO_UINT32 curIoTlbIdx = 0, BlkBegin, BlkLen, BlkBeginIdx,
        curConsBndIdx = 0, CurIoTlbItemEnd, CurIoTlbItemBeg;

    *bnd_arr_size = 0;

    /* find neightbor areas, that can be reads/writes per one access */

    while(curIoTlbIdx < io_tlb_len && *bnd_arr_size < bnd_arr_size_max) {
        if((KRAMIOTLB_IN_OUT_TYPE)SWAP_D(io_tlb[curIoTlbIdx]->io_out_type) & io_t)
            BlkBegin = SWAP_D(io_tlb[curIoTlbIdx]->data_offset);
        else
            BlkBegin = SWAP_D(io_tlb[curIoTlbIdx]->iocs_offset);

        BlkLen   = 0;

        for( ; 1; curIoTlbIdx++) {
            if(curIoTlbIdx >= io_tlb_len)
                break;

            /* except IRT */
            if((KRAMIOTLB_IOSYNC_TYPE)SWAP_D(io_tlb[curIoTlbIdx]->io_sync_type) == KRAMIOTLB_IO_SYNC)
                continue;

            if((KRAMIOTLB_IN_OUT_TYPE)SWAP_D(io_tlb[curIoTlbIdx]->io_out_type) & io_t) {
                CurIoTlbItemEnd = SWAP_D(io_tlb[curIoTlbIdx]->data_offset) +
                    SWAP_D(io_tlb[curIoTlbIdx]->data_length) +
                    SWAP_D(io_tlb[curIoTlbIdx]->iops_length);
                CurIoTlbItemBeg = SWAP_D(io_tlb[curIoTlbIdx]->data_offset);
            } else {
                CurIoTlbItemEnd = SWAP_D(io_tlb[curIoTlbIdx]->iocs_offset) +
                    SWAP_D(io_tlb[curIoTlbIdx]->iocs_length);
                CurIoTlbItemBeg = SWAP_D(io_tlb[curIoTlbIdx]->iocs_offset);
            }

            if(CurIoTlbItemEnd - BlkBegin > 255 ||
                (CurIoTlbItemBeg - (BlkBegin + BlkLen)) > 0) /* respect gap */
                break;

            BlkLen = CurIoTlbItemEnd - BlkBegin;
        }

        bnd_arr[*bnd_arr_size].d_offset = BlkBegin;
        bnd_arr[*bnd_arr_size].d_length = BlkLen;

        (*bnd_arr_size)++;
    }


    qsort(bnd_arr, *bnd_arr_size, sizeof(bnd_arr[0]), compare_bnd);

    /* find areas boundaries for locking, put areas together, to avoid to many locks */
    /* lock areas for ERTEC400 can not be longer 256 bytes */
    while(curConsBndIdx < *bnd_arr_size) {
        BlkBeginIdx = curConsBndIdx;
        BlkBegin = bnd_arr[curConsBndIdx].d_offset;
        BlkLen   = 0;

        /* bnd_arr[curConsBndIdx].l_offset = BlkBegin; */

        /* compute maximal posible length whith less 256 */
        while(curConsBndIdx < *bnd_arr_size && ((bnd_arr[curConsBndIdx].d_offset +
            bnd_arr[curConsBndIdx].d_length - BlkBegin) < 256)) {
            BlkLen = bnd_arr[curConsBndIdx].d_offset + bnd_arr[curConsBndIdx].d_length - BlkBegin;

            /* go to next bound */
            curConsBndIdx++;
            /*printf("   3w:curConsBndIdx = %i\n", curConsBndIdx);  */
            /*printf("   3w:BlkLen = %i\n", BlkLen); */
        }

        bnd_arr[BlkBeginIdx].l_offset = BlkBegin;
        bnd_arr[BlkBeginIdx].l_length = BlkLen;
        bnd_arr[BlkBeginIdx].d_cnt = curConsBndIdx - BlkBeginIdx;
    }

    /*for(unsigned long i=0; i<*bnd_arr_size; i++)
      printf(" %s Idx=%i l_off=%06i l_len=%06i d_cnt=%03i d_off=%06i d_len=%i\n",
                 (io_t == KRAMIOTLB_IO_IN ? "IN" : "OUT"),
                 *bnd_arr_size,
                 bnd_arr[i].l_offset,
                 bnd_arr[i].l_length,
                 bnd_arr[i].d_cnt,
                 bnd_arr[i].d_offset,
                 bnd_arr[i].d_length);*/
}

/*-----------------------------------------------------------------------------
 * Name  : IODU_ctrl_cache_init
 *
 *     PL: AP00866265: Changes in calling KRAMIOTLB_GetHandleItemsAdvance():
 *         Because of problem to write "packed" modules via cache, we exclude
 *         diagnosis modules (addresses) from cache. Read access to this addresses
 *         is redirect directly to KRAM.
 */
PNIO_UINT32 IODU_ctrl_cache_init(IODU_Item *pIODUItem,
    PNIO_UINT8 **pOutCache, IODU_CONS_BND **pOutBndArr, PNIO_UINT32 *OutBndArrSize,
    PNIO_UINT8 **pInCache, IODU_CONS_BND **pInBndArr, PNIO_UINT32 *InBndArrSize,

    KRAMIOTLB_Item *ItemsListToExclude,
    PNIO_UINT32     ItemsListToExcludeSize)
{
    IODATA_adr_info *cp_info = &pIODUItem->adr_info;
    KRAMIOTLB_Item **locIoTlb = 0; /* array of pointers to KRAMIOTLB_Item */
    IODU_CONS_BND *ConsBound = 0; /* array of IODU_CONS_BND */

    PNIO_UINT32 areaLen = 0, locIoTlbLen = 0;
    PNIO_UINT32 curConsIdx, minConsIdx, maxConsIdx, i;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    /* PL: AP00866265: second param changed from 0 -> 1: no diag adresses in cache. See also IODU_ctrl_data_read_ex() */
    KRAMIOTLB_GetHandleItemsAdvance(pIODUItem->Handle, 1, &locIoTlbLen, 0,
         ItemsListToExclude, ItemsListToExcludeSize, KRAMIOTLB_IsItemValidContrHost,
          &(pIODUItem->krtlbHeader));
    if(!locIoTlbLen){
      /* no io items found, nothing to do */
      return PNIO_OK;
    }

    locIoTlb = (KRAMIOTLB_Item **)malloc(locIoTlbLen * sizeof(locIoTlb[0]));
    if(!locIoTlb){
      return PNIO_ERR_OS_RES;
    }

    /* PL: AP00866265: second param changed from 0 -> 1: no diag adresses in cache. See also IODU_ctrl_data_read_ex() */
    KRAMIOTLB_GetHandleItemsAdvance(pIODUItem->Handle, 1, &locIoTlbLen, locIoTlb,
                                     ItemsListToExclude, ItemsListToExcludeSize, KRAMIOTLB_IsItemValidContrHost,
                                      &(pIODUItem->krtlbHeader));

    /*for(unsigned long i=0; i< locIoTlbLen; i++)
    {
        printf("io_tlb: d:%2i s:%2i ss:%2i io_t:%c d_len:%i "
            "addr:%06i d_off:%06i iocs_off:%06i\n",
            locIoTlb[i]->device_ar, locIoTlb[i]->slot_nr, locIoTlb[i]->subslot_nr,
                (locIoTlb[i]->io_out_type == KRAMIOTLB_IO_IN)?'r':'w',
                locIoTlb[i]->data_length, locIoTlb[i]->log_addr,
                locIoTlb[i]->data_offset, locIoTlb[i]->iocs_offset);
    }*/

    /* find lock boundaries for input area */
    /* allocate locIoTlb count of ConsBounds items,
       because count of ConsBouns can't be higher as locIoTlbLen */
    ConsBound = (IODU_CONS_BND *)malloc(locIoTlbLen * sizeof(ConsBound[0]));
    if(!ConsBound){
      free(locIoTlb);
      return PNIO_ERR_OS_RES;
    }

    memset(ConsBound, 0, locIoTlbLen * sizeof(ConsBound[0]));
    create_bnd_array(locIoTlb, locIoTlbLen, ConsBound, locIoTlbLen,
        &curConsIdx, KRAMIOTLB_IO_IN);

    /* search lowest and greatest offset */
    minConsIdx = maxConsIdx = 0;
    for(i=1; i< curConsIdx; i++) {
        if(ConsBound[i].d_offset < ConsBound[minConsIdx].d_offset)
            minConsIdx = i;
        if(ConsBound[i].d_offset > ConsBound[maxConsIdx].d_offset)
            maxConsIdx = i;
    }

    areaLen = ConsBound[maxConsIdx].d_offset +
        ConsBound[maxConsIdx].d_length - ConsBound[minConsIdx].d_offset;

    *pInCache = new PNIO_UINT8[areaLen];
    ASSERT(*pInCache);

    *InBndArrSize = curConsIdx;
    *pInBndArr = new IODU_CONS_BND[*InBndArrSize];
    ASSERT(*pInBndArr != 0);

    memcpy(*pInBndArr, ConsBound, sizeof(IODU_CONS_BND) * curConsIdx);

    /* find lock boundaries for output area */
    memset(ConsBound, 0, locIoTlbLen * sizeof(ConsBound[0]));
    create_bnd_array(locIoTlb, locIoTlbLen, ConsBound, locIoTlbLen,
        &curConsIdx, KRAMIOTLB_IO_OUT);

    /* search lowest and greatest offset */
    minConsIdx = maxConsIdx = 0;
    for(i=1; i< curConsIdx; i++) {
        if(ConsBound[i].d_offset < ConsBound[minConsIdx].d_offset)
            minConsIdx = i;
        if(ConsBound[i].d_offset > ConsBound[maxConsIdx].d_offset)
            maxConsIdx = i;
    }
    areaLen = ConsBound[maxConsIdx].d_offset +
        ConsBound[maxConsIdx].d_length - ConsBound[minConsIdx].d_offset;

    *pOutCache = new PNIO_UINT8[areaLen];
    ASSERT(*pOutCache);

    /* copy output data from kram in output cache */
    /* synchronise output cache and kram */
    memcpy(*pOutCache, cp_info->KRAM_base + ConsBound[0].d_offset, areaLen);

    *OutBndArrSize = curConsIdx;
    *pOutBndArr = new IODU_CONS_BND[*OutBndArrSize];
    ASSERT(*pOutBndArr != 0);

    memcpy(*pOutBndArr, ConsBound, sizeof(IODU_CONS_BND) * curConsIdx);

    free(locIoTlb);
    free(ConsBound);

    return PNIO_OK;
} /* end of IODU_ctrl_cache_init */

/*-----------------------------------------------------------------------------
 * IODU_ctrl_data_write_cache_flush:  Transfer cache content to KRAM
 */
void IODU_ctrl_data_write_cache_flush(IODU_Item *pIODUItem,
    PNIO_UINT8 *pOutCache, IODU_CONS_BND *pOutBndArr, PNIO_UINT32 OutBndArrSize)
{
    IODATA_adr_info *cp_info = &pIODUItem->adr_info;
    unsigned long i, iBlkEndIdx;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return;

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
    #endif

    for(i=0;i<OutBndArrSize;) {
        CnsBlockWrite(cp_info->EREG_base, pOutBndArr[i].l_offset,
            pOutBndArr[i].l_length, CNS_MAX_POLL_TIMEOUT);
        /*printf("w_lock(%i,%i)\n", pOutBndArr[i].l_offset, pOutBndArr[i].l_length);*/
        for(iBlkEndIdx = i + pOutBndArr[i].d_cnt; i < iBlkEndIdx; i++) {
        memcpy(cp_info->KRAM_base + pOutBndArr[i].d_offset,
            pOutCache + (pOutBndArr[i].d_offset - pOutBndArr[0].d_offset),
            pOutBndArr[i].d_length);
            /*printf("w_memcpy(%i,%i)\n", pOutBndArr[i].d_offset, pOutBndArr[i].d_length);*/
        }
        CnsBlockFreeWrite(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);
    }

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #endif

    return;
} /* end of IODU_ctrl_data_write_cache_flush */

/*-----------------------------------------------------------------------------
 * IODU_ctrl_data_read_cache_refresh
 */
void IODU_ctrl_data_read_cache_refresh(IODU_Item *pIODUItem,
    PNIO_UINT8 *pInCache, IODU_CONS_BND *pInBndArr, PNIO_UINT32 InBndArrSize)
{
    IODATA_adr_info *cp_info = &pIODUItem->adr_info;
    unsigned long i, iBlkEndIdx;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return ;

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
    #endif

    for(i=0;i<InBndArrSize;) {
        /* ATTENTION: It is not possible to lock big input area from host.
            In this case APDU-Stutus of Devices will be lockt.
            EDD can not access them and generate Device Failure

            UNFORTUNATELY we must lock each separate area.
            To improve performance configure first all input sumbodules and
            all output submodules to end

            CnsBlockRead(cp_info->EREG_base, pInBndArr[i].l_offset,
                pInBndArr[i].l_length, CNS_MAX_POLL_TIMEOUT);*/

        /* printf("r_lock(%i,%i)\n", pInBndArr[i].l_offset, pInBndArr[i].l_length);*/
        for(iBlkEndIdx = i + pInBndArr[i].d_cnt; i < iBlkEndIdx; i++) {
            CnsBlockRead(cp_info->EREG_base, pInBndArr[i].d_offset,
                pInBndArr[i].d_length, CNS_MAX_POLL_TIMEOUT);

            /* check correctness of destination area */
            if(pInBndArr[i].d_offset < pInBndArr[0].d_offset){
              /* prevent overwriting of memory befor pInCache */
              ASSERT("prevent overwriting of memory befor pInCache" == 0);
            }

            memcpy(pInCache + (pInBndArr[i].d_offset - pInBndArr[0].d_offset),
                    cp_info->KRAM_base + pInBndArr[i].d_offset,
                     pInBndArr[i].d_length);
            CnsBlockFreeRead(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);
            /*printf("r_memcpy(%i,%i)\n", pInBndArr[i].d_offset, pInBndArr[i].d_length);*/
        }
        /*CnsBlockFreeRead(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);*/
    }

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #endif

    return;
} /* end of IODU_ctrl_data_read_cache_refresh */

/*-----------------------------------------------------------------------------
 * Name  : IODU_ctrl_data_read_ex
 * Descr : Reads process input data from cache
 *     PL: AP00866265.
 *         Changes because of problems to write "packed" modules via cache.
 *         Now diagnosis modules are not transfered to the cache and this
 *         read function reads diagnosis module (i.e. modules without data,
 *         data length == 0) from the KRAM. See also IODU_ctrl_cache_init().
 * Param :
 *  [ IN]: pIODUItem: Pointer to IODUItem retrieved by IODU_ctrl_open.
 *  [ IN]: log_addr : Logical address of the input module to be read.
 *  [ IN]: BufLen   : Available buffer size [bytes] for received data.
 *  [ IN]: IOlocState: Consumer status given by PNIO user         :
 *  [OUT]: pBuffer  : Ptr to receive buffer. (data destination ptr)
 *  [OUT]: pDataLen : Number of bytes read or total module length (see note belowe)
 *                    At input: the value of the output variable is set to the 'BufLen' input
 *                    parameter from the PNIO_data_read/write call. !!!
 *  [OUT]: pIOremState: Remote status ==provider state. State of received data.
 *  ...    xxxCache : Cache boundery table parameters
 *
 *  NOTE:  pDataLen : IO-Base Manual Text:
 *       1.Gelesene Länge des Datenpuffers (in Byte) Wenn die Länge der IO-Device-Daten
 *         kleiner oder gleich "BufLen" ist, steht in "*pDataLen" die Länge der gelesenen IO-Daten.
 *       2.Wenn die Länge der IO-Device-Daten größer als der zur Verfügung stehende Datenpuffer ist,
 *         werden so viele Daten in den Datenpuffer gelesen, wie hineingehen (BufLen).
 *         In "*pDataLen" steht aber die Länge des Datenpuffers, die benötigt worden wäre,
 *         um alle IO-Daten zu lesen.
 *
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_xxx
 */
PNIO_UINT32 IODU_ctrl_data_read_ex(IODU_Item *pIODUItem, PNIO_UINT32 log_addr,
   PNIO_UINT32 BufLen, PNIO_UINT32 * pDataLen, PNIO_UINT8 * pBuffer,
   PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
   PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
   PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    PNIO_UINT32      copy_len;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pDataLen || (!pBuffer && BufLen))
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_IN, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pDataLen = 0;
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC)
        return IODU_ctrl_irt_data_read(pIODUItem, pItem, BufLen, pDataLen, pBuffer,
                                        0, &IOlocState, pIOremState);

    /* PL: AP00866265: read/write "packed" modules */
    if(SWAP_D(pItem->data_length) == 0) {
        /* we are reading dianosis modul -> read directly from KRAM */
        PNIO_IOXS  IOlocStateToRead_dummy = PNIO_S_GOOD;

        return IODU_kram_read(pIODUItem, pItem, KRAMIOTLB_CONTR, BufLen, pDataLen, pBuffer,
                              &IOlocStateToRead_dummy, &IOlocState, /* local state ==consumer state will be set */
                              pIOremState  /* remote ==provider state */
                              );
    }
    /* PL: AP00866265 end */

   /* PL: Fix of problem detected during 'controller byte slice acces (CBSA)' test. ->
    * Output parameter wasn't set correctly.
    * Set data length as output to the max available module length. (see IO-Base docu):
    */
    *pDataLen = SWAP_D(pItem->data_length);

   /* Note: If the available receive buffer is smaller than the available module length,
    *       we copy only 'BufLen' number of bytes. (to not overwrite destination mem).
    *       But the *pDataLen still contains the max. available modu length, to show the
    *       user, how long the receive buffer should be, to receive all module data.
    *       PL: why we return PNIO_OK is not clear. - historical reasons ???
    */
    copy_len = BufLen < *pDataLen ? BufLen : *pDataLen;

    /* check correctness of source area */
    if(SWAP_D(pItem->data_offset) < BegOfInCache){
      /* prevent reading of memory befor pInCache */
      return PNIO_ERR_INTERNAL;
    }

    /* read remote provider status from input cache */
    *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pInCache + SWAP_D(pItem->data_offset) +
                                             SWAP_D(pItem->data_length) - BegOfInCache,
                                              SWAP_D(pItem->iops_length));
    /* read data from input cache */
    memcpy((void *)pBuffer, pInCache + SWAP_D(pItem->data_offset) - BegOfInCache, copy_len);

    /* Set the consumer status in output cache */
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX)){

      /* check correctness of destination area */
      if(SWAP_D(pItem->iocs_offset) < BegOfOutCache){
         /* prevent overwriting of memory befor pOutCache */
         return PNIO_ERR_INTERNAL;
      }

      IODU_SET_IOXS(pOutCache + SWAP_D(pItem->iocs_offset) - BegOfOutCache,
        SWAP_D(pItem->iocs_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
    }

    return PNIO_OK;
} /* end of IODU_ctrl_data_read_ex */


#ifdef IO_ROUTER
/*-----------------------------------------------------------------------------
 * Name  : IODU_ctrl_CACHE_read
 * Descr : Reads io data from cache, is used by iorouter watchdog.
 * Param :
 *  [ IN]: log_addr: if(log_addr & 0x8000)->output (MSB must be set for output)
 *  [OUT]:
 * Return: PNIO_OK or PNIO_ERR_xxx
 */
PNIO_UINT32 IODU_ctrl_CACHE_read(IODU_Item *pIODUItem, PNIO_UINT32 log_addr,
   PNIO_UINT32 BufLen, PNIO_UINT32 * pDataLen, PNIO_UINT8 * pBuffer,
   PNIO_IOXS *pIOlocState, PNIO_IOXS * pIOremState,
   PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
   PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    PNIO_UINT32      copy_len;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pDataLen || (!pBuffer && BufLen))
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_IN, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pDataLen = 0;
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC)
    {
      ASSERT("KRAMIOTLB_IO_SYNC not supported for IODU_ctrl_KRAM_read" == 0);
    }

    *pDataLen = SWAP_D(pItem->data_length);

    copy_len = BufLen < *pDataLen ? BufLen : *pDataLen;

    if(log_addr & 0x8000) /* output */
    {
      /* check correctness of source area */
      if(SWAP_D(pItem->data_offset) < BegOfOutCache){
         /* prevent reading of memory befor pOutCache */
         return PNIO_ERR_INTERNAL;
      }

      /* read data from output cache */
      memcpy((void *)pBuffer, (void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache), copy_len);

      /* check correctness of source area */
      if((SWAP_D(pItem->data_offset) + SWAP_D(pItem->data_length)) < BegOfInCache){
        /* prevent reading of memory befor pInCache */
        return PNIO_ERR_INTERNAL;
      }

      /* read remote consumer status from input cache */
      *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pInCache + SWAP_D(pItem->data_offset) +
                                               SWAP_D(pItem->data_length) - BegOfInCache, SWAP_D(pItem->iops_length));

      /* get the provider status in output cache */
      if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX)){
       *pIOlocState = IODU_GET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
                                     SWAP_D(pItem->data_length) - BegOfOutCache, SWAP_D(pItem->iops_length));
      }
    }
    else /* input */
    {
      /* check correctness of source area */
      if(SWAP_D(pItem->data_offset) < BegOfInCache){
        /* prevent reading of memory befor pInCache */
        return PNIO_ERR_INTERNAL;
      }

      /* read data from input cache */
      memcpy((void *)pBuffer, pInCache + SWAP_D(pItem->data_offset) - BegOfInCache, copy_len);

      /* read remote provider status from input cache */
      *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pInCache + SWAP_D(pItem->data_offset) +
                                               SWAP_D(pItem->data_length) - BegOfInCache, SWAP_D(pItem->iops_length));

      /* get the consumer status in output cache */
      if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX)){

        /* check correctness of source area */
        if(SWAP_D(pItem->iocs_offset) < BegOfOutCache){
           /* prevent reading of memory befor pOutCache */
           return PNIO_ERR_INTERNAL;
        }

        *pIOlocState = IODU_GET_IOXS(pOutCache + SWAP_D(pItem->iocs_offset) - BegOfOutCache,
         SWAP_D(pItem->iocs_length));
      }
    }

    return PNIO_OK;
} /* end of IODU_ctrl_CACHE_read */
#endif /* IO_ROUTER */

/*-----------------------------------------------------------------------------
 * Name  : IODU_ctrl_data_write_ex
 * Descr : IO-Base host interface function. It writes output data to cache
 *         not directly to the KRAM (this is done by 'IODU_ctrl_data_write()'.
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: PNIO_OK, PNIO_ERR_xxx
 */
PNIO_UINT32 IODU_ctrl_data_write_ex(IODU_Item *pIODUItem,
    PNIO_UINT32  log_addr, PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
    PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pBuffer && BufLen)
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_OUT, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    /* read iocs from input cache */
    if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
      *pIOremState = PNIO_S_GOOD;
    else{
      /* check correctness of source area */
      if(SWAP_D(pItem->iocs_offset) < BegOfInCache){
        /* prevent reading of memory befor pInCache */
        return PNIO_ERR_INTERNAL;
      }

      *pIOremState = (PNIO_IOXS)IODU_GET_IOXS( pInCache + SWAP_D(pItem->iocs_offset) - BegOfInCache,
                                                 SWAP_D(pItem->iocs_length));
    }

    /* check correctness of destination area */
    if(SWAP_D(pItem->data_offset) < BegOfOutCache){
       /* prevent overwriting of memory befor pOutCache */
       return PNIO_ERR_INTERNAL;
    }

    /* CBSA: Controller Byte Slice Access extension */
    if ( (pIODUItem->nFlags & PNIO_CEP_SLICE_ACCESS) == 0 ) {
        // module slice access is off (not allowed, this is the default case):
        // Because of module consistency, user have to write whole module at ones.
        if(SWAP_D(pItem->data_length) != BufLen) {
            return PNIO_ERR_VALUE_LEN;
        }
    }
    else {
        // module slice access is on (is allowed),user must be able
        // to write less bytes than the total available module data length
        if(SWAP_D(pItem->data_length) < BufLen) {
            return PNIO_ERR_VALUE_LEN;   // Fix of AP01103504
        }
    }

    /*HR 22.01.2009 synchronize access to OUTPUT CACHE,
    because IODU_ctrl_data_write_ex will be called from at least 2 contextes:
    1. User writes IO-DAta in OUTPUT-Cache(PNIO_data_write_cache)
    2. User call PNIO_data_write_cache_flush and in them context
       IODU_ctrl_data_write_ex can be called too for IO-Router submodules

    we need to synchronize to avoid output data partially from first
    and partially from second context */

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
    #endif

    /* write output data in output cache */
    memcpy((void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache),
        (void *)pBuffer, BufLen);

    /* write iops in output cache */
    IODU_SET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
        SWAP_D(pItem->data_length) - BegOfOutCache,
        SWAP_D(pItem->iops_length),
        IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

    #if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
    #endif

    return PNIO_OK;

} /* end of IODU_ctrl_data_write_ex */

#endif /* IODU_SUPPORT_IO_CACHE_CONTROLLER */

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_irt_data_write
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                pKramItem - in, KRAMIOTLB_Item(real data offsets)
*                BufLen - in, Length of the data buffer
*                pBuffer - in, Pointer to the data buffer of the IO data to be written
*                IOlocState - in, Local status of the IO data to be written
*                pIOremState - out, Remote status of the read IO data
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_irt_data_write(IODU_Item *pIODUItem, KRAMIOTLB_Item *pItem,
    PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState)
{
  char preAccessStatus = IRT_ACCESS_STATUS_OUTSIDE, postAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;

  /* check: IrtAccessWindow (after OpStart but before OpDone/OpFault) */
  preAccessStatus = *(pIODUItem->adr_info.IrtAccessStatus);

  if (pIODUItem->item_valid == ITEM_INVALID)
    return PNIO_ERR_WRONG_HND;

  /* get consumer status */
  if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
      *pIOremState = PNIO_S_GOOD;
  else {

    *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(pIODUItem->adr_info.DMA_base +
                               SWAP_D(pItem->iocs_offset), SWAP_D(pItem->iocs_length));
    /* get transfer and data status value from Frame APDU status*/
    if( *pIOremState == PNIO_S_GOOD) {
      *pIOremState = IODU_GET_CONSUMER_QUALITY(
         pIODUItem->adr_info.DMA_base + SWAP_D(pItem->consumer_frame_apdu_stat_offset));
    }
  }

  /* set data */
  memcpy((void *)(pIODUItem->adr_info.DMA_base + SWAP_D(pItem->data_offset)),
          (void *)pBuffer, BufLen);

  /* set provider status  */
  IODU_SET_IOXS(pIODUItem->adr_info.DMA_base + SWAP_D(pItem->data_offset) +
      SWAP_D(pItem->data_length), SWAP_D(pItem->iops_length),
      IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

  postAccessStatus = *(pIODUItem->adr_info.IrtAccessStatus);

  if(preAccessStatus == IRT_ACCESS_STATUS_OUTSIDE ||
      postAccessStatus == IRT_ACCESS_STATUS_OUTSIDE)
    return PNIO_WARN_IRT_INCONSISTENT;
  else
    return PNIO_OK;
}


/**************************************************************************
* F u n c t i o n:  IODU_ctrl_data_write
*
* D e s c r i p t i o n :
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                log_addr - in, logical address
*                BufLen - in, Length of the data buffer
*                pBuffer - in, Pointer to the data buffer of the IO data to be written
*                IOlocState - in, Local status of the IO data to be written
*                pIOremState - out, Remote status of the read IO data
*
* Return Value:
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_data_write(IODU_Item *pIODUItem, PNIO_UINT32  log_addr,
    PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT8 * pOutCache, PNIO_UINT32 BegOfOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    unsigned char   *pData;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pBuffer && BufLen)
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_OUT, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    /* CBSA: Controller Byte Slice Access extension */
    if ( (pIODUItem->nFlags & PNIO_CEP_SLICE_ACCESS) == 0 ) {
        // module slice access is off (not allowed, it is default case):
        // Because of module consistency, user have to write whole module at ones.
        if(SWAP_D(pItem->data_length) != BufLen) {
            return PNIO_ERR_VALUE_LEN;
        }
    }
    else {
        // module slice access is on (is allowed),user must be able
        // to write less bytes than the total module date length
        if(SWAP_D(pItem->data_length) < BufLen) {
            //BufLen = SWAP_D(pItem->data_length);    // corrected (decreased) 'tobe write len'
            return PNIO_ERR_VALUE_LEN;   // Fix: AP01103504
        }
    }

    /* in first stage must be one byte, therefore no locking */
    ASSERT(SWAP_D(pItem->iops_length) == 1);
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
    {
        ASSERT(SWAP_D(pItem->iocs_length) == 1);
    }

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC &&
        pIODUItem->adr_info.DMA_base != 0) {
        return IODU_ctrl_irt_data_write(pIODUItem, pItem, BufLen, pBuffer,
                                        IOlocState, pIOremState);
    }

    pData = (unsigned char *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset));

    /* write consistency is guaranteed for 2 byte aligned access (1 byte data + 1 byte iops) <- HW constrains
     * PL: correction for Ctrl Byte Slice access (CBSA): User can write 1 byte into the middle of the IO-module,
     * but IOPS must be written after the last module data byte.
     * (Note: IODUItem.data_length specifies always the whole available module length from the logical
     * io-address given by the user until the module end. IOPS is the next byte after the last module data byte)
     * => Optimized access is only possible for 'one byte modules'  or for CBSA for 'one byte write request'
     *    at the end of the module (i.e.: SWAP_D(pItem->data_length) == 1) !!!
     */
    if( ((((unsigned long )pData) & 1) == 0) &&
        (BufLen == 1) && (SWAP_D(pItem->data_length) == 1) && (SWAP_D(pItem->iops_length) == 1) ) {
        unsigned char mybuf[2];

        /* get consumer status */
        if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
          *pIOremState = PNIO_S_GOOD;
        else
          *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(
            cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
            SWAP_D(pItem->iocs_length));

        /* now be careful, compose one byte data + one byte status
           to one word on the stack */
        mybuf[0] = *((unsigned char *)pBuffer);
        IODU_SET_IOXS(&mybuf[1],
            SWAP_D(pItem->iops_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

        /* set data and provider status (1+1 byte) */
        *((short *)pData)=*((short *)mybuf);

        /* write output data in output cache, update output cache */
        if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){

            /* check correctness of destination area */
            if(SWAP_D(pItem->data_offset) < BegOfOutCache){
               /* prevent overwriting of memory befor pOutCache */
               return PNIO_ERR_INTERNAL;
            }

            memcpy((void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache),
                (void *)pBuffer, BufLen);

            /* provider status */
            IODU_SET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
                SWAP_D(pItem->data_length) - BegOfOutCache,
                SWAP_D(pItem->iops_length),
                IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
        }
    } else {
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

        /* get consumer status */
        if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
          *pIOremState = PNIO_S_GOOD;
        else
          *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
                            SWAP_D(pItem->iocs_length));

        /* Lock write */
        CnsBlockWrite(cp_info->EREG_base, SWAP_D(pItem->data_offset),
            SWAP_D(pItem->data_length) + SWAP_D(pItem->iops_length),
            CNS_MAX_POLL_TIMEOUT);

        /* set data */      /* pData==(cp_info->KRAM_base + SWAP_D(pItem->data_offset)) ,see above*/
        memcpy((void *)pData, (void *)pBuffer, BufLen);

        /* provider status  */
        IODU_SET_IOXS( ((PNIO_UINT8*)pData) + SWAP_D(pItem->data_length), SWAP_D(pItem->iops_length),
            IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

        /* Unlock write */
        CnsBlockFreeWrite(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);

        /* write output data in output cache, update output cache */
        if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){

            /* check correctness of destination area */
            if(SWAP_D(pItem->data_offset) < BegOfOutCache){
               /* prevent overwriting of memory befor pOutCache */
               return PNIO_ERR_INTERNAL;
            }

            memcpy((void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache),
               (void *)pBuffer, BufLen);

            /* provider status */
            IODU_SET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
                SWAP_D(pItem->data_length) - BegOfOutCache,
                SWAP_D(pItem->iops_length),
                IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
        }
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
    }


    return PNIO_OK;
}

void IODU_ctrl_set_device_ioxs(IODU_Item * pIODUHnd, PNIO_UINT16 dev_nr,
                               KRAMIOTLB_IN_OUT_TYPE subslot_type, PNIO_IOXS ioxs_status,
                               PNIO_UINT32 length_less_as)
{
  PNIO_UINT32 ioxs_offset, ioxs_length;
  PNIO_UINT8 * pIOBase = 0;

  #define SubSlotArrFixLength 128

  KRAMIOTLB_Item * SubSlotArrFix[SubSlotArrFixLength];

  KRAMIOTLB_Item ** SubSlotArr = SubSlotArrFix;
  PNIO_UINT32 i, item_cnt = SubSlotArrFixLength;

  PNIO_UINT8 SubSlotArrMustBeFreed = 0;

  KRAMIOTLB_GetHandleDeviceItems(pIODUHnd->Handle, dev_nr, subslot_type,
                                  &item_cnt, SubSlotArr, &(pIODUHnd->krtlbHeader));

  /* ASSERT(item_cnt < SubSlotArrFixLength); */
  if(item_cnt > SubSlotArrFixLength)
  {
    SubSlotArr = (KRAMIOTLB_Item **)malloc(sizeof(KRAMIOTLB_Item *) * item_cnt);
    ASSERT(SubSlotArr);
    SubSlotArrMustBeFreed = 1;

    KRAMIOTLB_GetHandleDeviceItems(pIODUHnd->Handle, dev_nr, subslot_type,
                                    &item_cnt, SubSlotArr, &(pIODUHnd->krtlbHeader));
  }

  for(i=0; i<item_cnt; i++)
  {
   if(SWAP_D(SubSlotArr[i]->data_length) >= length_less_as) continue;

   if(SWAP_D(SubSlotArr[i]->io_sync_type) == KRAMIOTLB_IO_SYNC && pIODUHnd->adr_info.DMA_base != 0)
      pIOBase = pIODUHnd->adr_info.DMA_base;
   else
      pIOBase = pIODUHnd->adr_info.KRAM_base;

   if(SWAP_D(SubSlotArr[i]->io_out_type) & KRAMIOTLB_IO_IN)
   {
     ioxs_offset = SWAP_D(SubSlotArr[i]->iocs_offset);
     ioxs_length = SWAP_D(SubSlotArr[i]->iocs_length);
   }
   else
   {
     ioxs_offset = SWAP_D(SubSlotArr[i]->data_offset) + SWAP_D(SubSlotArr[i]->data_length);
     ioxs_length = SWAP_D(SubSlotArr[i]->iops_length);
   }

   /* now set the ioxs state */
   IODU_SET_IOXS( pIOBase + ioxs_offset, ioxs_length, ioxs_status, PNIO_IOXS_DETECT_BY_CONTROLLER);

   /*printf("IODU_ctrl_set_device_ioxs %d/%d/%x(0-G,1-B) dev_nr=%d s=%d ss=%d"
          " typ=0x%x(1-I,2-O,4-PDEV,8-DDEX) log_addr=%d P_ioxs=0x%x\n",
           ioxs_status,
           IODU_GET_IOXS(pIOBase + ioxs_offset, ioxs_length),
           (unsigned char)((pIOBase + ioxs_offset)[0]),
           SubSlotArr[i]->device_ar, SubSlotArr[i]->slot_nr,
           SubSlotArr[i]->subslot_nr, SubSlotArr[i]->io_out_type,
           SubSlotArr[i]->log_addr, (unsigned long)(pIOBase + ioxs_offset));*/
  }

  if(SubSlotArrMustBeFreed == 1) {
    free(SubSlotArr);
  }
}

#if 0
#ifdef IO_ROUTER

/**************************************************************************
* F u n c t i o n:  IODU_ctrl_data_write_kram  (IO-Router extension P.L.)
*
* D e s c r i p t i o n :
*   This function is similar to the original IODU_ctrl_data_write function.
*   The only difference is: it dos not write data into the write-cache, it
*   writes only into the KRAM.
*   This funktion is used by the watchdog to update the KRAM without disturbing
*   the content of the cache which could be newer (olready updated by the user
*   calling the function PNIO_ctrl_data_write_cach.
*   See also IODU_ctrl_data_write
*
* A r g u m e n t s:
*                Handle - in, handle of the controller
*                log_addr - in, logical address
*                BufLen - in, Length of the data buffer
*                pBuffer - in, Pointer to the data buffer of the IO data to be written
*                IOlocState - in, Local status of the IO data to be written
*                pIOremState - out, Remote status of the read IO data
*
* Return Value: PNIO_OK, PNIO_ERR_xxx
*
***************************************************************************/
PNIO_UINT32 IODU_ctrl_data_write_kram(IODU_Item *pIODUItem, PNIO_UINT32  log_addr,
    PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT8 * pOutCache, PNIO_UINT32 BegOfOutCache)
{
    KRAMIOTLB_Item  *pItem = NULL;
    IODATA_adr_info *cp_info;
    KRAMIOTLB_Ret    kr_ret;
    unsigned char   *pData;

    if (pIODUItem->item_valid == ITEM_INVALID)
      return PNIO_ERR_WRONG_HND;

    cp_info = &pIODUItem->adr_info;

    /* validate in params */
    if(!pBuffer && BufLen)
        return PNIO_ERR_PRM_BUF;
    if(!pIOremState)
        return PNIO_ERR_PRM_RSTATE;

    /* get the controller item from  Logical address of the module */
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle, log_addr,
        KRAMIOTLB_IO_OUT, &pItem, &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL)
        return PNIO_ERR_UNKNOWN_ADDR;

    if(SWAP_D(pItem->data_length) != BufLen)
        return PNIO_ERR_VALUE_LEN;

    /* in first stage must be one byte, therefore no locking */
    ASSERT(SWAP_D(pItem->iops_length) == 1);
    if( !(SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX) )
    {
      ASSERT(SWAP_D(pItem->iocs_length) == 1);
    }

    if(SWAP_D(pItem->available) == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        *pIOremState = PNIO_S_BAD;
        return PNIO_ERR_NO_CONNECTION;
    }

#if 0 /* removed, router has no sync modules */

    if(SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_SYNC &&
        pIODUItem->adr_info.DMA_base != 0)
        return IODU_ctrl_irt_data_write(pIODUItem, pItem, BufLen, pBuffer,
                                        IOlocState, pIOremState);
#endif

    pData = (unsigned char *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset));
    if(((((unsigned long )pData) & 1) == 0) &&
        (BufLen == 1) && SWAP_D(pItem->iops_length) == 1) {
        unsigned char mybuf[2];

        /* get consumer status */
        if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
          *pIOremState = PNIO_S_GOOD;
        else
          *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(
            cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
            SWAP_D(pItem->iocs_length));

        /* now be careful, compose one byte data + one byte status
           to one word on the stack */
        mybuf[0] = *((unsigned char *)pBuffer);
        IODU_SET_IOXS(&mybuf[1],
            SWAP_D(pItem->iops_length), IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

        /* set data and provider status */
        *((short *)pData)=*((short *)mybuf);

#if 0
        /* write output data in output cache, update output cache */
        if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){

            /* check correctness of destination area */
            if(SWAP_D(pItem->data_offset) < BegOfOutCache){
               /* prevent overwriting of memory befor pOutCache */
               return PNIO_ERR_INTERNAL;
            }

            memcpy((void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache),
                (void *)pBuffer, BufLen);

            /* provider status */
            IODU_SET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
                SWAP_D(pItem->data_length) - BegOfOutCache,
                SWAP_D(pItem->iops_length),
                IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
        }
#endif /* #if 0 */
    } else {
#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_LOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_LOCK(pIODUItem->MutexObj);
#endif

        /* get consumer status */
        if( SWAP_D(pItem->io_out_type) & KRAMIOTLB_IO_DDEX )
          *pIOremState = PNIO_S_GOOD;
        else
          *pIOremState = (PNIO_IOXS)IODU_GET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->iocs_offset),
                            SWAP_D(pItem->iocs_length));

        /* Lock write */
        CnsBlockWrite(cp_info->EREG_base, SWAP_D(pItem->data_offset),
            SWAP_D(pItem->data_length) + SWAP_D(pItem->iops_length),
            CNS_MAX_POLL_TIMEOUT);

        /* set data */
        memcpy((void *)(cp_info->KRAM_base + SWAP_D(pItem->data_offset)),
                (void *)pBuffer, BufLen);

        /* provider status  */
        IODU_SET_IOXS(cp_info->KRAM_base + SWAP_D(pItem->data_offset) +
            SWAP_D(pItem->data_length), SWAP_D(pItem->iops_length),
            IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);

        /* Unlock write */
        CnsBlockFreeWrite(cp_info->EREG_base, CNS_MAX_POLL_TIMEOUT);

#if 0
        /* write output data in output cache, update output cache */
        if(pOutCache && SWAP_D(pItem->io_sync_type) == KRAMIOTLB_IO_ASYNC){

            /* check correctness of destination area */
            if(SWAP_D(pItem->data_offset) < BegOfOutCache){
               /* prevent overwriting of memory befor pOutCache */
               return PNIO_ERR_INTERNAL;
            }

            memcpy((void *)(pOutCache + SWAP_D(pItem->data_offset) - BegOfOutCache),
               (void *)pBuffer, BufLen);

            /* provider status */
            IODU_SET_IOXS(pOutCache + SWAP_D(pItem->data_offset) +
                SWAP_D(pItem->data_length) - BegOfOutCache,
                SWAP_D(pItem->iops_length),
                IOlocState, PNIO_IOXS_DETECT_BY_CONTROLLER);
        }
#endif /* #if 0 */

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
        DPR_INTERPROCESS_MUTEX_UNLOCK(pIODUItem->MutexObj);
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
        DPR_MUTEX_UNLOCK(pIODUItem->MutexObj);
#endif
    }


    return PNIO_OK;
} /* end of IODU_ctrl_data_write_kram */


#endif /* IO_ROUTER */
#endif

#endif /*KRAMIOTLB_SUPPORT_HOST_CONTROLLER*/


