/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : krtlb_host.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : KRAM IO table module.
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
#define NOTRC
#include "string.h"
#include "kramiotlb.h"

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH

KRAMIOTLB_Item **contrHashTlb = 0;

/* Create hash table */
KRAMIOTLB_Ret KRAMIOTLB_CreateContrHash(PNIO_UINT32 hnd, const KRAMIOTLB_Header *header)
{
    KRAMIOTLB_Item *pTmpItem, *pCurMaxItem = &header->pFirstItem[SWAP_D(*header->pMaxItemPos)];
    long validitemcnt = 0;

    contrHashTlb = (KRAMIOTLB_Item **)malloc(sizeof(contrHashTlb[0]) * 0xFFFF);

    if(!contrHashTlb)
      return KRAMIOTLB_ERR_MEMORY;

    memset(contrHashTlb, 0, sizeof(contrHashTlb[0]) * 0xFFFF);

    for (pTmpItem = header->pFirstItem;
         pTmpItem <= pCurMaxItem;
         pTmpItem++) {
        if (SWAP_D(pTmpItem->iotlb_hndl) == hnd  &&
             SWAP_D(pTmpItem->used) == 1) {
            ASSERT(SWAP_D(pTmpItem->log_addr) < 0xFFFF);
            KTLB_TRC_2("KRAMIOTLB_CreateContrHash item hnd=0x%08x log_addr=0x%08x",
                hnd, (PNIO_UINT32)SWAP_D(pTmpItem->log_addr));
            contrHashTlb[SWAP_D(pTmpItem->log_addr)] = pTmpItem;
            validitemcnt++;
        }
    }

    KTLB_TRC_2("hash tlb created hnd=0x%08x validitemcnt=%ld", hnd, validitemcnt);

    return KRAMIOTLB_OK;
}

/* Free hash table */
KRAMIOTLB_Ret KRAMIOTLB_FreeContrHash(PNIO_UINT32 hnd, const KRAMIOTLB_Header *header)
{
    KTLB_TRC_1("hash tlb freed hnd=0x%08x", hnd);

    if(contrHashTlb)
    {
      free(contrHashTlb);
      contrHashTlb = 0;
    }

    return KRAMIOTLB_OK;
}

#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH */

/* host fuctions */
KRAMIOTLB_Ret KRAMIOTLB_FindContrItemByLogAddr(
    PNIO_UINT32 hnd,                  /* in, iobase instance */
    PNIO_UINT32 log_addr,             /* in */
    KRAMIOTLB_IN_OUT_TYPE inout_type, /* in */
    KRAMIOTLB_Item **p_item,          /* out */
    const KRAMIOTLB_Header *header)   /* in */
{
#ifndef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH
    KRAMIOTLB_Item *pTmpItem, *pCurMaxItem = &header->pFirstItem[SWAP_D(*header->pMaxItemPos)];
#endif
    *p_item = NULL;

    ASSERT(p_item);
    KTLB_TRC_2("find item hnd=0x%08x log_addr=0x%08x", hnd, log_addr);


#ifndef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH
    for (pTmpItem = header->pFirstItem;
         pTmpItem <= pCurMaxItem;
         pTmpItem++) {
        if (SWAP_D(pTmpItem->log_addr) == log_addr &&
            SWAP_D(pTmpItem->iotlb_hndl) == hnd &&
            (SWAP_D(pTmpItem->io_out_type) & inout_type) &&
            SWAP_D(pTmpItem->used) == 1) {
            *p_item = pTmpItem;
            break;
        }
    }
#else
    ASSERT(log_addr < 0xFFFF);
    if (inout_type == KRAMIOTLB_IO_OUT)
        log_addr = log_addr | 0x8000;
    if (contrHashTlb[log_addr])
        *p_item = contrHashTlb[log_addr];
#endif /* HASH_SORT */

    if (*p_item) {
        KTLB_TRC_2("found item: log_addr=0x%08x pointer=%p",log_addr, *p_item);
#ifndef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH
        KTLB_TRC_2("  slot=%03i, subslot=%03i",SWAP_D(pTmpItem->slot_nr), SWAP_D(pTmpItem->subslot_nr));
        KTLB_TRC_2("  iotype=%i, len=%03i",SWAP_D(pTmpItem->io_out_type), SWAP_D(pTmpItem->data_length));
        KTLB_TRC_1("  offset=0x%x",SWAP_D(pTmpItem->data_offset));
#endif
    }
    else {
        KTLB_TRC_1("KRAMIOTLB_FindContrItemByLogAddr: not found: log_addr=0x%08x WARNING",log_addr);
    }

    return KRAMIOTLB_OK;
}

void KRAMIOTLB_SetDeviceItemsAvailable(
        PNIO_UINT32           hnd,          /* in */
        PNIO_UINT32           device_nr,    /* in */
        KRAMIOTLB_AVAILABLE_T avail,        /* in */
        const KRAMIOTLB_Header     *header)       /* in */
{
    KRAMIOTLB_Item *pTmpItem, *pCurMaxItem = &header->pFirstItem[SWAP_D(*header->pMaxItemPos)];
    for (pTmpItem = header->pFirstItem;
         pTmpItem <= pCurMaxItem;
         pTmpItem++) {
        if (SWAP_D(pTmpItem->iotlb_hndl) == hnd &&
            SWAP_D(pTmpItem->device_ar) == device_nr &&
            SWAP_D(pTmpItem->used) == 1) {
            pTmpItem->available = avail;
        }
    }
}


#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER */

#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE

/* host function (device) */
KRAMIOTLB_Ret KRAMIOTLB_GetDeviceItems(
    PNIO_UINT32 hnd,
    KRAMIOTLB_IOSYNC_TYPE sync_type,  /* in */
    KRAMIOTLB_IN_OUT_TYPE inout_type, /* in */
    PNIO_UINT32 *items_count,         /* in,out */
    KRAMIOTLB_Item **p_item,          /* out */
    KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, optional, default 0 */
    PNIO_UINT32           addition_cmp_list_len, /* in, optional, default 0 */
    KRAMIOTLB_IsItemValid p_fct_is_item_valid, /* in, optional, default 0 */
    const KRAMIOTLB_Header *header)         /* in */
{
    unsigned long RealNumber = 0;
    KRAMIOTLB_Item *pTmpItem, *pCurMaxItem = &header->pFirstItem[SWAP_D(*header->pMaxItemPos)];
    KRAMIOTLB_Ret ret = KRAMIOTLB_OK;

    if(!items_count)
      return KRAMIOTLB_ERR_PRM;

    for (pTmpItem = header->pFirstItem;
         pTmpItem <= pCurMaxItem;
         pTmpItem++) {

        KTLB_TRC_2("exists s00ss %08x io_t %u",
                   ((pTmpItem->slot_nr)<<16)|pTmpItem->subslot_nr, pTmpItem->io_out_type);

        if ( (  sync_type == KRAMIOTLB_IO_UNDEFSYNC ||
                (KRAMIOTLB_IOSYNC_TYPE)SWAP_D(pTmpItem->io_sync_type) == sync_type ) &&

             SWAP_D(pTmpItem->iotlb_hndl) == hnd &&

             ((KRAMIOTLB_IN_OUT_TYPE)SWAP_D(pTmpItem->io_out_type) & inout_type) &&

             SWAP_D(pTmpItem->used) == 1 &&

             (p_fct_is_item_valid == 0 || /* no function, no validation */
               p_fct_is_item_valid(pTmpItem, p_addition_cmp_list, addition_cmp_list_len))
                                                                                          )
        {
                RealNumber++;
                if (p_item && *items_count > (RealNumber -1)) {

                   KTLB_TRC_2(" adds s00ss %08x io_t %u",
                                ((pTmpItem->slot_nr)<<16)|pTmpItem->subslot_nr, pTmpItem->io_out_type);
                   *p_item = pTmpItem;
                   (p_item)++;
                }
        }
    }

    *items_count = RealNumber;
    KTLB_TRC_2("get device items hnd=0x%08x sync_type=%d", hnd, sync_type);
    KTLB_TRC_2("                 inout_type=%d real_items_cnt=%ld", inout_type, RealNumber);

    return ret;
}

int KRAMIOTLB_IsItemValidContrHost(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of excluded submodules */
        PNIO_UINT32           addition_cmp_list_len)
{
  unsigned long i;
  if(addition_cmp_list_len == 0) return 1; /* valid, list of excluded submodules is empty */

  if(!p_addition_cmp_list ){
    /* if list not empty -> addition_cmp_list_len != 0, p_addition_cmp_list can not be = 0 */
    return KRAMIOTLB_ERR_PRM;
  }

  if(p_item_to_check == 0) return KRAMIOTLB_ERR_ADDR;

  for(i = 0; i< addition_cmp_list_len; i++)
  {
    if(SWAP_D(p_item_to_check->log_addr) == p_addition_cmp_list[i].log_addr &&
        (KRAMIOTLB_IN_OUT_TYPE)SWAP_D(p_item_to_check->io_out_type) == p_addition_cmp_list[i].io_out_type){
      /* this item must be excluded */
      /* printf("KRAMIOTLB_IsItemValidContrHost addr=%u not valid\n", p_item_to_check->log_addr); */
      return 0; /* not valid, must be excluded */
    }
  }

  return 1; /* valid */
}

int KRAMIOTLB_IsItemValidDeviceHost(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of available submodules */
        PNIO_UINT32           addition_cmp_list_len)
{
  unsigned long i;
  if(addition_cmp_list_len == 0 ||
      p_addition_cmp_list == 0 ) return 0; /* not valid, no list for validation */

  if(p_item_to_check == 0) return KRAMIOTLB_ERR_ADDR;

  for(i = 0; i< addition_cmp_list_len; i++)
  {
    if(SWAP_D(p_item_to_check->slot_nr) == p_addition_cmp_list[i].slot_nr &&
        SWAP_D(p_item_to_check->subslot_nr) == p_addition_cmp_list[i].subslot_nr &&
         (KRAMIOTLB_IN_OUT_TYPE)SWAP_D(p_item_to_check->io_out_type) == p_addition_cmp_list[i].io_out_type){
      /* this item is available in additional list */
      return 1; /* valid */
    }
  }

  /* printf("KRAMIOTLB_IsItemValidDeviceHost s=%u ss=%u in_out=%u\n",
       p_item_to_check->slot_nr, p_item_to_check->subslot_nr, p_item_to_check->in_out_type); */

  return 0; /* item was not found in available list, not valid */
}

int KRAMIOTLB_IsItemValidWithoutIORouter(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of available submodules */
        PNIO_UINT32           addition_cmp_list_len)
{
  addition_cmp_list_len = 0; /* not used */
  p_addition_cmp_list = 0;   /* not used */

  if(p_item_to_check == 0) return 0;

  if(SWAP_D(p_item_to_check->io_out_type) & KRAMIOTLB_IO_ROUTER){
    /* this item belong exclusive to IO_ROUTER therefore not valid */
    KTLB_TRC_2("item s00ss=0x%08x sync_type=%d not valid",
               (SWAP_D(p_item_to_check->slot_nr)<<16)|SWAP_D(p_item_to_check->subslot_nr), SWAP_D(p_item_to_check->io_sync_type));
    KTLB_TRC_1("                  io_dir=%x", SWAP_D(p_item_to_check->io_out_type));
    return 0; /* not valid */
  }
  else{
    KTLB_TRC_2("item s00ss=0x%08x sync_type=%d valid",
               (SWAP_D(p_item_to_check->slot_nr)<<16)|SWAP_D(p_item_to_check->subslot_nr), SWAP_D(p_item_to_check->io_sync_type));
    return 1; /* valid */
  }
}


#endif /* KRAMIOTLB_SUPPORT_HOST_DEVICE */
