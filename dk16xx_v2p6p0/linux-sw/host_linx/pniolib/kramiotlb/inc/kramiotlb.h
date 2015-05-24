/*****************************************************************************
 DESCRIPTION: KRAM IO TABLE
    to perform io data exchange between firmware and host
 DATE:        29.11.2004
                                                                             */
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

#ifndef KRAMIOTLB_H
#define KRAMIOTLB_H

#define IODU_SOFTEDD  100
#define IODU_ERTEC200 200
#define IODU_ERTEC400 400

#include "kramiotlb_cfg.h"

typedef enum {
    KRAMIOTLB_OK = 0,
    KRAMIOTLB_ERR_MEMORY, /* not enouth memory */
    KRAMIOTLB_ERR_ADDR,   /* item with device_nr, module_nr, submodule_nr not found */
    KRAMIOTLB_ERR_PRM     /* parameter(s) wrong */
} KRAMIOTLB_Ret;

typedef enum {
    KRAMIOTLB_IO_SYNC,
    KRAMIOTLB_IO_ASYNC,

    KRAMIOTLB_IO_UNDEFSYNC = 0xFFFFFFFF /* used only for search functions, to find itemt independently of io type */
} KRAMIOTLB_IOSYNC_TYPE;

typedef enum {            /* ATTENTION BIT-FIELD !!! */
    KRAMIOTLB_IO_IN   = 0x01,
    KRAMIOTLB_IO_OUT  = 0x02,
    KRAMIOTLB_IO_PDEV = 0x04,
    KRAMIOTLB_IO_DDEX = 0x08,    /* DIRECT_DATA_EX; Ctrl <-> Ctrl communication */
    KRAMIOTLB_IO_ROUTER = 0x10  /* submodule belong exclusive to IO-ROUTER */
} KRAMIOTLB_IN_OUT_TYPE;

typedef enum {
    KRAMIOTLB_CONTR,
    KRAMIOTLB_DEVICE
} KRAMIOTLB_ITYPE;

typedef enum {
    KRAMIOTLB_SUBM_AVAILABLE,
    KRAMIOTLB_SUBM_NOT_AVAILABLE
} KRAMIOTLB_AVAILABLE_T;

typedef union {
    PNIO_UINT32     index;  /* for ERTEC200 EDD*/
    PNIO_UINT32     handle; /* for SOFT EDD */
} KRAMIOTLB_FRM_HND;        /* internal frame identifier */

typedef enum {
   KRAMIOTLB_VALID     = 0,
   KRAMIOTLB_NOT_VALID = 1
} KRAMIOTLB_VALIDITY_T;

typedef struct {
/* only for io-table managment */
    PNIO_UINT32                        used;       /* 1 if the item used, otherwise 0 */
    PNIO_UINT32                        iotlb_hndl; /* one unique for iotld handle     */

    PNIO_UINT16                        flags;      /* bit0 - io_validity, KRAMIOTLB_(NOT_)VALID
                                                      bit1-15 - reserved */
/* provided by pnio controller le */
    PNIO_UINT16                        api;        /* api number */

    KRAMIOTLB_AVAILABLE_T              available;  /* KRAMIOTLB_SUBM_(NOT)_AVAILABLE  remote IOX=GOOD(BAD) */
    PNIO_UINT32                        log_addr;   /* logical address
                                                      !!!with MSB set according to IO-type!!! */
    PNIO_UINT32                        reduct_ratio; /* irt reduction ratio */
    PNIO_UINT32                        phase;        /* irt phase */

/* provided by pnio device le */
    PNIO_UINT32                        device_ar;
    PNIO_UINT32                        slot_nr;    /* slot number, only for device interface */
    PNIO_UINT32                        subslot_nr; /* subslot number,only for device interface */

/* provided by both controller and device
   all offsets are relating to IO-KRAM Base address (IRTE_BASE) */
    KRAMIOTLB_IN_OUT_TYPE              io_out_type;  /* input, output */
    KRAMIOTLB_IOSYNC_TYPE              io_sync_type; /* access-type of data: sync(IRT) / async(RT) */

/* to read/write io-data and frame stutus
   lock from data_offset with lenght = data_length + iops_length */
    PNIO_UINT32                        data_offset; /* offset of data for edd lock for access */
    PNIO_UINT32                        data_length; /* length of IO data to be read or write */

                                       /* iops_offset = data_offset + data_length */
    PNIO_UINT32                        iops_length; /* length of iops */
    PNIO_UINT32                        consumer_frame_apdu_stat_offset;   /* offset to apdu status of consumer frame in K-RAM */

/* to read/write stutus from another frame
   lock from iocs_offset with lenght = iocs_length */
    PNIO_UINT32                        iocs_offset; /* offset of iocs for edd lock for access */
    PNIO_UINT32                        iocs_length; /* length of iocs */

    KRAMIOTLB_FRM_HND                  data_buffer; /* data/ps frame buffer internal handle */
    KRAMIOTLB_FRM_HND                  cs_buffer;   /* cs frame buffer internal handle */


} KRAMIOTLB_Item;

#define KRAMTLB_GET_IO_VALIDITY(pKRAMIOTRL_Item)               ((KRAMIOTLB_VALIDITY_T)(pKRAMIOTRL_Item->flags & SWAP_W(0x1)))
#define KRAMTLB_SET_IO_VALIDITY(pKRAMIOTRL_Item, io_validity)   if(io_validity == KRAMIOTLB_NOT_VALID){\
                                                                  pKRAMIOTRL_Item->flags |= SWAP_W(0x1);\
                                                                }else{ pKRAMIOTRL_Item->flags &= SWAP_W(0xFFFE);}

/* global parameters for iotlb */

/* KRAMIOTLB_Header: KRAM I/O table access control structure. It's initialised by KRAMIOTLB_init().
 *                   Pointer to this header is passed to every KRAM handling function.
 */
typedef struct {
    PNIO_UINT32    *pMaxItemPos; /* pointer to variable which holds the last currently used array index */
    KRAMIOTLB_Item *pFirstItem;  /* position of first item */
    KRAMIOTLB_Item *pLastItem;   /* last possible possition. I.e. ptr to the last (max. possible) item  */
} KRAMIOTLB_Header;

#ifdef __cplusplus
extern "C" {
#endif

/* host/target functions
   table memory must cleared in target -> memset(beginn, 0, lenght) */
void KRAMIOTLB_init(
        void             * p_iotlb,  /* in, beginning of the io table */
        unsigned long      iotlb_len,
        KRAMIOTLB_Header * kramHdr); /* in/out */

void KRAMIOTLB_deinit(
        KRAMIOTLB_Header * kramHdr); /* in/out */

KRAMIOTLB_Ret KRAMIOTLB_GetHandleItems(
        PNIO_UINT32           hnd,
        PNIO_UINT32       min_data_length, /* in, use 1 if you don't need diagnostic addresses */
        PNIO_UINT32         * items_count, /* in,out */
        KRAMIOTLB_Item     ** p_item,      /* out */
        const KRAMIOTLB_Header    * kramHdr);      /* in */

typedef int (*KRAMIOTLB_IsItemValid)
       (KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list,
        PNIO_UINT32           addition_cmp_list_len);

KRAMIOTLB_Ret KRAMIOTLB_GetHandleItemsAdvance(
        PNIO_UINT32           hnd,         /* wicht muss be equal to iotlb_hndl from KRAMIOTLB_Item */
        PNIO_UINT32           min_data_length, /* in, use 1 if you don't need diagnostic addresses */
        PNIO_UINT32         * items_count, /* in,out */
        KRAMIOTLB_Item     ** p_item,      /* out */
        KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, optional, default 0 */
        PNIO_UINT32           addition_cmp_list_len, /* in, optional, default 0 */
        KRAMIOTLB_IsItemValid p_fct_is_item_valid,   /* in, optional, default 0 */
        const KRAMIOTLB_Header    * kramHdr); /* in */


KRAMIOTLB_Ret KRAMIOTLB_GetHandleDeviceItems(
        PNIO_UINT32           hnd,
        PNIO_UINT32           device_nr,
        KRAMIOTLB_IN_OUT_TYPE io_type,     /* in */
        PNIO_UINT32         * items_count,  /* in,out */
        KRAMIOTLB_Item     ** p_item,       /* out */
        const KRAMIOTLB_Header    * kramHdr);      /* in */

KRAMIOTLB_Ret KRAMIOTLB_GetDataOffset(
        PNIO_UINT32           hnd,         /* in, user, application handle */
        KRAMIOTLB_IN_OUT_TYPE io_type,     /* in */
        KRAMIOTLB_IOSYNC_TYPE sync_type,   /* in */
        PNIO_UINT32         * min_data_offset, /* out */
        PNIO_UINT32         * max_data_offset, /* out */
        const KRAMIOTLB_Header    * kramHdr);  /* in */

KRAMIOTLB_Ret KRAMIOTLB_IsIoSyncConfigured(
        PNIO_UINT32           hnd,         /* in, user, application handle */
        KRAMIOTLB_IOSYNC_TYPE sync_type,   /* in */
        PNIO_UINT8          * configured,  /* out, 0 - no, 1 - yes */
        const KRAMIOTLB_Header    * kramHdr); /* in */

KRAMIOTLB_Ret KRAMIOTLB_FindItemByGeoAddr(
        PNIO_UINT32           hnd,            /* in, iobase instance */
        PNIO_UINT32           slot_nr,        /* in */
        PNIO_UINT32           subslot_nr,     /* in */
        KRAMIOTLB_IN_OUT_TYPE inout_type,   /* in */
        KRAMIOTLB_Item     ** p_item,         /* out */
        const KRAMIOTLB_Header    * kramHdr); /* in */

/* target functions */
KRAMIOTLB_Ret KRAMIOTLB_AddItem(
        PNIO_UINT32        hnd,            /* in */
        KRAMIOTLB_Item   * pItem,          /* in */
        const KRAMIOTLB_Header * kramHdr);       /* in */

KRAMIOTLB_Ret KRAMIOTLB_SetItemAvailable(
        PNIO_UINT32           hnd,         /* in */
        PNIO_UINT32           log_addr,    /* in */
        KRAMIOTLB_IN_OUT_TYPE io_type,     /* in */
        PNIO_UINT32           device_nr,   /* in */
        PNIO_UINT32           slot_nr,     /* in */
        PNIO_UINT32           subslot_nr,  /* in */
        KRAMIOTLB_AVAILABLE_T avail,       /* in */
        const KRAMIOTLB_Header    * kramHdr);    /* in */

KRAMIOTLB_Ret KRAMIOTLB_DelItemByLogaddr(
        PNIO_UINT32           hnd,         /* in */
        PNIO_UINT32           logaddr,     /* in */
        const KRAMIOTLB_Header    * kramHdr);    /* in */

KRAMIOTLB_Ret KRAMIOTLB_FreeItems(
        PNIO_UINT32           hnd,         /* in */
        const KRAMIOTLB_Header    * kramHdr);    /* in */

KRAMIOTLB_Ret KRAMIOTLB_FreeItemsByAr(
        PNIO_UINT32           hnd,         /* in */
        PNIO_UINT32           device_ar,   /* in */
        const KRAMIOTLB_Header    * kramHdr);    /* in */

/* host fuctions (controller) */
#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT

/* Create hash table */
KRAMIOTLB_Ret KRAMIOTLB_CreateContrHash(
        PNIO_UINT32           hnd,
        const KRAMIOTLB_Header    * kramHdr);

/* Free hash table */
KRAMIOTLB_Ret KRAMIOTLB_FreeContrHash(
        PNIO_UINT32           hnd,
        const KRAMIOTLB_Header    * kramHdr);

#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT */

KRAMIOTLB_Ret KRAMIOTLB_FindContrItemByLogAddr(
        PNIO_UINT32           hnd,          /* in, iobase instance */
        PNIO_UINT32           log_addr,     /* in */
        KRAMIOTLB_IN_OUT_TYPE inout_type,   /* in */
        KRAMIOTLB_Item     ** p_item,       /* out */
        const KRAMIOTLB_Header    * kramHdr);      /* in */

void KRAMIOTLB_SetDeviceItemsAvailable(
        PNIO_UINT32           hnd,          /* in, iobase instance */
        PNIO_UINT32           device_nr,    /* in */
        KRAMIOTLB_AVAILABLE_T avail,        /* in */
        const KRAMIOTLB_Header    * kramHdr);  /* in */


int KRAMIOTLB_IsItemValidContrHost(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of excluded submodules */
        PNIO_UINT32           addition_cmp_list_len);

#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER */

#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE

/* host function (device) */
KRAMIOTLB_Ret KRAMIOTLB_GetDeviceItems(
        PNIO_UINT32           hnd,
        KRAMIOTLB_IOSYNC_TYPE sync_type,    /* in */
        KRAMIOTLB_IN_OUT_TYPE inout_type,   /* in */
        PNIO_UINT32         * items_count,  /* in,out */
        KRAMIOTLB_Item     ** p_item,       /* out */
        KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, available items */
        PNIO_UINT32           addition_cmp_list_len, /* in, number of available */
        KRAMIOTLB_IsItemValid p_fct_is_item_valid, /* in */
        const KRAMIOTLB_Header    * kramHdr);      /* in */

int KRAMIOTLB_IsItemValidDeviceHost(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of available submodules */
        PNIO_UINT32           addition_cmp_list_len);

int KRAMIOTLB_IsItemValidWithoutIORouter(
        KRAMIOTLB_Item      * p_item_to_check,
        KRAMIOTLB_Item      * p_addition_cmp_list, /* list of available submodules */
        PNIO_UINT32           addition_cmp_list_len);

#endif /* KRAMIOTLB_SUPPORT_HOST_DEVICE */

#ifdef __cplusplus
}
#endif

#endif /* KRAMIOTLB_H */
