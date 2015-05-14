/*******************************************************************
*
*  file: iodataupdate.h
*
*******************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN */
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

#ifndef _IODATAUPDATE_H_
#define _IODATAUPDATE_H_

#include "kramiotlb.h"

#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE
#include "pniousrd.h"
#endif

typedef enum {
    ITEM_INVALID =0,
    ITEM_VALID   =1
} IODU_VALIDITY;

typedef enum {
    WITHOUT_LOCK = 0,
    WITH_LOCK = 1,
} IODU_LOCK_TYPE;

#define IRT_ACCESS_STATUS_OUTSIDE  0x00
#define IRT_ACCESS_STATUS_INSIDE   0x01

typedef struct {
    PNIO_UINT8   *IOTLB_base; /* base pointer of the io data address table (kramiotlb) */
    PNIO_UINT32   iotlb_len;  /* length of io data address table (kramiotlb) */
    PNIO_UINT8   *EREG_base;  /* base pointer of the ERTEC register area */
    PNIO_UINT8   *KRAM_base;  /* base address for IO data in KRAM */
    PNIO_UINT8   *DMA_base;   /* base address for DMA, in FW = 0 */
    PNIO_UINT8   *IrtAccessStatus; /* if *(IODATA_adr_info::IrtAccessStatus) == IRT_ACCESS_STATUT_INSIDE, */
                                   /* then you are between OpStart and SetOpdone/OpFault */
                                   /* otherwise the access to irt data is unconsistent */
    KRAMIOTLB_Item *host_mem_KRAMTLB; /* memory to copy KRAMIOTLB from DPRAM to host user memory space
                                         (improve io performance) */
    PNIO_UINT32     host_mem_KRAMTLB_max_cnt; /* size of host_mem_KRAMTLB in KRAMIOTLB_Item elements */

} IODATA_adr_info;

typedef struct {
    PNIO_UINT32  l_offset; /* data offset from begin of io base */
    PNIO_UINT32  l_length; /* data length */

    PNIO_UINT32  d_cnt;    /* 0 - data boundary,
                                > 0 lock boundary, data_cnt count of folow data boundaries */
    PNIO_UINT32  d_offset; /* data offset from begin of io base */
    PNIO_UINT32  d_length; /* data length */
} IODU_CONS_BND;

#define IODU_MAX_SUBSLOT_CNT 256

typedef PNIO_IOXS    (*IODU_CBF_INIT_IOXS)  /* get initial IOXS status */
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_DEV_ADDR      * pAddr);         /* geographical address */

typedef struct {
    IODU_VALIDITY  item_valid;  /* 0 - invalid , 1 - valid */
    /* IODU_VALIDITY  io_validity; if(io_validity == ITEM_INVALID) return PNIO_ERR_SEQUENCE */
    PNIO_UINT32  cpid;        /* cp id */
    PNIO_UINT32  Handle;    /* handel for KRAMTLB of the item */
    PNIO_UINT32  UsrHandle; /* handle for user callbacks  */
    PNIO_UINT32  itemtyp;   /* device or controller */
    PNIO_UINT32  initStatPDEV; /* PDEVs is ardeady initialized (1) or not */

    IODATA_adr_info adr_info;
#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE
    PNIO_CBF_DATA_READ dataRead;    /* user callback function pointer for data read , for device only*/
    PNIO_CBF_DATA_WRITE dataWrite;    /* user callback function pointer for data write , for device only */
	IODU_CBF_INIT_IOXS  getInitPDEVStatus; /* get initial status for subslots without io data */ 
#endif
    PNIO_UINT32 nFlags;     /* DMA or KRAM access, ASYNC or SYNC */

    PNIO_UINT32   synINDevCount;
    KRAMIOTLB_Item  *synINDevLst[IODU_MAX_SUBSLOT_CNT]; /* array of pointer to KRAMIOTLB_Item */

    PNIO_UINT32 synOUTDevCount;
    KRAMIOTLB_Item  *synOUTDevLst[IODU_MAX_SUBSLOT_CNT]; /* array of pointer to KRAMIOTLB_Item */

    PNIO_UINT32 asynINDevCount;
    KRAMIOTLB_Item  *asynINDevLst[IODU_MAX_SUBSLOT_CNT]; /* array of pointer to KRAMIOTLB_Item */

    PNIO_UINT32 asynOUTDevCount;
    KRAMIOTLB_Item *asynOUTDevLst[IODU_MAX_SUBSLOT_CNT]; /* array of pointer to KRAMIOTLB_Item */

    KRAMIOTLB_Header krtlbHeader; /* header from DPRAM-based KRAMIOTLB */

#if defined(IODU_ENABLE_INTERPROCESS_LOCKING)
    DPR_INTERPROCESS_MUTEX MutexObj;
#elif defined(IODU_ENABLE_MULTIUSER_LOCKING)
    DPR_MUTEX MutexObj;
#endif
#if (IODU_ERTEC_TYPE == IODU_ERTEC200) || (IODU_ERTEC_TYPE == IODU_SOFTEDD)
    PNIO_UINT32 hDDB;
    PNIO_UINT32 DeviceConsumerID;
    PNIO_UINT32 DeviceProviderID;
    PNIO_UINT8 *lockedData;
    PNIO_UINT8 *lockedCS;
#endif
} IODU_Item;

#ifdef __cplusplus
extern "C" {
#endif

void IODU_Init(void);

IODU_Item *IODU_GetItem(PNIO_UINT32 cpid);

void IODU_SetItemEmpty(IODU_Item *pIODUItem);

/* to read input/output device/controller data */
PNIO_UINT32 IODU_kram_read(IODU_Item *pIODUItem, KRAMIOTLB_Item  *pItem, KRAMIOTLB_ITYPE item_type,
    PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen, PNIO_UINT8 *pBuffer,
    PNIO_IOXS  *pIOlocStateToRead,
    PNIO_IOXS  *pIOlocStateToSet, /* IO-lokale State will be set, if pIOlocStateToSet != 0 */
    PNIO_IOXS  *pIOremStateToRead);

/* to write output device/controller data */
PNIO_UINT32 IODU_kram_write(IODU_Item *pIODUItem, KRAMIOTLB_Item  *pItem, KRAMIOTLB_ITYPE item_type,
    PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocStateToSet, PNIO_IOXS * pIOremStateToRead);

#ifdef KRAMIOTLB_SUPPORT_HOST_CONTROLLER

PNIO_UINT32 IODU_ctrl_open(PNIO_UINT32 cpid,  /* in, cp index, FW = 1 */
    PNIO_UINT32 Handle,           /* in, handle, that was used by KRAMIOTLB_AddItem */
    PNIO_UINT32 nFlags,           /* in, nFlags = ExtPar from PNIO_controller/device_open */
    IODATA_adr_info *adr_info,    /* in, see iodataupdate.h */
    IODU_Item **ppIODUItem);      /* out, pointer to pointer to IO-Handle,            */
                                    /*      that will be used by all IODU_... functions */

PNIO_UINT32 IODU_ctrl_data_read(IODU_Item *pIODUItem,
    PNIO_UINT32  log_addr, PNIO_UINT32 BufLen,
    PNIO_UINT32 * pDataLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT8 * pOutCache, PNIO_UINT32 BegOfOutCache);

/* read io data (input or output) from KRAM */
PNIO_UINT32 IODU_ctrl_kram_read(IODU_Item *pIODUItem,
    PNIO_UINT32 log_addr, PNIO_IO_TYPE io_type, PNIO_UINT32 BufLen, PNIO_UINT32 *pDataLen,
    PNIO_UINT8 *pBuffer, PNIO_IOXS *pIOlocState, PNIO_IOXS *pIOremState);

PNIO_UINT32 IODU_ctrl_data_write(IODU_Item *pIODUItem,
    PNIO_UINT32  log_addr,  PNIO_UINT32 BufLen,
    PNIO_UINT8 * pBuffer, PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT8 * pOutCache, PNIO_UINT32 BegOfOutCache);

PNIO_UINT32 IODU_ctrl_remstat_read(IODU_Item *pIODUItem, PNIO_UINT32 log_addr,
    KRAMIOTLB_IN_OUT_TYPE  inoutt, PNIO_IOXS * pIOremState);

void IODU_ctrl_set_device_ioxs(IODU_Item * pIODUHnd, PNIO_UINT16 dev_nr,
                               KRAMIOTLB_IN_OUT_TYPE subslot_type, PNIO_IOXS ioxs_status,
                               PNIO_UINT32 length_less_as);

#ifdef IODU_SUPPORT_IO_CACHE_CONTROLLER

void IODU_ctrl_data_read_cache_refresh(IODU_Item *pIODUItem,
    PNIO_UINT8 *pInCache,
    IODU_CONS_BND *pInBndArr,
    PNIO_UINT32 InBndArrSize);

PNIO_UINT32 IODU_ctrl_data_read_ex(IODU_Item *pIODUItem, PNIO_UINT32 log_addr,
   PNIO_UINT32 BufLen, PNIO_UINT32 * pDataLen, PNIO_UINT8 * pBuffer,
   PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
   PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
   PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache);

#ifdef IO_ROUTER
/* read io data (input or output) from cache */
PNIO_UINT32 IODU_ctrl_CACHE_read(IODU_Item *pIODUItem, PNIO_UINT32 log_addr,
   PNIO_UINT32 BufLen, PNIO_UINT32 * pDataLen, PNIO_UINT8 * pBuffer,
   PNIO_IOXS *pIOlocState, PNIO_IOXS * pIOremState,
   PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
   PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache);
#endif /* IO_ROUTER */

void IODU_ctrl_data_write_cache_flush(IODU_Item *pIODUItem,
    PNIO_UINT8 *pOutCache,
    IODU_CONS_BND *pOutBndArr,
    PNIO_UINT32 OutBndArrSize);

PNIO_UINT32 IODU_ctrl_data_write_ex(IODU_Item *pIODUItem, PNIO_UINT32  log_addr,
    PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState, PNIO_IOXS * pIOremState,
    PNIO_UINT32 BegOfInCache, PNIO_UINT32 BegOfOutCache,
    PNIO_UINT8 *pInCache, PNIO_UINT8 * pOutCache);

PNIO_UINT32 IODU_ctrl_cache_init(
    IODU_Item *pIODUItem,
    PNIO_UINT8 **pOutCache,
    IODU_CONS_BND **pOutBndArr,
    PNIO_UINT32 *OutBndArrSize,
    PNIO_UINT8 **pInCache,
    IODU_CONS_BND **pInBndArr,
    PNIO_UINT32 *InBndArrSize,
    KRAMIOTLB_Item *ItemsListToExclude,
    PNIO_UINT32     ItemsListToExcludeSize);
#endif /* IODU_SUPPORT_IO_CACHE_CONTROLLER */

PNIO_UINT32 IODU_ctrl_close(IODU_Item *pIODUItem);
#endif /* KRAMIOTLB_SUPPORT_HOST_CONTROLLER */

#ifdef KRAMIOTLB_SUPPORT_HOST_DEVICE
PNIO_UINT32 IODU_dev_open(PNIO_UINT32 cpid,     /* in, cp index, FW = 1 */
    PNIO_UINT32 IodHandle,             /* in, handle, that was used by KRAMIOTLB_AddItem */
    PNIO_UINT32 UsrHandle,             /* in, handel, with them IODATAUPDATE calls dataRead_cbf, .. ; FW = IodHandle*/
    PNIO_CBF_DATA_READ dataRead_cbf,   /* in, cbf, to deliever input data to user */
    PNIO_CBF_DATA_WRITE dataWrite_cbf, /* in, cbf, to pick up output data from user */
    IODU_CBF_INIT_IOXS  initStatus_cbf, /* in, cbf, to get initial status for modules without io data */
    void *reserved2,                   /* reserved */
    PNIO_UINT32 nFlags,                /* in, nFlags = ExtPar from PNIO_controller/device_open */
    IODATA_adr_info *adr_info,         /* in, see iodataupdate.h */
    IODU_Item **ppIODUItem);           /* out, pointer to pointer to IO-Handle,            */
                                       /*      that will be used by all IODU_... functions */


PNIO_UINT32 IODU_update_subslot_tbl(IODU_Item *pIODUItem);

PNIO_UINT32 IODU_update_subslot_tbl_adv(IODU_Item *pIODUItem,
        KRAMIOTLB_Item      * p_addition_cmp_list,   /* in, optional, default 0 */
        PNIO_UINT32           addition_cmp_list_len, /* in, optional, default 0 */
        KRAMIOTLB_IsItemValid p_fct_is_item_valid);

PNIO_UINT32 IODU_intiate_data_read(IODU_Item *pIODUItem);

PNIO_UINT32 IODU_initiate_data_read_ext(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR *    pAddr,
    PNIO_ACCESS_ENUM   AccessType);

PNIO_UINT32 IODU_intiate_data_write(IODU_Item *pIODUItem);

PNIO_UINT32 IODU_initiate_data_write_ext(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR *    pAddr,
    PNIO_ACCESS_ENUM   AccessType);

/* read subslot input or output data for subslot spezified by pAddr
   unlocked for ERTER400,
   locked for ERTEC200,
   not supported for simultaneously FIRMWARE/HOST operation by ERTEC200 */
PNIO_UINT32 IODU_dev_read_subslot(IODU_Item *pIODUItem,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 BufLen, /* in, length of pBuffer */
    PNIO_UINT8 *pBuffer,
    PNIO_UINT32 *pDataLen, /* out, realy length of subslot data */
    PNIO_IOXS  *pIOlocStateToRead, /* out, destenation of local state */
    PNIO_IOXS  *pIOremStateToRead); /* out, destenation of remote state */

/* will be called from IOD by PNIO_sub_pull/plug 
   by PNIO_sub_pull io_validity = ITEM_INVALID
     IODU: set Consumer/Provider stutus to PNIO_BAD and 
           remove subslot from subslot update list
           IO-Base user don't gets callback for this subslot by PNIO_initialize_read/write
   by PNIO_sub_plug io_validity = ITEM_VALID
     IODU: put subslot in subslot update list
          IO-Base user gets callback for this subslot by PNIO_initialize_read/write */
PNIO_UINT32 IODU_set_io_validity_flag(IODU_Item *pIODUItem, 
                                 PNIO_DEV_ADDR * pAddr,
                                 IODU_VALIDITY io_validity);

PNIO_UINT32 IODU_set_io_validity_data(IODU_Item *pIODUItem, 
                                 PNIO_DEV_ADDR * pAddr,
                                 IODU_VALIDITY io_validity);

PNIO_UINT32 IODU_dev_close(IODU_Item *pIODUItem);
#endif /* KRAMIOTLB_SUPPORT_HOST_DEVICE */


#ifdef KRAMIOTLB_SUPPORT_TARGET

#ifdef IO_ROUTER

/* Functions for IO Router in Firmware */

PNIO_UINT32 IODU_router_open(PNIO_UINT32 cpid, PNIO_UINT32 Handle,
    PNIO_UINT32 nFlags, IODATA_adr_info *adr_info, IODU_Item **ppIODUItem);

PNIO_UINT32 IODU_router_close(IODU_Item *pIODUItem);

PNIO_UINT32 IODU_router_write_shadow(IODU_Item *pIODUItem, KRAMIOTLB_Item *pItem,
    PNIO_UINT32 BufLen, PNIO_UINT8 *pBuffer, PNIO_UINT8 *pShadow, PNIO_UINT8 *pStatus,
    PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState);

#endif /* IO_ROUTER */

#endif /* KRAMIOTLB_SUPPORT_TARGET */


#ifdef __cplusplus
}
#endif

#endif /* _IODATAUPDATE_H_ */
