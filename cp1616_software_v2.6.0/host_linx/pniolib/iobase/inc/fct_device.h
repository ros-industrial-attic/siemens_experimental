/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : fct_device.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : Device class help structures
*****************************************************************************/
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

#include "pniousrd.h"

#undef ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 4 )
 #define ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt fct_device.h header for your compiler
#endif

typedef struct {
    PNIO_UINT32          blk_len;
    SYNCHD_CHNL_OP       opcode;
    PNIO_UINT32          handle;    /* in, from IO-Base lib */
    PNIO_DAGENT_RET_TYPE agent_ret; /* if == PNIO_DAGET_RET_OK, analyse resp_ret */
    PNIO_UINT32          resp_ret;

    union {
        t_rqd_open_device         open_device;   /* deprecated */
        t_rqd_open_device_ext     open_device_ext;
        t_resp_open_device        resp_open_device;
        t_rqd_set_dev_state        set_dev_state;
        t_rqd_api_add              api_add;
        t_rqd_api_remove           api_remove;
        t_rqd_set_appl_state_ready set_appl_state_ready;
        t_rqd_device_ar_abort      ar_abort;
        t_rqd_pull                 pull;
        t_rqd_plug                 plug;
        t_rqd_build_channel_properties  build_ch_prop;
        t_rqd_diag_channel_add          diag_ch_add;
        t_rqd_diag_ext_channel_add      diag_ext_ch_add;
        /*t_rqd_diag_generic_add          diag_gen_add;*/ 
        t_rqd_diag_remove               diag_remove;
        /*t_rqd_alarm_send                alarm_send;*/
        t_rqd_ret_of_sub_alarm_send     ret_of_sub_alarm_send;
        t_rqd_test_ping                 test_ping;
    } u;

} ATTR_PACKED light_T_SYNCHD_CHNL; /* this struct was defined to avoid stack consumption,
                                      t_rqd_diag_generic_add and t_rqd_alarm_send have big
                                      static element t_rqd_diag_generic_add::Infodata and
                                      t_rqd_alarm_send::data */

typedef struct {
    PNIO_UINT32 Flink;
    PNIO_UINT32 Blink;
} ATTR_PACKED fw_PNIO_LIST_ENTRY_TYPE;

typedef struct {
    fw_PNIO_LIST_ENTRY_TYPE  ap_list;
} ATTR_PACKED fw_PNIO_APPL_READY_LIST_TYPE;

typedef struct {
    fw_PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT32  api;
    fw_PNIO_LIST_ENTRY_TYPE  module_list;
} ATTR_PACKED fw_PNIO_APPL_READY_AP_TYPE;

typedef struct {
    fw_PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT16  slot_nr;
    fw_PNIO_LIST_ENTRY_TYPE  submodule_list;
} ATTR_PACKED fw_PNIO_APPL_READY_MODULE_TYPE;

typedef struct {
    fw_PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT16  subslot_nr;
} ATTR_PACKED fw_PNIO_APPL_READY_SUBMODULE_TYPE;

typedef  struct {
    PNIO_UINT32  SlotNum;                   /* related slot */
    PNIO_UINT32  SubslotNum;                /* related subslot */
    PNIO_UINT32  reserved;                  /* internal used */
} ATTR_PACKED fw_PNIO_IOCS_TYPE;

typedef struct {
    PNIO_UINT32             CycleTime;
    PNIO_IOCR_TYPE_ENUM     Direction;      /* see PNIO_IOCR_TYPE_ENUM */
    PNIO_IOCR_PROP_ENUM     IOCRProperties; /* see PNIO_IOCR_PROP_ENUM */

    PNIO_UINT32             SendClock;
    PNIO_UINT32             ReductioFactor;
    PNIO_UINT32             Phase;
    PNIO_UINT32             NumOfIoCs;      /* number of IoCs in IoCsList */
    PNIO_UINT32             pIoCsList;      /* ignore pointer to array of IOCS */
    PNIO_UINT32             reserved[3];    /* internal use only */
} ATTR_PACKED fw_PNIO_IOCR_TYPE;

typedef struct {
    PNIO_UINT32           SlotNum;        /* slot number */
    PNIO_UINT32           SubSlotNum;     /* subslot number */
    PNIO_UINT32           SubMProperties; /* module properties */
    PNIO_UINT32           SubMIdent;      /* module ident number */

    PNIO_UINT32           InDatLen;       /* length of the Input data */
    PNIO_UINT32           InIopsLen;      /* length of input provider status */
    PNIO_UINT32           InIocsLen;      /* length of input consumer status */

    PNIO_UINT32           OutDatLen;      /* length of the Output data */
    PNIO_UINT32           OutIopsLen;     /* length of output provider status */
    PNIO_UINT32           OutIocsLen;     /* length of output consumer status */

    PNIO_UINT32           reserved[8];    /* internal use only */

} ATTR_PACKED fw_PNIO_SUBMOD_TYPE;

typedef struct {
    PNIO_UINT32           Api;
    PNIO_UINT32           SlotNum;
    PNIO_UINT32           NumOfSubmod;
    PNIO_UINT32           ModProperties;
    PNIO_UINT32           ModIdent;
    PNIO_UINT32           pSubList;         /* ignore pointer */
} ATTR_PACKED fw_PNIO_MODULE_TYPE;

typedef struct
{
    PNIO_UINT32           pNextAr;          /* ignore pointer */
    PNIO_UINT16           ArNumber;
    PNIO_UINT16           SessionKey;
    PNIO_UINT32           NumOfIocr;
    PNIO_UINT32           pIoCrList;        /* ignore pointer */
    PNIO_UINT32           NumOfMod;
    PNIO_UINT32           pModList;         /* ignore pointer */
} ATTR_PACKED fw_PNIO_AR_TYPE;

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif
