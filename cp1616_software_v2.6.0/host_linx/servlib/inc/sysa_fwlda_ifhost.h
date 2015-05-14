/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  FILE NAME    : sysa_fwlda_ifhost.h                                       */
/*****************************************************************************/
/*  DESCRIPTION  : local download agent interface functions                  */
/*****************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraendert weitergegeben */
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

#ifndef SYSA_FWLDA_IFHOST_H
#define SYSA_FWLDA_IFHOST_H

#undef ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 1 )
 #define ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((packed))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt pniobase.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
    #define PNIO_CODE_ATTR __stdcall
#else
    #define PNIO_CODE_ATTR
#endif

#define FWLDA_SUBSYS_INDEX 0xA4

typedef enum {
  FWLDA_OP_UNKNOWN = 0,
  FWLDA_OP_DOWNLOAD_START_RQB, /* parameter: SDB nummber to download */
  FWLDA_OP_DOWNLOAD_START_CNF,
  FWLDA_OP_DOWNLOAD_SDB_RQB,
  FWLDA_OP_DOWNLOAD_SDB_CNF,   /* parameter: SDB data
                                  response must be sendet to initiater.
                                  cp1616-host library send that request if user
                                  application call PNIO_download_data(DLD_CONFIG) function */

  FWLDA_OP_DOWNLOAD_END_RQB,    /* parameter: FWLDA_DOWNLOAD_END_DAT */
  FWLDA_OP_DOWNLOAD_END_CNF,
  FWLDA_OP_COPY_RAM_TO_ROM_RQB, /* parameter  FWLDA_RQB::ovs_inst_id*/
  FWLDA_OP_COPY_RAM_TO_ROM_CNF,
  FWLDA_OP_START_MGT_RQB,
  FWLDA_OP_START_MGT_CNF,

  FWLDA_OP_GET_FW_INFO_RQB,
  FWLDA_OP_GET_FW_INFO_CNF,
  FWLDA_OP_GET_SDB_RQB,         /* obsolete -> FWLDA_OP_GET_SDB_XL_RQB  */
  FWLDA_OP_GET_SDB_CNF,
  FWLDA_OP_DELALL_SDBS_RQB,
  FWLDA_OP_DELALL_SDBS_CNF,
  FWLDA_OP_RESET_TO_FACTORY_RQB,
  FWLDA_OP_RESET_TO_FACTORY_CNF,

  FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM, /* additional 4 opcodes for XDBs segmented transfer */
  FWLDA_OP_DOWNLOAD_SDB_SEGM_CNF,
  FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM_END,
  FWLDA_OP_DOWNLOAD_SDB_SEGM_END_CNF,
  FWLDA_OP_DOWNLOAD_SDB_LAST_OPCODE , /* CP-DOWNLOAD-SDB interface OPCODEs END MARKER  == 23*/

  FWLDA_OP_GET_SDB_XL_RQB       = 24, /* */
  FWLDA_OP_GET_SDB_XL_SEG_CNF   = 25,
  //FWLDA_OP_GET_SDB_XL_NXT_CNF   = 26,
  //FWLDA_OP_GET_SDB_XL_END_CNF   = 27,

  FWLDA_OP_CPINFO_OPEN_RQB = 41, /* CPINFO interface */
  FWLDA_OP_CPINFO_OPEN_CNF,
  FWLDA_OP_CPINFO_CLOSE_RQB,
  FWLDA_OP_CPINFO_CLOSE_CNF,
  FWLDA_OP_CPINFO_REGISTR_EVENTTYPE_RQB,
  FWLDA_OP_CPINFO_REGISTR_EVENTTYPE_CNF,
  FWLDA_reserved_0,
  FWLDA_OP_CPINFO_NOTIFICATION, /* firmware inform host about registered events */
  FWLDA_OP_CPINFO_LAST_OPCODE , /* CPINFO interface OPCODEs END MARKER          */

  FWLDA_OP_CP_SET_TIME_RQB            = 50,
  FWLDA_OP_CP_SET_TIME_CNF            = 51,
  FWLDA_OP_CP_SET_TYPE_OF_STATION_RQB = 52,
  FWLDA_OP_CP_SET_TYPE_OF_STATION_CNF = 53,
  FWLDA_OP_CP_GET_FW_INFO_RQB         = 54,  /* see ServLib: SERV_CP_get_fw_info() */
  FWLDA_OP_CP_GET_FW_INFO_CNF         = 55,  /* see ServLib: SERV_CP_get_fw_info() */

} FWLDA_OPCODE;


typedef enum {
  SUBSYS_FWLDA_OK = 0x0000,
  SUBSYS_FWLDA_ERR_RESOURCE,
  SUBSYS_FWLDA_ERR_SEQ,
  SUBSYS_FWLDA_ERR_INTERNAL,  /* examine firmware trace to find error reason */
  SUBSYS_FWLDA_ERR_WRONG_HND,
  SUBSYS_FWLDA_ERR_ALREADY_DONE,
  SUBSYS_FWLDA_ERR_OPEN_NOT_ALLOWED,
  SUBSYS_FWLDA_ERR_ERROR_PARAM
} SUBSYS_FWLDA_RET_TYPE;

#define FWLDA_OVS_CP_PROJECT   0
#define FWLDA_OVS_CP_REMANENT  1
#define FWLDA_OVS_STM_PROJECT  3

typedef struct {
  DPR_UINT32      rqb_len;     /* sizeof(FWLDA_RQB) + additional data len */
  FWLDA_OPCODE    opcode;
  DPR_UINT32      ovs_inst_id; /* vmdid from XDB-file */
  DPR_UINT32      response;
} ATTR_PACKED FWLDA_RQB;

typedef struct {
  DPR_MSG_HDR dpr_hdr;      /* dpram channel header */
  FWLDA_RQB   fwlda_rqb;    /* fwlda header */
                            /* possible additional data */
} ATTR_PACKED FWLDA_DPR_RQB;

typedef struct {
  DPR_UINT32    sdb_cnt;
  /* DPR_UINT16   sdb_nr[sdb_cnt]; */
} ATTR_PACKED FWLDA_DOWNLOAD_END_DAT;

#define FWLDA_SDB_TIMESTAMP_LEN 6

typedef struct {
  DPR_UINT32    sdb_length;
  DPR_UINT16    ovs_inst; /* FWLDA_OVS_... */
  DPR_UINT16    nr;
  DPR_UINT16    context;
  DPR_CHAR      timestamp[FWLDA_SDB_TIMESTAMP_LEN];
  DPR_UINT16    reserved;
} ATTR_PACKED FWLDA_SDB_DAT;

typedef struct {
  DPR_UINT16    t1;
  DPR_UINT16    t2;
  DPR_UINT16    t3;
  DPR_UINT16    t4;
  DPR_UINT16    t5;
} ATTR_PACKED FWLDA_FW_VERS;

#define FWLDA_SER_NUMBER_LEN 16
#define FWLDA_MLFB_LEN       14

/* constants for FWLDA_GET_FW_INFO_DAT::cp_state */
#define FWLDA_CP_STATE_HWINIT           0x05     /* CP-Hardware-Initialization  */
#define FWLDA_CP_STATE_STARTUP          0x06     /* CP-initialization           */
#define FWLDA_CP_STATE_RUN              0x08     /* CP-RUN without errors       */
#define FWLDA_CP_STATE_STOPPING         0x02     /* CP goes from RUN to STOP    */
#define FWLDA_CP_STATE_STOP             0x03     /* CP-STOP without errors      */
#define FWLDA_CP_STATE_STOP_ERROR       0x04     /* CP-STOP with error          */
#define FWLDA_CP_STATE_FATAL_ERROR      0xFF     /* FATAL-ERROR in HW or FW     */

/* constants for FWLDA_GET_FW_INFO_DAT::ext_par Bitfield !!! */

#define FWLDA_EXTPAR_NO_CP_EXCHANGE     0x01     /* do not perform CP-EXCHANGE Behavior, do not restore FW and S7Config bei CP-Exchange */

typedef struct {
  DPR_UINT32       blk_len;
  FWLDA_FW_VERS    fw_version; /* "V[2.1.0.1.2] - five tags */
  DPR_CHAR         cp_ser_nr[FWLDA_SER_NUMBER_LEN]; /* unique hw identification id, written by hw production process */
  DPR_UINT16       hw_version;
  DPR_CHAR         reserved1;
  DPR_CHAR         reserved2;
  DPR_CHAR         cp_state; /* FWLDA_CP_STATE_... */

  DPR_CHAR         ext_par;  /* Bitfield, FWLDA_EXTPAR_NO_CP_EXCHANGE, ... */

  DPR_CHAR         mlfb[FWLDA_MLFB_LEN];
  DPR_CHAR         mac[6];

  DPR_CHAR         pnio_ip[4];
  DPR_CHAR         pnio_net_mask[4];
  DPR_CHAR         pnio_gw_ip[4];

  DPR_CHAR         reserved3[8];
  DPR_UINT16       sdb_cnt;
 /* FWLDA_SDB_DAT sdbs[sdb_cnt] */
} ATTR_PACKED FWLDA_GET_FW_INFO_DAT; /* 78 bytes */


/* FWLDA_CP_FW_INFO_TYPE:
 *     Structure to delivere fw parameters to SERV_CP_get_fw_info()
 *     function call. Values received from the firmware are copied
 *     to the ServLib user buffer: SERV_CP_FW_INFO_TYPE.
 */
typedef struct fwlda_cp_fw_info_s {
    DPR_UINT32       blk_len;
    FWLDA_FW_VERS    fw_version;
    DPR_CHAR         cp_ser_nr[FWLDA_SER_NUMBER_LEN]; /* max. 16 */
    DPR_CHAR         mlfb[FWLDA_MLFB_LEN];            /* 14 */
    DPR_UINT16       hw_version;
    DPR_CHAR         mac[6];
    DPR_UINT32       ip_addr;
    DPR_UINT32       ip_mask;
    DPR_UINT32       default_router;
    DPR_CHAR         station_name[256];
    DPR_CHAR         type_of_station[256];
} ATTR_PACKED FWLDA_CP_FW_INFO_TYPE;         /* total len= */


typedef struct {
  DPR_UINT32      blk_len;
  DPR_UINT32      subsytem_id; /* 0 - ip subsystem group, 1 - sdb-verteiler */
  DPR_UINT32      reserved1;
  DPR_UINT32      reserved2;
} ATTR_PACKED FWLDA_START_SUBSYSTEM;

typedef struct {
  DPR_UINT32      blk_len;
  DPR_UINT32      host_version;
  DPR_UINT32      reserved1[6];
} ATTR_PACKED FWLDA_CPINFO_OPEN;

typedef struct {
  DPR_UINT32      blk_len;
  DPR_UINT32      event_type;
  DPR_CHAR        reg_unreg; /* 0 - unregister, 1 - register */
  DPR_CHAR        reserved1[3];
  DPR_UINT32      reserved2[5];
} ATTR_PACKED FWLDA_CPINFO_REGISTER;

typedef struct {
  DPR_UINT32      blk_len;
  DPR_UINT32      event_type;

  /* event type, event data
  SERV_CIB_LED_STATE or
  SERV_CIB_PDEV_DATA or
  SERV_CIB_PDEV_INIT_DATA */

} ATTR_PACKED FWLDA_CPINFO_REPORT;

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#undef ATTR_PACKED


#endif /* SYSA_FWLDA_IFHOST_H */
