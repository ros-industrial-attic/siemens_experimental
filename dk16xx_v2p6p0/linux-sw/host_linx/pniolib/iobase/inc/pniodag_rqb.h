/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                pniodag_rqb.h                                     */
/*****************************************************************************/
/*  Dualport ring buffer interface functions                                 */
/*****************************************************************************/
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

#ifndef PNIODAG_RQB_H
#define PNIODAG_RQB_H

#undef ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 4 )
 #define  ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned(4)))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt pniodag_rqb.h header for your compiler
#endif

#define PNIOD_DREC_MAX_SIZE           4096
#define PNIOD_ALRM_DATA_MAX_SIZE      1408       /* will be used as a buffer length */

#define PNIOD_ALRM_USER_DATA_MAX_LEN (1432 - 28) /* maximum posible alarm user data length to be send */

typedef enum {
    PNIO_DAGET_RET_OK,
    PNIO_DAGENT_RET_OPEN_NOT_ALLOWED,
    PNIO_DAGENT_RET_REBOOT_AFTER_EXCEPTION,
    PNIO_DAGENT_RET_ERROR_PARAM = 7
} PNIO_DAGENT_RET_TYPE;

typedef enum {
    SCOD_OPEN_DEVICE = 100,    /* first 100 for Controller Agent */
    SCOD_CLOSE,
    SCOD_SET_DEV_STATE,
    SCOD_START,                /* PNIO_device_start */
    SCOD_SET_APPL_STATE_READY, /* PNIO_set_appl_state_ready */
    SCOD_DEVICE_AR_ABORT,         /* PNIO_device_ar_abort */
    SCOD_API_ADD,
    SCOD_API_REMOVE,
    SCOD_PLUG,         /* module or submodule */
    SCOD_PULL,         /* module or submodule */
    SCOD_BUILD_CH_PROP,
    SCOD_DIAG_CH_ADD,
    SCOD_DIAG_GENERIC_ADD,
    SCOD_DIAG_REMOVE, /* channel or generic or ext*/
    SCOD_ALARM_SEND, /* process or diag */
    SCOD_RET_SUB_ALARM_SEND,
    SCOD_STOP,
    SCOD_TEST_PING,
    SCOD_REGISTER_START_OP_CBF,
    SCOD_PLUG_SUB_EXT,
    SCOD_DIAG_EXT_CH_ADD,
    SCOD_PLUG_SUB_EXT_IM,
    SCOD_OPEN_DEVICE_EXT
} SYNCHD_CHNL_OP;

typedef enum {
    MCOD_CBF_LED = 100,        /* start, stop */
    MCOD_CBF_INFO,             /* AR_INDATA_IN, AR_ABORT_IND, AR_OFFLINE_IND */
    MCOD_CBF_APDU_STATUS_IND,
    MCOD_CBF_STOP_REQUEST,     /* user application have to deinitialize IO-Base */
    MCOD_CBF_PRM_END_IND,
    MCOD_CBF_DEVICE_STOPPED,
    MCOD_CBF_PULL_PLUG_CNF,
    MCOD_SET_IO_VALIDITY  /* IO-Base calls IODU_set_io_validity_data */
} MODED_CHNL_OP;

typedef enum {
    DCOD_CBF_REC_READ = 100,
    DCOD_CBF_REC_READ_RESPONSE,
    DCOD_CBF_REC_WRITE,
    DCOD_CBF_REC_WRITE_RESPONSE,
    DCOD_CBF_ALARM_ACK, /* PNIO_CBF_REQ_DONE */
    DCOD_CBF_ALARM_ACK_RESPONSE,
    DCOD_CBF_CHECK_IND,
    DCOD_CBF_CHECK_IND_RESPONSE,
} DATARECD_CHNL_OP;

typedef enum {
    ACOD_CBF_AR_INFO_IND = 100,
    ACOD_CBF_AR_CHECK_IND
} ALARMD_CHNL_OP;

inline const char *SCOD2S(SYNCHD_CHNL_OP c) {
    switch(c){
    case SCOD_OPEN_DEVICE:         return "SCOD_OPEN_DEVICE";
    case SCOD_CLOSE:               return "SCOD_CLOSE";
    case SCOD_SET_DEV_STATE:       return "SCOD_SET_DEV_STATE";
    case SCOD_START:               return "SCOD_START";
    case SCOD_SET_APPL_STATE_READY:return "SCOD_SET_APPL_STATE_READY";
    case SCOD_DEVICE_AR_ABORT:     return "SCOD_DEVICE_AR_ABORT";
    case SCOD_API_ADD:             return "SCOD_API_ADD";
    case SCOD_API_REMOVE:          return "SCOD_API_REMOVE";
    case SCOD_PLUG:                return "SCOD_PLUG";
    case SCOD_PULL:                return "SCOD_PULL";
    case SCOD_BUILD_CH_PROP:       return "SCOD_BUILD_CH_PROP";
    case SCOD_DIAG_CH_ADD:         return "SCOD_DIAG_CH_ADD";
    case SCOD_DIAG_GENERIC_ADD:    return "SCOD_DIAG_GENERIC_ADD";
    case SCOD_DIAG_REMOVE:         return "SCOD_DIAG_REMOVE";
    case SCOD_ALARM_SEND:          return "SCOD_ALARM_SEND";
    case SCOD_RET_SUB_ALARM_SEND:  return "SCOD_RET_SUB_ALARM_SEND";
    case SCOD_STOP:                return "SCOD_STOP";
    case SCOD_TEST_PING:           return "SCOD_TEST_PING";
    case SCOD_REGISTER_START_OP_CBF: return "SCOD_REGISTER_START_OP";
    case SCOD_PLUG_SUB_EXT:        return "SCOD_PLUG_SUB_EXT";
    case SCOD_DIAG_EXT_CH_ADD:     return "SCOD_DIAG_EXT_CH_ADD";
    case SCOD_OPEN_DEVICE_EXT:     return "SCOD_OPEN_DEVICE_EXT";
    default:                       return "????";
    }
}

inline const char *MCOD2S(MODED_CHNL_OP c) {
    switch(c){
    case MCOD_CBF_LED:               return "MCOD_CBF_LED";
    case MCOD_CBF_INFO:              return "MCOD_CBF_INFO";
    case MCOD_CBF_APDU_STATUS_IND:   return "MCOD_CBF_APDU_STATUS_IND";
    case MCOD_CBF_STOP_REQUEST:      return "MCOD_CBF_STOP_REQUEST";
    case MCOD_CBF_PRM_END_IND:       return "MCOD_CBF_PRM_END_IND";
    case MCOD_CBF_DEVICE_STOPPED:    return "MCOD_CBF_DEVICE_STOPPED";
    case MCOD_CBF_PULL_PLUG_CNF:     return "MCOD_CBF_PULL_PLUG_CNF";
    case MCOD_SET_IO_VALIDITY:       return "MCOD_SET_IO_VALIDITY";
    default:                         return "????";
    }
};

inline const char *ACOD2S(ALARMD_CHNL_OP c) {
    switch(c){
    case ACOD_CBF_AR_INFO_IND:       return "ACOD_CBF_AR_INFO_IND";
    case ACOD_CBF_AR_CHECK_IND:      return "ACOD_CBF_AR_CHECK_IND";
    default:                         return "????";
    }
};

inline const char *DCOD2S(DATARECD_CHNL_OP c) {
    switch(c){
    case DCOD_CBF_REC_READ:           return "DCOD_CBF_REC_READ";
    case DCOD_CBF_REC_READ_RESPONSE:  return "DCOD_CBF_REC_READ_RESPONSE";
    case DCOD_CBF_REC_WRITE:          return "DCOD_CBF_REC_WRITE";
    case DCOD_CBF_REC_WRITE_RESPONSE: return "DCOD_CBF_REC_WRITE_RESPONSE";
    case DCOD_CBF_ALARM_ACK:          return "DCOD_CBF_ALARM_ACK";
    case DCOD_CBF_ALARM_ACK_RESPONSE: return "DCOD_CBF_ALARM_ACK_RESPONSE";
    case DCOD_CBF_CHECK_IND:          return "DCOD_CBF_CHECK_IND";
    case DCOD_CBF_CHECK_IND_RESPONSE: return "DCOD_CBF_CHECK_IND_RESPONSE";
    default:                          return "????";
    }
};

/* values vor t_rqd_open_device::ext_agent_info */
#define EPRM_CP_STOP_REGISTERED    0x00000001
#define EPRM_PULL_PLUG_REGISTERED  0x00000002

typedef struct {
    PNIO_UINT32          cp_index;
    PNIO_UINT32          ext_prm;
    PNIO_UINT32          vendor_id;
    PNIO_UINT32          device_id;
    PNIO_UINT32          instance_id;
    PNIO_UINT32          max_ar;
    PNIO_ANNOTATION      dev_annot;
    PNIO_UINT32          ext_agent_info;
    PNIO_UINT32          dma_mem_addr;
    PNIO_UINT32          dma_mem_len;
} ATTR_PACKED t_rqd_open_device;

typedef struct {
    PNIO_UINT32          cp_index;
    PNIO_UINT32          ext_prm;
    PNIO_UINT32          vendor_id;
    PNIO_UINT32          device_id;
    PNIO_UINT32          instance_id;
    PNIO_UINT32          max_ar;
    PNIO_ANNOTATION      dev_annot;
    PNIO_UINT32          ext_agent_info;
    PNIO_UINT32          dma_mem_addr;
    PNIO_UINT32          dma_mem_len;
    PNIO_UINT32          host_version;
    PNIO_UINT32          reserved[4];
} ATTR_PACKED t_rqd_open_device_ext;

typedef struct {
    PNIO_UINT32          device_hnd; /* SCOD_OPEN_DEVICE */
} ATTR_PACKED t_resp_open_device;

typedef struct {
    PNIO_UINT32          dev_state;
} ATTR_PACKED t_rqd_set_dev_state;

typedef struct {
    PNIO_UINT32          api;
    PNIO_UINT16          max_slots;
    PNIO_UINT16          max_subslots;
} ATTR_PACKED t_rqd_api_add;

typedef struct {
    PNIO_UINT32          api;
} ATTR_PACKED t_rqd_api_remove;

typedef struct {
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          ArCnt;
    PNIO_UINT32          ModulesCnt;
    PNIO_UINT32          SubmodulesCnt;

    /* data with valiable length
    PNIO_APPL_READY_AP_TYPE  Ars[ArCnt];
    PNIO_APPL_READY_MODULE_TYPE Modules[ModulesCnt];
    PNIO_APPL_READY_SUBMODULE_TYPE Submodules[SubmodulesCnt];
    */
} ATTR_PACKED t_rqd_set_appl_state_ready;

typedef struct {
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
} ATTR_PACKED t_rqd_device_ar_abort;

typedef struct {
    PNIO_UINT8           obj_type; /* 0 - module, 1 - submodule */
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
} ATTR_PACKED t_rqd_pull;

typedef struct {
    PNIO_UINT8           obj_type; /* 0 - module, 1 - submodule */
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT32          Ident;
    PNIO_UINT32          AlarmType; /* only for SCOD_PLUG_SUB_WRONG */
    PNIO_PLUG_IM0_BITS   IM0_bits;
} ATTR_PACKED t_rqd_plug;

typedef struct {
    PNIO_UINT16          Type;
    PNIO_UINT16          Spec;
    PNIO_UINT16          Dir;
} ATTR_PACKED t_rqd_build_channel_properties;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT16          ChannelNum;
    PNIO_UINT16          ChannelProp;
    PNIO_UINT32          ChannelErrType;
    PNIO_UINT16          DiagTag;
} ATTR_PACKED t_rqd_diag_channel_add;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT16          ChannelNum;
    PNIO_UINT16          ChannelProp;
    PNIO_UINT32          ChannelErrType;
    PNIO_UINT16          DiagTag;

    PNIO_UINT16          ExtChannelErrType;
    PNIO_UINT32          ExtChannelAddValue;
} ATTR_PACKED t_rqd_diag_ext_channel_add;


typedef struct {
    PNIO_UINT8           diag_type; /* 0 - channel, 1 - generic , 2 - ext_channel */
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT16          DiagTag;
} ATTR_PACKED t_rqd_diag_remove;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT16          ChannelNum;
    PNIO_UINT16          ChannelProp;
    PNIO_UINT16          DiagTag;
    PNIO_UINT16          UserStructIdent;
    PNIO_UINT32          InfoDataLen;
    PNIO_UINT8           InfoData[PNIOD_ALRM_DATA_MAX_SIZE];
} ATTR_PACKED t_rqd_diag_generic_add;

typedef struct {
    PNIO_UINT8           alarm_type; /* 0 - process, 1 - diag */
    PNIO_UINT32          Api;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          AlarmState;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT16          UserStructIdent;
    PNIO_UINT32          UserHndl;

    PNIO_UINT32          DataLen;
    PNIO_UINT8           Data[PNIOD_ALRM_DATA_MAX_SIZE];
} ATTR_PACKED t_rqd_alarm_send;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT32          UserHndl;
} ATTR_PACKED t_rqd_ret_of_sub_alarm_send;

typedef struct {
    PNIO_UINT32          DataLen;
    PNIO_UINT8           Data[64];
} ATTR_PACKED t_rqd_test_ping;

typedef enum {
    AR_INDATA  = 0,
    AR_ABORT   = 1,
    AR_OFFLINE = 2
} MCOD_INFO_TYPE;

typedef struct {
    PNIO_UINT8           ArIndType; /* AR_INDATA, AR_ABORT, AR_OFFLINE */
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_AR_REASON       ReasonCode;
} ATTR_PACKED t_cbfd_ar_ind; /* AR_INDATA_IND, AR_ABORT_IND, AR_OFFLINE_IND */

typedef struct {
    PNIO_UINT32          UserHndl;
    PNIO_UINT32          Status;
} ATTR_PACKED t_cbfd_req_done;

typedef struct {
    PNIO_UINT32          UserHndl;
    PNIO_UINT32          Status;
    PNIO_ERR_STAT        PnioState;
} ATTR_PACKED t_cbfd_req_done_resp;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT32          ModIdent;
    PNIO_UINT16          ModState;
    PNIO_UINT32          SubIdent;
    PNIO_UINT16          SubState;
} ATTR_PACKED t_cbfd_check_ind;

typedef struct {
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_APDU_STATUS_IND ApduStatus;
} ATTR_PACKED t_cbfd_apdu_status_ind;

typedef struct {
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          Api;
    PNIO_UINT16          SlotNr;
    PNIO_UINT16          SubslotNr;
} ATTR_PACKED t_cbfd_prm_end_ind;

typedef struct {
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          Api;
    PNIO_UINT16          SlotNr;
    PNIO_UINT16          SubslotNr;
    PNIO_UINT32          Response;
} ATTR_PACKED t_cbfd_prm_end_ind_resp;

typedef struct {
    PNIO_UINT32          Start; /* if(Start) call StartLedFlash(Frequency)
                                else      call StopLedFlash() */
    PNIO_UINT32          Frequency;
} ATTR_PACKED t_cbfd_led;

/*typedef struct {
    PNIO_DEV_ALARM_DATA al_blk;
    PNIO_UINT8           user_adata[PNIOD_ALRM_MAX_SIZE];
} ATTR_PACKED t_cbfd_alarm_ind;*/

typedef struct {
    PNIO_UINT32          BlkLen;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          ArDataLen; /* length of data after PNIO_AR_TYPE Ar */
    PNIO_AR_TYPE         Ar;

    /* packed ArData follow, with variable structure */
    /* PNIO_IOCR_TYPE IoCrList[IoCrListLen] */
    /* PNIO_MODULE_TYPE ModList[xx] */
    /* PNIO_SUBMOD_TYPE PNIO_SUBMOD_TYPE[yy] */

} ATTR_PACKED t_cbfd_ar_info_ind;

typedef struct {
    PNIO_UINT32          HostIp;
    PNIO_UINT16          ArType;
    PNIO_UUID_TYPE       ArUUID;
    PNIO_UINT32          ArProperties;
    PNIO_UUID_TYPE       CmiObjUUID;
    PNIO_UINT16          CmiStationNameLenght;
    PNIO_UINT8           CmiStationName[128];
    PNIO_UINT32          ArDataLen; /* length of data after PNIO_AR_TYPE Ar */
    PNIO_AR_TYPE         Ar;

    /* packed ArData follow, with variable structure */
    /* PNIO_IOCR_TYPE IoCrList[IoCrListLen] */
    /* PNIO_MODULE_TYPE ModList[xx] */
    /* PNIO_SUBMOD_TYPE PNIO_SUBMOD_TYPE[yy] */

} ATTR_PACKED t_cbfd_ar_check_ind;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          SequenceNum;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT32          RecordIndex;
    PNIO_UINT32          BufLen;
} ATTR_PACKED t_cbfd_rec_read;

typedef struct {
    PNIO_UINT32          SequenceNum;
    PNIO_ERR_STAT        PnioState;
    PNIO_UINT32          BufLen;
    PNIO_UINT8           Buffer[PNIOD_DREC_MAX_SIZE];
} ATTR_PACKED t_cbfd_rec_read_resp;

typedef struct {
    PNIO_UINT32          Api;
    PNIO_UINT16          ArNumber;
    PNIO_UINT16          SessionKey;
    PNIO_UINT32          SequenceNum;
    PNIO_DEV_ADDR        Addr;
    PNIO_UINT32          RecordIndex;
    PNIO_UINT32          BufLen;
    PNIO_UINT8           Buffer[PNIOD_DREC_MAX_SIZE];
} ATTR_PACKED t_cbfd_rec_write;

typedef struct {
    PNIO_UINT32          SequenceNum;
    PNIO_ERR_STAT        PnioState;
    PNIO_UINT32          BufLen;
} ATTR_PACKED t_cbfd_rec_write_resp;

typedef struct {
    PNIO_UINT32          Reserved;
} ATTR_PACKED t_cbfd_dev_stoppped;

typedef struct {
  PNIO_UINT32            Api;
  PNIO_DEV_ACTION        Action;
  PNIO_DEV_ADDR          Addr;
  PNIO_UINT32            ErrorCode;
} ATTR_PACKED t_cbfd_pull_plug_cnf;

typedef struct {
  PNIO_DEV_ADDR          Addr;
  PNIO_UINT32            Validity; /* 1- valid, 0-invalid */
} ATTR_PACKED t_intern_set_io_validity;

typedef struct PNIOD_SYNCH_CHNL {
    PNIO_UINT32          blk_len;
    SYNCHD_CHNL_OP       opcode;
    PNIO_UINT32          handle;    /* in, from IO-Base lib */
    PNIO_DAGENT_RET_TYPE agent_ret; /* if == PNIO_DAGET_RET_OK, analyse resp_ret */
    PNIO_UINT32          resp_ret;

    union {
        t_rqd_open_device         open_device;
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
        t_rqd_diag_generic_add          diag_gen_add;
        t_rqd_diag_remove               diag_remove;
        t_rqd_alarm_send                alarm_send;
        t_rqd_ret_of_sub_alarm_send     ret_of_sub_alarm_send;
        t_rqd_test_ping                 test_ping;
    } u;

} ATTR_PACKED T_SYNCHD_CHNL;

#define HDR_SHCHNL_LEN 20

typedef struct {
    PNIO_UINT32          blk_len;
    MODED_CHNL_OP        opcode;
    PNIO_UINT32          handle;    /* in, from IO-Base lib */

    union {
        t_cbfd_ar_ind            ar_ind;
        t_cbfd_apdu_status_ind   apdu_status_ind;
        t_cbfd_led               led;
        t_cbfd_prm_end_ind       prm_end_ind;
        t_cbfd_dev_stoppped      dev_stopped;
        t_cbfd_pull_plug_cnf     pull_plug_cnf;
        t_intern_set_io_validity set_io_validity;
    } u;

} ATTR_PACKED T_MODED_CHNL;

#define HDR_MDCHNL_LEN 12

typedef struct {
    PNIO_UINT32          blk_len;
    DATARECD_CHNL_OP     opcode;
    PNIO_UINT32          handle;     /* in, from IO-Base lib */

    union {
        t_cbfd_rec_read          rec_read;
        t_cbfd_rec_read_resp     rec_read_resp;
        t_cbfd_rec_write         rec_write;
        t_cbfd_rec_write_resp    rec_write_resp;
        t_cbfd_req_done          req_done;
        t_cbfd_req_done_resp     req_done_resp;
        t_cbfd_check_ind         check_ind;
        t_cbfd_check_ind         check_ind_resp;
    } u;
} ATTR_PACKED T_DATAREC_CHNL;

#define HDR_DRCHNL_LEN 12

typedef struct {
    PNIO_UINT32          blk_len;
    ALARMD_CHNL_OP       opcode;
    PNIO_UINT32          handle;    /* in, from IO-Base lib */

    union {
        t_cbfd_ar_info_ind  ar_info_ind;
        t_cbfd_ar_check_ind ar_check_ind;
    } u;
} ATTR_PACKED T_ALARM_CHNL;

#define HDR_ALCHNL_LEN 12

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#undef ATTR_PACKED

#endif /* PNIODAG_RQB_H */
