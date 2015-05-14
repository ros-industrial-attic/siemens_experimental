/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                pnioag_rqb.h                                      */
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

#ifndef PNIOAG_RQB_H
#define PNIOAG_RQB_H

#undef  ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 4 )
 #define  ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned(4)))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt pnioag_rqb.h header for your compiler
#endif

#define PNIOI_DREC_MAX_SIZE 4096
#define PNIOI_ALRM_MAX_SIZE 1472

typedef enum {
    PNIO_AGETPC_RET_OK,
    PNIO_AGETPC_RET_CONFIG_INCONS,
    PNIO_AGETPC_RET_OPEN_NOT_ALLOWED,
    PNIO_AGETPC_RET_NO_PNIO_CONFIG,
    PNIO_AGETPC_RET_ALREADY_OPENED,
    PNIO_AGETPC_RET_NOT_OPENED,
    PNIO_AGETPC_RET_REBOOT_AFTER_EXCEPTION,
    PNIO_AGETPC_RET_ERROR_PARAM
} PNIO_AGETPC_RET_TYPE;

typedef enum {
    SCO_UNKNOWN,
    SCO_OPEN_CONTROLLER,
    SCO_SET_MODE,
    SCO_RW_DR,
    SCO_CLOSE,
    SCO_TRANSFORM_ADDR,
    SCO_CTRL_ALARM_CONFIRM,
    SCO_SET_DEV_STATE,
    SCO_REGISTER_STOP_CBF,
    SCO_TEST_PING,
    SCO_CTRL_DIAG,
    SCO_REGISTER_START_OP_CBF,
    SCO_OPEN_CONTROLLER_EXT,
} SYNCH_CHNL_OP;

typedef enum {
    MCO_UNKNOWN,
    MCO_CTRL_MODE_CHANGED,
    MCO_CTRL_DEV_ACT,
    MCO_STOP_REQUEST,  // user application have to deinitialize IO-Base,
                       // (PNIO_set_mode(OFFLINE), PNIO_close())
    MCO_LED
} MODE_CHNL_OP;

typedef enum {
     DREC_UNKNOWN,
     DREC_REC,         // datarecord
     DREC_CTRL_DIAG    // diagnostic data
} DREC_CHNL_OP;

//t_rq_open_ctrl::ext_prm
#define CTRL_OPEN_EPAR_DIRECT      0x00000000
#define CTRL_OPEN_EPAR_ALARMHND    0x00000001

inline const char *SCO2S(SYNCH_CHNL_OP c) {
    switch(c){
    case SCO_UNKNOWN:              return "SCO_UNKNOWN";
    case SCO_OPEN_CONTROLLER:      return "SCO_OPEN_CONTROLLER";
    case SCO_SET_MODE:             return "SCO_SET_MODE";
    case SCO_RW_DR:                return "SCO_RW_DR";
    case SCO_CLOSE:                return "SCO_CLOSE";
    case SCO_TRANSFORM_ADDR:       return "SCO_TRANSFORM_ADDR";
    case SCO_CTRL_ALARM_CONFIRM:   return "SCO_CTRL_ALARM_CONFIRM";
    case SCO_SET_DEV_STATE:        return "SCO_SET_DEV_STATE";
    case SCO_REGISTER_STOP_CBF:    return "SCO_REGISTER_STOP_CBF";
    case SCO_TEST_PING:            return "SCO_TEST_PING";
    case SCO_CTRL_DIAG:            return "SCO_CTRL_DIAG";
    case SCO_REGISTER_START_OP_CBF:return "SCO_REGISTER_START_OP_CBF";
    case SCO_OPEN_CONTROLLER_EXT:  return "SCO_OPEN_CONTROLLER_EXT";
    default:                       return "????";
    }
}

typedef struct {
    PNIO_UINT32       id;
    PNIO_UINT32       ext_prm;
    PNIO_UINT32       dma_mem_addr;
    PNIO_UINT32       dma_mem_len;
} ATTR_PACKED t_rq_open_ctrl;


typedef struct {
    PNIO_UINT32       id;
    PNIO_UINT32       ext_prm;
    PNIO_UINT32       dma_mem_addr;
    PNIO_UINT32       dma_mem_len;
    PNIO_UINT32       host_version;
    PNIO_UINT32       reserved[3];
} ATTR_PACKED t_rq_open_ctrl_ext;

typedef struct {
    PNIO_UINT8        EmergencyClose; // if 1 perform close without send confirm.
	  PNIO_UINT8        pad[3];
} ATTR_PACKED t_rq_close_ctrl;

typedef enum {
    PNAGC_READ_DR,
    PNAGC_WRITE_DR
} PNAGC_DR_RQ_TYPE;

typedef struct {
    PNIO_REF          UserRef;          // in
    PNAGC_DR_RQ_TYPE  ActionType;
    PNIO_ADDR         Addr;             // in
    PNIO_UINT32       RecordIndex;      // in
    PNIO_UINT32       Length;           // in
    PNIO_UINT8        Buffer[PNIOI_DREC_MAX_SIZE]; // in, out
} ATTR_PACKED t_read_write_dr;

typedef struct {
    PNIO_ERR_STAT     err;
    t_read_write_dr   rqb;
} ATTR_PACKED t_read_write_dr_resp;

typedef struct {
    PNIO_CTRL_DIAG    rqb;
    PNIO_UINT32       ErrorCode;
    //PNIO_UINT32     DiagDataBufferLen;
    //PNIO_UINT8*     pDiagDataBuffer;
    union {
        PNIO_UINT32           CycleTime;   // for PNIO_CTRL_DIAG_CONFIG_SUBMODULE_LIST
        PNIO_UINT32           DiagDataLen; // for PNIO_CTRL_DIAG_RESERVED
        PNIO_CTRL_DIAG_DEVICE DeviceState; // for PNIO_CTRL_DIAG_DEVICE_STATE
        PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO_DATA CtrlNameAddrInfo;
    } u;

} ATTR_PACKED t_ctrl_diag_resp;

typedef struct ctrl_alarm_confirm {
    PNIO_UINT32       ind_ref;
} ATTR_PACKED t_ctrl_alarm_confirm;

typedef struct {
    PNIO_UINT32       DataLen;
    PNIO_UINT8        Data[64];
} ATTR_PACKED t_rq_test_ping;

typedef struct {
    PNIO_UINT32       iodataupdate_hnd;
} ATTR_PACKED t_resp_open_ctrl;

typedef struct {
    PNIO_ADDR         Addr;
    PNIO_DEV_ACT_TYPE DevActMode;
} ATTR_PACKED t_rq_dev_act;

typedef struct {
    PNIO_UINT32       Start; /* if(Start) call StartLedFlash(Frequency)
                                else      call StopLedFlash() */
    PNIO_UINT32       Frequency;
} ATTR_PACKED t_rq_led;

typedef struct {
    PNIO_UINT32       blk_len;
    SYNCH_CHNL_OP     opcode;
    PNIO_UINT32       handle;   // in, will be set by IOBase Lib,
                            //     this handle got IOBase Lib from Agent by OPEN
                            //     in responce case value is undefind
    PNIO_AGETPC_RET_TYPE agent_ret;
    PNIO_UINT32       resp_ret; // only for response


    union {
        t_rq_open_ctrl            open_ctrl;
        t_rq_open_ctrl_ext        open_ctrl_ext;
        t_resp_open_ctrl          resp_open_ctrl;
        t_rq_close_ctrl           close_ctrl;
        PNIO_MODE_TYPE            set_mode;
        t_rq_dev_act              dev_state;

        t_read_write_dr           rw_dr;
        t_ctrl_alarm_confirm      ctrl_al_confirm;
        t_rq_test_ping            test_ping;
        PNIO_CTRL_DIAG            ctrl_diag;
    } u;

} ATTR_PACKED T_SYNCH_COMMAND;


typedef struct {
    PNIO_UINT32       blk_len;
    MODE_CHNL_OP      opcode;
    PNIO_UINT32       handle;   // internal Agent Handle
    PNIO_UINT32       resp_ret; // only for response
    union {
        PNIO_MODE_TYPE    set_mode;
        t_rq_dev_act      dev_state;
        t_rq_led          led;
    }u;

} ATTR_PACKED T_MODE_CHNL_DATA;

#define MODE_CH_HDR_LEN (4 * sizeof(PNIO_UINT32))

/* typedef struct {
    PNIO_UINT16 BlkType;
    PNIO_UINT16 BlkLen;
    PNIO_UINT16 BlkVersion;
    PNIO_UINT16 AlarmType;
    PNIO_UINT32 ApplicationProcessIdentifier;
    PNIO_UINT16 Slot;
    PNIO_UINT16 Subslot;
    PNIO_UINT32 ModulIdentification;
    PNIO_UINT32 SubmodulIdentification;
    PNIO_UINT16 AlarmSpecifier;
    PNIO_UINT16 UserStructureIdentifier;

} PNIO_ALARM_NORM_HEADER; byte packing */

/*typedef enum {
    PNIO_AOFF_BLKTYPE                        = 0,
    PNIO_AOFF_BLKLEN                         = 2,
    PNIO_AOFF_BLKVERSION                     = 4,
    PNIO_AOFF_ALARMTYPE                      = 6,
    PNIO_AOFF_APPLICATIONPROCESSIDENTIFIER   = 8,
    PNIO_AOFF_SLOT                           = 12,
    PNIO_AOFF_SUBSLOT                        = 14,
    PNIO_AOFF_MODULIDENTIFICATION            = 16,
    PNIO_AOFF_SUBMODULIDENTIFICATION         = 20,
    PNIO_AOFF_ALARMSPECIFIER                 = 24,
    PNIO_AOFF_USERSTRUCTUREIDENTIFIER        = 26,
    PNIO_ALARM_NORM_HEADER_OFFSETS_LEN       = 28

} PNIO_ALARM_NORM_HEADER_OFFSETS;*/

typedef struct {
    PNIO_UINT32       blk_len;
    PNIO_UINT32       handle; // internal Agent Handle

    PNIO_REF ind_ref;
    PNIO_ADDR            addr;
    PNIO_CTRL_ALARM_DATA rqb;
} ATTR_PACKED T_CTRL_RECEIVE_ALARM;

typedef  struct {
    PNIO_UINT32       blk_len;
    DREC_CHNL_OP      opcode;
    PNIO_UINT32       handle; // internal Agent Handle

    union {
      t_read_write_dr_resp rw_dr_resp;
      t_ctrl_diag_resp     diag_resp;
    } u;
} ATTR_PACKED T_DREC_CHNL_DATA;


#define PNIOI_SYNC_CHNL_SIZE sizeof(T_SYNCH_COMMAND)
#define PNIOI_MODE_CHNL_SIZE sizeof(T_MODE_CHNL_DATA)
#define PNIOI_DREC_CHLN_SIZE sizeof(T_DREC_CHNL_DATA)
#define PNIOI_ALRM_CHNL_SIZE sizeof(T_CTRL_RECEIVE_ALARM)

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#undef ATTR_PACKED

#endif /* PNIOAG_RQB_H */
