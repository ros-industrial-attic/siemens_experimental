/****************************************************************************/
/*                                                                          */
/*    Copyright (C) SIEMENS CORP., 2015 All rights reserved.*/
/*                                                                          */
/*    file: PNIOUSRX.H                                                      */
/*                                                                          */
/*    Description:                                                          */
/*    data types and function declarations for IO-Base                      */
/*    controller user interface                                             */
/*                                                                          */
/****************************************************************************/

#ifndef PNIOUSRX_H
#define PNIOUSRX_H

#include "pniobase.h"

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
 #error please adapt pniousrx.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
    #define PNIO_CODE_ATTR __stdcall
#else
    #define PNIO_CODE_ATTR
#endif

/* constants specificaton */

#define PNIO_MAX_IO_LEN           254
#define PNIO_MAX_REC_LEN         4096

/* data types specification */

typedef enum {
    PNIO_MODE_OFFLINE = 0,
    PNIO_MODE_CLEAR   = 1,
    PNIO_MODE_OPERATE = 2
} PNIO_MODE_TYPE;

typedef enum {
    PNIO_DA_FALSE = 0,
    PNIO_DA_TRUE  = 1
} PNIO_DEV_ACT_TYPE;

typedef struct {
    PNIO_ADDR_TYPE  AddrType;
    PNIO_IO_TYPE IODataType;
    union {
        PNIO_UINT32 Addr;  /* logical address */

        PNIO_UINT32  Reserved [5];
    } u;
} ATTR_PACKED PNIO_ADDR;

typedef PNIO_ALARM_TINFO PNIO_CTRL_RALRM_TINFO;
typedef PNIO_ALARM_INFO PNIO_CTRL_ALARM_INFO;

typedef struct {
    PNIO_ALARM_TYPE  AlarmType;         /* original alarm-identifier from alarm-data-block                  */
    PNIO_APRIO_TYPE  AlarmPriority;     /* alarm priority for net, high or low -> see cm defines            */
    PNIO_UINT16      DeviceNum;         /* station number                                                   */
    PNIO_UINT16      SlotNum;           /* slot-no form alarm-data-block or 0                               */
    PNIO_UINT16      SubslotNum;        /* subslot-no from alarm-data-block or 0                            */

    PNIO_UINT16  CpuAlarmPriority;      /* only relevant with process alarm, ob number                      */
    PNIO_UINT32  PnioCompatModtype;     /* only relevant for plug/pull alarms: real == expected => 0x8101   */


    PNIO_CTRL_RALRM_TINFO  AlarmTinfo;   /* see struct definition*/

    PNIO_UINT8       DiagDs[4];         /* only relevant with diagnostic alarm, all 0x0 otherwise         */
    PNIO_UINT8       PrAlarmInfo[4];    /* only relevant with process alarm, all 0x0 otherwise            */

    PNIO_CTRL_ALARM_INFO   AlarmAinfo;   /* see struct definition, only relevant if not device failure/return alarm  */
} ATTR_PACKED PNIO_CTRL_ALARM_DATA;

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
typedef struct {
    PNIO_ADDR    Address;

    PNIO_UINT32     DataLength; /* data length of IO-Data input, output */
    PNIO_DATA_TYPE  DataType;   /* data type of IO-Data IRT, RT */
    PNIO_COM_TYPE   ComType;    /* communication type: controller-device, direct data exchange */

    PNIO_UINT32   Api;
    PNIO_UINT32   ReductionFactor;
    PNIO_UINT32   Phase;
    PNIO_UINT32   CycleTime;      /* microseconds */

    PNIO_UINT32   Reserved1[2];
    PNIO_UINT32   StatNo;
    PNIO_UINT32   Slot;
    PNIO_UINT32   Subslot;
    PNIO_UINT32   Reserved2[2];
} ATTR_PACKED PNIO_CTRL_DIAG_CONFIG_SUBMODULE;


typedef struct {
    PNIO_ADDR     Address;
    PNIO_UINT16   BitOffset;
    PNIO_UINT16   BitLength;
} ATTR_PACKED PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE;


#define  PNIO_CTRL_DEV_DIAG_OFFLINE            0x00000001 /* device isn't switched on (or)
                                                                    isn't attainable over the net (or)
                                                                    has wrong device name */
#define  PNIO_CTRL_DEV_DIAG_ADDR_INFO_OK       0x00000010 /* name resolution succeded (device name -> IP) */
#define  PNIO_CTRL_DEV_DIAG_AR_CONNECTING      0x00000100 /* intermediate state */
#define  PNIO_CTRL_DEV_DIAG_AR_IN_DATA         0x00000200 /* SRT: connection was built up (or)
                                                             RT: intermediate state (or)
                                                             RT: device is attached by wrong port (or)
                                                             RT: network configuration is not equal to STEP7 project */
#define  PNIO_CTRL_DEV_DIAG_AR_OFFLINE         0x00000400 /* SRT: see PNIO_CTRL_DIAG_DEVICE::Reason (or)
                                                             RT: intermediate state */
#define  PNIO_CTRL_DEV_DIAG_AR_RTC3_FOLLOWS    0x00001000 /* RT: intermediate state (or)
                                                             RT: device don't send APPLICATION_READY confirmation */


#define  PNIO_CTRL_REASON_NONE                             0x0000
#define  PNIO_CTRL_REASON_NARE_ERR_MULTIPLE_IP_USE         0x0101
#define  PNIO_CTRL_REASON_NARE_ERR_IP_IN_USE               0x0102
#define  PNIO_CTRL_REASON_NARE_ERR_DCP_MULTIPLE_STATIONS   0x0103
#define  PNIO_CTRL_REASON_NARE_ERR_DCP_SET_FAILED          0x0104
#define  PNIO_CTRL_REASON_NARE_ERR_DCP_SET_TIMEOUT         0x0105
#define  PNIO_CTRL_REASON_NARE_ERR_DCP_STATION_NOT_FOUND   0x0106
#define  PNIO_CTRL_REASON_NARE_ERR_OPCODE                  0x0107
#define  PNIO_CTRL_REASON_NARE_ERR_CHANNEL_USE             0x0108
#define  PNIO_CTRL_REASON_NARE_ERR_DNS_FAILED              0x0109
#define  PNIO_CTRL_REASON_NARE_ERR_LL                      0x010a
#define  PNIO_CTRL_REASON_NARE_ERR_NO_OWN_IP               0x010b
#define  PNIO_CTRL_REASON_NARE_ERR_DCP_IDENT_TLV_ERROR     0x010c
#define  PNIO_CTRL_REASON_NARE_ERR_IP_RESOLVE_NO_ANSWER    0x010d
#define  PNIO_CTRL_REASON_CLRPC_ERR_FAULTED                0x0201
#define  PNIO_CTRL_REASON_CLRPC_ERR_REJECTED               0x0202
#define  PNIO_CTRL_REASON_CLRPC_ERR_IN_ARGS                0x0203
#define  PNIO_CTRL_REASON_CLRPC_ERR_OUT_ARGS               0x0204
#define  PNIO_CTRL_REASON_CLRPC_ERR_DECODE                 0x0205
#define  PNIO_CTRL_REASON_CLRPC_ERR_PNIO_OUT_ARGS          0x0206
#define  PNIO_CTRL_REASON_CLRPC_ERR_TIMEOUT                0x0207
#define  PNIO_CTRL_REASON_CM_ADDR_INFO_CLASS_LINK          0x0301
#define  PNIO_CTRL_REASON_CM_ADDR_INFO_CLASS_MISS          0x0401
#define  PNIO_CTRL_REASON_CM_ADDR_INFO_CLASS_COMPANION     0x0501
#define  PNIO_CTRL_REASON_CM_ADDR_INFO_CLASS_OTHER         0x0601

#define  PNIO_CTRL_REASON_ERR_MULTIPLE_IP_USE              0x0700
#define  PNIO_CTRL_REASON_ERR_MULTIPLE_STATIONS            0x0701
#define  PNIO_CTRL_REASON_REASON_LINK_DOWN                 0x0702
#define  PNIO_CTRL_REASON_MAU_MISMATCH                     0x0703
#define  PNIO_CTRL_REASON_RPC_ERROR                        0x0704
#define  PNIO_CTRL_REASON_PNIO_CONFIGURATION               0x0705
#define  PNIO_CTRL_REASON_RPC_TIMEOUT                      0x0706
#define  PNIO_CTRL_REASON_IP_SET_ABORTED                   0x0707
#define  PNIO_CTRL_REASON_IOD_NOT_FOUND                    0x0708
#define  PNIO_CTRL_REASON_OUT_OF_RESOURCES                 0x0709
#define  PNIO_CTRL_REASON_MISS                             0x070A
#define  PNIO_CTRL_REASON_NOS_SET_ABORTED                  0x070B
#define  PNIO_CTRL_REASON_IOC_ABORTED                      0x070C
#define  PNIO_CTRL_REASON_IOD_ABORTED                      0x070D
#define  PNIO_CTRL_REASON_VENDORID_ABORTED                 0x070E
#define  PNIO_CTRL_REASON_MP_IN_PROGRESS                   0x070F
#define  PNIO_CTRL_REASON_MP_NOS_MISSING                   0x0710
#define  PNIO_CTRL_REASON_MP_IPv4_MISSING                  0x0711
#define  PNIO_CTRL_REASON_MP_NOS_INVALID                   0x0712
#define  PNIO_CTRL_REASON_MP_IPv4_INVALID                  0x0713
#define  PNIO_CTRL_REASON_MP_TAILORING_MISSING             0x0714
#define  PNIO_CTRL_REASON_MP_PDEV_ERROR                    0x0715
#define  PNIO_CTRL_REASON_MP_DEVICE_DISABLED               0x0716

typedef struct {
    PNIO_DEV_ACT_TYPE  Mode;
    PNIO_UINT32        DiagState; /* BitField, see PNIO_CTRL_DEV_DIAG_XXX */
    PNIO_UINT32        Reason;    /* see PNIO_CTRL_REASON_XXX */
    PNIO_UINT32        Reserved1[12];
} ATTR_PACKED PNIO_CTRL_DIAG_DEVICE;

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
typedef struct {
    PNIO_UINT8    name[256];           /* ctrl 'device name' (Geraetename) StationName  */
    PNIO_UINT8    TypeOfStation[256];  /* ctrl 'device type' (Geraetetyp ) TypeOfStation*/
    PNIO_UINT32   ip_addr;
    PNIO_UINT32   ip_mask;
    PNIO_UINT32   default_router;
} ATTR_PACKED PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO_DATA;
#endif /* PNIO_SOFTNET */

typedef enum {
    PNIO_CTRL_DIAG_RESERVED                = 0,
    PNIO_CTRL_DIAG_CONFIG_SUBMODULE_LIST   = 1,
    PNIO_CTRL_DIAG_DEVICE_STATE            = 2,
    PNIO_CTRL_DIAG_CONFIG_IOROUTER_PRESENT = 3,
    PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE_LIST= 4,
#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
    PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO   = 5 , /* FW V2.5.2.0 and greater */
    PNIO_CTRL_DIAG_GET_COMM_COUNTER_DATA   = 6,
    PNIO_CTRL_DIAG_RESET_COMM_COUNTERS   = 7    

#endif /* PNIO_SOFTNET */
} PNIO_CTRL_DIAG_ENUM;

typedef struct {
    PNIO_CTRL_DIAG_ENUM DiagService;
    union {
       PNIO_UINT32      Reserved1[8];
       PNIO_ADDR        Addr;          /* for PNIO_CTRL_DIAG_DEVICE_STATE */
    }u;
    PNIO_REF            ReqRef;
    PNIO_UINT32         Reserved2;
} ATTR_PACKED PNIO_CTRL_DIAG;

#endif /* not supported for PNIO SOFTNET */

/* structure types for the callback events */

typedef enum {
    PNIO_CBE_MODE_IND       = 1,
    PNIO_CBE_ALARM_IND      = 2,
    PNIO_CBE_REC_READ_CONF  = 3,
    PNIO_CBE_REC_WRITE_CONF = 4,
    PNIO_CBE_DEV_ACT_CONF   = 5,
    PNIO_CBE_CP_STOP_REQ    = 6
#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
    ,
    PNIO_CBE_START_LED_FLASH = 7,
    PNIO_CBE_STOP_LED_FLASH = 8,
    PNIO_CBE_CTRL_DIAG_CONF = 9
#endif /* not supported for PNIO SOFTNET */
} PNIO_CBE_TYPE;

typedef struct {
    PNIO_MODE_TYPE      Mode;
} ATTR_PACKED PNIO_CBE_PRM_MODE_IND;

typedef struct {
    PNIO_ADDR *         pAddr;
    PNIO_UINT32         RecordIndex;
    PNIO_REF            ReqRef;
    PNIO_ERR_STAT       Err;
    PNIO_UINT32         Length;
    const PNIO_UINT8 *  pBuffer;
} ATTR_PACKED PNIO_CBE_PRM_REC_READ_CONF;

typedef struct {
    PNIO_ADDR *         pAddr;
    PNIO_UINT32         RecordIndex;
    PNIO_REF            ReqRef;
    PNIO_ERR_STAT       Err;
} ATTR_PACKED PNIO_CBE_PRM_REC_WRITE_CONF;

typedef struct {
    PNIO_ADDR *                  pAddr;
    PNIO_REF                     IndRef;
    const PNIO_CTRL_ALARM_DATA * pAlarmData;
} ATTR_PACKED PNIO_CBE_PRM_ALARM_IND;

typedef struct {
    PNIO_ADDR *         pAddr;
    PNIO_DEV_ACT_TYPE   Mode;
    PNIO_UINT32         Result;
} ATTR_PACKED PNIO_CBE_PRM_DEV_ACT_CONF;

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
typedef struct {
    PNIO_UINT32         Frequency;
} ATTR_PACKED PNIO_CBE_PRM_LED_FLASH;

typedef struct {
    PNIO_CTRL_DIAG *    pDiagData;
    PNIO_UINT32         DiagDataBufferLen;
    PNIO_UINT8 *        pDiagDataBuffer;    /* cast to PNIO_CTRL_DIAG_CONFIG_SUBMODULE[...] or
                                                    to PNIO_CTRL_DIAG_DEVICE[1] */
    PNIO_UINT32         ErrorCode;
} ATTR_PACKED PNIO_CBE_PRM_CTRL_DIAG_CONF;
#endif /* not supported for PNIO SOFTNET */

typedef struct {
    PNIO_CBE_TYPE       CbeType;
    PNIO_UINT32         Handle;
    union {
        PNIO_CBE_PRM_MODE_IND       ModeInd;
        PNIO_CBE_PRM_REC_READ_CONF  RecReadConf;
        PNIO_CBE_PRM_REC_WRITE_CONF RecWriteConf;
        PNIO_CBE_PRM_ALARM_IND      AlarmInd;
        PNIO_CBE_PRM_DEV_ACT_CONF   DevActConf;
#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
        PNIO_CBE_PRM_LED_FLASH      LedFlash;
        PNIO_CBE_PRM_CTRL_DIAG_CONF CtrlDiagConf;
#endif /* not supported for PNIO SOFTNET */
    };
} ATTR_PACKED PNIO_CBE_PRM;

typedef void (*PNIO_CBF) (PNIO_CBE_PRM * pCbfPrm);

/* controller function specification */

#ifdef __cplusplus
extern "C" {
#endif

PNIO_UINT32 PNIO_CODE_ATTR PNIO_controller_open(
        PNIO_UINT32    CpIndex,            /* in */
        PNIO_UINT32    ExtPar,             /* in */
        PNIO_CBF       cbf_rec_read_conf,  /* in */
        PNIO_CBF       cbf_rec_write_conf, /* in */
        PNIO_CBF       cbf_alarm_ind,      /* in */
        PNIO_UINT32 *  Handle);            /* out */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_activate(
        PNIO_UINT32    Handle,           /* in */
        PNIO_ADDR *    pAddr,            /* in */
        PNIO_DEV_ACT_TYPE DeviceMode);   /* in */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_set_mode(
        PNIO_UINT32    Handle,           /* in */
        PNIO_MODE_TYPE Mode);            /* in */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_rec_read_req(
        PNIO_UINT32    Handle,            /* in */
        PNIO_ADDR *    pAddr,             /* in */
        PNIO_UINT32    RecordIndex,       /* in */
        PNIO_REF       ReqRef,            /* in */
        PNIO_UINT32    Length);           /* in */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_rec_write_req(
        PNIO_UINT32    Handle,            /* in */
        PNIO_ADDR *    pAddr,             /* in */
        PNIO_UINT32    RecordIndex,       /* in */
        PNIO_REF       ReqRef,            /* in */
        PNIO_UINT32    Length,            /* in */
        PNIO_UINT8 *   pBuffer);          /* in */

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_ctrl_diag_req(
        PNIO_UINT32    Handle,            /* in */
        PNIO_CTRL_DIAG * pDiagReq);       /* in */

#endif /* not supported for PNIO SOFTNET */


#ifdef PNIO_SOFTNET
    #define PNIO_controller_close PNIO_close
    PNIO_UINT32 PNIO_CODE_ATTR PNIO_close(PNIO_UINT32 Handle); /* in */
#else
    #define PNIO_close PNIO_controller_close
    PNIO_UINT32 PNIO_CODE_ATTR PNIO_controller_close(PNIO_UINT32 Handle); /* in */
#endif


PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_read(
        PNIO_UINT32   Handle,            /* in */
        PNIO_ADDR *   pAddr,             /* in */
        PNIO_UINT32   BufLen,            /* in */
        PNIO_UINT32 * pDataLen,          /* out */
        PNIO_UINT8 *  pBuffer,           /* out */
        PNIO_IOXS     IOlocState,        /* in */
        PNIO_IOXS *   pIOremState);      /* out */

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_output_data_read(
        PNIO_UINT32   Handle,            /* in */
        PNIO_ADDR *   pAddr,             /* in */
        PNIO_UINT32   BufLen,            /* in */
        PNIO_UINT32 * pDataLen,          /* out */
        PNIO_UINT8 *  pBuffer,           /* out */
        PNIO_IOXS *   pIOlocState,       /* out */
        PNIO_IOXS *   pIOremState);      /* out */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_read_cache_refresh(
        PNIO_UINT32 Handle);             /* in */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_read_cache(
        PNIO_UINT32 Handle,              /* in */
        PNIO_ADDR * pAddr,               /* in */
        PNIO_UINT32 BufLen,              /* in */
        PNIO_UINT32 * pDataLen,          /* out */
        PNIO_UINT8 * pBuffer,            /* out */
        PNIO_IOXS   IOlocState,          /* in */
        PNIO_IOXS * pIOremState);        /* out */
#endif /* not supported for PNIO SOFTNET */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_write(
        PNIO_UINT32   Handle,            /* in */
        PNIO_ADDR *   pAddr,             /* in */
        PNIO_UINT32   BufLen,            /* in */
        PNIO_UINT8 *  pBuffer,           /* in */
        PNIO_IOXS     IOlocState,        /* in */
        PNIO_IOXS *   pIOremState);      /* out */

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_write_cache_flush(
        PNIO_UINT32   Handle);            /* in */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_write_cache(
        PNIO_UINT32   Handle,            /* in */
        PNIO_ADDR *   pAddr,             /* in */
        PNIO_UINT32   BufLen,            /* in */
        PNIO_UINT8 *  pBuffer,           /* in */
        PNIO_IOXS     IOlocState,        /* in */
        PNIO_IOXS *   pIOremState);      /* out */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_cache_exchange(
        PNIO_UINT32 Handle);
#endif /* not supported for PNIO SOFTNET */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_register_cbf(
        PNIO_UINT32   Handle,            /* in */
        PNIO_CBE_TYPE CbeType,           /* in */
        PNIO_CBF      Cbf);              /* in */

#ifdef __cplusplus
}
#endif

/* PROFI_ENERGY: This macro must be defined. Do not undefine it.
 *   PROFIenerg functionality is a part of PNIO Controller Ifc
 *   in Version V2.5.2 and higher.
 */
#ifndef PROFI_ENERGY
#define PROFI_ENERGY
#endif

#ifdef PROFI_ENERGY
/* PE (PROFIenergy) Extensions: PE data types and functions
 */

/* Config Data Record:
 * PE Parameter Record  to configure PROFIenergy Power Moduls.
 * Standard PNIO_rec_write_req() is used to configure a PE device:
 * Siemens IM 151-3BA23 Version 01 parameters
 */
#define PNIO_PE_CFG_DEV_NUM_SLOTS_MAX     8
#define PNIO_PE_CFG_SLOT_EMPTY            0x00
#define PNIO_PE_CFG_SLOT_MODE_POWER_ON    0x00
#define PNIO_PE_CFG_SLOT_MODE_POWER_OFF   0x01

#if defined(_MSC_VER)
 #pragma pack( push, safe_usrx_prev_packing, 2 )
 //#pragma pack(show)
 #define ATTR_PACKED
#elif defined(__GNUC__)
 #undef ATTR_PACKED
 #define ATTR_PACKED  __attribute__ ((aligned(2))) __attribute__ ((packed))
#endif

typedef struct {
    PNIO_UINT8          slot_num;     /* 0=empty  1...62= valid slot numbers */
    PNIO_UINT8          power_mode;   /* 0x00=power on, 0x01=power off during power save mode */
} ATTR_PACKED PNIO_PE_CFG_SLOT;

#if 0
// ----------  Block header is not required - is set by the firmware
    PNIO_UINT16         block_type;    /* 0x0800 RecordDataWrite, 0x0801 RecordDataRead  */
    PNIO_UINT16         block_length;  /* Num bytes without counting the fields BlockType and BlockLength   */
    PNIO_UINT8          version_high;  /* 0x01 */
    PNIO_UINT8          version_low;   /* 0x00 */
#endif

typedef struct {
    PNIO_UINT8          reserved1;                            /* reserved for internal use - version num */
    PNIO_UINT8          num_cfg_slots;                        /* current number of slots in the array    */
    PNIO_PE_CFG_SLOT    slot[PNIO_PE_CFG_DEV_NUM_SLOTS_MAX];  /* array of slot config blocks             */
} ATTR_PACKED PNIO_PE_PRM_CONFIG_DEVICE_REQ;


/* Alignment test: sizeof PNIO_PE_PRM_CONFIG_DEVICE_REQ must be == 18
*/
typedef char _size_tst_config_dev_rec_ [ ((sizeof(PNIO_PE_PRM_CONFIG_DEVICE_REQ) == 18) ? 1 : -1) ];

#if defined(_MSC_VER)
 #pragma pack( pop, safe_usrx_prev_packing )
#elif defined(__GNUC__)
 #undef ATTR_PACKED
 #define ATTR_PACKED  __attribute__ ((aligned(4)))
#endif

/* PE (PROFIenergy service related parameters: */

/* PE_SAP_DR_INDEX : subslot specific PROFIenergy Service Access Point 'Data Record Index'
 * PE_DR_INDEX_MASK: 0x80A1..0x80AF: subslot specific record data reserved for further use
 */
#define PE_SAP_DR_INDEX   0x80A0
#define PE_DR_INDEX_MASK  0xFFF0    /* PL: 19.09.2014: Fix: PR 1113532: 0x80A0 ---> 0xFFF0 */

/* PNIO_PE_SERVICE_REQ_LIFETIME_DEFAULT: Time in sec. to wait for Service Response
*/
#define PNIO_PE_SERVICE_REQ_LIFETIME_DEFAULT     10   /* [sec] */


/* PE Service Request IDs ==Command IDs */
typedef enum {
    PNIO_PE_CMD_RESERVED            = 0,
    PNIO_PE_CMD_START_PAUSE         = 1,
    PNIO_PE_CMD_END_PAUSE           = 2,
    PNIO_PE_CMD_Q_MODES             = 3,
    PNIO_PE_CMD_PEM_STATUS          = 4,
    PNIO_PE_CMD_PE_IDENTIFY         = 5,
    PNIO_PE_CMD_Q_MSRMT             = 0x10,
} PNIO_PE_CMD_ENUM;

/* PE Command Modifiers */
typedef enum {
    PE_CMD_MODIF_NOT_USED           = 0,
    PE_CMD_MODIF_Q_MODE_LIST_ALL    = 1,
    PE_CMD_MODIF_Q_MODE_GET_MODE    = 2,
    PE_CMD_MODIF_Q_MSRMT_LIST_ALL   = 1,
    PE_CMD_MODIF_Q_MSRMT_GET_VAL    = 2,
} PNIO_PE_CMD_MODIFIER_ENUM;


/*--- PE Service Request Parameters ---*/

/* START_PAUSE
 */
/* REQ:  */
typedef struct {
    PNIO_UINT32         time_ms;     /* requested pause time in ms */
} ATTR_PACKED PNIO_PE_PRM_START_PAUSE_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT32         pe_mode_id;  /* energy-saving mode selected by the device */
} ATTR_PACKED PNIO_PE_PRM_START_PAUSE_CONF;

/* END_PAUSE
 */
/* REQ: NULL */
/* CONF: */
typedef struct {
    PNIO_UINT32         time_ms;     /* time to leave energy-saving mode */
} ATTR_PACKED PNIO_PE_PRM_END_PAUSE_CONF;

/* PE_IDENTIFY
 */
/* REQ: NULL */
/* CONF: */
typedef struct {
    PNIO_UINT8          numServices;    /* number of supported services */
    PNIO_UINT8          serviceId[254]; /* array of service ids PNIO_PE_CMD_xxx  */
} ATTR_PACKED PNIO_PE_PRM_PE_IDENTIFY_CONF;


/* PEM_STATUS
 */
/* REQ: NULL */
/* CONF: */
typedef struct {
    PNIO_UINT8          PE_Mode_ID_Source;
    PNIO_UINT8          PE_Mode_ID_Destination;
    PNIO_UINT32         Time_to_operate;                   /* max time to reach 'operate' state */
    PNIO_UINT32         Remaining_time_to_destination;
    float               Mode_Power_Consumption;
    float               Energy_Consumption_to_Destination;
    float               Energy_Consumption_to_operate;
} ATTR_PACKED PNIO_PE_PRM_PEM_STATUS_CONF;


/* PE_QUERY_MODES: LIST_ALL    Service_Modifier= 0x01
 */
/* REQ: NULL */
/* CONF: */
typedef struct {
    PNIO_UINT8          numModeIds;
    PNIO_UINT8          peModeId[254];     /* array of supported PE_Mode_Ids  MAX=254 */

} ATTR_PACKED PNIO_PE_PRM_Q_MODE_LIST_ALL_CONF;


/* PE_QUERY_MODES: GET_MODE    Service_Modifier= 0x02
 */
/* REQ:  */
typedef struct {
    PNIO_UINT8          peModeId;
    PNIO_UINT8          reserved;
} ATTR_PACKED PNIO_PE_PRM_Q_MODE_GET_MODE_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8          peModeId;
    PNIO_UINT8          PE_Mode_Attributes;
    PNIO_UINT32         Time_min_Pause;
    PNIO_UINT32         Time_to_Pause;
    PNIO_UINT32         Time_to_operate;
    PNIO_UINT32         Time_min_length_of_stay;
    PNIO_UINT32         Time_max_length_of_stay;
    float               Mode_Power_Consumption;
    float               Energy_Consumption_to_pause;
    float               Energy_Consumption_to_operate;
} ATTR_PACKED PNIO_PE_PRM_Q_MODE_GET_MODE_CONF;


/* PE command Query_Measurement:  Q_MSRMT_GET_LIST     Service_Modifier= 0x01 == Get_Measurement_List
 */

#define PE_MSRMT_IDS_MAX   254

/* REQ: NULL */
/* CONF: */
typedef struct {
    PNIO_UINT16         Measurement_ID;
    PNIO_UINT8          Accuracy_Domain;
    PNIO_UINT8          Accuracy_Class;
    float               Range;
} ATTR_PACKED PNIO_PE_MSRMT_ID_ELEM;

typedef struct {
    PNIO_UINT8          numMeasurementIds;
    PNIO_UINT8          reserved;
    PNIO_PE_MSRMT_ID_ELEM elem[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PNIO_PE_PRM_Q_MSRMT_GET_LIST_CONF;


/* PE command Query_Measurement:  Q_MSRMT_GET_VAL      Service_Modifier= 0x02 == Get_Measurement_Values
 */
/* REQ: */
typedef struct {
    PNIO_UINT8          numMeasurementIds;
    PNIO_UINT8          reserved;
    PNIO_UINT16         Measurement_ID[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PNIO_PE_PRM_Q_MSRMT_GET_VAL_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT16         Measurement_ID;
    PNIO_UINT16         Status_of_Measurement_Value;
    float               Transmission_Data_Type;
    PNIO_UINT32         End_of_demand;
    PNIO_UINT32         reserved;
} ATTR_PACKED PNIO_PE_MSRMT_VAL_ELEM;

typedef struct {
    PNIO_UINT8          numMeasurementValues;
    PNIO_UINT8          reserved;
    PNIO_PE_MSRMT_VAL_ELEM value[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PNIO_PE_PRM_Q_MSRMT_GET_VAL_CONF;


typedef struct {
    PNIO_UINT8    err_code;
    PNIO_UINT8    reserved;
} ATTR_PACKED PNIO_PE_PRM_GENERIC_CONF_NEG;

/* PNIO_PE_CBE_PRM:  PE confirmation parameter (callback param)
 */

/* PE_CONF_HDR: PE Service Response (confirmation) Header */
typedef struct {
    PNIO_PE_CMD_ENUM    CmdId;   /* PE Service id (CommandId) 'int'   */
    PNIO_UINT32         Handle;  /* Hndl returned by PNIO_ctrl_open() */
    PNIO_ADDR           Addr;
    PNIO_REF            Ref;
    PNIO_ERR_STAT       Err;
    PNIO_UINT16         state;         /* Response state:1= ok, 2=err */
    PNIO_UINT16         struct_id;     /* param. struct id,  0xff=err */
    PNIO_UINT32         length;  /* num bytes of the following params */
} ATTR_PACKED PE_CBE_HDR;

/* PE Confirmation Struct Ids */
#define PE_CNF_STRUCT_ID_RESERVED        0
#define PE_CNF_STRUCT_ID_ERROR           0xFF
#define PE_CNF_STRUCT_ID_UNIQUE          1
#define PE_CNF_STRUCT_ID_QMODE_LIST_ALL  1
#define PE_CNF_STRUCT_ID_QMODE_GET_MODE  2
#define PE_CNF_STRUCT_ID_QMSRMT_LIST_ALL 1
#define PE_CNF_STRUCT_ID_QMSRMT_GET_VAL  2


typedef struct {
    PE_CBE_HDR                              pe_hdr; /* pe callback header */
    union {
        PNIO_PE_PRM_GENERIC_CONF_NEG        RespNegative;
        PNIO_PE_PRM_START_PAUSE_CONF        StartPause;
        PNIO_PE_PRM_END_PAUSE_CONF          EndPause;
        PNIO_PE_PRM_PE_IDENTIFY_CONF        PeIdentify;
        PNIO_PE_PRM_PEM_STATUS_CONF         PemStatus;
        PNIO_PE_PRM_Q_MODE_LIST_ALL_CONF    ModeList;
        PNIO_PE_PRM_Q_MODE_GET_MODE_CONF    ModeGet;
        PNIO_PE_PRM_Q_MSRMT_GET_LIST_CONF   MeasurementList;
        PNIO_PE_PRM_Q_MSRMT_GET_VAL_CONF    MeasurementValue;
    } pe_prm;
} ATTR_PACKED PNIO_PE_CBE_PRM;



/* PNIO_PE_REQ_PRM: PE command request parameter
*/
typedef struct {
    PNIO_PE_CMD_ENUM          CmdId;
    PNIO_PE_CMD_MODIFIER_ENUM CmdModifier;
    union {
        PNIO_PE_PRM_START_PAUSE_REQ         StartPause;
        PNIO_PE_PRM_Q_MODE_GET_MODE_REQ     ModeGet;
        PNIO_PE_PRM_Q_MSRMT_GET_VAL_REQ     MeasurementValue;
    } rq;
} ATTR_PACKED PNIO_PE_REQ_PRM;


/* PE Command Request Funnction
 */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_pe_cmd_req(
        PNIO_UINT32    Handle,            /* in */
        PNIO_ADDR     *pAddr,             /* in */
        PNIO_REF       ReqRef,            /* in */
        PNIO_PE_REQ_PRM *pPeReqPrm        /* in */
        );

/* PNIO_PE_CBF: PE Callback function
 */
typedef void (*PNIO_PE_CBF) (PNIO_PE_CBE_PRM * pCbfPrm);


PNIO_UINT32 PNIO_CODE_ATTR PNIO_register_pe_cbf(
        PNIO_UINT32    Handle,            /* in */
        PNIO_PE_CBF    Cbf                /* in */
        );


#endif /* PROFI_ENERGY */
/* PE (PROFIenergy) Extensions END */



typedef struct {
	PNIO_UINT32		SndNoError;	
	PNIO_UINT32		SndCollision;	
	PNIO_UINT32		SndOtherError;	
	PNIO_UINT32		RcvNoError;	
	PNIO_UINT32		RcvResError;	
	PNIO_UINT32		RcvRejected;	
} ATTR_PACKED PNIO_CTRL_COMM_PORT_COUNTER_DATA;

typedef struct {
	PNIO_CTRL_COMM_PORT_COUNTER_DATA 	Port[4];
	PNIO_UINT16					NumberOfPorts;
} ATTR_PACKED PNIO_CTRL_COMM_COUNTER_DATA;

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#endif /* PNIOUSRX_H */


