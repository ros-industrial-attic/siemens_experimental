/****************************************************************************/
/*                                                                          */
/*    Copyright (C) SIEMENS CORP., 2015 All rights reserved.*/
/*                                                                          */
/*    file: PNIOBASE.H                                                      */
/*                                                                          */
/*    Description:                                                          */
/*    data types and function declarations for IO-Base user interface       */
/*                                                                          */
/****************************************************************************/

#ifndef PNIOBASE_H
#define PNIOBASE_H

//#define PNIO_ALARM_OLD_STRUC

#if defined(WIN32)
    typedef unsigned __int8 uint8_t;
    typedef unsigned __int16 uint16_t;
    typedef unsigned __int32 uint32_t;
#elif defined(WIN64)
    typedef unsigned __int8 uint8_t;
    typedef unsigned __int16 uint16_t;
    typedef unsigned __int32 uint32_t;
#else
  #include <inttypes.h>
#endif

/* base data type specification */
typedef uint8_t  PNIO_UINT8;
typedef uint16_t PNIO_UINT16;
typedef uint32_t PNIO_UINT32;
typedef uint32_t PNIO_REF;

#include "pnioerrx.h"

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
 #error please adapt pniobase.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
    #define PNIO_CODE_ATTR __stdcall
#else
    #define PNIO_CODE_ATTR
#endif

/* deprecated definition; don't use this define */
/*
#ifdef __cplusplus
    #define PNIO_EXTRN extern "C"
#else
    #define PNIO_EXTRN extern
#endif*/

#define PNIO_API_VERSION 230

#define PNIO_MAX_ALARM_DATA_LEN  1472

typedef enum {
    PNIO_ADDR_LOG = 0,
    PNIO_ADDR_GEO = 1
} PNIO_ADDR_TYPE;

typedef enum {
    PNIO_IO_IN  = 0,
    PNIO_IO_OUT = 1
} PNIO_IO_TYPE;

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */
typedef enum {
    PNIO_DATA_RT  = 0,
    PNIO_DATA_IRT = 1
} PNIO_DATA_TYPE;

typedef enum {
    PNIO_COM_UNICAST        = 0, /* controller-device */
    PNIO_COM_DIRECT_DATA_EX = 1  /* direct data exchange */
} PNIO_COM_TYPE; /* communication type */

typedef enum {
    PNIO_ACCESS_ALL_WITH_LOCK = 0,    /* not supported */
    PNIO_ACCESS_ALL_WITHOUT_LOCK = 1,

    PNIO_ACCESS_RT_WITH_LOCK    = 2,
    PNIO_ACCESS_RT_WITHOUT_LOCK = 3,

    PNIO_ACCESS_IRT_WITH_LOCK = 4,    /* not supported */
    PNIO_ACCESS_IRT_WITHOUT_LOCK = 5
} PNIO_ACCESS_ENUM;
#endif /* not supported for PNIO SOFTNET */

typedef enum {
    PNIO_S_GOOD = 0,
    PNIO_S_BAD  = 1
} PNIO_IOXS; /*represents PNIO IO remote/local status */

/* PNIO_ERR_STAT:
 * a) The first four parameters represent the PNIO Status.
 *    For details refer to IEC 61158-6 chapter "Coding of the field PNIOStatus"
 * b) The last two parameters correspond to AdditionalValue1 and AdditionalValue2,
 *    see IEC 61158-6 chapter "Coding of the field AdditionalValue1 and AdditionalValue2":
 *    The values shall contain additional user information within negative responses.
 *    The value zero indicates no further information.
 *    For positive read responses, the value 1 of the field AdditionalValue1 indicates
 *    that the Record Data Object contains more data than have been read.
 * c) For the two callback events PNIO_CBE_REC_READ_CONF and
 *    PNIO_CBE_REC_WRITE_CONF, the following special cases apply to
 *    the "Err" parameter:
 *    Case 1
 *      ErrorCode=0xC0 and
 *      ErrorDecode=0x80 and
 *      ErrCode1=0xA3 and
 *      ErrCode2=0x0
 *      Cause: A connection to the target device cannot be established
 *             or was interrupted.
 *    Case 2
 *      ErrorCode=0xDE/0xDF and
 *      ErrorDecode=0x80 and
 *      ErrCode1=0xC3 and
 *      ErrCode2=0x0
 *      Cause: The target device is currently busy with other jobs.
 *             Repeat the job.
 *    Case 3
 *      For all other situations, refer to the error codes of the
 *      PROFINET IO standard IEC 61158-6, Section "Coding of the field PNIOStatus".
 */
typedef struct {
    PNIO_UINT8     ErrCode;   /* ErrorCode: Most significant word, most significant byte of PNIO Status */
    PNIO_UINT8     ErrDecode; /* ErrorDecode: Most significant word, least significant byte of PNIO Status */
    PNIO_UINT8     ErrCode1;  /* ErrorDecode: Least significant word, most significant byte of PNIO Status */
    PNIO_UINT8     ErrCode2;  /* ErrorCode2: Least significant word, least significant byte of PNIO Status */
    PNIO_UINT16    AddValue1;
    PNIO_UINT16    AddValue2;
} ATTR_PACKED PNIO_ERR_STAT;

typedef enum {
    PNIO_ALARM_DIAGNOSTIC                   = 0x01,
    PNIO_ALARM_PROCESS                      = 0x02,
    PNIO_ALARM_PULL                         = 0x03,
    PNIO_ALARM_PLUG                         = 0x04,
    PNIO_ALARM_STATUS                       = 0x05,
    PNIO_ALARM_UPDATE                       = 0x06,
    PNIO_ALARM_REDUNDANCY                   = 0x07,
    PNIO_ALARM_CONTROLLED_BY_SUPERVISOR     = 0x08,
    PNIO_ALARM_RELEASED_BY_SUPERVISOR       = 0x09,
    PNIO_ALARM_PLUG_WRONG                   = 0x0A,
    PNIO_ALARM_RETURN_OF_SUBMODULE          = 0x0B,
    PNIO_ALARM_DIAGNOSTIC_DISAPPEARS        = 0x0C,
    PNIO_ALARM_MCR_MISMATCH                 = 0x0D,
    PNIO_ALARM_PORT_DATA_CHANGED            = 0x0E,
    PNIO_ALARM_SYNC_DATA_CHANGED            = 0x0F,
    PNIO_ALARM_ISOCHRONE_MODE_PROBLEM       = 0x10,
    PNIO_ALARM_NETWORK_COMPONENT_PROBLEM    = 0x11,
    PNIO_ALARM_UPLOAD_AND_STORAGE           = 0x1E,
    PNIO_ALARM_PULL_MODULE                  = 0x1F,
    PNIO_ALARM_DEV_FAILURE                  = 0x00010000,
    PNIO_ALARM_DEV_RETURN                   = 0x00010001
} PNIO_ALARM_TYPE;

typedef enum {
    PNIO_APRIO_LOW  = 0,
    PNIO_APRIO_HIGH = 1
} PNIO_APRIO_TYPE;

typedef struct {
	PNIO_UINT16 ChannelNumber ; /* 0x0000 ... 0x7fff manufacturer specific */
								/* 0x8000 Submodule */
								/* 0x8001 .. 0xffff reserved */
	PNIO_UINT16 ChannelProperties; /* see FDIS 61158-6-10 chapter 6.2.7.4 for coding Table 520 - 524 */
	PNIO_UINT16 ChannelErrorType; /* see FDIS 61158-6-10 chapter 6.2.7.2 for coding Table 518 */
} PNIO_ALARM_DATA_DIAGNOSIS2 ;	/* diagnosis Alarm */

typedef struct {
    PNIO_UINT16 BlockType;
    PNIO_UINT16 BlockLength;
    PNIO_UINT8  BlockVersionHigh;
    PNIO_UINT8  BlockVersionLow;
} PNIO_BLOCK_HEADER;


typedef struct {
	PNIO_BLOCK_HEADER BlockHeader; /* 6 Byte don't care */
	PNIO_UINT8 padd;				/* don't care */
	PNIO_UINT8 padd1;				/* don't care */
    PNIO_UINT32 MaintenanceStatus; /* bit 0 = maintance required if 1 */
								   /* bit 1 = maintance demanded if 1 */
} PNIO_MAINTENANCE ;

typedef struct {
	PNIO_UINT16 ChannelNumber ; /* 0x0000 ... 0x7fff manufacturer specific */
								/* 0x8000 Submodule */
								/* 0x8001 .. 0xffff reserved */
	PNIO_UINT16 ChannelProperties; /* see FDIS 61158-6-10 chapter 6.2.7.4 for coding Table 520 - 524 */
	PNIO_UINT16 ChannelErrorType; /* see FDIS 61158-6-10 chapter 6.2.7.2 for coding Table 518 */
} PNIO_ALARM_DATA_DIAGNOSIS ;	/* diagnosis Alarm */

typedef struct {
	PNIO_MAINTENANCE maintInfo;
	PNIO_UINT16 UserStrucIdent;      /* user structure identifier for user alarm data         */
                                        /* 0x8000 :  diagnosis alarm                             */
                                        /* 0x8002 :  extended diagnosis alarm                    */
                                        /* 0x0000  ... 0x7fff :  Manufacturer specific diagnosis or process alarm    */

	PNIO_UINT16 ChannelNumber ; /* 0x0000 ... 0x7fff manufacturer specific */
								/* 0x8000 Submodule */
								/* 0x8001 .. 0xffff reserved */
	PNIO_UINT16 ChannelProperties; /* see FDIS 61158-6-10 chapter 6.2.7.4 for coding Table 520 - 524 */
	PNIO_UINT16 ChannelErrorType; /* see FDIS 61158-6-10 chapter 6.2.7.2 for coding Table 518 */
} PNIO_ALARM_DATA_MAINTENANCE_DIAGNOSIS ;	/* maintenance diagnosis Alarm */

typedef struct {
	PNIO_UINT16 ChannelNumber ;
	PNIO_UINT16 ChannelProperties;
	PNIO_UINT16 ChannelErrorType;
	PNIO_UINT16 ExtChannelErrorType; /* see FDIS 61158-6-10 chapter 6.2.7.5 for coding Table 525 - 537 */
	PNIO_UINT32 ExtChannelAddValue;  /* see FDIS 61158-6-10 chapter 6.2.7.6 for coding  Table 538 -541 */
} PNIO_ALARM_DATA_EXT_DIAGNOSIS ;	/* extended diagnosis Alarm */

typedef struct {
	PNIO_MAINTENANCE maintInfo;
	PNIO_UINT16 UserStrucIdent;
	PNIO_UINT16 ChannelNumber ;
	PNIO_UINT16 ChannelProperties;
	PNIO_UINT16 ChannelErrorType;
	PNIO_UINT16 ExtChannelErrorType; /* see FDIS 61158-6-10 chapter 6.2.7.5 for coding Table 525 - 537 */
	PNIO_UINT32 ExtChannelAddValue;  /* see FDIS 61158-6-10 chapter 6.2.7.6 for coding  Table 538 -541 */
} PNIO_ALARM_DATA_MAINTENANCE_EXT_DIAGNOSIS ;	/* extended diagnosis Alarm */

typedef struct {
    PNIO_UINT16    BlockType;           /* block type from alarm data block */
    PNIO_UINT16    BlockVersion;        /* version from alarm data block */
    PNIO_UINT32    Api;                 /* for PNIO: Application Process Identifier from database           */
    PNIO_UINT16    AlarmSpecifier;      /* alarm-specifier from alarm data block
                                         *         Bit 14-15: reserved
                                         *         Bit 13:    Submodul-Diagnosis State
                                         *         Bit 12:    Generic Diagnosis
                                         *         Bit 11:    Channel Diagnosis
                                         *         Bit  0-10: sequence number                               */

    PNIO_UINT32    ModIdent;            /* modul-identifikation from alarm-data-block                       */
    PNIO_UINT32    SubIdent;            /* submodul-identification from alarm-data-block                    */

    PNIO_UINT16    UserStrucIdent;      /* user structure identifier for user alarm data         */
                                        /* 0x8000 :  diagnosis alarm                             */
                                        /* 0x8002 :  extended diagnosis alarm                    */
                                        /* 0x8100 :  maintenance diagnosis alarm                 */
                                        /* 0x0000  ... 0x7fff :  Manufacturer specific diagnosis or process alarm    */

    PNIO_UINT16    UserAlarmDataLen;                        /* length of the user alarm data     */
#ifdef PNIO_ALARM_OLD_STRUC
    PNIO_UINT8     UserAlarmData[PNIO_MAX_ALARM_DATA_LEN];  /* array of the user alarm data      */
#else
    union {
			PNIO_ALARM_DATA_MAINTENANCE_DIAGNOSIS m_diag;	/* maintenance Alarm */
			PNIO_ALARM_DATA_MAINTENANCE_EXT_DIAGNOSIS mext_diag;	/* extended Maintenance Alarm */
            PNIO_ALARM_DATA_DIAGNOSIS diag;	/* diagnosis Alarm */
            PNIO_ALARM_DATA_EXT_DIAGNOSIS ext_diag; /* extended diagnosis Alarm */
            PNIO_UINT8     UserAlarmData[PNIO_MAX_ALARM_DATA_LEN];  /* Manufacturer specific diagnosis or process alarm */
    } UAData;

#endif
} ATTR_PACKED PNIO_ALARM_INFO;

typedef struct {
    PNIO_UINT16    CompatDevGeoaddr;    /* bit 0-10 = device_no,                                */
                                        /* bit 11-14 = io-subsys-nr,                            */
                                        /* bit 15 = 1 (pnio identifier)                         */
                                        /* bit 11-14 = io-subsys-no,                            */
                                        /* bit 15 = 1 (pnio identifier)                         */
    PNIO_UINT8     ProfileType;         /* 0x08 => bit 0-3 for PNIO, bit 4-7 for DP             */
    PNIO_UINT8     AinfoType;           /* bit 0-3 alinfotyp = 0x00 (transparent)               */
    PNIO_UINT8     ControllerFlags;     /* bit 0 = 1 (ext. dp-interface), bit 1-7 reserved      */
    PNIO_UINT8     DeviceFlag;          /* bit 0 for PNIO: apdu-stat-failure, bit 1-7 reserved  */
    PNIO_UINT16    PnioVendorIdent;     /* for PNIO: vendor-identification                      */
    PNIO_UINT16    PnioDevIdent;        /* for PNIO: device-identification                      */
    PNIO_UINT16    PnioDevInstance;     /* for PNIO: instance of device                         */

} ATTR_PACKED PNIO_ALARM_TINFO;

/* constants specification for PNIO_controller_open, PNIO_device_open ExtPar */
#define PNIO_CEP_MODE_CTRL                 0x00000002      /* allow use of PNIO_set_mode */

#ifndef PNIO_SOFTNET /* not supported for PNIO SOFTNET */

#define PNIO_CEP_DEV_MAXAR_BY_TYPE         0x00000004      /* allow specificate maximal AR nummber by AR-Type, only for PNIO_device_open */
#define PNIO_CEP_SLICE_ACCESS              0x00000008      /* allow module slice read/write */
#define PNIO_CEP_DEV_MAXAR_BY_TYPE_ADV     0x00000010      /* allow specification of maximum AR number by AR-Type */
                                                           /* (RT, DA), only for PNIO_device_open()               */
#define PNIO_CEP_PE_MODE_ENABLE            0x00000020      /* allow Host-PC shutdown for PROFIEnergy  */
#define PNIO_CEP_SET_MRP_ROLE_OFF          0x00000040      /* sets MRP role to state OFF  */


typedef void (*PNIO_CBF_APPL_WATCHDOG)
       (PNIO_UINT32 CpIndex);

typedef enum {
    PNIO_CP_CBE_UNKNOWN     = 0,
    PNIO_CP_CBE_STARTOP_IND = 1,
    PNIO_CP_CBE_OPFAULT_IND = 2,
    PNIO_CP_CBE_NEWCYCLE_IND = 3
} PNIO_CP_CBE_TYPE;

typedef struct {
    PNIO_UINT32       CycleCount;
    PNIO_UINT32       ClockCount;
    PNIO_UINT32       CountSinceCycleStart;
} PNIO_CYCLE_INFO;

typedef struct {
    PNIO_UINT32       AppHandle;
    PNIO_CYCLE_INFO   CycleInfo;
} PNIO_CP_CBE_PRM_STARTOP_IND;

typedef struct {
    PNIO_UINT32       AppHandle;
    PNIO_CYCLE_INFO   CycleInfo;
} PNIO_CP_CBE_PRM_OPFAULT_IND;

typedef struct {
    PNIO_UINT32       AppHandle;
    PNIO_CYCLE_INFO   CycleInfo;
} PNIO_CP_CBE_PRM_NEWCYCLE_IND;

typedef struct {
    PNIO_CP_CBE_TYPE  CbeType;
    PNIO_UINT32       CpIndex;
    union {
        PNIO_CP_CBE_PRM_STARTOP_IND StartOp;
        PNIO_CP_CBE_PRM_OPFAULT_IND OpFault;
        PNIO_CP_CBE_PRM_NEWCYCLE_IND NewCycle;

        /* to be extended */
    } u;
} PNIO_CP_CBE_PRM;

typedef void (*PNIO_CP_CBF)
       (PNIO_CP_CBE_PRM *pCbfPrm);

#ifdef __cplusplus
extern "C" {
#endif

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_set_appl_watchdog(
        PNIO_UINT32              CpIndex,
        PNIO_UINT32              wdTimeOutInMs,
        PNIO_CBF_APPL_WATCHDOG   pnio_appl_wd_cbf);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_trigger_watchdog(
        PNIO_UINT32              CpIndex);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_register_cbf(
        PNIO_UINT32              AppHandle,
        PNIO_CP_CBE_TYPE         CbeType,
        PNIO_CP_CBF              Cbf);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_set_opdone(
        PNIO_UINT32              AppHandle,
        PNIO_CYCLE_INFO        * pCycleInfo
        );

#ifdef __cplusplus
}
#endif


/* Typedefs and Functions for development purposes.
 * Not needed for normal operation. */
#define PNIO_CI_FAULT_TIME_LEN 20

typedef enum {
    PNIO_FALSE = 0,
    PNIO_TRUE = 1
} PNIO_BOOL;

typedef enum {
    PNIO_CP_CI_UNKNOWN = 0,
    PNIO_CP_CI_STARTOP = 1,
    PNIO_CP_CI_OPFAULT = 2,
    PNIO_CP_CI_NEWCYCLE = 3,
    PNIO_CP_CI_OPDONE = 4
} PNIO_CP_CI_TYPE;

typedef struct {
    PNIO_BOOL        Valid; /* true: Min/Max valid */
    PNIO_UINT32      Min;
    PNIO_UINT32      Max;
} PNIO_LIMITS;

typedef struct {
    PNIO_CP_CI_TYPE  Type;
    PNIO_CYCLE_INFO  CycleInfo;
} PNIO_CI_ENTRY;

typedef struct {
    PNIO_BOOL        FaultOccurred; /* true: all counters stopped */
    char             FaultTime[PNIO_CI_FAULT_TIME_LEN];
    PNIO_LIMITS      StartOp;
    PNIO_LIMITS      OpDone;
    PNIO_LIMITS      ApplDelay;
} PNIO_CYCLE_STAT;

#ifdef __cplusplus
extern "C" {
#endif

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_cycle_stat(
        PNIO_UINT32              AppHandle,
        int                      MeasureNr,
        PNIO_CYCLE_STAT        * pCycleStat);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_CP_cycle_info(
        PNIO_UINT32              AppHandle,
        PNIO_CI_ENTRY          * pCycleInfoEntry,
        int                      MeasureNr,
        PNIO_UINT32              Offset);

#ifdef __cplusplus
}
#endif

#endif /* not supported for PNIO SOFTNET */

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#endif /* PNIOBASE_H */
