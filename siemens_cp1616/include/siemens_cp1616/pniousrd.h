/****************************************************************************/
/*                                                                          */
/*    Copyright (C) SIEMENS CORP., 2015 All rights reserved.*/
/*                                                                          */
/*    file: PNIOUSRD.H                                                      */
/*                                                                          */
/*    Description:                                                          */
/*    data types and function declarations for IO-Base                      */
/*    device user interface                                                 */
/*                                                                          */
/****************************************************************************/

#ifndef PNIOUSRD_H
#define PNIOUSRD_H

#include "pniobase.h"

#define PNIO_CHAN_PROP_MAINT_REQUIRED_BIT   0x0200
#define PNIO_CHAN_PROP_MAINT_DEMANDED_BIT 0x0400

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
 #error please adapt pniousrd.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
    #define PNIO_CODE_ATTR __stdcall
#else
    #define PNIO_CODE_ATTR
#endif

/* communication relations */
typedef enum {
    PNIO_IOCR_TYPE_RESERVED_0          = 0x0000,
    PNIO_IOCR_TYPE_INPUT               = 0x0001,
    PNIO_IOCR_TYPE_OUTPUT              = 0x0002,
    PNIO_IOCR_TYPE_MULTICAST_PROVIDER  = 0x0003,
    PNIO_IOCR_TYPE_MULTICAST_CONSUMER  = 0x0004
} PNIO_IOCR_TYPE_ENUM;

typedef enum  {
    PNIO_IOCR_PROP_RT_CLASS_MASK       = 0x0000000F,  /* Bit 0 - 3: IOCR-Properties.RTClass */
    PNIO_IOCR_PROP_RT_CLASS_1          = 0x00000001,  /* RT */
    PNIO_IOCR_PROP_RT_CLASS_2          = 0x00000002,  /* RT */
    PNIO_IOCR_PROP_RT_CLASS_3          = 0x00000003,  /* IRT */
    PNIO_IOCR_PROP_RT_CLASS_1_UDP      = 0x00000004,  /* RToverUDP */
} PNIO_IOCR_PROP_ENUM;

typedef  struct
{
    PNIO_UINT32  SlotNum;             /* related slot */
    PNIO_UINT32  SubslotNum;          /* related subslot */
    PNIO_UINT32  reserved;            /* internal used */
} ATTR_PACKED PNIO_IOCS_TYPE;


/* IOCR */
typedef struct {
    PNIO_UINT32             CycleTime;
    PNIO_IOCR_TYPE_ENUM     Direction;          /* see PNIO_IOCR_TYPE_ENUM */
    PNIO_IOCR_PROP_ENUM     IOCRProperties;     /* see PNIO_IOCR_PROP_ENUM */

    PNIO_UINT32             SendClock;
    PNIO_UINT32             ReductioFactor;
    PNIO_UINT32             Phase;
    PNIO_UINT32             NumOfIoCs;      /* number of IoCs in IoCsList */
    PNIO_IOCS_TYPE *        pIoCsList;      /* pointer to array of IOCS */
    PNIO_UINT32             reserved[3];        /* internal use only */
} ATTR_PACKED PNIO_IOCR_TYPE;

/* AR Type */
typedef enum  {
    PNIO_AR_TYPE_SINGLE                 = 0x00000001,
    PNIO_AR_TYPE_SUPERVISOR             = 0x00000006,
    PNIO_AR_TYPE_SINGLE_RTC3            = 0x00000010
} PNIO_AR_TYPE_ENUM;


/* modules and submodules */

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

} ATTR_PACKED PNIO_SUBMOD_TYPE;

typedef struct {
    PNIO_UINT32           Api;            /* Api number  */
    PNIO_UINT32           SlotNum;        /* slot number */
    PNIO_UINT32           NumOfSubmod;
    PNIO_UINT32           ModProperties;  /* module properties */
    PNIO_UINT32           ModIdent;       /* module ident number */
    PNIO_SUBMOD_TYPE    * pSubList;       /* list of submodules */
} ATTR_PACKED PNIO_MODULE_TYPE;

typedef struct {
    PNIO_UINT16 BlockType;
    PNIO_UINT16 BlockLength;
    PNIO_UINT8  BlockVersionHigh;
    PNIO_UINT8  BlockVersionLow;
} PNIO_BLOCK_HEADER2;

typedef struct {
    PNIO_BLOCK_HEADER BlockHeader;              /* BlockType: 0x0020 */
    PNIO_UINT8        VendorIDHigh;
    PNIO_UINT8        VendorIDLow;
    PNIO_UINT8        OrderID[20];
    PNIO_UINT8        IM_Serial_Number[16];
    PNIO_UINT16       IM_Hardware_Revision;     /* (big-endian) */
    PNIO_UINT8        IM_Software_Revision[4];
    PNIO_UINT16       IM_Revision_Counter;      /* (big-endian) */
    PNIO_UINT16       IM_Profile_ID;            /* (big-endian) */
    PNIO_UINT16       IM_Profile_Specific_Type; /* (big-endian) */
    PNIO_UINT8        IM_Version_Major;
    PNIO_UINT8        IM_Version_Minor;
    PNIO_UINT16       IM_Supported;             /* Bitliste Bit 1 --> Supports IM1... (big-endian) */
} ATTR_PACKED PNIO_IM0_TYPE;

typedef struct {
    PNIO_BLOCK_HEADER BlockHeader;            /*  BlockType: 0x0021 */
    PNIO_UINT8        IM_Tag_Function[32];
    PNIO_UINT8        IM_Tag_Location[22];
 } ATTR_PACKED PNIO_IM1_TYPE;

typedef struct {
    PNIO_BLOCK_HEADER BlockHeader;           /*  BlockType: 0x0022 */
    PNIO_UINT8        IM_Date[16];
 } ATTR_PACKED PNIO_IM2_TYPE;

typedef struct {
    PNIO_BLOCK_HEADER BlockHeader;          /*  BlockType: 0x0023 */
    PNIO_UINT8        IM_Descriptor[54];
 } ATTR_PACKED PNIO_IM3_TYPE;

typedef struct {
    PNIO_BLOCK_HEADER BlockHeader;          /*  BlockType: 0x0024 */
    PNIO_UINT8        IM_Signature[54];
 } ATTR_PACKED PNIO_IM4_TYPE;

 enum PNIO_SUB_PROPERTIES_ENUM_USER {
	CM_SUB_PROP_USER_TYPE_MASK			= 0x0003,  /* Bit 0 - 1: SubmoduleProperties.Type */
	CM_SUB_PROP_USER_TYPE_NO_DATA		= 0x0000,
	CM_SUB_PROP_USER_TYPE_INPUT			= 0x0001,
	CM_SUB_PROP_USER_TYPE_OUTPUT			= 0x0002,
	CM_SUB_PROP_USER_TYPE_INPUT_OUTPUT	= 0x0003,

	CM_SUB_PROP_USER_SHARED_INP_MASK		= 0x0004,  /* Bit 2: SubmoduleProperties.SharedInput */
	CM_SUB_PROP_USER_SHARED_INP_NO		= 0x0000,
	CM_SUB_PROP_USER_SHARED_INP_YES		= 0x0004,

	CM_SUB_PROP_USER_REDUCE_INP_LEN_MASK = 0x0008,  /* Bit 3: SubmoduleProperties.ReduceInputSubmoduleDataLength */
	CM_SUB_PROP_USER_REDUCE_INP_LEN_NO   = 0x0000,  /* Use expected input SubmoduleDataLength for I-CR */
	CM_SUB_PROP_USER_REDUCE_INP_LEN_YES  = 0x0008,  /* Reduce input SubmoduleDataLength to zero for I-CR */

	CM_SUB_PROP_USER_REDUCE_OUT_LEN_MASK = 0x0010,  /* Bit 4: SubmoduleProperties.ReduceOutputSubmoduleDataLength */
	CM_SUB_PROP_USER_REDUCE_OUT_LEN_NO   = 0x0000,  /* Use expected output SubmoduleDatalength for O-CR */
	CM_SUB_PROP_USER_REDUCE_OUT_LEN_YES  = 0x0010,  /* Reduce output SubmoduleDatalength to zero for O-CR */

	CM_SUB_PROP_USER_DISCARD_IOXS_MASK   = 0x0020,  /* Bit 5: SubmoduleProperties.DiscardIOXS */
	CM_SUB_PROP_USER_DISCARD_IOXS_NO     = 0x0000,  /* Use IOXS-length */
	CM_SUB_PROP_USER_DISCARD_IOXS_YES    = 0x0020,  /* Treat IOXS-Length as 0 */

	CM_SUB_PROP_USER_UNCHECKED_6_7       = 0x00C0,  /* Bit 6 - 7: reserved_1, set to zero but do not check */

	CM_SUB_PROP_USER_RESERVED_8_15       = 0xFF00   /* Bit 8 - 15: reserved_2, set to zero and test for zero */
};

/* application relations */

typedef struct {
    void                * pNextAr;    /* pointer to next AR (don't care for application) */
    PNIO_UINT16           ArNumber;   /* ar - handle */
    PNIO_UINT16           SessionKey;
    PNIO_UINT32           NumOfIocr;  /* number of Iocr in IoCrList */
    PNIO_IOCR_TYPE      * pIoCrList;  /* pointer to Array of IOCR */
    PNIO_UINT32           NumOfMod;   /* number of Modules in ModList */
    PNIO_MODULE_TYPE    * pModList;   /* pointer to array of modules */
} ATTR_PACKED PNIO_AR_TYPE;

typedef struct {
    PNIO_UINT16   ArNumber;      /* AR-Handle, see PNIO_device_open() or SV_EVENT_CHECK_IND */
    PNIO_UINT16   SessionKey;    /* see CL_EVENT_IN_DATA_IND or SV_EVENT_CHECK_IND */
    PNIO_UINT32   Api;           /* application process identifier */
    PNIO_UINT16   SlotNum;       /* 0 = device substitute */
    PNIO_UINT16   SubslotNum;    /* 0 = module substitute */
    PNIO_UINT16   AlarmPriority; /* CM_ALARM_PRIORITY_LOW or CM_ALARM_PRIORITY_HIGH */
    PNIO_UINT16   AlarmType;     /* see CM_ALARM_TYPE_... */
    PNIO_UINT16   AlarmSequence; /* see "AlarmSpecifier" bits 0-10   */
    PNIO_UINT8    DiagChannelAvailable;  /* see "AlarmSpecifier" bit 11 */
    PNIO_UINT8    DiagGenericAvailable;  /* see "AlarmSpecifier" bit 12 */
    PNIO_UINT8    DiagSubmodAvailable;   /* see "AlarmSpecifier" bit 13 */
    PNIO_UINT8    Reserved;               /* see "AlarmSpecifier" bit 14 */
    PNIO_UINT8    ArDiagnosisState;      /* see "AlarmSpecifier" bit 15 */
    PNIO_UINT32   ModIdent;
    PNIO_UINT32   SubIdent;
    PNIO_UINT16   UserStructIdent;     /* user-structure-tag for alarm_data, see
                                         CM_ALARM_TAG_CHANNEL_DIAGNOSIS */
    PNIO_UINT16   UserAlarmDataLength; /* length of alarm_data */
    PNIO_UINT8  * UserAlarmData;       /* see Alarm-PDU, see CM_ALARM_OFFSET_DATA */
    PNIO_UINT32   CmPnioErr;           /* alarm-ack only, see the makro CM_PNIO_ERR_MAKE() in file cm_err.h */
} ATTR_PACKED PNIO_DEV_ALARM_DATA;


/* reason code for event indication of AR-Offline, AR-Abort */
typedef enum {
    PNIO_AR_REASON_NONE    =  0,
    PNIO_AR_REASON_RESERVED1 = 1, /* internal use */
    PNIO_AR_REASON_RESERVED2 = 2, /* internal use */
    /***/
    PNIO_AR_REASON_MEM     =  3, /* out of mem */
    PNIO_AR_REASON_FRAME   =  4, /* add provider or consumer failed */
    PNIO_AR_REASON_MISS    =  5, /* miss (consumer) */
    PNIO_AR_REASON_TIMER   =  6, /* cmi timeout */
    PNIO_AR_REASON_ALARM   =  7, /* alarm-open failed */
    PNIO_AR_REASON_ALSND   =  8, /* alarm-send.cnf(-) */
    PNIO_AR_REASON_ALACK   =  9, /* alarm-ack-send.cnf(-) */
    PNIO_AR_REASON_ALLEN   =  10, /* alarm-data too long */
    PNIO_AR_REASON_ASRT    =  11, /* alarm.ind(err) */
    PNIO_AR_REASON_RPC     = 12, /* rpc-client call.cnf(-) */
    PNIO_AR_REASON_ABORT   = 13, /* ar-abort.req */
    PNIO_AR_REASON_RERUN   = 14, /* re-run aborts existing */
    PNIO_AR_REASON_REL     = 15, /* got release.ind */
    PNIO_AR_REASON_PAS     = 16, /* device passivated */
    PNIO_AR_REASON_RMV     = 17, /* device/ar removed */
    PNIO_AR_REASON_PROT    = 18, /* protocol violation */
    PNIO_AR_REASON_NARE    = 19, /* NARE error */
    PNIO_AR_REASON_BIND    = 20, /* RPC-Bind error */
    PNIO_AR_REASON_CONNECT = 21, /* RPC-Connect error */
    PNIO_AR_REASON_READ    = 22, /* RPC-Read error */
    PNIO_AR_REASON_WRITE   = 23, /* RPC-Write error */
    PNIO_AR_REASON_CONTROL = 24, /* RPC-Control error */
    PNIO_AR_REASON_PULLPLUG  = 25, /* forbidden pull or plug after check.rsp and before in-data.ind */
    PNIO_AR_REASON_AP_RMV    = 26, /* deprecated! AP removed */
    PNIO_AR_REASON_LNK_DWN   = 27, /* link "down" */
    PNIO_AR_REASON_MMAC      = 28, /* could not register multicast-mac */
    PNIO_AR_REASON_SYNC      = 29, /* not synchronized (cannot start companion-ar) */
    PNIO_AR_REASON_TOPO      = 30, /* wrong topology (cannot start companion-ar) */
    PNIO_AR_REASON_DCP_NAME  = 31, /* dcp, station-name changed */
    PNIO_AR_REASON_DCP_RESET = 32, /* dcp, reset to factory-settings */
    PNIO_AR_REASON_PRM       = 33, /* deprecated! cannot start companion-AR because a 0x8ipp submodule in the first AR... */
                                   /* - has appl-ready-pending (erroneous parameterisation) */
                                   /* - is locked (no parameterisation) */
                                   /* - is wrong or pulled (no parameterisation) */
    PNIO_AR_REASON_IRDATA    = 34, /* no irdata record yet */
    /***/
    PNIO_AR_REASON_MAX = 255       /*!*/

} PNIO_AR_REASON ;

/* APDU status code for event indication of status change */
typedef enum  {
    PNIO_EVENT_APDU_STATUS_STOP            = 0x0001,
    PNIO_EVENT_APDU_STATUS_RUN             = 0x0002,
    PNIO_EVENT_APDU_STATUS_STATION_OK      = 0x0004,
    PNIO_EVENT_APDU_STATUS_STATION_PROBLEM = 0x0008,
    PNIO_EVENT_APDU_STATUS_PRIMARY         = 0x0010,
    PNIO_EVENT_APDU_STATUS_BACKUP          = 0x0020
} PNIO_APDU_STATUS_IND;

typedef enum {
    PNIO_MOD_STATE_NO_MODULE           = 0,
    PNIO_MOD_STATE_WRONG_MODULE        = 1,
    PNIO_MOD_STATE_PROPER_MODULE       = 2,
    PNIO_MOD_STATE_SUBSTITUTED_MODULE  = 3,
    /***/
    PNIO_MOD_STATE_RESERVED
} PNIO_MODULE_STATE;

/* the following defines are reading in the AR_INfO IND */
typedef enum {
    PNIO_SUB_STATE_IDENT_MASK                     = 0x7800,

    PNIO_SUB_STATE_IDENT_NO_SUBMODULE             = 0x1800,
    PNIO_SUB_STATE_IDENT_WRONG                    = 0x1000,
    PNIO_SUB_STATE_IDENT_SUBSTITUTE               = 0x0800,
    PNIO_SUB_STATE_IDENT_OK                       = 0x0000

} PNIO_SUBMODULE_STATE;

#define PNIO_SUB_STATE_FORMAT_1                     0x8000

/* the following defines are for use in the check.RSP */
typedef enum  { /* subset of cm_sub_state_enum */
    PNIO_SUB_CHECK_NO_SUBMODULE           = (PNIO_SUB_STATE_FORMAT_1 | PNIO_SUB_STATE_IDENT_NO_SUBMODULE),
    PNIO_SUB_CHECK_WRONG_SUBMODULE        = (PNIO_SUB_STATE_FORMAT_1 | PNIO_SUB_STATE_IDENT_WRONG),
    PNIO_SUB_CHECK_SUBSTITUTED_SUBMODULE  = (PNIO_SUB_STATE_FORMAT_1 | PNIO_SUB_STATE_IDENT_SUBSTITUTE),
    PNIO_SUB_CHECK_PROPER_SUBMODULE       = (PNIO_SUB_STATE_FORMAT_1 | PNIO_SUB_STATE_IDENT_OK)
}PNIO_SUB_CHECK_STATE;

/* the following defines are used in PNIO_sub_plug_ext */
#define PNIO_ALARM_TYPE_PLUG                      0x0004
#define PNIO_ALARM_TYPE_PLUG_WRONG                0x000A

/* the following defines are used in PNIO_sub_plug_ext_IM */
typedef enum  {       /* see I&M0FilterData (record index F840) */
	PNIO_PLUG_IM0_BITS_NOTHING   = 0x00, /* this submodule does not contain I&M data */
	PNIO_PLUG_IM0_BITS_SUBMODULE = 0x01, /* this submodule contains I&M data */
  PNIO_PLUG_IM0_BITS_SUBMODULE_REP_MODULE  = 0x03, /* modifier: this submodule stands for the representative module  */
  PNIO_PLUG_IM0_BITS_SUBMODULE_REP_MOD_DEV = 0x07  /* modifier: this submodule stands for the representative module and device */
}PNIO_PLUG_IM0_BITS;


/* geographical addressing */
typedef struct {
    PNIO_ADDR_TYPE  AddrType;
    PNIO_IO_TYPE    IODataType;
    union {
        PNIO_UINT32 reserved;
        struct {
            PNIO_UINT32 reserved1[2];
            PNIO_UINT32 Slot;
            PNIO_UINT32 Subslot;
            PNIO_UINT32 reserved2;
        } Geo;                   /* geographical address */
    } u;
} ATTR_PACKED PNIO_DEV_ADDR;

/* Alarm-Item.UserStructureIdentifier of an Alarm notification PDU */
/* values   from            0  to   0x7fff     are manufacturer specific */
#define PNIO_USR_STRUC_ID_CHANNEL_DIAG          0x8000  /* channel diagnostic, see IEC 61158 */
#define PNIO_USR_STRUC_ID_MULTI_MANUFAC_DIAG    0x8001  /* multiple channel diagnostic, see IEC 61158 */
#define PNIO_USR_STRUC_ID_EXTCHANNEL_DIAG       0x8002  /* extended channel diagnostic, see IEC 61158 */

/* PART 1:  functions, called by the user application */

/* annotation */
#define MAX_DEVICE_TYPE_LENGTH 25
#define MAX_ORDER_ID_LENGTH    20

typedef struct {
    PNIO_UINT8  deviceType [MAX_DEVICE_TYPE_LENGTH + 1];
    PNIO_UINT8  orderId [MAX_ORDER_ID_LENGTH + 1];
    PNIO_UINT16 hwRevision;
    PNIO_UINT8  swRevisionPrefix;
    PNIO_UINT16 swRevision1;
    PNIO_UINT16 swRevision2;
    PNIO_UINT16 swRevision3;
} ATTR_PACKED PNIO_ANNOTATION;

/* register callback functions */
/* EVENTS: */
/* all these functions are called by the pnio stack in case of a special alarm or event. */
/* the user can implement his own code into this functions. */

typedef  void (*PNIO_CBF_CHECK_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT32          Api,           /* in */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_DEV_ADDR      * pAddr,         /* geographical address */
        PNIO_UINT32        * pModIdent,     /* [out] Ptr to module identifier */
        PNIO_UINT16        * pModState,     /* [out] Ptr to module state */
        PNIO_UINT32        * pSubIdent,     /* [out] Ptr to submodule identifier */
        PNIO_UINT16        * pSubState);    /* [out] Ptr to submodule state */

typedef struct {
    PNIO_UINT32 TimeLow;
    PNIO_UINT16 TimeMid;
    PNIO_UINT16 TimeHiAndVersion;
    PNIO_UINT8  ClockSeqHiAndReserved;
    PNIO_UINT8  ClockSeqLow;
    PNIO_UINT8  Node[6];
} ATTR_PACKED PNIO_UUID_TYPE;


typedef  void (*PNIO_CBF_AR_CHECK_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT32          HostIp,        /* in */
        PNIO_UINT16          ArType,        /* see PNIO_AR_TYPE */
        PNIO_UUID_TYPE       ArUUID,
        PNIO_UINT32          ArProperties,
        PNIO_UUID_TYPE       CmiObjUUID,
        PNIO_UINT16          CmiStationNameLenght,
        PNIO_UINT8         * pCmiStationName,
        PNIO_AR_TYPE       * pAr);

typedef  void (*PNIO_CBF_AR_INFO_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_AR_TYPE       * pAr);    /* [in] AR properties, do not modify !!! */

/*  data exchange has been started */
typedef  void (*PNIO_CBF_AR_INDATA_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey);

typedef  void (*PNIO_CBF_AR_ABORT_IND)
       (PNIO_UINT32          DevHndl,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_AR_REASON       ReasonCode);  /*  AR abort after ArInData-indication*/

typedef  void (*PNIO_CBF_AR_OFFLINE_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_AR_REASON       ReasonCode);  /* AR abort before ArInData-indication */

typedef  void (*PNIO_CBF_APDU_STATUS_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_APDU_STATUS_IND ApduStatus);  /* status change indication */

typedef  void (*PNIO_CBF_PRM_END_IND)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_UINT32          Api,
        PNIO_UINT16          SlotNum,
        PNIO_UINT16          SubslotNum);

typedef  void (*PNIO_CBF_REQ_DONE)
       (PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          UserHndl,       /* user defined handle */
        PNIO_UINT32          Status,
        PNIO_ERR_STAT      * pPnioState);

typedef  void (*PNIO_CBF_CP_STOP_REQ)
       (PNIO_UINT32          DevHndl);      /* Handle for Multidevice */

typedef void (*PNIO_CBF_DEVICE_STOPPED)
       (PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          ErrorCode);

typedef  void (*PNIO_CBF_START_LED_FLASH)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT32          Frequency);

typedef  void (*PNIO_CBF_STOP_LED_FLASH)
       (PNIO_UINT32          DevHndl);      /* Handle for Multidevice */


/* READ RECORD, WRITE RECORD: */
typedef  void  (*PNIO_CBF_REC_READ)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_UINT32          SequenceNum,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT32          RecordIndex,
        PNIO_UINT32        * pBufLen,        /* in: length to read, out: length, read by user */
        PNIO_UINT8         * pBuffer,        /* buffer pointer */
        PNIO_ERR_STAT      * pPnioState);    /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                   ErrCode2), see IEC61158-6 */
 typedef  void  (*PNIO_CBF_REC_WRITE)
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_UINT32          SequenceNum,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT32          RecordIndex,
        PNIO_UINT32        * pBufLen,        /* in: length to write, out: length, written by user */
        PNIO_UINT8         * pBuffer,        /* buffer pointer */
        PNIO_ERR_STAT      * pPnioState);    /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                   ErrCode2), see IEC61158-6 */

/* data exchange functions */
typedef PNIO_IOXS    (*PNIO_CBF_DATA_WRITE) /* write data to IO stack (local ==> remote) */
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_DEV_ADDR      * pAddr,         /* geographical address */
        PNIO_UINT32          BufLen,        /* length of the submodule input data */
        PNIO_UINT8         * pBuffer,       /* Ptr to data buffer to write to */
        PNIO_IOXS            Iocs);         /* remote (io controller) consumer status */

typedef PNIO_IOXS    (*PNIO_CBF_DATA_READ)  /* read data from IO stack (remote ==> local) */
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_DEV_ADDR      * pAddr,         /* geographical address */
        PNIO_UINT32          BufLen,        /* length of the submodule input data */
        PNIO_UINT8         * pBuffer,       /* Ptr to data buffer to read from */
        PNIO_IOXS            Iops);         /* (io controller) provider status */

typedef enum {
        PNIO_MOD_PULL = 1,
        PNIO_MOD_PLUG = 2,
        PNIO_SUB_PULL = 3,
        PNIO_SUB_PLUG = 4
} PNIO_DEV_ACTION;

typedef  void  (*PNIO_CBF_PULL_PLUG_CONF)
       (PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ACTION      Action,         /* PNIO_MOD_PULL, PNIO_SUB_PULL ..  */
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT32          ErrorCode);

/* structure for callback function block */
typedef struct {
    PNIO_UINT32               size;                           /* size of struct = sizeof(PNIO_CBF_FUNCTIONS) */
    PNIO_CBF_DATA_WRITE       cbf_data_write;                 /* mandatory */
    PNIO_CBF_DATA_READ        cbf_data_read;                  /* mandatory */
    PNIO_CBF_REC_READ         cbf_rec_read;                   /* mandatory */
    PNIO_CBF_REC_WRITE        cbf_rec_write;                  /* mandatory */
    PNIO_CBF_REQ_DONE         cbf_alarm_done;                 /* mandatory */
    PNIO_CBF_CHECK_IND        cbf_check_ind;                  /* mandatory */
    PNIO_CBF_AR_CHECK_IND     cbf_ar_check_ind;               /* mandatory */
    PNIO_CBF_AR_INFO_IND      cbf_ar_info_ind;                /* mandatory */
    PNIO_CBF_AR_INDATA_IND    cbf_ar_indata_ind;              /* mandatory */
    PNIO_CBF_AR_ABORT_IND     cbf_ar_abort_ind;               /* mandatory */
    PNIO_CBF_AR_OFFLINE_IND   cbf_ar_offline_ind;             /* mandatory */
    PNIO_CBF_APDU_STATUS_IND  cbf_apdu_status_ind;            /* mandatory */
    PNIO_CBF_PRM_END_IND      cbf_prm_end_ind;                /* mandatory */
    PNIO_CBF_CP_STOP_REQ      cbf_cp_stop_req;                /* mandatory */
    PNIO_CBF_DEVICE_STOPPED   cbf_device_stopped;             /* mandatory */
    PNIO_CBF_START_LED_FLASH  cbf_start_led_flash;            /* optional */
    PNIO_CBF_STOP_LED_FLASH   cbf_stop_led_flash;             /* optional */
    PNIO_CBF_PULL_PLUG_CONF   cbf_pull_plug_conf;             /* optional */
} ATTR_PACKED PNIO_CBF_FUNCTIONS;

/* spelling error, compatibility fix */
#define PNIO_CFB_FUNCTIONS PNIO_CBF_FUNCTIONS

#ifdef __cplusplus
extern "C" {
#endif

#define PNIO_DEVICE_SET_MAXAR_BY_TYPE(MaxArClass1_2, MaxArClass3, MaxArDeviceAccess) \
    ( ((MaxArDeviceAccess & 0xF)<<8) | ((MaxArClass3 & 0xF)<<4) | (MaxArClass1_2 & 0xF) )

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_open (
        PNIO_UINT32          CpIndex,        /* [in]  reserved for futere use */
        PNIO_UINT32          ExtPar,         /* [in]  reserved for futere use */
        PNIO_UINT16          VendorId,       /* [in]  vendor id (see gsdml file) */
        PNIO_UINT16          DeviceId,       /* [in]  device id (see gsdml file) */
        PNIO_UINT16          InstanceId,     /* [in]  InstanceId */
        PNIO_UINT32          MaxAR,          /* [in]  max count AR */
        PNIO_ANNOTATION    * pDevAnnotation, /* [in]  annotation */
        PNIO_CBF_FUNCTIONS * pCbf,           /* [in]  cbf */
        PNIO_UINT32        * pDevHndl);      /* [out] device handle */

#ifdef PNIO_DEVICE_IF_EXT_01
PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_open_ext (
        PNIO_UINT32          CpIndex,        /* [in]  reserved for futere use */
        PNIO_UINT32          ExtPar,         /* [in]  reserved for futere use */
        PNIO_UINT16          VendorId,       /* [in]  vendor id (see gsdml file) */
        PNIO_UINT16          DeviceId,       /* [in]  device id (see gsdml file) */
        PNIO_UINT16          InstanceId,     /* [in]  InstanceId */
        PNIO_UINT32          MaxAR,          /* [in]  max count AR */
        PNIO_ANNOTATION    * pDevAnnotation, /* [in]  annotation */
        PNIO_CBF_FUNCTIONS * pCbf,           /* [in]  cbf */
        PNIO_UINT16          highSlotNum,    /* [in]  highest valid slot nr */
        PNIO_UINT16          highSubslotNum, /* [in]  highest valid subslot nr */
        PNIO_UINT32        * pDevHndl);      /* [out] device handle */
#endif /* PNIO_DEVICE_IF_EXT_01 */

/*----------------------------------------------------------------------------*/

/* strcucture defnitionen */
typedef struct pnio_list_entry_tag {
    struct pnio_list_entry_tag  * Flink; /* forward link */
    struct pnio_list_entry_tag  * Blink; /* backward link */
}PNIO_LIST_ENTRY_TYPE;

typedef struct pnio_appl_ready_tag {
    PNIO_LIST_ENTRY_TYPE  ap_list;  /* with list from type PNIO_APPL_READY_AP_TYPE  */
} PNIO_APPL_READY_LIST_TYPE;

typedef struct pnio_appl_ready_ap_tag {
    PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT32  api;
    PNIO_LIST_ENTRY_TYPE  module_list;        /* with list from type PNIO_APPL_READY_MODULE_TYPE  */
} PNIO_APPL_READY_AP_TYPE;

typedef struct pnio_appl_ready_module_tag {
    PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT16  slot_nr;
    PNIO_LIST_ENTRY_TYPE  submodule_list;     /* with list from type PNIO_APPL_READY_SUBMODULE_TYPE  */
} PNIO_APPL_READY_MODULE_TYPE;

typedef struct pnio_appl_redy_submodule_tag {
    PNIO_LIST_ENTRY_TYPE  link;
    PNIO_UINT16  subslot_nr;
} PNIO_APPL_READY_SUBMODULE_TYPE;

/* device control macros */

#define PNIO_PTR_ARE_EQUAL(ptr1_,  ptr2_) \
   ( (PNIO_UINT32)(ptr1_) == (PNIO_UINT32)(ptr2_) )

#define PNIO_LIST_INITIALIZE(ListHead_) {\
	(ListHead_)->Flink = (ListHead_)->Blink = (ListHead_);\
	}

#define PNIO_LIST_IS_EMPTY(ListHead_) (\
	PNIO_PTR_ARE_EQUAL((ListHead_)->Flink, (ListHead_))\
	)

#define PNIO_LIST_REMOVE_ENTRY(Entry_) {\
	PNIO_LIST_ENTRY_TYPE *EX_Entry_;\
	EX_Entry_ = (Entry_);\
	EX_Entry_->Blink->Flink = EX_Entry_->Flink;\
	EX_Entry_->Flink->Blink = EX_Entry_->Blink;\
	}

#define PNIO_LIST_REMOVE_HEAD(ListHead_, Entry_, Type_) {\
	(Entry_) = (Type_)((ListHead_)->Flink);\
	PNIO_LIST_REMOVE_ENTRY((ListHead_)->Flink);\
	}

#define PNIO_LIST_INSERT_HEAD(ListHead_, Entry_) {\
	(Entry_)->Flink = (ListHead_)->Flink;\
	(Entry_)->Blink = (ListHead_);\
	(ListHead_)->Flink->Blink = (Entry_);\
	(ListHead_)->Flink = (Entry_);\
	}

#define PNIO_LIST_INSERT_TAIL(ListHead_, Entry_) {\
	(Entry_)->Flink = (ListHead_);\
	(Entry_)->Blink = (ListHead_)->Blink;\
	(ListHead_)->Blink->Flink = (Entry_);\
	(ListHead_)->Blink = (Entry_);\
	}

#define PNIO_LIST_INSERT_BEFORE(Existing_, Entry_) {\
	PNIO_LIST_INSERT_TAIL (Existing_, Entry_);\
	}

#define PNIO_LIST_FIRST(ListHead_, Type_) (\
	(Type_)(PNIO_LIST_IS_EMPTY(ListHead_) ? NULL : (ListHead_)->Flink)\
	)

#define PNIO_LIST_NEXT(ListHead_, Entry_, Type_) (\
	(Type_)(PNIO_PTR_ARE_EQUAL((Entry_)->Flink, (ListHead_)) ? NULL : (Entry_)->Flink)\
	)

#define PNIO_LIST_PREV_OR_HEAD(Entry_, Type_) (\
	(Type_)((Entry_)->Blink)\
	)

#define PNIO_LIST_LENGTH(ListHead_, LenPtr_) {\
	PNIO_LIST_ENTRY_TYPE* el = PNIO_LIST_FIRST (ListHead_, PNIO_LIST_ENTRY_TYPE*);\
	*(LenPtr_) = 0;\
	while (! PNIO_PTR_ARE_EQUAL(el,NULL)) {\
		*(LenPtr_) += 1;\
		el = PNIO_LIST_NEXT (ListHead_, el, PNIO_LIST_ENTRY_TYPE);\
	}}

#define PNIO_LIST_APPEND(ListHead_, ListHeadToAppend_) {\
	if (! PNIO_LIST_IS_EMPTY (ListHeadToAppend_)) {\
		(ListHead_)->Blink->Flink = (ListHeadToAppend_)->Flink;\
		(ListHeadToAppend_)->Flink->Blink = (ListHead_)->Blink;\
		(ListHead_)->Blink = (ListHeadToAppend_)->Blink;\
		(ListHead_)->Blink->Flink = (ListHead_);\
		PNIO_LIST_INITIALIZE (ListHeadToAppend_);\
	}}

/*----------------------------------------------------------------------------*/

/* device control */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_set_appl_state_ready(
        PNIO_UINT32          DevHndl,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_APPL_READY_LIST_TYPE * pList);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_ar_abort(
        PNIO_UINT32          DevHndl,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_close(
        PNIO_UINT32          DevHndl);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_start(
        PNIO_UINT32          DevHndl);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_stop(
        PNIO_UINT32          DevHndl);

/* set device state  CLEAR/OPERATE and OK/STATION_PROBLEM */
#define PNIO_DEVSTAT_OK                     1
#define PNIO_DEVSTAT_STATION_PROBLEM        2

PNIO_UINT32 PNIO_CODE_ATTR PNIO_set_dev_state(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          DevState);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_api_add(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_UINT16          MaxSlots,
        PNIO_UINT16          MaxSubslots
        );

#ifdef PNIO_DEVICE_IF_EXT_01
PNIO_UINT32 PNIO_CODE_ATTR PNIO_api_add_ext (
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api);
#endif /* PNIO_DEVICE_IF_EXT_01 */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_api_remove(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api);

/* plug and pull modules and submodules */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_mod_pull(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr);   /* geographical or logical address */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_pull(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice old function */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,
        PNIO_UINT32          SubIdent);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug_ext(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,
        PNIO_UINT32          SubIdent,
        PNIO_UINT32          AlarmType);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug_ext_IM(
        PNIO_UINT32         DevHndl,         /* Handle for Multidevice */
        PNIO_UINT32         Api,
        PNIO_DEV_ADDR       *pAddr,
        PNIO_UINT32         SubIdent,
        PNIO_UINT32         AlarmType,
        PNIO_PLUG_IM0_BITS  IM0_bits);


PNIO_UINT32 PNIO_CODE_ATTR PNIO_mod_plug(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,
        PNIO_UINT32          ModIdent);

/* set diagnostic data in pnio stack */
#define PNIO_DIAG_CHANPROP_TYPE_SUBMOD                     0x00
#define PNIO_DIAG_CHANPROP_TYPE_1BIT                       0x01
#define PNIO_DIAG_CHANPROP_TYPE_2BIT                       0x02
#define PNIO_DIAG_CHANPROP_TYPE_4BIT                       0x03
#define PNIO_DIAG_CHANPROP_TYPE_BYTE                       0x04
#define PNIO_DIAG_CHANPROP_TYPE_WORD                       0x05
#define PNIO_DIAG_CHANPROP_TYPE_DWORD                      0x06
#define PNIO_DIAG_CHANPROP_TYPE_LWORD                      0x07

/* This attribute indicates the the Channel Diagnosis appears */
#define PNIO_DIAG_CHAN_SPEC_ERROR_APPEARS                        0x0001   /* DiagnosisAlarm and Read Device Diagnosis */
#define PNIO_DIAG_CHAN_SPEC_ERROR_DISAPP_FREE                    0x0002   /* DiagnosisAlarm and Read Device Diagnosis */
#define PNIO_DIAG_CHAN_SPEC_ERROR_DISAPP_REMAIN                  0x0003   /* only DiagnosisAlarm */

/* This attribute indicates the direction of the channel to which the Channel Diagnosis object is related */
#define PNIO_DIAG_CHAN_DIRECTION_MANUFACTURE                     0x0000
#define PNIO_DIAG_CHAN_DIRECTION_INPUT                           0x0001
#define PNIO_DIAG_CHAN_DIRECTION_OUTPUT                          0x0002
#define PNIO_DIAG_CHAN_DIRECTION_BIDIRECT                        0x0003

/* This attribute indicates the error type of the Channel Related Diagnosis */
#define PNIO_DIAG_CHAN_ERROR_SHORT_CIRCUIT                       0x0001    /* short circuit */
#define PNIO_DIAG_CHAN_ERROR_UNDERVOLTAGE                        0x0002    /* Undervoltage */
#define PNIO_DIAG_CHAN_ERROR_OVERVOLTAGE                         0x0003    /* Overvoltage */
#define PNIO_DIAG_CHAN_ERROR_OVERLOAD                            0x0004    /* Overload  */
#define PNIO_DIAG_CHAN_ERROR_OVERTEMPERATURE                     0x0005    /* Overtemperature */
#define PNIO_DIAG_CHAN_ERROR_LINE_BREAK                          0x0006    /* line break */
#define PNIO_DIAG_CHAN_ERROR_UPPER_LIMIT_VALUE_EXCEEDED          0x0007    /* upper limit value exceeded */
#define PNIO_DIAG_CHAN_ERROR_LOWER_LIMIT_VALUE_EXCEEDED          0x0008    /* lower limit value exceeded */
#define PNIO_DIAG_CHAN_ERROR                                     0x0009    /* Error */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_PARAM         0x0010    /* parametrization fault */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_SUPPLY        0x0011    /* power supply fault */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_FUSE          0x0012    /* fuse blown / open */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_MANUF         0x0013    /* manufacturer specific  */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_GROUND        0x0014    /* ground fault */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_REF_POINT     0x0015    /* reference point lost */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_PROCESS       0x0016    /* process event lost / sampling error" */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_THRES         0x0017    /* threshold warning */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_OUTPUT        0x0018    /* output disabled */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_SAFETY        0x0019    /* safety event */
#define PNIO_DIAG_CHAN_ERROR_MANUFACTURER_SPECIFIC_EXTERNAL      0x001A    /* external fault */

#define PNIO_DIAG_CHAN_ERROR_DATA_TRANSMISSION                   0x8000    /* Data Transmission Impossible */
#define PNIO_DIAG_CHAN_ERROR_REMOTE_MISMATCH                     0x8001    /* Remote Mismatch */
#define PNIO_DIAG_CHAN_ERROR_MEDIA_REDUND                        0x8002    /* Media Redundancy Mismatch */
#define PNIO_DIAG_CHAN_ERROR_SYNC_MISMATCH                       0x8003    /* Sync Mismatch */
#define PNIO_DIAG_CHAN_ERROR_ISOCH_MISMATCH                      0x8004    /* IsochronousMode Mismatch */
#define PNIO_DIAG_CHAN_ERROR_MULTICAST_CR                        0x8005    /* Multicast CR Mismatch */
#define PNIO_DIAG_CHAN_ERROR_MULTICAST_PROV                      0x8006    /* Multicast Provider Status Mismatch */


PNIO_UINT16 PNIO_CODE_ATTR PNIO_build_channel_properties(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT16          Type,           /* channel properties.type */
        PNIO_UINT16          Spec,           /* channel properties.specifier */
        PNIO_UINT16          Dir);           /* channel properties.direction */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_channel_add(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          ChannelNum,     /* channel number */
        PNIO_UINT16          ChannelProp,    /* channel properties */
        PNIO_UINT32          ChannelErrType, /* channel error type */
        PNIO_UINT16          DiagTag);       /* user defined diag tag != 0 */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_channel_remove(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          DiagTag);       /* user defined diag tag, 0 = remove all */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_generic_add(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          ChannelNum,
        PNIO_UINT16          ChannelProp,
        PNIO_UINT16          DiagTag,        /* user defined diag tag != 0 */
        PNIO_UINT16          UserStructIdent,/* structure of info data (user defined) */
        PNIO_UINT8         * pInfoData,      /* diag data */
        PNIO_UINT32          InfoDataLen);   /* length of diag data in bytes */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_generic_remove(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          DiagTag);       /* user defined diag tag, 0 = remove all */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_ext_channel_add(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          ChannelNum,     /* channel number */
        PNIO_UINT16          ChannelProp,    /* channel properties */

        PNIO_UINT16          ChannelErrType,     /* channel error type */
        PNIO_UINT16          ExtChannelErrType,  /* ext channel error type */
        PNIO_UINT32          ExtChannelAddValue, /* ext channel add value */
        PNIO_UINT16          DiagTag);           /* user defined diag tag != 0 */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_ext_channel_remove(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT16          DiagTag);       /* user defined diag tag, 0 = remove all */

/* diag alarms and process alarms */
#define  PNIO_STATE_ALARM_APPEARS   1  /* new diag-alarm has appeared */
#define  PNIO_STATE_ALARM_DISAPPEARS 2 /* diag alarm has disappeared */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_process_alarm_send(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_DEV_ADDR      * pAddr,          /* geographical or address */
        PNIO_UINT8         * pData,         /* AlarmItem.Data */
        PNIO_UINT32          DataLen,        /* length of AlarmItem.Data */
        PNIO_UINT16          UserStructIdent,/* AlarmItem.UserStructureIdentifier, s. IEC61158-6 */
        PNIO_UINT32          UserHndl);      /* user defined handle */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_alarm_send(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_UINT32          AlarmState,     /* alarm appears/disappears */
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT8         * pData,         /* AlarmItem.Data */
        PNIO_UINT32          DataLen,        /* length of AlarmItem.Data */
        PNIO_UINT16          UserStructIdent,/* AlarmItem.UserStructureIdentifier, s. IEC61158-6 */
        PNIO_UINT32          UserHndl);      /* user defined handle */

PNIO_UINT32 PNIO_CODE_ATTR PNIO_ret_of_sub_alarm_send(
        PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
        PNIO_UINT32          Api,
        PNIO_UINT16          ArNumber,
        PNIO_UINT16          SessionKey,
        PNIO_DEV_ADDR      * pAddr,          /* geographical address */
        PNIO_UINT32          UserHndl);      /* user defined handle */


/* data exchange functions, called by the user */
PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_read(
        PNIO_UINT32          DevHndl);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_read_ext(
        PNIO_UINT32        DevHndl,        /* Handle for Multidevice */
        PNIO_DEV_ADDR*     pAddr,          /* geographical address */
        PNIO_ACCESS_ENUM   AccessType);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_write(
        PNIO_UINT32          DevHndl);

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_write_ext(
        PNIO_UINT32        DevHndl,        /* Handle for Multidevice */
        PNIO_DEV_ADDR*     pAddr,          /* geographical address */
        PNIO_ACCESS_ENUM   AccessType);

#ifdef __cplusplus
}
#endif

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#endif /* PNIOUSRD_H */
