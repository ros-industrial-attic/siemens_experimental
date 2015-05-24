/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           : PNIO                                                */
/*   Filename          : easy_dev_cert.c                                     */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Description: This device applcation implements "PNIO Device" for          */
/*             PNO certification.                                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all      */
/*             actions should be synchronized.                               */
/*---------------------------------------------------------------------------*/
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

#ifdef WIN32
    #include <windows.h>
    #include <stdio.h>
    #include <conio.h>
	#include <time.h>

#else
    #include <stdio.h>
    #include <unistd.h>
    #include <stdlib.h>
    #include <poll.h>
    #include <termios.h>
    #include <string.h>
    #include <time.h>
    #define Sleep(x) usleep(x*1000)
#endif

#include <assert.h>

#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

#include "easy_dev_cfg.h"

//#define PROFI_ENERGY_ENABLE // enable PROFIENERGY extensions
//#define PE_PRINTF printf
//#define PE_PRINTF 

/*----------------------------------------------------------------------------------------------------*/
/*    FUNCTION PROTOTYPES                                                                             */
/*----------------------------------------------------------------------------------------------------*/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub);
int getCharWithTimeout();
void do_after_prm_end_ind_cbf();

#ifdef PROFI_ENERGY_ENABLE
void PNIO_cbf_rec_write_pe  (
    PNIO_UINT32          DevHndl,
    PNIO_UINT32          Api,
    PNIO_UINT16          ArNumber,
    PNIO_UINT16          SessionKey,
    PNIO_UINT32          SequenceNum,
    PNIO_DEV_ADDR      * pAddr,          /* geographical address */
    PNIO_UINT32          RecordIndex,
    PNIO_UINT32        * pBufLen,        /* [in, out] in: length to read, out: length, read by user */
    PNIO_UINT8         * pBuffer,        /* [in] buffer pointer */
    PNIO_ERR_STAT      * pPnioState)  ;   /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                ErrCode2), see IEC61158-6 */
void PNIO_cbf_rec_read_pe  (
    PNIO_UINT32          DevHndl,
    PNIO_UINT32          Api,
    PNIO_UINT16          ArNumber,
    PNIO_UINT16          SessionKey,
    PNIO_UINT32          SequenceNum,
    PNIO_DEV_ADDR      * pAddr,          /* geographical address */
    PNIO_UINT32          RecordIndex,
    PNIO_UINT32        * pBufLen,        /* [in, out] in: length to read, out: length, read by user */
    PNIO_UINT8         * pBuffer,        /* [out] buffer pointer */
    PNIO_ERR_STAT      * pPnioState) ;    /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, */

 PNIO_UINT32  PE_Initialize(void);

#endif // PROFI_ENERGY_ENABLE

/*----------------------------------------------------------------------------------------------------*/
/*    DEFINES                                                                                         */
/*----------------------------------------------------------------------------------------------------*/
#define NUMOF_SLOTS              4     /* slot 0...8   +  1 */
#define NUMOF_SUBSLOTS            2     /* Every slot has 1 subslot */
#define NUMOF_BYTES_PER_SUBSLOT   256   /* Maximum data length as configured in the sample */
#define NUMOF_PERSISTENT_IM_ITEMS 6
#define LINE_LENGTH_MAX           255   /* Maximum line length of persistent im data file */

#define DS_NUM_REC_INP_DATA_OBJ_ELEM  0x8028
#define DS_NUM_REC_OUTP_DATA_OBJ_ELEM 0x8029

#define STRNCMP_EQUAL 0

#ifndef SWAP16
    #define SWAP_16(var)                  \
    (  ((( (var)) & 0xFF00L) >>  8)   \
    + ((( (var)) & 0x00FFL) << 8))
#endif /* SWAP_16 */

#ifndef SWAP_32
    #define SWAP_32(var)                     \
    (  ((( (var)) & 0xFF000000L) >> 24)  \
    + ((( (var)) & 0x00FF0000L) >>  8)   \
    + ((( (var)) & 0x0000FF00L) <<  8)   \
    + ((( (var)) & 0x000000FFL) << 24))
#endif /* SWAP_32 */

// todo: move to iobase header
///////////////////////////////////////////////////////////////////////////////////////////////////////
#define LEN_IM_ORDERID 20
#define LEN_IM_SERIAL_NUMBER 16
#define LEN_IM_SOFTWARE_REVISION 4
#define LEN_IM_TAG_FUNCTION 32
#define LEN_IM_TAG_LOCATION 22
#define LEN_IM_DATE 16
#define LEN_IM_DESCRIPTOR 54
#define LEN_IM_SIGNATURE 54



/*----------------------------------------------------------------------------------------------------*/
/*    TYPES                                                                                           */
/*----------------------------------------------------------------------------------------------------*/

//Datentyp fuer die Diagnosedaten
typedef struct {
    unsigned short chanNum;
    unsigned short chanProp;
    unsigned short chanErrType;
} diagAlarmData_t;

typedef struct tag_IM_DATA {
    PNIO_IM0_TYPE   im0;
    PNIO_IM1_TYPE   im1;
    PNIO_IM2_TYPE   im2;
    PNIO_IM3_TYPE   im3;
    PNIO_IM4_TYPE   im4;
} IM_DATA;

typedef enum {
    IM0 = 0xAFF0,
    IM1 = 0xAFF1,
    IM2 = 0xAFF2,
    IM3 = 0xAFF3,
    IM4 = 0xAFF4
} IM0_idx_e;

typedef enum {
    IM_Revision_Counter =       0,
    IM_Tag_Function =           1,
    IM_Tag_Location =           2,
    IM_Date =                   3,
    IM_Descriptor =             4,
    IM_Signature =              5
} IM_PERSISTENT_DATA_e;

typedef enum {
    IM_EQUAL = 0,
    IM_READ_DIFFER = 1,
    IM_WRITE_DIFFER = 2,
    IM_READ_WRITE_DIFFER = 3
} IM_CMP_E;

typedef struct tag_IM_PERSISTENT_DATA { /* persistent IM data */
    PNIO_UINT16     IM_Revision_Counter;
    PNIO_UINT8      IM_Tag_Function[LEN_IM_TAG_FUNCTION];
    PNIO_UINT8      IM_Tag_Location[LEN_IM_TAG_LOCATION];
    PNIO_UINT8      IM_Date[LEN_IM_DATE];
    PNIO_UINT8      IM_Descriptor[LEN_IM_DESCRIPTOR];
    PNIO_UINT8      IM_Signature[LEN_IM_SIGNATURE];
} IM_PERSISTENT_DATA;

/*----------------------------------------------------------------------------------------------------*/
/*    GLOBALS                                                                                         */
/*----------------------------------------------------------------------------------------------------*/
PNIO_UINT32 g_hDevice = 0;              /* global handle for device */
PNIO_UINT16 g_SessionKey = 0;           /* session identifier, this is obtained in application-relation
                                           information callback i.e. PNIO_cbf_ar_info_ind. */
PNIO_UINT16 g_arNumber = 0;             /* application relation number */





















/* Global variables to count callback events */
volatile static PNIO_UINT32  g_OpFaultCount      = 0;
volatile static PNIO_UINT32  g_StartOpCount      = 0;
volatile static PNIO_UINT32  g_DataExchangeCount = 0;

/* Global variables to count errors */
volatile static PNIO_UINT32  g_readErrors        = 0;
volatile static PNIO_UINT32  g_writeErrors       = 0;
volatile static PNIO_UINT32  g_badRemoteState    = 0;

volatile int first_startop_done = 0;
volatile static int g_bIsochron = 0;
volatile int g_DeviceReady = 0;        /* application ready already sent flag == AR establishing */

int stopreq=0, g_pno_test=0;   /* 'g_pno_test' is a flag for certification test (see below) */

/*
The structure described below contains all the module/submodule information. For simplicity, this structure
is hard-coded in the configuration file, in the actual program it can be built via the '.ini' file.
*/
static device_data_set_t g_device_data_set[] =
{
    DEVICE_DATA
};

static device_data_t *g_device_data = NULL;
static int gDevArraySize = 0;  /* Total no of slots as configured in the sample Step7 project */
int idxTbl[DEVICE_DATA_ENTRIES_MAX];                 /* an array of slot ids, sub-slot entries will contain -1 */

/*
Some callback-flags for the callbacks we have to wait for
*/
volatile static int AR_INFO_IND_flag = 0;             /* Callback: Verbindung zum Controller ist aufgebaut */
volatile static int PRM_END_IND_flag = 0;             /* Callback: Ende Parametrierphase */
volatile static int INDATA_IND_flag = 0;              /* Callback: erste Datenuebermittlung von IO-Controller*/
volatile static int DEVICE_STOPPED_flag = 0;          /* Callback: Geraet wurde angehalten */


/**** Output Data  (IO Controller ==> IO Device) */
PNIO_UINT8     OutData        [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    OutDatLen      [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     OutDatIocs     [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     OutDatIops     [NUMOF_SLOTS][NUMOF_SUBSLOTS];

/**** Input Data  (IO Device ==> IO Controller) */
PNIO_UINT8     InData         [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    InDatLen       [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     InDatIops      [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     InDatIocs      [NUMOF_SLOTS][NUMOF_SUBSLOTS];

PNIO_BOOL SubmodulesThatExistInStep7Project[NUMOF_SLOTS][NUMOF_SUBSLOTS];
void InitSubmoduleExistsInStep7Project(void)
{
    int slot,subslot;
    for (slot=0; slot<NUMOF_SLOTS; slot++)
    {
        for (subslot=0; subslot<NUMOF_SUBSLOTS;subslot++)
        {
            SubmodulesThatExistInStep7Project[slot][subslot] = PNIO_TRUE;
        }
    }
}
PNIO_BOOL GetSubmoduleExistsInStep7Project(PNIO_UINT32 slot, PNIO_UINT32 subslot)
{
    if ((slot < NUMOF_SLOTS) && (subslot < NUMOF_SUBSLOTS))
    {
        return SubmodulesThatExistInStep7Project[slot][subslot];
    }
    return PNIO_FALSE;
}
void SetSubmoduleDoesntExistInStep7Project(PNIO_UINT32 slot, PNIO_UINT32 subslot)
{
    if ((slot < NUMOF_SLOTS) && (subslot < NUMOF_SUBSLOTS))
    {
        printf("  Submodule %u/%u doesn't exist both in device configuration and step7 project\n", slot, subslot);
        SubmodulesThatExistInStep7Project[slot][subslot] = PNIO_FALSE;
    }
    else
    {
        printf("  Submodule %u/%u exceeds NUMOF_SLOTS/NUMOF_SUBSLOTS define\n", slot, subslot);
    }
}


/**** I&M-Data */
static IM_DATA im_work[DEVICE_DATA_ENTRIES_MAX];             /* current IM data */
static const char *name_IM_PERSISTENT_DATA[] = {    /* names of persistent IM data items */
    "IM_Revision_Counter",
    "IM_Tag_Function",
    "IM_Tag_Location",
    "IM_Date",
    "IM_Descriptor",
    "IM_Signature"
};
static const IM_DATA im_default =   /* default IM data */
{
    {   /* I&M0 */
        { 0x0020,                     /* BlockHeader.BlockType */
          0x0038,                     /* BlockHeader.BlockLength */
          0x01,                       /* BlockHeader.BlockVersionHigh */
          0x00},                      /* BlockHeader.BlockVersionLow */
          0x00,                       /* VendorIDHigh */
          0x2A,                       /* VendorIDLow */
        {'6','G','K','1',' ',
         '1','6','0','-','4',
         'A','A','0','1',' ',
              ' ',' ',' ',' ',' '},      /* OrderID[20] -> CP1616 */
        {'0','0','0','-','0',
         '0','0','-','0','0',
         '0','-','0','0','0',
         '1'},                      /* IM_Serial_Number[16] */
          ANNOT_HW_REV,               /* IM_Hardware_Revision */
        {'V', 0x02, 0x06, 0x00},    /* IM_Software_Revision[4] */
          0x0000,                     /* IM_Revision_Counter */
          0x0000,                     /* IM_Profile_ID -> Generic Device */
          0x0004,                     /* IM_Profile_Specific_Type -> Communication Module */
          0x01,                       /* IM_Version_Major */
          0x01,                       /* IM_Version_Minor */
          0x001e                      /* IM_Supported -> IM0 bis IM4 */
    },
    {   /* I&M1 */
        { 0x0021,                     /* BlockHeader.BlockType */
          0x0038,                     /* BlockHeader.BlockLength */
          0x01,                       /* BlockHeader.BlockVersionHigh */
          0x00},                      /* BlockHeader.BlockVersionLow */
        {' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' '},                  /* IM_Tag_Function[32] */
        {' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' '}                   /* IM_Tag_Location[22] */
    },
    {  /* I&M2 */
        { 0x0022,                     /* BlockHeader.BlockType */
          0x0012,                     /* BlockHeader.BlockLength */
          0x01,                       /* BlockHeader.BlockVersionHigh */
          0x00},                      /* BlockHeader.BlockVersionLow */
        {' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' '}                       /* IM_Date[16] */
    },
    {  /* I&M3 */
        { 0x0023,                     /* BlockHeader.BlockType */
          0x0038,                     /* BlockHeader.BlockLength */
          0x01,                       /* BlockHeader.BlockVersionHigh */
          0x00},                      /* BlockHeader.BlockVersionLow */
        {' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' ',' ',
         ' ',' ',' ',' '}           /* IM_Descriptor[54] */
    },
    {   /* I&M4 */
        { 0x0024,                     /* BlockHeader.BlockType */
          0x0038,                     /* BlockHeader.BlockLength */
          0x01,                       /* BlockHeader.BlockVersionHigh */
          0x00},                      /* BlockHeader.BlockVersionLow */
        {'\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0','\0',
         '\0','\0','\0','\0'}           /* IM_Signature[54] */
    }
};

#define DS_BLOCK_TYPE_SUBST_VALUE           0x0014
#define DS_BLOCK_TYPE_INP_DATA_OBJ_ELEM     0x0015
#define DS_BLOCK_TYPE_OUTP_DATA_OBJ_ELEM    0x0016
#define IEC_DS_VERSION_HIGH_1               0x01
#define IEC_DS_VERSION_LOW_0                0x00
#define SUBST_ACTIVE_FLAG_OFF               0x0000



static PNIO_UINT8 IoxsIoBaseToNet(PNIO_IOXS ioxs)
{
    return((ioxs==PNIO_S_GOOD)?0x80:0x00);
}

static void DsInit_InputDataObject(IEC_DS_READ_INP_DATA_OBJ_ELEM * pContent, PNIO_DEV_ADDR* pAddr)
{
    PNIO_UINT16 ioDataLength = (PNIO_UINT16)InDatLen[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot];

    /* block header */
    pContent->dsBlockType = SWAP_16(DS_BLOCK_TYPE_INP_DATA_OBJ_ELEM);
    pContent->dsBlockLength =
      sizeof(pContent->dsBlockVersionHi)
    + sizeof(pContent->dsBlockVersionLo)
    + sizeof(pContent->lengthIocs)
    + sizeof(pContent->iocs)
    + sizeof(pContent->lengthIops)
    + sizeof(pContent->iops)
    + sizeof(pContent->lengthData)
    + ioDataLength;
    pContent->dsBlockLength = SWAP_16(pContent->dsBlockLength);
    pContent->dsBlockVersionHi = IEC_DS_VERSION_HIGH_1;
    pContent->dsBlockVersionLo = IEC_DS_VERSION_LOW_0;

    /* block body */
    pContent->lengthIocs = sizeof (PNIO_UINT8);
    pContent->iocs = IoxsIoBaseToNet(InDatIocs[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot]);
    pContent->lengthIops = sizeof (PNIO_UINT8);
    pContent->iops = IoxsIoBaseToNet(InDatIops[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot]);
    pContent->lengthData = SWAP_16(ioDataLength);
    pContent->pData = &InData[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot][0];
}

static void DsInit_OutputDataObject(IEC_DS_READ_OUTP_DATA_OBJ_ELEM * pContent, PNIO_DEV_ADDR* pAddr)
{
    PNIO_UINT16 ioDataLength = (PNIO_UINT16)OutDatLen[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot];

    /* block length - no swap to network format here */
    pContent->dsBlockLengthSubst =
      sizeof(pContent->dsBlockVersionHiSubst)
    + sizeof(pContent->dsBlockVersionLoSubst)
    + sizeof(pContent->substMode)
    + sizeof(pContent->substIocs)
    + ioDataLength
      + sizeof(pContent->substDataValid);
    pContent->dsBlockLength =
      sizeof(pContent->dsBlockVersionHi)
    + sizeof(pContent->dsBlockVersionLo)
    + sizeof(pContent->substActiveFlag)
    + sizeof(pContent->lengthIocs)
    + sizeof(pContent->lengthIops)
    + sizeof(pContent->lengthData)
    + sizeof(pContent->iocs)
    + ioDataLength
      + sizeof(pContent->iops)
    + sizeof(pContent->dsBlockTypeSubst)
    + sizeof(pContent->dsBlockLengthSubst)
    + pContent->dsBlockLengthSubst;

    /* block header, swap to network format */
    pContent->dsBlockType = SWAP_16(DS_BLOCK_TYPE_OUTP_DATA_OBJ_ELEM);
    pContent->dsBlockLength = SWAP_16(pContent->dsBlockLength);
    pContent->dsBlockVersionHi = IEC_DS_VERSION_HIGH_1;
    pContent->dsBlockVersionLo = IEC_DS_VERSION_LOW_0;

    /* block body */
    pContent->substActiveFlag = SUBST_ACTIVE_FLAG_OFF;
    pContent->lengthIocs = sizeof (PNIO_UINT8);
    pContent->lengthIops = sizeof (PNIO_UINT8);
    pContent->lengthData = SWAP_16(ioDataLength);
    pContent->iocs = IoxsIoBaseToNet(OutDatIocs[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot]);
    pContent->pData = &OutData[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot][0];
    pContent->iops = IoxsIoBaseToNet(OutDatIops[pAddr->u.Geo.Slot][pAddr->u.Geo.Subslot]);

    /* substitute block header */
    pContent->dsBlockTypeSubst = SWAP_16(DS_BLOCK_TYPE_SUBST_VALUE);
    pContent->dsBlockLengthSubst = SWAP_16(pContent->dsBlockLengthSubst);
    pContent->dsBlockVersionHiSubst = IEC_DS_VERSION_HIGH_1;
    pContent->dsBlockVersionLoSubst = IEC_DS_VERSION_LOW_0;

    /*substitute block body */
    pContent->substMode = SWAP_16(0);
    pContent->substIocs = pContent->iocs;
    pContent->pSubstData = pContent->pData;
    pContent->substDataValid = 0x40; /* bad */
}

static void DsFill_InputDataObject(IEC_DS_READ_INP_DATA_OBJ_ELEM *pContent, PNIO_UINT32 *pBufLen, PNIO_UINT8* pBuf, PNIO_ERR_STAT* pPnioState)
{
    PNIO_UINT8* pAct = pBuf;
    PNIO_UINT16 dsLen = SWAP_16(pContent->dsBlockLength) + sizeof(pContent->dsBlockType) + sizeof(pContent->dsBlockLength);

    if ( *pBufLen < (PNIO_UINT32)dsLen ) {
        pPnioState->ErrCode   = 0xde;  /* IODReadRes */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
        pPnioState->ErrCode1  = 0xC0;  /* Resource / read constrain conflict */
        pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
        pPnioState->AddValue1 = 0;     /* here dont care                                                             */
        pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        return;
    }

    /* fill block header */
    memcpy(pAct, &pContent->dsBlockType, sizeof(pContent->dsBlockType));
    pAct += sizeof(pContent->dsBlockType);
    memcpy(pAct, &pContent->dsBlockLength, sizeof(pContent->dsBlockLength));
    pAct += sizeof(pContent->dsBlockLength);
    *pAct++ = pContent->dsBlockVersionHi;
    *pAct++ = pContent->dsBlockVersionLo;

    /* fill block data */
    *pAct++ = pContent->lengthIocs;
    *pAct++ = pContent->iocs;
    *pAct++ = pContent->lengthIops;
    *pAct++ = pContent->iops;
    memcpy(pAct, &pContent->lengthData, sizeof(pContent->lengthData));
    pAct += sizeof(pContent->lengthData);
    memcpy(pAct, pContent->pData, SWAP_16(pContent->lengthData));

    memset(pPnioState, 0, sizeof(pPnioState));
    *pBufLen = (PNIO_UINT32)dsLen;
}

static void DsFill_OutputDataObject(IEC_DS_READ_OUTP_DATA_OBJ_ELEM *pContent, PNIO_UINT32 *pBufLen, PNIO_UINT8* pBuf, PNIO_ERR_STAT* pPnioState)
{
    PNIO_UINT8* pAct = pBuf;
    PNIO_UINT16 dsLen =
      SWAP_16(pContent->dsBlockLength)
    + sizeof(pContent->dsBlockType)
    + sizeof(pContent->dsBlockLength);

    printf("dsBlockLength=%d, dsBlockLengthSubst=%d dsLen=%d bufLen=%d\n",
           SWAP_16(pContent->dsBlockLength), SWAP_16(pContent->dsBlockLengthSubst),
           dsLen, *pBufLen);

    if ( *pBufLen < (PNIO_UINT32)dsLen ) {
        pPnioState->ErrCode   = 0xde;  /* IODReadRes */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
        pPnioState->ErrCode1  = 0xC0;  /* Resource / read constrain conflict */
        pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
        pPnioState->AddValue1 = 0;     /* here dont care                                                             */
        pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        return;
    }

    /* fill block header */
    memcpy(pAct, &pContent->dsBlockType, sizeof(pContent->dsBlockType));
    pAct += sizeof(pContent->dsBlockType);
    memcpy(pAct, &pContent->dsBlockLength, sizeof(pContent->dsBlockLength));
    pAct += sizeof(pContent->dsBlockLength);
    *pAct++ = pContent->dsBlockVersionHi;
    *pAct++ = pContent->dsBlockVersionLo;

    /* fill block data */
    memcpy(pAct, &pContent->substActiveFlag, sizeof(pContent->substActiveFlag));
    pAct += sizeof(pContent->substActiveFlag);
    *pAct++ = pContent->lengthIocs;
    *pAct++ = pContent->lengthIops;
    memcpy(pAct, &pContent->lengthData, sizeof(pContent->lengthData));
    pAct += sizeof(pContent->lengthData);
    *pAct++ = pContent->iocs;
    memcpy(pAct, pContent->pData, SWAP_16(pContent->lengthData));
    pAct += SWAP_16(pContent->lengthData);
    *pAct++ = pContent->iops;

    /* fill substitute block header */
    memcpy(pAct, &pContent->dsBlockTypeSubst, sizeof(pContent->dsBlockTypeSubst));
    pAct += sizeof(pContent->dsBlockTypeSubst);
    memcpy(pAct, &pContent->dsBlockLengthSubst, sizeof(pContent->dsBlockLengthSubst));
    pAct += sizeof(pContent->dsBlockLengthSubst);
    *pAct++ = pContent->dsBlockVersionHiSubst;
    *pAct++ = pContent->dsBlockVersionLoSubst;

    /* fill substitute block body */
    memcpy(pAct, &pContent->substMode, sizeof(pContent->substMode));
    pAct += sizeof(pContent->substMode);
    memset(pAct, 0, sizeof(pContent->iocs) + SWAP_16(pContent->lengthData) + sizeof(pContent->substDataValid));

    memset(pPnioState, 0, sizeof(pPnioState));
    *pBufLen = (PNIO_UINT32)dsLen;
}

void IM_initImRecords(IM_DATA *pImWork, const IM_DATA *pImDefault)
{
    int i;

    for ( i=0; i<DEVICE_DATA_ENTRIES_MAX; i++ ) {
        pImWork[i] = *pImDefault;
        if ( i>0 ) {
            pImWork[i].im0.IM_Profile_Specific_Type = 0x03; /* io-module -> www.profibus.com/IM/Device_ID_Table_6102.xml */
        }
    }
}

static void IM_incRevCounter(PNIO_UINT16 *pRevCounter)
{
    if ( *pRevCounter == 0 ) {
        *pRevCounter = 1;
    }
}

/*----------------------------------------------------------------------------------------------------*/
/*                  I&M helper functions                                                              */
/*----------------------------------------------------------------------------------------------------*/
IM_CMP_E IM_CmpIM0(const PNIO_IM0_TYPE * pLeft, const PNIO_IM0_TYPE * pRight)
{
    if (
       pLeft->BlockHeader.BlockLength == pRight->BlockHeader.BlockLength ||
       pLeft->BlockHeader.BlockType == pRight->BlockHeader.BlockType ||
       pLeft->BlockHeader.BlockVersionHigh == pRight->BlockHeader.BlockVersionHigh ||
       pLeft->BlockHeader.BlockVersionLow == pRight->BlockHeader.BlockVersionLow ||
       pLeft->IM_Hardware_Revision == pRight->IM_Hardware_Revision ||
       pLeft->IM_Profile_ID == pRight->IM_Profile_ID ||
       pLeft->IM_Profile_Specific_Type == pRight->IM_Profile_Specific_Type ||
       pLeft->IM_Revision_Counter == pRight->IM_Revision_Counter ||
       pLeft->IM_Supported == pRight->IM_Supported ||
       pLeft->IM_Version_Major == pRight->IM_Version_Major ||
       pLeft->IM_Version_Minor == pRight->IM_Version_Minor ||
       STRNCMP_EQUAL == strncmp((char *)pLeft->OrderID, (char *)pRight->OrderID, LEN_IM_ORDERID) ||
       pLeft->VendorIDHigh == pRight->VendorIDHigh ||
       pLeft->VendorIDLow == pRight->VendorIDLow ||
       STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Serial_Number, (char *)pRight->IM_Serial_Number, LEN_IM_SERIAL_NUMBER) ||
       STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Software_Revision, (char *)pRight->IM_Software_Revision, LEN_IM_SOFTWARE_REVISION)
       ) {
        return IM_EQUAL;
    }

    return IM_READ_DIFFER;
}

IM_CMP_E IM_CmpIM1(const PNIO_IM1_TYPE * pLeft, const PNIO_IM1_TYPE * pRight)
{
    PNIO_BOOL readDiffer = PNIO_TRUE;
    PNIO_BOOL writeDiffer = PNIO_TRUE;

    if ( pLeft->BlockHeader.BlockLength == pRight->BlockHeader.BlockLength ||
         pLeft->BlockHeader.BlockType == pRight->BlockHeader.BlockType ||
         pLeft->BlockHeader.BlockVersionHigh == pRight->BlockHeader.BlockVersionHigh ||
         pLeft->BlockHeader.BlockVersionLow == pRight->BlockHeader.BlockVersionLow ) {
        readDiffer = PNIO_FALSE;
    }

    if ( STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Tag_Function, (char *)pRight->IM_Tag_Function, LEN_IM_TAG_FUNCTION) &&
         STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Tag_Location, (char *)pRight->IM_Tag_Location, LEN_IM_TAG_LOCATION ) ) {
        writeDiffer = PNIO_FALSE;
    }

    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_READ_WRITE_DIFFER;
    }
    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_FALSE) ) {
        return IM_READ_DIFFER;
    }
    if ( (readDiffer == PNIO_FALSE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_WRITE_DIFFER;
    }
    return IM_EQUAL;
}

IM_CMP_E IM_CmpIM2(const PNIO_IM2_TYPE * pLeft, const PNIO_IM2_TYPE * pRight)
{
    PNIO_BOOL readDiffer = PNIO_TRUE;
    PNIO_BOOL writeDiffer = PNIO_TRUE;

    if ( pLeft->BlockHeader.BlockLength == pRight->BlockHeader.BlockLength ||
         pLeft->BlockHeader.BlockType == pRight->BlockHeader.BlockType ||
         pLeft->BlockHeader.BlockVersionHigh == pRight->BlockHeader.BlockVersionHigh ||
         pLeft->BlockHeader.BlockVersionLow == pRight->BlockHeader.BlockVersionLow ) {
        readDiffer = PNIO_FALSE;
    }

    if ( STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Date, (char *)pRight->IM_Date, LEN_IM_DATE) ) {
        writeDiffer = PNIO_FALSE;
    }

    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_READ_WRITE_DIFFER;
    }
    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_FALSE) ) {
        return IM_READ_DIFFER;
    }
    if ( (readDiffer == PNIO_FALSE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_WRITE_DIFFER;
    }
    return IM_EQUAL;
}

IM_CMP_E IM_CmpIM3(const PNIO_IM3_TYPE * pLeft, const PNIO_IM3_TYPE * pRight)
{
    PNIO_BOOL readDiffer = PNIO_TRUE;
    PNIO_BOOL writeDiffer = PNIO_TRUE;

    if ( pLeft->BlockHeader.BlockLength == pRight->BlockHeader.BlockLength ||
         pLeft->BlockHeader.BlockType == pRight->BlockHeader.BlockType ||
         pLeft->BlockHeader.BlockVersionHigh == pRight->BlockHeader.BlockVersionHigh ||
         pLeft->BlockHeader.BlockVersionLow == pRight->BlockHeader.BlockVersionLow ) {
        readDiffer = PNIO_FALSE;
    }

    if ( STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Descriptor, (char *)pRight->IM_Descriptor, LEN_IM_DESCRIPTOR) ) {
        writeDiffer = PNIO_FALSE;
    }

    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_READ_WRITE_DIFFER;
    }
    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_FALSE) ) {
        return IM_READ_DIFFER;
    }
    if ( (readDiffer == PNIO_FALSE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_WRITE_DIFFER;
    }
    return IM_EQUAL;
}

IM_CMP_E IM_CmpIM4(const PNIO_IM4_TYPE * pLeft, const PNIO_IM4_TYPE * pRight)
{
    PNIO_BOOL readDiffer = PNIO_TRUE;
    PNIO_BOOL writeDiffer = PNIO_TRUE;

    if ( pLeft->BlockHeader.BlockLength == pRight->BlockHeader.BlockLength ||
         pLeft->BlockHeader.BlockType == pRight->BlockHeader.BlockType ||
         pLeft->BlockHeader.BlockVersionHigh == pRight->BlockHeader.BlockVersionHigh ||
         pLeft->BlockHeader.BlockVersionLow == pRight->BlockHeader.BlockVersionLow ) {
        readDiffer = PNIO_FALSE;
    }

    if ( STRNCMP_EQUAL == strncmp((char *)pLeft->IM_Signature, (char *)pRight->IM_Signature, LEN_IM_SIGNATURE) ) {
        writeDiffer = PNIO_FALSE;
    }

    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_READ_WRITE_DIFFER;
    }
    if ( (readDiffer == PNIO_TRUE) && (writeDiffer == PNIO_FALSE) ) {
        return IM_READ_DIFFER;
    }
    if ( (readDiffer == PNIO_FALSE) && (writeDiffer == PNIO_TRUE) ) {
        return IM_WRITE_DIFFER;
    }
    return IM_EQUAL;
}

PNIO_UINT8 * IM_RemTrailBlanks(const PNIO_UINT8 *pBuf, int bufLen)
{
    static PNIO_UINT8 pNoBlanks[LINE_LENGTH_MAX];
    int idx = bufLen - 1;
    int copyLen=0;

    if ( bufLen >= LINE_LENGTH_MAX ) {
        idx = LINE_LENGTH_MAX-2;
    }

    for ( ;idx >= 0; idx-- ) {
        if ( pBuf[idx] != ' ' ) {
            copyLen = idx + 1;
            break;
        }
        if ( idx == 0 ) {     /* solely blanks found */
            copyLen = 0;
            break;
        }
    }
    strncpy((char *)pNoBlanks, (char *)pBuf, copyLen);
    pNoBlanks[copyLen] = '\0';

    return pNoBlanks;
};


/* pSrc has to be null-terminated */
PNIO_BOOL IM_CopyQuotedString(PNIO_UINT8 *pDst, PNIO_UINT8 *pSrc, size_t maxSize)
{
    PNIO_UINT8 *pTmp;
    size_t copyLen;

    pSrc = (PNIO_UINT8 *) strchr((char *)pSrc, '"');
    if ( !pSrc ) {
        return PNIO_FALSE;
    }
    else {
        pSrc++;
    }
    pTmp = (PNIO_UINT8 *)strrchr((char *)pSrc, '"');
    if ( !pTmp ) {
        return PNIO_FALSE;
    }

    copyLen = pTmp - pSrc;
    if ( copyLen > maxSize ) {
        copyLen = maxSize;
    }

    strncpy((char *)pDst, (char *)pSrc, copyLen);
    return PNIO_TRUE;
}

void IM_DataFromStore(const char *pName, IM_DATA *pImArray)
{
    FILE *pFile;
    PNIO_UINT8 line[LINE_LENGTH_MAX];
    IM_PERSISTENT_DATA_e idx;
    int sModIdx;

    pFile = fopen(pName, "r");
    if ( pFile == NULL )
        return;

    for ( sModIdx=0; sModIdx<gDevArraySize; sModIdx++ ) {
        IM_DATA *pIm = &pImArray[sModIdx];

        /* override default IM values by stored values */
        while ( fgets((char *)line, LINE_LENGTH_MAX, pFile) != NULL ) {
            char *pPos = NULL;
            int sectionIdx;

            /* ignore comments -> read next line */
            if ( (line[0] == ';') || (line[0] == '#') )
                continue;

            /* continue until submodule section is found */
            sscanf((char *)line, "[SUBMOD%i]", &sectionIdx);
            if ( sectionIdx != sModIdx ) {
                continue;
            }
            /* find IM item in current line */
            for ( idx=0; idx<NUMOF_PERSISTENT_IM_ITEMS; idx++ ) {
                size_t size = strlen(name_IM_PERSISTENT_DATA[idx]);
                if ( STRNCMP_EQUAL ==  strncmp((char *)line, name_IM_PERSISTENT_DATA[idx], size) ) {
                    pPos = (char*)line + size;
                    /* ignore blanks etc */
                    // todo: immer " fordern!!!
                    while ( (*pPos == ' ') || (*pPos == '=') )
                        pPos++;
                    break;
                }
            }

            if ( pPos == NULL )   /* no IM item found */
                continue;

            /* read IM value */
            // todo: nur maximal sizeof(im-element) Bytes einlesen!!!!
            switch ( idx ) {
                case IM_Revision_Counter:
                    {
                        int val;
                        sscanf(pPos, "%i", &val);
                        if ( (val > 0) && (val <= 0xffff) ) {
                            pIm->im0.IM_Revision_Counter = (PNIO_UINT16) val;
                        }
                        break;
                    }
                case IM_Tag_Function:
                    IM_CopyQuotedString(pIm->im1.IM_Tag_Function, line, sizeof(pIm->im1.IM_Tag_Function));
                    break;
                case IM_Tag_Location:
                    IM_CopyQuotedString(pIm->im1.IM_Tag_Location, line, sizeof(pIm->im1.IM_Tag_Location));
                    break;
                case IM_Date:
                    IM_CopyQuotedString(pIm->im2.IM_Date, line, sizeof(pIm->im2.IM_Date));
                    break;
                case IM_Descriptor:
                    IM_CopyQuotedString(pIm->im3.IM_Descriptor, line, sizeof(pIm->im3.IM_Descriptor));
                    break;
                case IM_Signature:
                    IM_CopyQuotedString(pIm->im4.IM_Signature, line, sizeof(pIm->im4.IM_Signature));
                    break;
                default:
                    assert(0);
            }
        }
    }

    fclose(pFile);
}

void IM_DataToStore(const char *pName, const IM_DATA *pImArray, const IM_DATA *pImDefault)
{
    FILE *pFile;
    int sModIdx;
    pFile = fopen(pName, "w");
    if ( pFile == NULL )
        return;

    for ( sModIdx=0; sModIdx<gDevArraySize; sModIdx++ ) {
        const IM_DATA *pIm = &pImArray[sModIdx];
        if ( sModIdx ) {
            fprintf(pFile, "\n");
        }
        fprintf(pFile, "[SUBMOD%i]\n", sModIdx);
        // todo: ev. Blanks an Ende wegschneiden
        fprintf(pFile, "%s = 0x%04x\n", name_IM_PERSISTENT_DATA[IM_Revision_Counter],
                pIm->im0.IM_Revision_Counter);
        if ( pIm->im0.IM_Supported & 0x02 ) {
            fprintf(pFile, "%s = \"%s\"\n", name_IM_PERSISTENT_DATA[IM_Tag_Function],
                    IM_RemTrailBlanks(pIm->im1.IM_Tag_Function,sizeof(pIm->im1.IM_Tag_Function)));
            fprintf(pFile, "%s = \"%s\"\n", name_IM_PERSISTENT_DATA[IM_Tag_Location],
                    IM_RemTrailBlanks(pIm->im1.IM_Tag_Location,sizeof(pIm->im1.IM_Tag_Location)));
        }
        if ( pIm->im0.IM_Supported&0x04 ) {
            fprintf(pFile, "%s = \"%s\"\n", name_IM_PERSISTENT_DATA[IM_Date],
                    IM_RemTrailBlanks(pIm->im2.IM_Date,sizeof(pIm->im2.IM_Date)));
        }
        if ( pIm->im0.IM_Supported&0x08 ) {
            fprintf(pFile, "%s = \"%s\"\n", name_IM_PERSISTENT_DATA[IM_Descriptor],
                    IM_RemTrailBlanks(pIm->im3.IM_Descriptor,sizeof(pIm->im3.IM_Descriptor)));
        }
        if ( pIm->im0.IM_Supported&0x10 ) {
            fprintf(pFile, "%s = \"%s\"\n", name_IM_PERSISTENT_DATA[IM_Signature],
                    IM_RemTrailBlanks(pIm->im4.IM_Signature,sizeof(pIm->im4.IM_Signature)));
        }
    }

    fclose(pFile);
}

static void IM_CopySwapIm1FromBuf(
                                 PNIO_IM1_TYPE      *pIm, /* [out] */
                                 PNIO_UINT8  *const pBuf) /* [in]  */
{
    *pIm =                          *(PNIO_IM1_TYPE *)pBuf;
    pIm->BlockHeader.BlockType =                SWAP_16(((PNIO_IM1_TYPE *)pBuf)->BlockHeader.BlockType);
    pIm->BlockHeader.BlockLength =              SWAP_16(((PNIO_IM1_TYPE *)pBuf)->BlockHeader.BlockLength);
}

static void IM_CopySwapIm2FromBuf(
                                 PNIO_IM2_TYPE      *pIm,        /* [out] */
                                 PNIO_UINT8  *const pBuf) /* [in]  */
{
    *pIm =                          *(PNIO_IM2_TYPE *)pBuf;
    pIm->BlockHeader.BlockType =                SWAP_16(((PNIO_IM2_TYPE *)pBuf)->BlockHeader.BlockType);
    pIm->BlockHeader.BlockLength =              SWAP_16(((PNIO_IM2_TYPE *)pBuf)->BlockHeader.BlockLength);
}

static void IM_CopySwapIm3FromBuf(
                                 PNIO_IM3_TYPE      *pIm, /* [out] */
                                 PNIO_UINT8  *const pBuf) /* [in]  */
{
    *pIm =                          *(PNIO_IM3_TYPE *)pBuf;
    pIm->BlockHeader.BlockType =                SWAP_16(((PNIO_IM3_TYPE *)pBuf)->BlockHeader.BlockType);
    pIm->BlockHeader.BlockLength =              SWAP_16(((PNIO_IM3_TYPE *)pBuf)->BlockHeader.BlockLength);
}

static void IM_CopySwapIm4FromBuf(
                                 PNIO_IM4_TYPE      *pIm, /* [out] */
                                 PNIO_UINT8  *const pBuf) /* [in]  */
{
    *pIm =                          *(PNIO_IM4_TYPE *)pBuf;
    pIm->BlockHeader.BlockType =                SWAP_16(((PNIO_IM4_TYPE *)pBuf)->BlockHeader.BlockType);
    pIm->BlockHeader.BlockLength =              SWAP_16(((PNIO_IM4_TYPE *)pBuf)->BlockHeader.BlockLength);
}

static PNIO_BOOL IM_CopySwapBufFromIm(
                                     PNIO_UINT8    *pBuf, /* [out]  */
                                     IM_DATA const *pIm,  /* [in] */
                                     IM0_idx_e     idx)   /* [in]  */
{
    switch ( idx ) {
        case IM0:
            *((PNIO_IM0_TYPE *)pBuf)           =                pIm->im0;
            ((PNIO_IM0_TYPE *)pBuf)->BlockHeader.BlockType =    SWAP_16(pIm->im0.BlockHeader.BlockType);
            ((PNIO_IM0_TYPE *)pBuf)->BlockHeader.BlockLength =  SWAP_16(pIm->im0.BlockHeader.BlockLength);
            ((PNIO_IM0_TYPE *)pBuf)->IM_Hardware_Revision =     SWAP_16(pIm->im0.IM_Hardware_Revision);
            /* ((PNIO_IM0_TYPE *)pBuf)->IM_Software_Revision =     SWAP_32(pIm->im0.IM_Software_Revision);*/
            ((PNIO_IM0_TYPE *)pBuf)->IM_Revision_Counter =      SWAP_16(pIm->im0.IM_Revision_Counter);
            ((PNIO_IM0_TYPE *)pBuf)->IM_Profile_Specific_Type = SWAP_16(pIm->im0.IM_Profile_Specific_Type);
            ((PNIO_IM0_TYPE *)pBuf)->IM_Profile_ID =            SWAP_16(pIm->im0.IM_Profile_ID);
            ((PNIO_IM0_TYPE *)pBuf)->IM_Supported =             SWAP_16(pIm->im0.IM_Supported);
            break;
        case IM1:
            *((PNIO_IM1_TYPE *)pBuf)           =                pIm->im1;
            ((PNIO_IM1_TYPE *)pBuf)->BlockHeader.BlockType =     SWAP_16(pIm->im1.BlockHeader.BlockType);
            ((PNIO_IM1_TYPE *)pBuf)->BlockHeader.BlockLength =   SWAP_16(pIm->im1.BlockHeader.BlockLength);
            break;
        case IM2:
            *((PNIO_IM2_TYPE *)pBuf)           =                pIm->im2;
            ((PNIO_IM2_TYPE *)pBuf)->BlockHeader.BlockType =     SWAP_16(pIm->im2.BlockHeader.BlockType);
            ((PNIO_IM2_TYPE *)pBuf)->BlockHeader.BlockLength =   SWAP_16(pIm->im2.BlockHeader.BlockLength);
            break;
        case IM3:
            *((PNIO_IM3_TYPE *)pBuf)           =                pIm->im3;
            ((PNIO_IM3_TYPE *)pBuf)->BlockHeader.BlockType =     SWAP_16(pIm->im3.BlockHeader.BlockType);
            ((PNIO_IM3_TYPE *)pBuf)->BlockHeader.BlockLength =   SWAP_16(pIm->im3.BlockHeader.BlockLength);
            break;
        case IM4:
            *((PNIO_IM4_TYPE *)pBuf)           =                pIm->im4;
            ((PNIO_IM4_TYPE *)pBuf)->BlockHeader.BlockType =     SWAP_16(pIm->im4.BlockHeader.BlockType);
            ((PNIO_IM4_TYPE *)pBuf)->BlockHeader.BlockLength =   SWAP_16(pIm->im4.BlockHeader.BlockLength);
            break;
        default:
            break;
    }

    return PNIO_TRUE;
}
//Diese Funktion gibt die Groesse des Datensatzes zurueck
static PNIO_UINT32 IM_SizeofImDatarec(PNIO_UINT32  RecordIndex)
{
    switch ( RecordIndex ) {
        case IM0:
            return(sizeof(PNIO_IM0_TYPE));
        case IM1:
            return(sizeof(PNIO_IM1_TYPE));
        case IM2:
            return(sizeof(PNIO_IM2_TYPE));
        case IM3:
            return(sizeof(PNIO_IM3_TYPE));
        case IM4:
            return(sizeof(PNIO_IM4_TYPE));
        default:
            return 0;
    }
}

// Diese Funktion gibt zurueck ob der uebergebene Datensatz unterstuetzt wird
static PNIO_BOOL IM_IndexSupported(PNIO_UINT32 RecordIndex, PNIO_UINT16 IM_Supported)
{//&& (IM_Supported&0x01)
    if ( ((RecordIndex == IM0) ) ||
         ((RecordIndex == IM1) && (IM_Supported&0x02)) ||
         ((RecordIndex == IM2) && (IM_Supported&0x04)) ||
         ((RecordIndex == IM3) && (IM_Supported&0x08)) ||
         ((RecordIndex == IM4) && (IM_Supported&0x10)) ) {
        return PNIO_TRUE;
    }
    return PNIO_FALSE;
}

// Diese Funktion setzt den Errorcode
static void setPnioRwError(PNIO_ERR_STAT* pPnioState, PNIO_UINT8 ErrCode1)
{
    /**** if an error occured, you must specify it according IEC 61158-6 */
    pPnioState->ErrCode   = 0xdf;  /* IODWriteRes with ErrorDecode = PNIORW */
    pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
    pPnioState->ErrCode1  = ErrCode1;
    pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
    pPnioState->AddValue1 = 0;     /* here dont care                                                             */
    pPnioState->AddValue2 = 0;     /* here dont care                                                             */
}

// Diese Funktion liest die Datensaetze von Buffer ein
static void IM_recWrite(PNIO_UINT32 RecordIndex, PNIO_DEV_ADDR *pAddr, PNIO_UINT32 *pBufLen, PNIO_UINT8 *pBuf, PNIO_ERR_STAT* pPnioState)
{
    IM_CMP_E    imCmp;
    int         sModIdx;
    PNIO_BOOL   addressValid    = PNIO_FALSE;
    IM_DATA     *pIm = NULL;

    *pBufLen = 0;

    for ( sModIdx=0; sModIdx<gDevArraySize; sModIdx++ ) {
        if ( ((unsigned int)g_device_data[sModIdx].slot == pAddr->u.Geo.Slot)&&((unsigned int)g_device_data[sModIdx].subslot == pAddr->u.Geo.Subslot) ) {
            addressValid = PNIO_TRUE;
            pIm = &im_work[sModIdx];
            break;
        }
    }

    // todo: ist das notwendig?
    /* Slot/Subslot should always be valid */
    if ( addressValid == PNIO_FALSE ) {
        setPnioRwError(pPnioState, 0xb2); /* access: invalid slot/sublslot*/
        return;
    }

    // doto: ist das notwendig?
    if ( (g_device_data[sModIdx].modState == 0) || (g_device_data[sModIdx].subState == 0) ) {
        setPnioRwError(pPnioState, 0xb2); /* access: invalid slot/sublslot*/
        return;
    }

    if ( !IM_IndexSupported(RecordIndex, im_work[sModIdx].im0.IM_Supported) ) {
        setPnioRwError(pPnioState, 0xa9); /* application: feature not supported */
        return;
    }

    switch ( RecordIndex ) {
        case IM0:
            setPnioRwError(pPnioState, 0xb6); /* access: access denied */
            return;
        case IM1: {
                PNIO_IM1_TYPE im;
                if ( *pBuf == 0 )
                    im = im_default.im1;
                else
                    IM_CopySwapIm1FromBuf(&im, pBuf);
                imCmp = IM_CmpIM1(&im, &pIm->im1);
                if ( (imCmp == IM_READ_DIFFER)||(imCmp == IM_READ_WRITE_DIFFER) ) {
                    setPnioRwError(pPnioState, 0xb4); /* Access: invalid area */
                }
                else if ( imCmp == IM_WRITE_DIFFER ) {
                    pIm->im1 = im;
                    IM_incRevCounter(&pIm->im0.IM_Revision_Counter);
                }
            }
            break;
        case IM2:
            {
                PNIO_IM2_TYPE im;
                if ( *pBuf == 0 )
                    im = im_default.im2;
                else
                    IM_CopySwapIm2FromBuf(&im, pBuf);
                imCmp = IM_CmpIM2(&im, &pIm->im2);
                if ( (imCmp == IM_READ_DIFFER)||(imCmp == IM_READ_WRITE_DIFFER) ) {
                    setPnioRwError(pPnioState, 0xb4); /* Access: invalid area */
                }
                else if ( imCmp == IM_WRITE_DIFFER ) {
                    pIm->im2 = im;
                    IM_incRevCounter(&pIm->im0.IM_Revision_Counter);
                }
            }
            break;
        case IM3:
            {
                PNIO_IM3_TYPE im;
                if ( *pBuf == 0 )
                    im = im_default.im3;
                else
                    IM_CopySwapIm3FromBuf(&im, pBuf);
                imCmp = IM_CmpIM3(&im, &pIm->im3);
                if ( (imCmp == IM_READ_DIFFER)||(imCmp == IM_READ_WRITE_DIFFER) ) {
                    setPnioRwError(pPnioState, 0xb4); /* Access: invalid area */
                }
                else if ( imCmp == IM_WRITE_DIFFER ) {
                    pIm->im3 = im;
                    IM_incRevCounter(&pIm->im0.IM_Revision_Counter);
                }
            }
            break;
        case IM4:
            {
                PNIO_IM4_TYPE im;
                if ( *pBuf == 0 )
                    im = im_default.im4;
                else
                    IM_CopySwapIm4FromBuf(&im, pBuf);
                imCmp = IM_CmpIM4(&im, &pIm->im4);
                if ( (imCmp == IM_READ_DIFFER)||(imCmp == IM_READ_WRITE_DIFFER) ) {
                    setPnioRwError(pPnioState, 0xb4); /* Access: invalid area */
                }
                else if ( imCmp == IM_WRITE_DIFFER ) {
                    pIm->im4 = im;
                    IM_incRevCounter(&pIm->im0.IM_Revision_Counter);
                }
            }
            break;
        default:
            break;
    }
    *pBufLen = IM_SizeofImDatarec(RecordIndex);
    return;
}

// Diese Funktion schreibt den uebergebenen Datensatz in den Buffer
static void IM_RecRead(PNIO_UINT32 RecordIndex, PNIO_DEV_ADDR *pAddr, PNIO_UINT32 *pBufLen, PNIO_UINT8 *pBuf, PNIO_ERR_STAT* pPnioState)
{
    int         sModIdx;
    PNIO_BOOL   addressValid = PNIO_FALSE;

    for ( sModIdx=0; sModIdx<gDevArraySize; sModIdx++ ) {
        if ( (((unsigned int)g_device_data[sModIdx].slot == pAddr->u.Geo.Slot)&&((unsigned int)g_device_data[sModIdx].subslot == pAddr->u.Geo.Subslot))
             ||((pAddr->u.Geo.Subslot == 32768)
                ||(pAddr->u.Geo.Subslot == 32769)
                ||(pAddr->u.Geo.Subslot == 32770)
                ||(pAddr->u.Geo.Subslot == 32771)
                ||(pAddr->u.Geo.Subslot == 32772)) ) {
            addressValid = PNIO_TRUE;
            break;
        }
    }

    // todo: ist das notwendig?
    /* Slot/Subslot should always be valid */
    if ( addressValid == PNIO_FALSE ) {
        setPnioRwError(pPnioState, 0xb2); /* access: invalid slot/sublslot*/
        return;
    }

    // doto: ist das notwendig?
    if ( (g_device_data[sModIdx].modState == 0) || (g_device_data[sModIdx].subState == 0) ) {
        setPnioRwError(pPnioState, 0xb2); /* access: invalid slot/sublslot*/
        return;
    }

    if ( !IM_IndexSupported(RecordIndex, im_work[sModIdx].im0.IM_Supported) ) {
        setPnioRwError(pPnioState, 0xa9); /* application: feature not supported */
        return;
    }

    // todo: was tun, wenn IM Record nicht in Puffer passt
    *pBufLen = IM_SizeofImDatarec(RecordIndex);
    IM_CopySwapBufFromIm(pBuf, &im_work[sModIdx], RecordIndex);

    return;
}


/*----------------------------------------------------------------------------------------------------*/
/*    CALLBACKS                                                                                       */
/*----------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/* Relevant callback functions for Initialize and UnInitialize of a PNIO device */
/* are defined here.                                                            */
/*------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_data_write (...)                                                    */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  Passes the input data from the application to the stack.                     */
/*  The application reads the data from the specified input module               */
/*  and handles it to the stack.                                                 */
/*  The function UsrReadInputData() is called by the PNIO stack.                 */
/*                                                                               */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
PNIO_IOXS PNIO_cbf_data_write(
                             PNIO_UINT32    DevHndl,            /* [in] Handle for device */
                             PNIO_DEV_ADDR* pAddr,              /* [in] geographical address */
                             PNIO_UINT32    BufLen,             /* [in] length of the submodule input data */
                             PNIO_UINT8*    pBuffer,            /* [out] Ptr to data buffer to write to */
                             PNIO_IOXS      Iocs)               /* [in] remote (io controller) consumer status */
{
    PNIO_UINT32 slot_num    = pAddr->u.Geo.Slot;
    PNIO_UINT32 subslot_num = pAddr->u.Geo.Subslot;

    /*printf("## PNIO_cbf_data_write(..., len=%u, Iocs=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iocs, DevHndl, slot_num, subslot_num);*/


    InDatLen  [slot_num][subslot_num] = BufLen;  /* save data length (only for debugging) */
    InDatIocs [slot_num][subslot_num] = Iocs;    /* consumer status (of remote IO controller) */

    InDatIops [slot_num][subslot_num] = PNIO_S_BAD; /* set local status to bad */

    if ( BufLen == 0 ) {
        /*printf(" BufLen = 0, nothing to write to..\n");*/
        if (GetSubmoduleExistsInStep7Project(slot_num, subslot_num))
        {
        	InDatIops [slot_num][subslot_num] = PNIO_S_GOOD;
    	}
    }
    else if ( BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT ) {
        if (GetSubmoduleExistsInStep7Project(slot_num, subslot_num))
        {
        	memcpy (pBuffer, &InData[slot_num][subslot_num][0], BufLen); /* Copy the application data to the stack */
        	InDatIops [slot_num][subslot_num] = PNIO_S_GOOD; /* assume everything is ok */
        }
    }
    else {
        printf("!!! PNIO_cbf_data_write: Buflen=%lu > allowed size (%u)!!! Abort writing..\n",
               (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
    }

    return(InDatIops [slot_num][subslot_num]); /* return local provider status */
}

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_data_read (...)                                                     */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  Passes the output data from the stack to the application.                    */
/*  The application takes the data and writes it to the specified                */
/*  output module.                                                               */
/*  function UsrWriteOutputData() is called by the PNIO stack.                   */
/*                                                                               */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
PNIO_IOXS PNIO_cbf_data_read(
                            PNIO_UINT32    DevHndl,            /* [in] Handle for Multidevice */
                            PNIO_DEV_ADDR* pAddr,              /* [in] geographical address */
                            PNIO_UINT32    BufLen,             /* [in] length of the submodule input data */
                            PNIO_UINT8*    pBuffer,            /* [in] Ptr to data buffer to read from */
                            PNIO_IOXS      Iops)               /* [in] (io controller) provider status */
{
    PNIO_UINT32 slot_num    = pAddr->u.Geo.Slot;
    PNIO_UINT32 subslot_num = pAddr->u.Geo.Subslot;

    /*printf("## PNIO_cbf_data_read(..., len=%u, Iops=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iops, DevHndl, slot_num, subslot_num);*/

    OutDatLen  [slot_num][subslot_num] = BufLen;  /* save data length (only for debugging) */
    OutDatIops [slot_num][subslot_num] = Iops;    /* provider status (of remote IO controller) */

    OutDatIocs [slot_num][subslot_num] = PNIO_S_BAD; /* set local status to bad */
    if ( BufLen == 0 ) {
        printf(" BufLen = 0, nothing to read..\n");
        if (GetSubmoduleExistsInStep7Project(slot_num, subslot_num))
        {
	        OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD;
    	}
    }
    else if ( BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT ) {
        if (GetSubmoduleExistsInStep7Project(slot_num, subslot_num))
        {
        	memcpy (&OutData[slot_num][subslot_num][0], pBuffer, BufLen); /*Copy the data from the stack to the application buffer*/
        	OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD; /* assume everything is ok */
        }
    }
    else {
        printf("!!! PNIO_cbf_data_read: Buflen=%lu > allowed size (%u)!!! Abort reading...  IOCS=  BAD !!! \n",
               (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
    }

    return( OutDatIocs [slot_num][subslot_num]); /* consumer state (of local IO device) */
}











































/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_rec_write (...)                                                     */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*  This callback is called to notify that a write record request                */
/*      has been received from the PNIO controller. The user has to              */
/*      read the record data from the specified source buffer.                   */
/*      The length of the provided data are specified in function parameter      */
/*      *pBufLen. The user has to change this pointer, if the size of            */
/*  the accepted data differs from the size of the provided data.                */
/*  After serving this function, the user returns the success state.             */
/*-------------------------------------------------------------------------------*/
void  PNIO_cbf_rec_write(
                        PNIO_UINT32            DevHndl,
                        PNIO_UINT32            Api,
                        PNIO_UINT16            ArNumber,
                        PNIO_UINT16            SessionKey,
                        PNIO_UINT32            SequenceNum,
                        PNIO_DEV_ADDR*         pAddr,          /* geographical address */
                        PNIO_UINT32            RecordIndex,
                        PNIO_UINT32*           pBufLen,        /* [in, out] in: length to write, out: length, written by user */
                        PNIO_UINT8*            pBuf,           /* [in] buffer pointer */
                        PNIO_ERR_STAT*         pPnioState)     /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                                    ErrCode2), see IEC61158-6 */
{
    int   i;
    PNIO_BOOL modPlugged = PNIO_FALSE;

    printf ("\n## WRITE_RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu\n",
            (unsigned long)Api, (unsigned long)pAddr->u.Geo.Slot, (unsigned long)pAddr->u.Geo.Subslot,
            (unsigned long)RecordIndex, (unsigned long)*pBufLen, (unsigned long)SequenceNum);

    memset(pPnioState, 0, sizeof(*pPnioState));

    /* target module plugged ? */
    for ( i=0; i<gDevArraySize; i++ ) {
        if ( ((PNIO_UINT32)g_device_data[i].slot == pAddr->u.Geo.Slot)&&((PNIO_UINT32)g_device_data[i].subslot == pAddr->u.Geo.Subslot) ) {
            modPlugged = PNIO_TRUE;
            break;
        }
    }

    if ( RecordIndex == IM0
         || RecordIndex == IM1
         || RecordIndex == IM2
         || RecordIndex == IM3
         || RecordIndex == IM4 ) {
        if ( (pAddr->u.Geo.Slot == 1) &&
             ((pAddr->u.Geo.Subslot == 1)
              ||(pAddr->u.Geo.Subslot == 0x8000)
              ||(pAddr->u.Geo.Subslot == 0x8001)
              ||(pAddr->u.Geo.Subslot == 0x8002)
              ||(pAddr->u.Geo.Subslot == 0x8003)
              ||(pAddr->u.Geo.Subslot == 0x8004)) ) { /* head module supports IM  */
            IM_recWrite(RecordIndex, pAddr, pBufLen, pBuf, pPnioState);
        }
        else {
            pPnioState->ErrCode   = 0xdf;  /* IODWriteRes */
            pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
            pPnioState->ErrCode1  = 0xB6;  /* invalid slot/subslot */
            pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
            pPnioState->AddValue1 = 0;     /* here dont care                                                             */
            pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        }
    }
    else {

#ifdef PROFI_ENERGY_ENABLE
PNIO_cbf_rec_write_pe(
                         DevHndl,
                        Api,
                        ArNumber,
                         SessionKey,
                         SequenceNum,
                         pAddr,          /* geographical address */
                        RecordIndex,
                        pBufLen,        /* [in, out] in: length to write, out: length, written by user */
                         pBuf,           /* [in] buffer pointer */
                        pPnioState) ;    /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, */

	}

#else
        PNIO_UINT8    WriteRecDummyData[50];

        /**** check data size (accepted data < provided data) */
        if ( *pBufLen > sizeof (WriteRecDummyData) ) {

            *pBufLen = sizeof (WriteRecDummyData);
            printf("## WARNING: Can not write all data, not enough space..\n");
        }

        /**** copy the record data into a buffer for further use ***/
        memcpy (WriteRecDummyData,      /* destination pointer for record data */
                pBuf,                       /* source pointer for record data      */
                *pBufLen);                  /* length of the accepted data         */

    }

    printf ("## RECORD_DATA written:");
    for ( i=0; i < (int)*pBufLen; i++ ) {
        if ( i%16 == 0 )
            printf("\n");
        printf ("0x%02lx ", (long)pBuf[i]);
    }
    printf("\n");
#endif // PROFI_ENERGY_ENABLE
}

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_rec_read (...)                                                      */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/* This callback is called to notify that a read record request                  */
/* has been received from the PNIO controller. This function has to              */
/* provide the record data and copy it to the specified buffer                   */
/* address. The maximum pBufLen is also provided in the function                 */
/* parameters and can not be exceeded !.                                         */
/* After serving this function, this function returns the real copied data       */
/* length and the success state (PNIO_OK),                                       */
/*-------------------------------------------------------------------------------*/
void  PNIO_cbf_rec_read(
                       PNIO_UINT32            DevHndl,
                       PNIO_UINT32            Api,
                       PNIO_UINT16            ArNumber,
                       PNIO_UINT16            SessionKey,
                       PNIO_UINT32            SequenceNum,
                       PNIO_DEV_ADDR*         pAddr,          /* geographical address */
                       PNIO_UINT32            RecordIndex,
                       PNIO_UINT32*           pBufLen,        /* [in, out] in: length to read, out: length, read by user */
                       PNIO_UINT8*            pBuf,           /* [out] buffer pointer */
                       PNIO_ERR_STAT*         pPnioState)     /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                                   ErrCode2), see IEC61158-6 */
{
    int i;
    PNIO_BOOL modPlugged = PNIO_FALSE;
    PNIO_IO_TYPE ioType = PNIO_IO_IN;

    printf ("\n## READ_RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, ioType=%d, Sequence_nr=%lu\n\r",
            (unsigned long)Api, (unsigned long)pAddr->u.Geo.Slot, (unsigned long)pAddr->u.Geo.Subslot,
            (unsigned long)RecordIndex, (unsigned long)*pBufLen, (int)pAddr->IODataType,    (unsigned long)SequenceNum);

    memset(pPnioState, 0, sizeof(*pPnioState));

    /* target module plugged ? */
    for ( i=0; i<gDevArraySize; i++ ) {
        if ( ((PNIO_UINT32)g_device_data[i].slot == pAddr->u.Geo.Slot)&&((PNIO_UINT32)g_device_data[i].subslot == pAddr->u.Geo.Subslot) ) {
            modPlugged = PNIO_TRUE;
            ioType = g_device_data[i].ioType;
            break;
        }
    }

    if ( RecordIndex == IM0
         || RecordIndex == IM1
         || RecordIndex == IM2
         || RecordIndex == IM3
         || RecordIndex == IM4 ) {
        IM_RecRead(RecordIndex, pAddr, pBufLen, pBuf, pPnioState);
    }
    else if ( RecordIndex == DS_NUM_REC_INP_DATA_OBJ_ELEM ) {
        if ( modPlugged == PNIO_TRUE ) {
            if ( (ioType == PNIO_IO_IN)||(ioType == 2/*PNIO_IO_INOUT*/) ) {
                IEC_DS_READ_INP_DATA_OBJ_ELEM content;
                DsInit_InputDataObject(&content, pAddr);
                if ( pPnioState->ErrCode == 0 ) {
                    DsFill_InputDataObject(&content, pBufLen, pBuf, pPnioState);
                }
            }
            else {
                pPnioState->ErrCode   = 0xde;  /* IODReadRes */
                pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
                pPnioState->ErrCode1  = 0xB0;  /*  */
                pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
                pPnioState->AddValue1 = 0;     /* here dont care                                                             */
                pPnioState->AddValue2 = 0;     /* here dont care                                                             */
            }
        }
        else {
            pPnioState->ErrCode   = 0xde;  /* IODReadRes */
            pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
            pPnioState->ErrCode1  = 0xB2;  /* invalid slot/subslot */
            pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
            pPnioState->AddValue1 = 0;     /* here dont care                                                             */
            pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        }
    }
    else if ( RecordIndex == DS_NUM_REC_OUTP_DATA_OBJ_ELEM ) {
        if ( modPlugged == PNIO_TRUE ) {
            if ( (ioType == PNIO_IO_OUT)||(ioType == 2/*PNIO_IO_INOUT*/) ) {
                IEC_DS_READ_OUTP_DATA_OBJ_ELEM content;
                DsInit_OutputDataObject(&content, pAddr);
                if ( pPnioState->ErrCode == 0 ) {
                    DsFill_OutputDataObject(&content, pBufLen, pBuf, pPnioState);
                }
            }
            else {
                pPnioState->ErrCode   = 0xde;  /* IODReadRes */
                pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
                pPnioState->ErrCode1  = 0xB0;  /*  */
                pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
                pPnioState->AddValue1 = 0;     /* here dont care                                                             */
                pPnioState->AddValue2 = 0;     /* here dont care                                                             */
            }
        }
        else {
            pPnioState->ErrCode   = 0xde;  /* IODReadRes */
            pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
            pPnioState->ErrCode1  = 0xB2;  /* invalid slot/subslot */
            pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
            pPnioState->AddValue1 = 0;     /* here dont care                                                             */
            pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        }
    }
    else {
#ifdef PROFI_ENERGY_ENABLE
		PNIO_cbf_rec_read_pe(
                        DevHndl,
                        Api,
                        ArNumber,
                        SessionKey,
                        SequenceNum,
                        pAddr,          /* geographical address */
                        RecordIndex,
                        pBufLen,        /* [in, out] in: length to read, out: length, read by user */
                        pBuf,           /* [out] buffer pointer */
                        pPnioState) ;    /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,*/

#else
        pPnioState->ErrCode   = 0xde;  /* IODReadRes */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
        pPnioState->ErrCode1  = 0xB0;  /* invalid index */
        pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
        pPnioState->AddValue1 = 0;     /* here dont care                                                             */
        pPnioState->AddValue2 = 0;     /* here dont care                                                             */
#endif // PROFI_ENERGY_ENABLE
	}

    if ( pPnioState->ErrCode != 0 ) {
        *pBufLen = 0;
    }

    printf ("## RECORD_DATA transmitted %d byte\n\r", *pBufLen);
    for ( i=0; i < (int)*pBufLen; i++ ) {
        if ( i%16 == 0 )
            printf("\n");

        printf ("0x%02lx ", (long)pBuf[i]);
    }
    printf("## <-READ_RECORD  DONE \n\r");
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_check_ind (...)                                                            */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called for each and every submodule of the IO Base Device      */
/*      interface for which the configuration does not match that of the IO controller. */
/*      The user program is consequently given the option of changing the submodule     */
/*      layout or marking the submodule as compatible or incorrect.                     */
/*      In case of agreement of the configurations, this callback is not called.        */
/*                                                                                      */
/*  This sample function can be modified accordingly                                    */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_check_ind(
                       PNIO_UINT32     DevHndl,       /* [in] Handle for Multidevice */
                       PNIO_UINT32     Api,           /* [in] Api number */
                       PNIO_UINT16     ArNumber,      /* [in] Application-relation number */
                       PNIO_UINT16     SessionKey,    /* [in] session key */
                       PNIO_DEV_ADDR*  pAddr,         /* [in] geographical address */
                       PNIO_UINT32*    pModIdent,     /* [out] Ptr to module identifier */
                       PNIO_UINT16*    pModState,     /* [out] Ptr to module state */
                       PNIO_UINT32*    pSubIdent,     /* [out] Ptr to submodule identifier */
                       PNIO_UINT16*    pSubState)     /* [out] Ptr to submodule state */
{
    int idx;

    printf("\nStep 10 - PNIO_CBF_CHECK_IND() slot=%u, subslot=%u, ModIdent=%u, State (%u), SubIdent=%u, State (%u)\n\r",
           pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, *pModIdent, *pModState, *pSubIdent, *pSubState);

    /* get the index int of our configuration */
    idx = GetSubmodNum(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);

    /*
    Check the configuration sent by controller against the configuration_data structure.
    If there is any mismatch, return error.
    */
    if ( (idx != -1) && ((unsigned int)g_device_data[idx].subslot == pAddr->u.Geo.Subslot) &&
         (g_device_data[idx].modId == *pModIdent) && (g_device_data[idx].subslotId == *pSubIdent) ) {
        *pModState = PNIO_MOD_STATE_PROPER_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_OK;
    }
    else {
        printf ("## the configuration of plugged modules is inconsistent to HWCONFIG\n");
        printf ("## please check your configuration first!\n");
        *pModState = PNIO_MOD_STATE_WRONG_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_WRONG;
        SetSubmoduleDoesntExistInStep7Project(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);
    }
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_check_ind (...)                                                         */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*  This callback is called by the IO Base Device interface as soon as an IO            */
/*      controller establishes a connection with the IO Base Device user programm       */
/*      and transmits its expected configuration for the IO Base Device user program.   */
/*      As a result of this callback, application-relation global parameters are        */
/*      transferred to the IO Base user program for inspection. In case of errors       */
/*      in the application-relation layout, the IO Base Device user program can         */
/*      terminate the application-retional global parameters;                           */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_check_ind(
                          PNIO_UINT32     DevHndl,              /* [in] Handle for Multidevice */
                          PNIO_UINT32     HostIp,               /* [in] ip adderss of the host controller */
                          PNIO_UINT16     ArType,               /* [in] Application-relation type */
                          PNIO_UUID_TYPE  ArUUID,               /* [in] Application-relation UUID */
                          PNIO_UINT32     ArProperties,         /* [in] Application-relation properties */
                          PNIO_UUID_TYPE  CmiObjUUID,           /* [in] UUID of application-relation initiator e.g. IO-controller */
                          PNIO_UINT16     CmiStationNameLength, /* [in] Length of the param station-name (next param) */
                          PNIO_UINT8*     pCmiStationName,      /* [in] Station name */
                          PNIO_AR_TYPE*   pAr)                  /* [in] pointer to application-relation structure PNIO_AR_TYPE */
{
    union {
        unsigned long l;
        unsigned char c[4];
    } lc;
    char stname[256];
    int  len = CmiStationNameLength < 256 ? CmiStationNameLength : 255;
    lc.l = HostIp;
    strncpy(stname, (const char *)pCmiStationName, len);
    stname[len] = '\0';
    printf("\nStep 10 - PNIO_CBF_AR_CHECK_IND() - Station %s, IP %d.%d.%d.%d\n\r",
           stname,lc.c[0], lc.c[1], lc.c[2], lc.c[3]);

    printf("\t ArType=%u  ArProperties=%#x \n\r",ArType,ArProperties);

    /* */
    if ( ArType == 0x10 ) {
        printf("\t PNIO_cbf_ar_check_ind: ---> IRT  \n\r");
        g_bIsochron = 1;  /* == IRT */
    }
    else {
        printf("\t PNIO_cbf_ar_check_ind: ---> RT   \n\r");


        g_bIsochron = 0;
    }


    return;
}

/* */
PNIO_SUBMOD_TYPE * GetPointerToSubModule(PNIO_AR_TYPE* pAr, PNIO_UINT32 SlotNum, PNIO_UINT32 SubSlotNum)
{
	PNIO_MODULE_TYPE * pMod = pAr->pModList;
	PNIO_UINT32  module_index;

	/* iterate all modules */
	for (module_index = 0; module_index < pAr->NumOfMod; ++module_index)
	{
		PNIO_SUBMOD_TYPE * pSub = pMod->pSubList;
		PNIO_UINT32 submodule_index;

		/* iterate all submodules in current module */
		for (submodule_index = 0; submodule_index < pMod->NumOfSubmod; ++submodule_index)
		{
			if ((SlotNum == pSub->SlotNum) && (SubSlotNum == pSub->SubSlotNum))
			{
				return pSub;
			}
			
			++pSub;
		}

		++pMod;
	}

	return NULL;
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_info_ind (...)                                                          */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface as soon as the          */
/*      application-relation for the IO controller is laid out. Consequently,           */
/*      the IO-Base-Device user program is informed about the modules and submodules    */
/*      that will be operated in this application relation..                            */
/*                                                                                      */
/*      In this callback function user can initialize IO data in submodule to initial   */
/*      value 0 and can also set local consumer and provider status                     */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_info_ind(
                         PNIO_UINT32     DevHndl,    /* [in] Handle for Multidevice */
                         PNIO_UINT16     ArNumber,   /* [in] Application-relation number */
                         PNIO_UINT16     SessionKey, /* [in] session key */
                         PNIO_AR_TYPE*   pAr)        /* [in] pointer to application-relation structure PNIO_AR_TYPE */
{
    printf("\n\rStep 11 - PNIO_CBF_AR_INFO_IND() - ArNumber=%d established. SessionKey=%x, Num.Modules=%d \n\r",
           ArNumber, SessionKey, pAr->NumOfMod);

    if ( pAr->NumOfMod == 0 ) {
        /* Supervisor AR */
        printf("\t PNIO_CBF_AR_INFO_IND(): Supervisor AR - No Action \n\r");
        return;
    }

#if 0 // Rueckgebaut, weil falsche Step7-Konfigurationen jetzt von der Firmware korrekt gehandhabt werden
	if (NULL != GetPointerToSubModule(pAr, 0, 0x8000))
	{
		printf("\t PNIO_CBF_AR_INFO_IND(): Wrong Step7 configuration, AR ignored. DAP configured in slot 0, expected in slot 1\n\r");
		return;
	}
#endif
	
	
	g_arNumber = ArNumber;     /* Store the AR number */
    g_SessionKey = SessionKey; /* Store the session key */

    AR_INFO_IND_flag = 1;


    printf("\t <- PNIO_CBF_AR_INFO_IND() - DONE!  AR_INFO_IND_flag = 1 \n\r");
    return;
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_indata_ind (...)                                                        */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*      This callback is called by the IO Base Device interface as soon as an           */
/*      IO controller has transmitted the IO data for the first time. It signals        */
/*      the beginning of cyclical data exchange.                                        */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_indata_ind(
                           PNIO_UINT32     DevHndl,        /* [in] Handle for Multidevice */
                           PNIO_UINT16     ArNumber,       /* [in] Application-relation number */
                           PNIO_UINT16     SessionKey)     /* [in] session key */
{
    printf ("## AR IN-Data event indication received, ArNumber = %x, Session Key = %x\n\r",
            ArNumber, SessionKey);

    INDATA_IND_flag = 1;
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_abort_ind (...)                                                         */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface as soon as the          */
/*      connection is terminated before a data exchange with the IO Controller began     */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_abort_ind(
                          PNIO_UINT32     DevHndl,     /* [in] Handle for Multidevice */
                          PNIO_UINT16     ArNumber,    /* [in] Application-relation number */
                          PNIO_UINT16     SessionKey,  /* [in] session key */
                          PNIO_AR_REASON  ReasonCode)  /* [in] reason code */
{
    g_DeviceReady = 0;
    /* AR abort after ArInData-indication */
    printf ("## AR ABORT indication, ArNumber = %x, Reason = %x\n\r",
            ArNumber, ReasonCode);

    /* abort indicaton by regular (non-supervisor) AR? -> reset connection flags */
    /* PL: check for INDATA_IND_flag removed because this is a callback 'before data exchange'
    */
    if ( /* INDATA_IND_flag && */ (ArNumber == g_arNumber) && (SessionKey == g_SessionKey) ) {
        AR_INFO_IND_flag = 0;
        PRM_END_IND_flag = 0;
        first_startop_done = 0;
        INDATA_IND_flag  = 0;
    }
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_offline_ind (...)                                                       */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface as soon as the          */
/*      connection is terminated after a data exchange with the IO Controller began    */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_offline_ind(
                            PNIO_UINT32     DevHndl,       /* [in] Handle for Multidevice */
                            PNIO_UINT16     ArNumber,      /* [in] Application-relation number */
                            PNIO_UINT16     SessionKey,    /* [in] session key */
                            PNIO_AR_REASON  ReasonCode)    /* [in] reason code */
{
    g_DeviceReady = 0;
    printf ("## AR Offline indication, ArNumber = %x, Reason = %x\n\r",
            ArNumber, ReasonCode);

    /* offline indicaton by regular (non-supervisor) AR? -> reset connection flags */

    /* PL: check for INDATA_IND_flag removed: because of race conditions: AR-INDATA and AR-OFFLINE
    */
    if ( /* INDATA_IND_flag && */ (ArNumber == g_arNumber) && (SessionKey == g_SessionKey) ) {
        AR_INFO_IND_flag = 0;
        PRM_END_IND_flag = 0;
        first_startop_done = 0;
        INDATA_IND_flag  = 0;
    }
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_prm_end_ind (...)                                                          */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*   This callback is called by the IO Base Device interface, as soon as an             */
/*   IO controller signals the end of the parametrizing phase.                          */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_prm_end_ind(
                         PNIO_UINT32 DevHndl,            /* [in] Handle for Multidevice */
                         PNIO_UINT16 ArNumber,           /* [in] Application-relation number */
                         PNIO_UINT16 SessionKey,         /* [in] session key */
                         PNIO_UINT32 Api,                /* [in] Associated API */
                         PNIO_UINT16 SlotNum,            /* [in] slot number */
                         PNIO_UINT16 SubslotNum)         /* [in] sub-slot number */
{
    printf("\nStep 13 - PNIO_CBF_PRM_END_IND() - ArNumber = %x\n\r",
           ArNumber);

    do_after_prm_end_ind_cbf();

    PRM_END_IND_flag = 1;

    printf("Step 13<- PNIO_CBF_PRM_END_IND() - DONE:  PRM_END_IND_flag -> 1 \n\r");
    return;
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_cp_stop_req ()                                                             */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*                                                                                      */
/*  Input:     ---                                                                      */
/*  Output:    ---                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_cp_stop_req(PNIO_UINT32 DevHndl)
{
    printf("## PNIO_cbf_cp_stop_req\n\r");
    stopreq=1;
    if ( g_pno_test == 0 )
        printf("press 'q' to quit...\n");
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_device_stopped ()                                                          */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface after                   */
/*  the device stop request is received.                                                */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_device_stopped(
                            PNIO_UINT32 DevHndl,    /* [in] Handle for Multidevice */
                            PNIO_UINT32 Reserved)   /* Reserved for future use */
{
    printf("## PNIO_cbf_device_stopped\n\r");

    /*
    synchronization - The main function calls device-stop-request function
    and then the main function waits for DEVICE_STOPPED_flag to be set
    */
    DEVICE_STOPPED_flag = 1;
}

/*-----------------------------------------------------------------*/
/* NOT USED CALLBACKS                                              */
/*-----------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------*/
/*                                                                                   */
/*  PNIO_cbf_req_done(...)                                                           */
/*                                                                                   */
/*-----------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface, as soon as an alarm */
/*      is sent                                                                      */
/*-----------------------------------------------------------------------------------*/
void PNIO_cbf_req_done(
                      PNIO_UINT32          DevHndl,        /* Handle for Multidevice */
                      PNIO_UINT32          UserHndl,       /* user defined handle */
                      PNIO_UINT32          Status,
                      PNIO_ERR_STAT      * pPnioState)
{
    printf ("## PNIO_cbf_req_done UserHndl = %x, Status = %x\n\r",
            UserHndl, Status);
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_apdu_status_ind()                                     */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This callback is called after the state of the Controller       */
/* has changed                                                     */
/*-----------------------------------------------------------------*/
void PNIO_cbf_apdu_status_ind(
                             PNIO_UINT32          DevHndl,
                             PNIO_UINT16          ArNumber,
                             PNIO_UINT16          SessionKey,
                             PNIO_APDU_STATUS_IND ApduStatus)
{
    printf("APDU Status Change\n\r");
}


/*-----------------------------------------------------------------*/
/*  IRT Start OP                                                   */
/*-----------------------------------------------------------------*/
/* This callback is called at the start of IRT cycle               */
/*-----------------------------------------------------------------*/
void Cbf_StartOP_ind(PNIO_CP_CBE_PRM *prm)
{
    PNIO_UINT32 ErrorCode;

    ++g_StartOpCount;

   /* we wait until the 'AR' is astablished and "PARAMETER-END" was received,
    * i.e. 'PNIO_cbf_prm_end_ind()' callbck was called.
    * This callback is called cyclic before the communication is possible,.
    * so return without action.                                                                    .
    */
    if ( !AR_INFO_IND_flag || !PRM_END_IND_flag ) {
        /* no connection yet
         * no action
         */
        PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);
        return;
    }

   /* 1st time we have to set IOXS to GOOD
    * and set the "State Flag" to signal to the main thread to send "APPLICATION-READY"
    */
    if ( !first_startop_done ) {
        /* Read data from controller */
        PNIO_initiate_data_read_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
        /* write data to controller */
        PNIO_initiate_data_write_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
        /* signal that data processing is done for this cycle */
        PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);

        first_startop_done = 1;
        printf("\nStep 16a- IOXS GOOD\n\r");
    }
    else {
        int slot, sub, idx;

        ++g_DataExchangeCount;

        /* Read data from controller */
        ErrorCode = PNIO_initiate_data_read_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
        if ( ErrorCode != PNIO_OK )
            ++g_readErrors;

        /* increment in data */
        for ( slot=0; slot<NUMOF_SLOTS; slot++ ) {
            for ( sub=0; sub<NUMOF_SUBSLOTS; sub++ ) {
                for ( idx=0; idx<(int)InDatLen[slot][sub];idx++ ) {
                    InData[slot][sub][idx]++;
                }
            }
        }

        /* write data to controller */
        ErrorCode = PNIO_initiate_data_write_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
        if ( ErrorCode != PNIO_OK )
            ++g_writeErrors;

        /* signal that data processing is done for this cycle */
        PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);
    }


    /* Diag Output only */
    if ( (g_StartOpCount % 10000) == 0 ) {
        printf("StartOp: cnt=%lu DX-cnt=%lu wErr=%lu PNIO-ERR=%#x \n\r", g_StartOpCount,g_DataExchangeCount, g_writeErrors, ErrorCode  );
    }

    return;
} /* end of Cbf_StartOP_ind */

/*-----------------------------------------------------------------*/
/*  IRT OP Fault                                                   */
/*-----------------------------------------------------------------*/
/* This callback is called if IRT data could not be sent/received  */
/* in the last IRT cycle. Outgoing data is marked as invalid.      */
/* This will cause an abort of the AR to the controller.           */
/*-----------------------------------------------------------------*/
void Cbf_OPFault_ind(PNIO_CP_CBE_PRM *prm)
{
    /* Opfault has come */
    ++g_OpFaultCount;
}

/*----------------------------------------------------------------------------------------------------*/
/*    FUNCTIONS                                                                                       */
/*----------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/*                                                                    */
/*  Function :      setNWOrder()                                      */
/*                                                                    */
/*--------------------------------------------------------------------*/
/*      This function checks the byte-order of the machine, corrects  */
/*      it if required.                                               */
/*--------------------------------------------------------------------*/

unsigned short setNWOrder(
                         unsigned short val) /* Value whose order is to be corrected */
{
    typedef union {
        unsigned short          sh;
        unsigned char           by[2];
    } TEST_NW_ORDER; /* structure to check the byte order */

    TEST_NW_ORDER   tstVal;

    tstVal.sh = 0x0102;     /* Some test value will do */

    if ( tstVal.by[0] == 0x01 ) {
        /* higher byte has gone at address 0 - byte ordering is present */
        return val;
    }
    else {
        /* reverse byte order */
        return(val >> 8) | ((val&0xff) << 8);
    }
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :      SendDiagnosticAlarm()                         */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function sends a diagnostic alarm message to the PNIO    */
/*      controller.                                               */
/*----------------------------------------------------------------*/
PNIO_UINT32 SendDiagnosticAlarm(
                               PNIO_UINT32     channelErrType, /* Type of error - alarm type */
                               PNIO_UINT16     Slot,
                               PNIO_UINT16     diagTag)            /* diagnostic tag/alarm handle */
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    PNIO_UINT16 uiDiagAlarmProp = 0; /* Channel property to be created and passed to the channel add function */
    int modIdx = Slot;  /* index of the module */
    diagAlarmData_t diagData;        /* diagnosis data */

    PNIO_DEV_ADDR   addr;            /* Address info of the module */

    addr.AddrType      = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
    addr.u.Geo.Slot    = modIdx;        /* module index*/
    addr.u.Geo.Subslot = 1;             /* for our test application always 1 */

    printf("Sending a diagnostic alarm...\n");

    /*--------------------------------------*/
    /* Build channel properties             */
    /*--------------------------------------*/
    uiDiagAlarmProp = PNIO_build_channel_properties(
                                                   g_hDevice,                             /* Device handle */
                                                   PNIO_DIAG_CHANPROP_TYPE_SUBMOD,        /* for our dev. always this  */
                                                   PNIO_DIAG_CHAN_SPEC_ERROR_APPEARS,     /* Der Fehler kommt (aus Sicht des IO-Controllers)*/
                                                   PNIO_DIAG_CHAN_DIRECTION_MANUFACTURE); /* 0: = specific */

    /*--------------------------------------*/
    /* Storing diagnosis data in a sub-slot */
    /*--------------------------------------*/
    ErrorCode = PNIO_diag_channel_add(
                                     g_hDevice,                             /* Device handle */
                                     g_device_data[modIdx].api,             /* Api number */
                                     &addr,                                 /* Address */
                                     1,                                     /* subslot number always 1 here */
                                     uiDiagAlarmProp,                       /* channel properties */
                                     channelErrType,                        /* errortype - function param */
                                     diagTag);                              /* diag tag/ alarm handle */

    if ( ErrorCode != PNIO_OK ) {
        printf("Error in adding a diagnostic channel\n");
        return ErrorCode;
    }

    /*--------------------------------------------------------------*/
    /* set the device state to PNIO_DEVSTAT_OPERATE_STATION_PROBLEM */
    /*--------------------------------------------------------------*/
    ErrorCode = PNIO_set_dev_state (g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in changing the device state\n");
        return ErrorCode;
    }

    /*------------------------------------------------------------------------------------------------*/
    /* Send diagnosis alarm to the PNIO controller, handle the rest in the callback PNIO_cbf_req_done */
    /*------------------------------------------------------------------------------------------------*/
    diagData.chanProp    = setNWOrder(uiDiagAlarmProp);
    diagData.chanNum     = setNWOrder(0x8000);
    diagData.chanErrType = setNWOrder((unsigned short)diagTag);

    ErrorCode = PNIO_diag_alarm_send(
                                    g_hDevice,                             /* Device Handle */
                                    g_device_data[modIdx].api,             /* Api number */
                                    g_arNumber,                            /* received in PNIO_cbf_ar_info_ind */
                                    g_SessionKey,                          /* received in PNIO_cbf_ar_info_ind */
                                    PNIO_STATE_ALARM_APPEARS,
                                    &addr,                                 /* location (slot, subslot) */
                                    (PNIO_UINT8 *)&diagData,               /* Alarm Data */
                                    sizeof(diagAlarmData_t),               /* size of the structure sent */
                                    0x8000,                                /* means, pData contains diagnosis information */
                                    diagTag);                          /* user handle, to identify user in multi-user scenario */

    if ( ErrorCode != PNIO_OK ) {
        printf("Error in sending a diagnostic alarm\n");
    }

    return ErrorCode;
}

/*************************************************************/
/*                                                           */
/*  Function :      getCharWithTimeout()                     */
/*                                                           */
/*-----------------------------------------------------------*/
/*                                                           */
/* Get a character form console                              */
/*                                                           */
/*************************************************************/
int getCharWithTimeout()
{
    int key = 0;
#ifndef WIN32
    struct pollfd pollfd[1];
    static int init = 0;
    static struct termios termiosOld;
    static struct termios termios;

    if ( !init ) {
        tcgetattr(fileno(stdin), &termios);
        memcpy(&termiosOld, &termios, sizeof (termios));
        cfmakeraw(&termios);
        init++;
    }
    /* put terminal into raw mode */
    tcsetattr(fileno(stdin), TCSANOW, &termios);
    pollfd->fd = fileno(stdin);
    pollfd->events = POLLIN;
    pollfd->revents = 0;
    poll(pollfd, 1, 100);
    if ( pollfd->revents & POLLIN )
        key = getchar();
    tcsetattr(fileno(stdin), TCSANOW, &termiosOld);
#else

    Sleep(100);
    if ( _kbhit() ) {
        key = _getch();
    }
#endif

    return key;
}


























































/*-----------------------------------------------------------------*/
/*                                                                 */
/*  Function :      do_after_prm_end_ind_cbf()                     */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This function is called after the PNIO_cbf_prm_end_ind callback */
/* has been called ( see main() ), it calls for the first time     */
/* PNIO_initiate_data_write(), PNIO_initiate_data_read and         */
/* PNIO_set_appl_state_ready                                       */
/*                                                                 */
/*-----------------------------------------------------------------*/

/* Wait for PNIO_cbf_prm_end_ind callback, then
   initiate first write and read and set_appl_state_ready */
void do_after_prm_end_ind_cbf(void)
{
    int i, j;
    PNIO_UINT32 ErrorCode;
//    PNIO_APPL_READY_LIST_TYPE readyListType;

    /*-------------------------------------------------------------------*/
    /*   set all IO data in submodules to inital value = 0              */
    /*-------------------------------------------------------------------*/
    memset (&InData,        0, sizeof (InData));        /* IO data (input) */
    memset (&InDatLen,      0, sizeof (InDatLen));      /* length of input data */
    memset (&InDatIops,     0, sizeof (InDatIops));     /* local provider status */
    memset (&InDatIocs,     0, sizeof (InDatIocs));     /* remote consumer status */

    memset (&OutData,       0, sizeof (OutData));       /* IO data (output) */
    memset (&OutDatLen,     0, sizeof (OutDatLen));     /* length of output data */
    memset (&OutDatIocs,    0, sizeof (OutDatIocs));    /* local consumer status */
    memset (&OutDatIops,    0, sizeof (OutDatIops));    /* remote provider status */

    /*----------------------------------------------------------------------*/
    /* set local provider status preset values for all input/output slots   */
    /* set local consumer status for all output slots                       */
    /*----------------------------------------------------------------------
     * gDevArraySize== numOfSubmodul== number of Modules of the current selected device
     */
    for ( i = 0; i < gDevArraySize ; i++ ) {
        for ( j = 0; j < 1 /*g_device_data[i].maxSubslots*/; j++ ) {
            /*** set local provider state = GOOD for input data***/
            if ( i == 0 ) {
                if ( g_device_data[i].modState == 1 ) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
                    OutDatIocs [i][j] = PNIO_S_GOOD;
                }
                else {
                    printf("## Warning do_after_prm_end_ind_cbf: modState ERROR. slot=%d modId=%#x \n",i,g_device_data[i].modId);
                    InDatIops [i][j] = PNIO_S_BAD;
                    OutDatIocs [i][j] = PNIO_S_BAD;
                }
            }
            else {
                if ( g_device_data[i].modState == 1 &&
                     g_device_data[i+j].subState == 1 ) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
                    OutDatIocs [i][j] = PNIO_S_GOOD;
                }
                else {
                    InDatIops [i][j] = PNIO_S_BAD;
                    OutDatIocs [i][j] = PNIO_S_BAD;
                }
            }
        }
    }

    /* PNIO_initiate_data_read/write is made here for RT only,
     * for IRT it is made in the Cbf_StartOP_ind callback
     */
     if (  !g_bIsochron ) {
        /* We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program  */
        /* can set the local status for functional submodules of all the outgoing data             */
        /* (from the perspective of the IO controller) to GOOD. For all non-functional submodules, */
        /* the local status has to be set to "BAD".                                                */
        ErrorCode = PNIO_initiate_data_read(g_hDevice);
        if ( PNIO_OK != ErrorCode ) {
            printf("## Error - PNIO_init_data_read. 0x%x\n", ErrorCode);
        }
        ErrorCode = PNIO_initiate_data_write(g_hDevice);
        if ( PNIO_OK != ErrorCode ) {
            printf("## Error - PNIO_init_data_write. 0x%x\n", ErrorCode);
        }
        printf("Step 14 - PNIO_initiate_data_read / data_write()\n\r");
    } /* end if RT */

    /* Note PNIO_set_appl_state_ready is called in the main thread (at the same place for IRT and RT */

    return;
















} /* end of do_after_prm_end_ind_cbf() */

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :      GetSubmodNum()                                */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function returns the index in the IO data array          */
/*  for the given submodule.                                      */
/*                                                                */
/*----------------------------------------------------------------*/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub)
{
    int entries = gDevArraySize;
    int i;
    int j;

    for ( i = 0; i < entries; i++ ) {
        if ( (int)mod == idxTbl[i] )
            break;
    }

    if ( i == entries )
        return -1;

    for ( j = 0; j < g_device_data[i].maxSubslots; j++ ) {
        if ( g_device_data[i+j].subslot == (int)sub )
            return j;
    }

    return -1;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :       AddModSubMod()                               */
/*                                                                */
/*----------------------------------------------------------------*/
/*  This function adds all the modules and submodules of the      */
/*      device in serial order.                                   */
/*----------------------------------------------------------------*/
PNIO_UINT32 AddModSubMod(void)
{
    PNIO_UINT32   status = PNIO_OK;
    PNIO_DEV_ADDR addr;  /* location (module/submodule) */
    int           slot=0;
    int           entries = gDevArraySize;
    int           i;

    addr.AddrType = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */

    InitSubmoduleExistsInStep7Project();

    /*------------------------------------------------------------------*/
    /*  add module 0 and corresponding submodule first                  */
    /*------------------------------------------------------------------*/

    addr.IODataType    = g_device_data[0].ioType;
    addr.u.Geo.Slot    = g_device_data[0].slot;    /* plug module 0 */
    addr.u.Geo.Subslot = g_device_data[0].subslot; /* get the corresponding sub-slot */

    printf("Step 3 - PNIO_mod_plug()\n");
    status = PNIO_mod_plug (g_hDevice,      /* device handle */
                            g_device_data[0].api,               /* api number */
                            &addr,                              /* location (slot, subslot) */
                            g_device_data[0].modId);            /* module 0 identifier */

    if ( status == PNIO_OK ) {
        printf("Module plug\t\t:");
        g_device_data[0].modState = 1;
    }
    else {
        printf("Module plug failed\t:");
        g_device_data[0].modState = 0;
    }

    printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
           g_device_data[0].api,
           g_device_data[0].slot,
           g_device_data[0].subslot,
           g_device_data[0].maxSubslots,
           g_device_data[0].modId);

    if ( !g_device_data[0].modState ) {
        printf("ERROR: Failure in plugging module 0 -> no other module / submodule will be plugged...\n");
        return(status);
    }

    // now plug submodule corresponding to module 0, im support
    status = PNIO_sub_plug_ext_IM (
                                  g_hDevice,                    /* device handle */
                                  g_device_data[0].api,         /* api number */
                                  &addr,                        /* location (slot, subslot) */
                                  g_device_data[0].subslotId,   /* submodule identifier */
                                  PNIO_ALARM_TYPE_PLUG,         /* submodule id correct */
                                  PNIO_PLUG_IM0_BITS_SUBMODULE);/* submodule contains IM data */

    if ( status == PNIO_OK ) {
        printf("Submodule plug\t\t:");
        g_device_data[0].subState = 1;
    }
    else {
        printf("Submodule plug failed\t:");
        g_device_data[0].subState = 0;
    }

    printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
           g_device_data[0].api,
           g_device_data[0].slot,
           g_device_data[0].subslot,
           g_device_data[0].maxSubslots,
           g_device_data[0].modId);

    if ( !g_device_data[0].subState ) {
        printf("ERROR: Failure in plugging the submodule corresponding to module 0\n");
        printf(" -> no other module / submodule will be plugged...\n");
        return(status);
    }

    printf("\nNow we can plug the other modules and submodules - first the modules, then the submodules..\n");

    /*-----------------------------------------*/
    /*  now add all modules                    */
    /*-----------------------------------------*/

    /*
    In the for-loop below, i is not incremented contineously.
    Instead, 'i' goes only through those entries of the g_device_data array, which are a
    slot and jumps over the sub-slots as we are adding ONLY modules in this loop
    */
    for ( i = 1; i < entries; ) {
        addr.IODataType    = g_device_data[i].ioType;
        addr.u.Geo.Slot    = g_device_data[i].slot;    /* plug module at correct slot */
        addr.u.Geo.Subslot = g_device_data[i].subslot; /* get the corresponding sub-slot*/



        status = PNIO_mod_plug (
                               g_hDevice,               /* device handle */
                               g_device_data[i].api,    /* api number */
                               &addr,                   /* location (slot, subslot) */
                               g_device_data[i].modId); /* module identifier */

        if ( status == PNIO_OK ) {
            printf("Module plug\t\t:");
            g_device_data[i].modState = 1;
        }
        else {
            printf("Module plug failed\t:");
            g_device_data[i].modState = 0;
        }







        printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
               g_device_data[i].api,
               g_device_data[i].slot,
               g_device_data[i].subslot,
               g_device_data[i].maxSubslots,
               g_device_data[i].modId);

        if ( status == PNIO_OK ) {
            /*
            advance in the g_device_data structure, jumping over all the submodule entries
            to reach the next module entry in the structure
            */
            i += g_device_data[i].maxSubslots;
        }
        else {
            /*
            go to the next entry in g_device_data table
            */
            i++;
        }
    }/*end for*/


    /*-----------------------------------------*/
    /*  add  submodules                        */
    /*-----------------------------------------*/
    for ( i = 1; i < entries; i++ ) {

        /*
        in the g_device_data structure, each module entry has got at least 1 sub-slot associated with it (maxSubslot > 0)
        whereas all the submodule entries are having no sub-slots (maxSubslots = 0).
        */
        if ( g_device_data[i].maxSubslots > 0 ) {

            /* beginning of a new slot */
            slot = i; /* index of the corresponding slot for a given subslot */

            g_device_data[slot].subState = 1; /* assume that the submodules for this slot are
                                                 going to be successfully added, if any module is not added
                                                 correctly, it will be later set to 0 */
        }

        if ( g_device_data[slot].modState ) {

            /* add submodule only if the module is added */
            addr.IODataType    = g_device_data[i].ioType;
            addr.u.Geo.Slot         = g_device_data[i].slot;
            addr.u.Geo.Subslot      = g_device_data[i].subslot;


            status = PNIO_sub_plug (
                                   g_hDevice,                    /* device handle */
                                   g_device_data[i].api,         /* api number */
                                   &addr,                        /* location (slot, subslot) */
                                   g_device_data[i].subslotId);  /* submodule identifier */

            if ( status == PNIO_OK ) {
                printf("Submodule plug\t\t:");

                g_device_data[i].subState = 1;
            }
            else {
                printf("Submodule plug failed\t:");

                g_device_data[i].subState = 0;
                g_device_data[slot].subState = 0;
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
                   g_device_data[i].api,
                   g_device_data[i].slot,
                   g_device_data[i].subslot,
                   g_device_data[i].maxSubslots,
                   g_device_data[i].modId);
        }
    }/*end for*/

    /*if not all the modules/submodules are plugged correctly, print warning*/
    for ( i = 0; i < entries; i++ ) {
        if ( g_device_data[i].subState == 0 ) {
            printf("WARNING: Not all modules or submodules were plugged correctly!!\n");
            break;
        }
    }

    return(status);
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :       RemoveModSubMod()                            */
/*                                                                */
/*----------------------------------------------------------------*/
/*  This function first removes the submodules and then the       */
/*  modules from the PNIO device in reverse order.                */
/*----------------------------------------------------------------*/
PNIO_UINT32 RemoveModSubMod(void)
{
    int i;
    PNIO_DEV_ADDR addr;  /* location (module/submodule) */
    int entries = gDevArraySize;
    PNIO_UINT32 status = PNIO_OK;

    printf("\n");

    /* Remove the modules/submodules in the reverse order */

    for ( i = entries - 1; i >= 0 && status == PNIO_OK; i-- ) {
        if ( g_device_data[i].subState == 1 ) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot; /* slot number */
            addr.u.Geo.Subslot  = g_device_data[i].subslot;




            /*-----------------------------------------*/
            /*  remove  submodules                     */
            /*-----------------------------------------*/
            status = PNIO_sub_pull (g_hDevice,g_device_data[i].api, &addr);

            if ( status == PNIO_OK ) {
                g_device_data[i].subState = 0;

                printf("Submodule pull\t\t:");
            }
            else {
                printf("Submodule pull failed\t:");
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
                   g_device_data[i].api,
                   g_device_data[i].slot,
                   g_device_data[i].subslot,
                   g_device_data[i].maxSubslots,
                   g_device_data[i].modId);

            /*
            It is very important to notify the controller that the device state is
            NOT-OK every time after removing a submodule.
            */
            status = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
        }

        if ( status == PNIO_OK && g_device_data[i].modState == 1 ) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot;
            addr.u.Geo.Subslot  = 1;                         /* dont care */

            /*-----------------------------------------*/
            /*  remove  modules                        */
            /*-----------------------------------------*/

            status = PNIO_mod_pull(g_hDevice,g_device_data[i].api,&addr);

            if ( status == PNIO_OK ) {
                printf("Module pull\t\t:");
                g_device_data[i].modState = 0;
            }
            else {
                printf("Module pull failed\t:");
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
                   g_device_data[i].api,
                   g_device_data[i].slot,
                   g_device_data[i].subslot,
                   g_device_data[i].maxSubslots,
                   g_device_data[i].modId);

            /*
            Notify the controller that the device state is NOT-OK.
            */
            status = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
        }
    }
    return status;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :       AddApi()                                     */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function adds all the api stated in our configuration    */
/*  structure for each module/submodule                           */
/*                                                                */
/*----------------------------------------------------------------*/
PNIO_UINT32 AddApi(void)
{
    int         i;
    int         j;
    int         highestSlotsNumber;
    int         highestSubslotNumber = 0;
    PNIO_UINT32 api;
    PNIO_UINT32 status = PNIO_OK;

    /* for each slot */
    for ( i = j = 0; i < gDevArraySize; i++ ) {
        /* read api from our configuration data */
        api = g_device_data[i].api;

        /* look if api added at a prior position */
        for ( j = 0; j < i; j++ ) {
            if ( api == g_device_data[j].api ) {
                /* api was added */
                break;
            }
        }

        if ( i == j ) { /* not added, add a new api */
            /* calculate highest slot and subslot number for this api */
            highestSlotsNumber   = g_device_data[j].slot;
            highestSubslotNumber = g_device_data[j].subslot;

            /*
            check if the api exists in the slots ahead,
            if yes, then update highest slot/subslot number accordingly
            */
            for ( j = i+1; j <  gDevArraySize; j++ ) {
                if ( api == g_device_data[j].api ) {
                    if ( g_device_data[j].slot > highestSlotsNumber )
                        highestSlotsNumber = g_device_data[j].slot;
                    if ( g_device_data[j].subslot > highestSubslotNumber )
                        highestSubslotNumber = g_device_data[j].subslot;
                }
            }

            printf("Step 2 - PNIO_api_add\n");
            status = PNIO_api_add(
                                 g_hDevice,              /* in */
                                 api,                    /* in */
                                 (PNIO_UINT16) highestSlotsNumber,     /* in */
                                 (PNIO_UINT16) highestSubslotNumber);  /* in */

            if ( status != PNIO_OK )
                printf("\t PNIO_api_add failed\n");
            else
                printf("\t PNIO_api_add successful\n");
        }
    }

    return status;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :      RemoveApi()                                   */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function removes all the api stated in our configuration */
/*  structure.                                                    */
/*                                                                */
/*----------------------------------------------------------------*/

PNIO_UINT32 RemoveApi(void)
{
    int         i;
    int         j;
    PNIO_UINT32 api;
    PNIO_UINT32 status = PNIO_OK;
    int         entries = gDevArraySize;

    /* for each slot */
    for ( i = j = 0; i < entries && status == PNIO_OK; i++ ) {
        /* read api from our configuration data */
        api = g_device_data[i].api;

        /* look if the api has been added at a prior position in our g_device_data structure */
        for ( j = 0; j < i; j++ ) {
            if ( api == g_device_data[j].api ) {
                /* api added at a prior position, hence it has already been removed*/
                break;
            }
        }

        if ( i == j ) { /* api not removed yet */
            status = PNIO_api_remove(g_hDevice, api);
            if ( status != PNIO_OK ) {
                printf("\nApi remove failed\t:");
            }
            else {
                printf("\nApi removed\t\t:");
            }
            printf(" api=%u\n\n", api);
        }
    }
    return status;
}

/*-------------------------------------------------------------------*/
/*                                                                   */
/*  Function :      Initialize()                                     */
/*                                                                   */
/*-------------------------------------------------------------------*/
/*      This function does the initialization of the PNIO            */
/*      device. Registration of callbacks is part of initialization  */
/*-------------------------------------------------------------------*/
PNIO_UINT32 Initialize(void)
{
    PNIO_CFB_FUNCTIONS      structCBFunctions;
    PNIO_UINT32 Handle;
    PNIO_UINT32 uiMaxAR = 2; /* maximum application relationships supported. */
    PNIO_UINT32 ErrorCode = PNIO_OK;


    //uiMaxAR = PNIO_DEVICE_SET_MAXAR_BY_TYPE(1,0,1);

    /* Initialize the annotation structure */
    PNIO_ANNOTATION structPNIOAnnotation = {
        ANNOT_NAME,
        ANNOT_ORDERID,
        ANNOT_HW_REV,
        ANNOT_SW_PREFIX,
        ANNOT_SW_REV_1,
        ANNOT_SW_REV_2,
        ANNOT_SW_REV_3};

    /* Initialize the callback structure */
    /* Set the callback function pointers */

    memset(&structCBFunctions, 0, sizeof(PNIO_CFB_FUNCTIONS));
    structCBFunctions.size                  = sizeof(PNIO_CFB_FUNCTIONS);
    structCBFunctions.cbf_data_write        = PNIO_cbf_data_write;
    structCBFunctions.cbf_data_read         = PNIO_cbf_data_read;

    structCBFunctions.cbf_rec_read          = PNIO_cbf_rec_read;
    structCBFunctions.cbf_rec_write         = PNIO_cbf_rec_write;
    structCBFunctions.cbf_alarm_done        = PNIO_cbf_req_done;
    structCBFunctions.cbf_check_ind         = PNIO_cbf_check_ind;
    structCBFunctions.cbf_ar_check_ind      = PNIO_cbf_ar_check_ind;
    structCBFunctions.cbf_ar_info_ind       = PNIO_cbf_ar_info_ind;
    structCBFunctions.cbf_ar_indata_ind     = PNIO_cbf_ar_indata_ind;
    structCBFunctions.cbf_ar_abort_ind      = PNIO_cbf_ar_abort_ind;
    structCBFunctions.cbf_ar_offline_ind    = PNIO_cbf_ar_offline_ind;
    structCBFunctions.cbf_apdu_status_ind   = PNIO_cbf_apdu_status_ind;
    structCBFunctions.cbf_prm_end_ind       = PNIO_cbf_prm_end_ind;
    structCBFunctions.cbf_cp_stop_req       = PNIO_cbf_cp_stop_req;
    structCBFunctions.cbf_device_stopped    = PNIO_cbf_device_stopped;
    structCBFunctions.cbf_start_led_flash   = NULL;
    structCBFunctions.cbf_stop_led_flash    = NULL;


    Handle = 0;  /*0 is invalid handle */

    printf("Step 1 - PNIO_device_open()\n");

    ErrorCode = PNIO_device_open(
                                /*in*/  CP_INDEX,                 /* index of communication processor */
#ifdef PROFI_ENERGY_ENABLE
                                /*in*/  PNIO_CEP_MODE_CTRL | PNIO_CEP_PE_MODE_ENABLE,       /* permission to change operation mode, PROFIenergy */
#else
                                /*in*/  PNIO_CEP_MODE_CTRL,       /* permission to change operation mode */
#endif // PROFI_ENERGY_ENABLE
                                /*in*/  VENDOR_ID,                /* vendor ID */
                                /*in*/  DEVICE_ID,                /* device ID */
                                /*in*/  INSTANCE_ID,              /* instance ID */
                                /*in*/  uiMaxAR,                  /* max AR count*/
                                /*in*/  &structPNIOAnnotation,    /* annotation */
                                /*in*/  &structCBFunctions,       /* callback functions information*/
                                /*out*/ &Handle);               /* device handle */


    if ( (ErrorCode != PNIO_OK) || (0 == Handle) ) {
        //printf("\t <-PNIO_device_open()  ERROR !!!  ErrorCode=%#x \n",ErrorCode);
        return ErrorCode;
    }

    g_hDevice = Handle;
    return  PNIO_OK;
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :      Uninitialize()                                  */
/*                                                                  */
/*------------------------------------------------------------------*/
/*      This function does the de-initialization of the PNIO device */
/*------------------------------------------------------------------*/
PNIO_UINT32 Uninitialize(void)
{
    printf("Uninitialize:  --> PNIO_device_close()\n");
    return PNIO_device_close(g_hDevice);
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :      ConfigureDeviceData()                           */
/*                                                                  */
/*------------------------------------------------------------------*/
/*  In the sample program #define is used to write the config       */
/*      data. This function creates a structure out of the config   */
/*  data, and fills the unfilled members whereever necessary.       */
/*      In practice the developer can create a structure similar to */
/*      this from the '.ini' file.                                  */
/*------------------------------------------------------------------*/
void ConfigureDeviceData(int configIndex)
{
    int i = 0;
    int beginNewSlot = 0;
    int idx = 0;      /* counter for idxTbl */

    /*
    copy the predefined structure to g_device_data as it is.
    This piece of code can be replaced by one with '.ini' file reading
    and creating the elementary structure g_device_data.
    */

    g_device_data = g_device_data_set[configIndex].deviceData;

    /* fill idxTbl with -1, only entries corresponding to a slot will have the right slot id. */
    memset(idxTbl, -1, DEVICE_DATA_ENTRIES_MAX * sizeof(int));
    idxTbl[idx] = g_device_data[0].slot;

    /*
    our predefined structure has the maxsubslots field set to 0, even though
    there are subslots for every module. Here we dynamically calculate the
    subslots in each module and update the g_device_data structure accordingly
    */

    /* browsing through the device_data structure */
    for ( i = 0; i < gDevArraySize; i++ ) {
        if ( g_device_data[i].slot == g_device_data[beginNewSlot].slot ) {
            /* we are still in same slot and new sub-slot */
            /*
            if we are in the same slot, then the information in this row
            is regarding a new subslot. So increment the sub-slot count by 1
            */
            g_device_data[beginNewSlot].maxSubslots++;
            idxTbl[idx++];

            /*
            we assign the slotId/modId to the corresponding sub-slot
            */
            g_device_data[i].modId = g_device_data[beginNewSlot].modId;
        }
        else {
            /* new slot information has started */
            beginNewSlot = i;                            /* index corresponding to the beginning of the new slot */
            g_device_data[beginNewSlot].maxSubslots = 1; /* every new module/slot has min one sub-slot */
            idxTbl[idx++] = g_device_data[i].slot;       /* store the entry of the new slot in idxTbl */
        }
    }
}













































































/*--------------------------------------------------------------------------------------------*/
/*------------------------------    main  entry  ---------------------------------------------*/
/*--------------------------------------------------------------------------------------------*/

int main(void)
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    PNIO_APPL_READY_LIST_TYPE readyListType;
    PNIO_UINT16 user_handle = 0x01;
    int ch = 0, i;
    int nrOfDeviceConfigurations = sizeof(g_device_data_set) / sizeof(device_data_set_t);
    int deviceConfigIndex = 0;
    unsigned long loopCnt = 0;


    printf("\n-----------------------------------------------------------------------------\n");
    printf("\n-------     PNIO Device I&M Sample Program fuer den PNIO-Tester     ---------\n");
    printf("\n-------     Version V02.00.00.02                                    ---------\n");
    printf("\n-----------------------------------------------------------------------------\n");
  
#ifdef PROFI_ENERGY_ENABLE
	PE_Initialize(); // PE initialize
#endif // PROFI_ENERGY_ENABLE

   /* fix settings for IRT Device certification.
    * Configuration of this Device-Application is made in the easy_dev_cfg.h header file.
    * This device configuration muss be identical with the settings in the "PNO Testbed controller"
    * Config params are set in the macro 'DEVICE_DATA' which defines the g_device_data_set.
    * Its an array of g_device_data cofigurations.
    *
    * For IRT certification test we use the 2nd configuration i.e. One with index '1'.
    */
    deviceConfigIndex = 1;
    gDevArraySize = g_device_data_set[deviceConfigIndex].nrOfSubmodules;
    g_pno_test=1;

    /*-------------------------------------------------*/
    /* initialize the g_device_data and I&M structures */
    /*-------------------------------------------------*/
    ConfigureDeviceData(deviceConfigIndex);
    IM_initImRecords(im_work, &im_default);
    IM_DataFromStore("im.conf", im_work);

    begin:

    /*-----------------------------------*/
    /* clear device state control flags  */
    /*-----------------------------------*/
    first_startop_done = 0;
    AR_INFO_IND_flag   = 0;
    PRM_END_IND_flag   = 0;
    INDATA_IND_flag    = 0;

    /*-----------------------------------*/
    /* Open device                       */
    /*-----------------------------------*/
    /* Initialize calls PNIO_device_open() and sets the global device handle: g_hDevice */
    ErrorCode = Initialize();               /*  g_hDevice  */
    if ( (ErrorCode == PNIO_ERR_NO_FW_COMMUNICATION) && (g_pno_test==1) ) {
       /* after the reset of the cp1616 firmware and/or after a "reset to factory defaults"
        * we have to wait until the fw is ready to communicate
        */
        for ( i=0; i<10; i++ ) {
            ch = getCharWithTimeout();
            if ( (ch == (int)'q')||(ch == (int)'Q') )
                break;
            Sleep (1000);
        }
        goto begin;
    }
    if ( (ErrorCode == PNIO_ERR_CONFIG_IN_UPDATE)&&(g_pno_test == 1) && (ch != (int) 'q') && (ch != (int)'Q') ) {
        // wait until the valid ip address is set in the firmware
        Sleep(2000);
        ch = getCharWithTimeout();
        if ( (ch == (int)'q') || (ch == (int) 'Q') )
            goto CLEAN_UP;
        goto begin;
    }

    if ( (ErrorCode != PNIO_OK) || (0 == g_hDevice) ) {
         printf("Error in initializing the device. Error# 0x%x\n", ErrorCode);
        return -1;
    }

    printf("Device initialized successfully. Handle=0x%x\n",g_hDevice);

    /*-----------------------------------*/
    /* Add Profile                       */
    /*-----------------------------------*/

    ErrorCode = AddApi();
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in AddApi(). Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }


    /*-----------------------------------------------*/
    /* Add modules and sub modules to the device     */
    /*-----------------------------------------------*/

    ErrorCode = AddModSubMod();
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in AddModSubMod(). Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }


    /*----------------------------------------------*/
    /* Start the device                             */
    /*----------------------------------------------*/

    printf("\nStep  8 - PNIO_device_start()\n");
    ErrorCode = PNIO_device_start(g_hDevice);
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in starting the device. Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }

    printf("\nStep  9 - PNIO_set_dev_state(PNIO_DEVSTAT_OK)\n");
    ErrorCode = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_OK);
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in setting device state OK. Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }
























    /*------------------------------------------------------------------------*/
    /* Attention : Callbacks are running concurrent in other threads so all   */
    /*             next steps in the main thred must be synchronized.         */
    /*------------------------------------------------------------------------*/

    /* Wait until AR Type is known, so wa can handle appropriately */
    loopCnt = 0;
    printf("*** waiting for AR_INFO_IND_flag (count=%lu) \n",loopCnt );
    while( !AR_INFO_IND_flag ) {
        if ( (loopCnt % 5) == 0) {
            printf("*** waiting for AR_INFO_IND_flag (count=%lu), type 'q' to exit...\r",loopCnt );
        }
        ch = getCharWithTimeout();
        if ((ch == (int) 'q')||(ch == ( int)'Q')) {
            printf("\n\n*** waiting aborted by user...\n");
            goto CLEAN_UP;
        }
        if(stopreq) {
            goto CLEAN_UP;
        }
        Sleep(100);
        loopCnt++;
    }
    printf("main: AR_INFO_IND_flag = %d \n",AR_INFO_IND_flag);

    /*----------------------------------------------*/
    /* Register IRT callbacks, if required          */
    /*----------------------------------------------*/
    if ( g_bIsochron ) {
        printf("main: Register IRT callback: 'Cbf_OPFault_ind' \n");
        ErrorCode = PNIO_CP_register_cbf(g_hDevice, PNIO_CP_CBE_OPFAULT_IND, Cbf_OPFault_ind);
        if ( ErrorCode != PNIO_OK ) {
            printf("\n\tError (0x%08x) while registering OpFault callback function, check if you are logged in as root\n",
                   ErrorCode);
            goto CLEAN_UP;
        }

        printf("main: Register IRT callback: 'Cbf_StartOP_ind' \n");
        ErrorCode = PNIO_CP_register_cbf(g_hDevice, PNIO_CP_CBE_STARTOP_IND, Cbf_StartOP_ind);
        if ( ErrorCode != PNIO_OK ) {
            printf("\n\tError (0x%08x) while registering StartOP callback function, check if you are logged in as root\n",
                   ErrorCode);
            goto CLEAN_UP;
        }
    } /* end Isochron */




   /* AR_ESTABLISH:  we return to this point after PNIO_cbf_ar_offline_ind() was called.
    *                Note: INDATA_IND_flag is reseted in this callback and is set
    *                in PNIO_cbf_ar_indata_ind() again.
    */
    AR_ESTABLISH:

   /* wait until the 'AR establishement' is finished and IOXS are set GOOD,
    * so we can call SET-APPLICATION-READY
    *
    * In case of 'RT' communication the Cbf_StartOP_ind() is not called (is not registered,
    * so we have to set the flag here
    */

    if ( !g_bIsochron ) {
        first_startop_done = 1;
    }
    loopCnt = 0;
    //printf("*** waiting for PRM_END_IND_flag (count=%lu) \n",loopCnt );
    while(!PRM_END_IND_flag || !first_startop_done) {
        if ( (loopCnt % 10) == 0) {
            printf("*** waiting for PRM_END_IND_flag (count=%lu), f1=%d f2=%d, type 'q' to exit... \r",loopCnt,PRM_END_IND_flag,first_startop_done );
        }

        fflush(stdout);
        ch = getCharWithTimeout();
        if(('q' == ch) || ('Q' == ch)) {
            /* abort by user */
            printf("\n\n*** waiting for PRM_END event aborted by user...\n");
            goto CLEAN_UP;
        }
        if(stopreq) {
            goto CLEAN_UP;
        }
        if ( AR_INFO_IND_flag == 0 ) {
            printf("\n*** waiting for  PRM_END aborted --> AR re-establishement...\n");
            Sleep(10);
            goto AR_ESTABLISH;
        }
        //Sleep(500);
        loopCnt++;
    } /* end while */


    /*----------------------------------------------*/
    /* Set Appl.State Ready                         */
    /*----------------------------------------------*/
    /*
     * PNIO_set_appl_state_ready() can be called only after 'do_after_prm_end_ind_cbf()'
     * was called. Herewith device signals readiness for data exchange to the IO controller.
     * Note IOPS must be GOOD immediatelly after this signal. See AP01157093.
     * IOXS are already set in the callback PNIO_cbf_prm_end_ind() for RT and
     * in Cbf_StartOP_ind() for IRT.  i.e. g_bIsochron == true
     */



    memset(&readyListType, 0, sizeof(readyListType));
    readyListType.ap_list.Flink = NULL;
    readyListType.ap_list.Blink = NULL;
    printf("\nStep 17 - PNIO_set_appl_state_ready(arNumber=%d, sessionKey=%d)...\n", g_arNumber, g_SessionKey);
    ErrorCode = PNIO_set_appl_state_ready(g_hDevice,g_arNumber, g_SessionKey, &readyListType);
    if ( ErrorCode == PNIO_OK ) {
        g_DeviceReady = 1;
        printf("\nDevice is ready...\n\n");
    }
    else {
        printf("\nError in setting appl state ready, errorcode=0x%x\n", ErrorCode);
    }

#if 0
    /* Wait Device is "IN-DATA". I.e. Cyclic IO-Communication is established */
    loopCnt = 0;
    printf("*** waiting for INDATA_IND_flag (count=%lu)\n",loopCnt );
    while( !INDATA_IND_flag ) {
        if ( (loopCnt % 5) == 0) {
            printf("*** waiting for INDATA_IND_flag (count=%lu)\r",loopCnt );
        }
        ch = getCharWithTimeout();
        if ((ch == (int) 'q')||(ch == ( int)'Q')) {
            printf("\n\n*** waiting aborted by user...\n");
            goto CLEAN_UP;
        }
        if(stopreq) {
            goto CLEAN_UP;
        }
        if ( AR_INFO_IND_flag == 0 ) {
            printf("\n*** waiting aborted --> AR re-establishement...\n");
            goto AR_ESTABLISH;
        }

        Sleep(900);
        loopCnt++;
    }
#endif /* 0 */

    /*----------------------------------------------*/
    /* work loop (IRT: read/write in callbacks)     */
    /*----------------------------------------------*/

    printf("\nType '1' To send Diagnostic Alarm or 'q' to exit... \n");

    /*loop and wait for the user input*/
    loopCnt = 0;
    do {
        if ( (loopCnt % 5) == 0) {
            printf("*** main loop count=%lu \r",loopCnt );
        }
        if ( g_DeviceReady == 0 ) {
            printf("*** main loop exit --> AR re-stablishement \n\r");
            goto AR_ESTABLISH;
        }
        if ( PRM_END_IND_flag != 0 && g_bIsochron == 0 ) {
            /* RT: update io-data */
            PNIO_initiate_data_write(g_hDevice);
            PNIO_initiate_data_read(g_hDevice);
        }
        ch = getCharWithTimeout();
        if ( stopreq == 1 )
            goto CLEAN_UP;
        if ( ch == (int) '1' ) {
            ErrorCode = SendDiagnosticAlarm(CH_ERR_INVAL_LINKUP, 1, user_handle++);


        }
        Sleep(10);
        loopCnt++;
    }while ( ((int)'q'!=ch) && ((int)'Q'!=ch) );

    /*----------------------------------------------*/
    /* CLEANUP - Device Stop                        */
    /*----------------------------------------------*/
    CLEAN_UP:

    IM_DataToStore("im.conf", im_work, &im_default);

    printf("\nStopping PNIO device...\n\n");

    /* Stop the device */
    ErrorCode = PNIO_device_stop(g_hDevice);
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in stopping the device. Error# 0x%x\n", ErrorCode);
        return -1;
    }

    /*----------------------------------------------*/
    /* Callback synchronization                     */
    /*----------------------------------------------*/

    /* wait for PNIO_cbf_device_stopped callback */
    while ( !DEVICE_STOPPED_flag ) {
        Sleep(100);
    }

    /* Remove the modules and submodules */
    ErrorCode = RemoveModSubMod();
    if ( ErrorCode != PNIO_OK ) {
        return -1;
    }

    /* Remove the API */
    ErrorCode = RemoveApi();
    if ( ErrorCode != PNIO_OK ) {
        return -1;
    }

    /* Uninitialize the device */
    ErrorCode = Uninitialize();
    if ( ErrorCode != PNIO_OK ) {
        printf("Error in uninitializing the device. Error# 0x%x\n", ErrorCode);
        return -1;
    }

    g_hDevice = 0;  /* set hDevice back to an invalid value */

    printf("Device uninitalized successfully\n\n");

    if ( (g_pno_test == 1) && (stopreq == 1) ) {
        printf("\nDevice restart .............\n\n");
        stopreq=0;
        Sleep(1000);
        goto begin;

    }

    return 0;
}

#ifdef PROFI_ENERGY_ENABLE

#define PE_MIN_PAUSE_TIME_IN_MS 3*60*1000


static char* turn_out_time ();
PNIO_UINT8 Service_Req_ID = 0x00;
PNIO_UINT8 host_down            = 0x00;






// PROFIenergy Data structures
#pragma pack(push)
#pragma pack(1)



typedef struct PE_Service_Data_Request_t
{
	union Service_Data_t
	{
		PNIO_UINT32 Data;
		PNIO_UINT32 Reserved;
	} Service_Data;
} PE_Service_Data_Request;
typedef struct PE_Service_Header_Request_t
{
	PNIO_UINT8 Modifier;
	PNIO_UINT8 Data_Structure_Identifier_RQ;
} PE_Service_Header_Request;
typedef struct PE_Request_Header_t
{
	PNIO_UINT8 Service_Req_ID;
	PNIO_UINT8 Request_Ref;
} PE_Request_Header;
typedef struct PE_Preamble_t
{
	PNIO_UINT16 BlockType;
	PNIO_UINT16 BlockLength;
	PNIO_UINT8 BlockVersionHigh;
	PNIO_UINT8 BlockVersionLow;
} PE_Preamble;

typedef struct PE_request_t
{
	PE_Preamble Preamble;
	PE_Request_Header Request_Header;
	PE_Service_Header_Request Service_Header_Request;
	PE_Service_Data_Request Service_Data_Request;
} PE_request;

PE_request* pe_req;

#define PNIO_PE_MAX_SLOTS     0x08
// data record 0x03 for parameter setting
typedef struct {
    PNIO_UINT8          Slot; // always 0x00 for CP16xx
    PNIO_UINT8          Mode; // 0x01 -> selected, 0x00 -> not selected
} PE_slot;
typedef struct PE_param_t
{
	PNIO_UINT8 Version; // Version always 0x01
	PNIO_UINT8 BlockNumber; // # number of elements in the array Slots
	PE_slot Slots[PNIO_PE_MAX_SLOTS];
} PE_param;
PE_param*   pe_param;
typedef struct PE_Service_Header_Response_t
{
	PNIO_UINT8 Status;
	PNIO_UINT8 Data_Structure_Identifier_RS;
} PE_Service_Header_Response;

typedef struct PE_Service_Data_Response_t
{
	PNIO_UINT8 Error_Code;
	PNIO_UINT8 Reserved;
} PE_Service_Data_Response;

typedef struct PE_response_failure_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_Data_Response Service_Data_Response;
} PE_response_failure;

PE_response_failure* pe_fail;
PNIO_UINT8 pe_status_power_save = 0x00;

typedef struct PE_Service_Start_Pause_Data_Response_t
{
	PNIO_UINT8 PE_Mode_ID;
	PNIO_UINT8 Reserved;
} PE_Service_Start_Pause_Data_Response;

// special PE data structures
typedef struct PE_Start_Pause_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_Start_Pause_Data_Response Service_Data_Response;
} PE_Service_Start_Pause_Response;

PE_Service_Start_Pause_Response* pe_start_pause;


void print_start_pause (char* prefix)
	{
    if (prefix == NULL) {
    	prefix = turn_out_time ();
    }
		printf("                       \n\r");
		printf("       START PAUSE     \n\r");
		printf("       received        \n\r");
		printf("       type q          \n\r");
		printf("       and shut down   \n\r");
		printf("       the PC          \n\r");
	}

PNIO_UINT32 convertToBigEndian(PNIO_UINT32 leval)
	{
		PNIO_UINT32 b0 = 0x00000000, b1 = 0x00000000, b2 = 0x00000000, b3 = 0x00000000, beval = 0x00000000;
		b0 = leval & 0x000000FF;
		b1 = (leval >> 0x08) & 0x000000FF;
		b2 = (leval >> 0x10) & 0x000000FF;
		b3 = (leval >> 0x18) & 0x000000FF;
		beval = (b0 << 0x18)|(b1 << 0x10)|(b2 << 0x08)|b3;
		// printf("leval = 0x%x, beval = 0x%x\n", leval, beval);
		return beval;
	}

char* readable_time (long unsigned int time_ms)
	{
		 static char readable_time_line[128];

		 if (time_ms < 1000)
			 sprintf(readable_time_line, "  (0x%lx %lu.)", time_ms, time_ms);
		 else if (1000 <= time_ms&&time_ms < 60000)
			 sprintf(readable_time_line, "%02ld secs %03ld ms (0x%lx %lu.)", time_ms/1000, time_ms%1000, time_ms, time_ms);
		 else
			 sprintf(readable_time_line, "%02ld min %02ld secs %03ld ms  (0x%lx %lu.)",
				 time_ms/60000, (time_ms%60000)/1000, (time_ms%60000)%1000, time_ms, time_ms);
		return readable_time_line;
	}


typedef struct PE_Service_PEM_Status_Data_Response_t
{
	PNIO_UINT8 PE_Mode_ID_Source;
	PNIO_UINT8 PE_Mode_ID_Destination;
	PNIO_UINT32 Time_To_Operate;
	PNIO_UINT32 Remaining_Time_To_Destination;
	float Mode_Power_Consumption;
	float Energy_Consumption_To_Destination;
	float Energy_Consumption_To_Operate;
} PE_Service_PEM_Status_Data_Response;



typedef struct PE_Service_PEM_Status_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_PEM_Status_Data_Response Service_Data_Response;
} PE_Service_PEM_Status_Response;

typedef struct PE_Service_Query_Mode_List_Energy_Saving_Modes_Data_Response_t
{
	PNIO_UINT8 Number_of_PE_Mode_IDs;
	PNIO_UINT8 PE_Mode_ID;
} PE_Service_Query_Mode_List_Energy_Saving_Modes_Data_Response;


typedef struct PE_Service_Query_Mode_List_Energy_Saving_Modes_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_Query_Mode_List_Energy_Saving_Modes_Data_Response Service_Data_Response;
} PE_Service_Query_Mode_List_Energy_Saving_Modes_Response;

typedef struct PE_Service_End_Pause_Data_Response_t
{
	PNIO_UINT32 Time_To_Operate;
} PE_Service_End_Pause_Data_Response;


typedef struct PE_Service_End_Pause_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_End_Pause_Data_Response Service_Data_Response;
} PE_Service_End_Pause_Response;

PE_Service_PEM_Status_Response*  pe_pem_status;
PE_Service_Query_Mode_List_Energy_Saving_Modes_Response* pe_list_energy_saving_modes;
PE_Service_End_Pause_Response*   pe_end_pause;
PNIO_UINT8 Modifier       = 0x03;

typedef struct PE_Service_Query_Mode_Get_Mode_Data_Response_t
{
	PNIO_UINT8 PE_Mode_ID;
	PNIO_UINT8 PE_Mode_Attributes;
	PNIO_UINT32 Time_Min_Pause;
	PNIO_UINT32 Time_To_Pause;
	PNIO_UINT32 Time_To_Operate;
	PNIO_UINT32 Time_Min_Length_of_Stay;
	PNIO_UINT32 Time_Max_Length_of_Stay;
	PNIO_UINT32 Mode_Power_Consumption;
	PNIO_UINT32 Energy_Consumption_To_Pause;
	PNIO_UINT32 Energy_Consumption_To_Operate;
} PE_Service_Query_Mode_Get_Mode_Data_Response;


typedef struct PE_Service_Query_Mode_Get_Mode_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_Query_Mode_Get_Mode_Data_Response Service_Data_Response;
} PE_Service_Query_Mode_Get_Mode_Response;


PE_Service_Query_Mode_Get_Mode_Response* pe_get_mode;

#define MAX_NUM_OF_SUPPORTED_SERVICES 0x05

typedef struct PE_Service_PE_Identify_Data_Response_t
{
	PNIO_UINT8 Count;
	PNIO_UINT8 SupportedServices[MAX_NUM_OF_SUPPORTED_SERVICES];
} PE_Service_PE_Identify_Data_Response;


typedef struct PE_Service_PE_Identify_Response_t
{
	PE_Preamble Preamble;
	PE_Request_Header Response_Header;
	PE_Service_Header_Response Service_Header_Response;
	PE_Service_PE_Identify_Data_Response Service_Data_Response;
} PE_Service_PE_Identify_Response;


#ifndef false
#define false 0
#endif
#ifndef true
#define true  1
#endif



PE_Service_PE_Identify_Response* pe_identify;
volatile int	is_shutdown_routine_internal_active = false;
volatile int	is_wake_up_routine_internal_active  = false;
volatile int	is_start_pause_n_modules_are_pulled = false;

void print_end_pause (char* prefix)
	{
    if (prefix == NULL) {
    	prefix = turn_out_time();
    }
		printf("%s                   \n", prefix);
		printf("%s    END PAUSE      \n", prefix);
		printf("%s                   \n", prefix);
	}
void print_query_modes (char* prefix)
	{
    if (prefix == NULL) {
    	prefix = turn_out_time ();
    }
		printf("%s                  \n", prefix);
		printf("%s   QUERY MODES    \n", prefix);
		printf("%s                  \n", prefix);
	}
static char* turn_out_time ()
  {
    time_t t = time(NULL);

    struct tm* timeStamp;

    static char timeFormat[80];

    timeStamp = localtime (&t);

    sprintf (timeFormat, "%04d.%02d.%02d %02d:%02d:%02d",
      timeStamp->tm_year + 1900,
      timeStamp->tm_mon + 1,
      timeStamp->tm_mday,
      timeStamp->tm_hour,
      timeStamp->tm_min,
      timeStamp->tm_sec);

    return timeFormat;
  }

void PE_print_error (char* prefix)
  {
	  if (prefix == NULL) {
	  	prefix = turn_out_time ();
	  }

    PE_PRINTF("%s             \n", prefix);
    PE_PRINTF("%s  ERROR      \n", prefix);
    PE_PRINTF("%s             \n", prefix);
  }

PNIO_UINT32 PE_Initialize(void)
{
    PNIO_UINT32 uiMaxAR=1; /* maximum application relationships supported. */
    PNIO_UINT32 return_code = PNIO_OK;


    pe_fail = (PE_response_failure*) malloc(sizeof(PE_response_failure));
    if (pe_fail == NULL) {
     	PE_print_error (NULL);
      PE_PRINTF("%s : Initialize> Error No space at (file:%s line %d).\n",
              turn_out_time(), __FILE__, __LINE__);
      return -100;
    }

    pe_identify = (PE_Service_PE_Identify_Response*) malloc(sizeof(PE_Service_PE_Identify_Response));
    if (pe_identify == NULL) {
     	PE_print_error (NULL);
      PE_PRINTF("%s : Initialize> Error No space at (file:%s line %d).\n",
              turn_out_time(), __FILE__, __LINE__);
      return -100;
    }

    pe_pem_status = (PE_Service_PEM_Status_Response*) malloc(sizeof(PE_Service_PEM_Status_Response));
    if (pe_pem_status == NULL)
      return -100;

    pe_get_mode = (PE_Service_Query_Mode_Get_Mode_Response*) malloc(sizeof(PE_Service_Query_Mode_Get_Mode_Response));
    if (pe_get_mode == NULL) {
     	PE_print_error (NULL);
      PE_PRINTF("%s : Initialize> Error No space at (file:%s line %d).\n",
              turn_out_time(), __FILE__, __LINE__);
      return -100;
    }

    pe_list_energy_saving_modes = (PE_Service_Query_Mode_List_Energy_Saving_Modes_Response*) malloc(sizeof(PE_Service_Query_Mode_List_Energy_Saving_Modes_Response));
    if (pe_list_energy_saving_modes == NULL)
      return -100;

    pe_end_pause = (PE_Service_End_Pause_Response*) malloc(sizeof(PE_Service_End_Pause_Response));
    if (pe_end_pause == NULL) {
     	PE_print_error (NULL);
      PE_PRINTF("%s : Initialize> Error No space at (file:%s line %d).\n",
              turn_out_time(), __FILE__, __LINE__);
      return -100;
    }

    pe_start_pause = (PE_Service_Start_Pause_Response*) malloc(sizeof(PE_Service_Start_Pause_Response));
    if (pe_start_pause == NULL) {
     	PE_print_error (NULL);
      PE_PRINTF("%s : Initialize> Error No space at (file:%s line %d).\n",
              turn_out_time(), __FILE__, __LINE__);
      return -100;
    }

	return return_code;
}


/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_rec_write()                                           */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This Callback is called if controller wants to write a record   */
/*                                                                 */
/*-----------------------------------------------------------------*/
void PNIO_cbf_rec_write_pe  (
    PNIO_UINT32          DevHndl,
    PNIO_UINT32          Api,
    PNIO_UINT16          ArNumber,
    PNIO_UINT16          SessionKey,
    PNIO_UINT32          SequenceNum,
    PNIO_DEV_ADDR      * pAddr,          /* geographical address */
    PNIO_UINT32          RecordIndex,
    PNIO_UINT32        * pBufLen,        /* [in, out] in: length to read, out: length, read by user */
    PNIO_UINT8         * pBuffer,        /* [in] buffer pointer */
    PNIO_ERR_STAT      * pPnioState)     /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                ErrCode2), see IEC61158-6 */
{
	unsigned int i = 0;

	pPnioState->ErrCode=0x00;
	pPnioState->ErrDecode=0x00;
	pPnioState->ErrCode1=0x00;
	pPnioState->ErrCode2=0x00;
	pPnioState->AddValue1=0x00;
	pPnioState->AddValue2=0x00;

	PE_PRINTF("%s : //===================================================================================\n", turn_out_time());
	PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK.\n", turn_out_time());


	switch (RecordIndex)	{

		case 0x03:
				return;
		break;

		case 0x80A0:

			pe_req = (PE_request*) pBuffer;

      PE_PRINTF("%s : :: +-------------------------------------------------------------------+\n", turn_out_time());
      PE_PRINTF("%s : :: :   Controller ----- 0x80A0 ------> Device                          :\n", turn_out_time());
      PE_PRINTF("%s : :: +-------------------------------------------------------------------+\n", turn_out_time());

      PE_PRINTF("%s : :: PNIO_cbf_rec_write> Request_Ref=0x%x, Service_Req_ID=0x%x\n", turn_out_time(), pe_req->Request_Header.Request_Ref, pe_req->Request_Header.Service_Req_ID);


			Service_Req_ID = pe_req->Request_Header.Service_Req_ID;


			switch (Service_Req_ID)
			{
				case 0x01: // Start_Pause

					//print_start_pause (NULL);

					PE_PRINTF("%s : :: PNIO_cbf_rec_write> Start_Pause received\n", turn_out_time());

					if ( ( host_down != 0x00) ) {  // if bad parameters or already in power save

						pe_fail->Response_Header.Request_Ref    = pe_req->Request_Header.Request_Ref;
						pe_fail->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
						pe_fail->Service_Header_Response.Data_Structure_Identifier_RS = 0xFF;
						pe_fail->Service_Header_Response.Status = 0x02;
						if (host_down != 0x00)
              pe_fail->Service_Data_Response.Error_Code = 0x01;

						PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
						PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;

					}	else {


            // 2012.08.17 merge result: change
						pe_req->Service_Data_Request.Service_Data.Data = convertToBigEndian(pe_req->Service_Data_Request.Service_Data.Data);

						if (pe_req->Service_Data_Request.Service_Data.Data < PE_MIN_PAUSE_TIME_IN_MS)
						{
							pe_fail->Response_Header.Request_Ref    = pe_req->Request_Header.Request_Ref;
							pe_fail->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
							pe_fail->Service_Header_Response.Data_Structure_Identifier_RS = 0xFF;
							pe_fail->Service_Header_Response.Status   = 0x02;
							pe_fail->Service_Data_Response.Error_Code = 0x01;
							PE_PRINTF("%s : :: PNIO_cbf_rec_write> !!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", turn_out_time());
							PE_PRINTF("%s : :: PNIO_cbf_rec_write> PAUSE TIME %s IS TOO SHORT, MUST BE AT LEAST %s\n",
                turn_out_time(),
                readable_time(pe_req->Service_Data_Request.Service_Data.Data),
                readable_time(PE_MIN_PAUSE_TIME_IN_MS)
              );
							PE_PRINTF("%s : :: PNIO_cbf_rec_write> !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!! <>\n", turn_out_time());
						}	else {
							print_start_pause (NULL);
							pe_status_power_save = 0x01;
							pe_start_pause->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
							pe_start_pause->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
							pe_fail->Response_Header.Service_Req_ID = 0x00;

							if ((pe_status_power_save == 0x01)&&(host_down != 0x01))
							{
								pe_start_pause->Service_Data_Response.PE_Mode_ID = 0x01;
								pe_pem_status->Service_Data_Response.PE_Mode_ID_Source = 0x01;// 0x01 -> power off mode with full PROFINET communication
								pe_pem_status->Service_Data_Response.PE_Mode_ID_Destination = 0xFF; // 0xFF -> power on mode with full PROFINET communication
								host_down = 0x01;
							}
							else
							{
								pe_start_pause->Service_Data_Response.PE_Mode_ID = 0xFF; // 0xFF -> power on mode with full PROFINET communication
								pe_pem_status->Service_Data_Response.PE_Mode_ID_Source = 0xFF; // 0xFF -> power on mode with full PROFINET communication
								pe_pem_status->Service_Data_Response.PE_Mode_ID_Destination = 0x01;// 0x01 -> power off mode with full PROFINET communication
							}
							PE_PRINTF("%s : :: PNIO_cbf_rec_write> PAUSE TIME %s is accepted.\n",
							  turn_out_time(),
							  readable_time(pe_req->Service_Data_Request.Service_Data.Data)
							);
						}
						PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
						PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					}
				  break;

				case 0x02: // End_Pause
				           // =========
					PE_PRINTF("%s : :: PNIO_cbf_rec_write> End_Pause received\n", turn_out_time());
					if ((host_down != 0x01))	{
						pe_fail->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
						pe_fail->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
						pe_fail->Service_Header_Response.Data_Structure_Identifier_RS = 0xFF;
						pe_fail->Service_Header_Response.Status = 0x02;
						if (host_down != 0x01) pe_fail->Service_Data_Response.Error_Code = 0x01;
						return;
					}	else {
						pe_status_power_save = 0x00;
						pe_end_pause->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
						pe_end_pause->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
						pe_fail->Response_Header.Service_Req_ID = 0x00;
						if ((pe_status_power_save == 0x00)&&(host_down != 0x00))	{
							printf("PNIO_cbf_rec_write> bin in wake up if (pe_status_power_save == 0x00)\n");
							pe_pem_status->Service_Data_Response.PE_Mode_ID_Source = 0xFF;
							pe_pem_status->Service_Data_Response.PE_Mode_ID_Destination = 0xFF;
							host_down = 0x00;
						}	else {
							printf("PNIO_cbf_rec_write> bin in wake up if (pe_status_power_save != 0x00)\n");
							pe_pem_status->Service_Data_Response.PE_Mode_ID_Source = 0x01;
							pe_pem_status->Service_Data_Response.PE_Mode_ID_Destination = 0x01;
						}
						PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
						PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					}
			  	break;




				case 0x03: // Query_Modes
				           // ===========
					
						Modifier = pe_req->Service_Header_Request.Modifier;
						switch (Modifier)
						{
							case 0x01: // list energy saving modes
								printf("PNIO_cbf_rec_write> Query_Mode, List mode received\n");
								pe_list_energy_saving_modes->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
								pe_list_energy_saving_modes->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
								pe_list_energy_saving_modes->Service_Header_Response.Status = 0x01;
								pe_list_energy_saving_modes->Service_Header_Response.Data_Structure_Identifier_RS = 0x01;
								pe_list_energy_saving_modes->Service_Data_Response.Number_of_PE_Mode_IDs = 0x01;
								pe_list_energy_saving_modes->Service_Data_Response.PE_Mode_ID = 0x01;
								pe_fail->Response_Header.Service_Req_ID = 0x00;
							break;
							case 0x02: // get mode
								printf("PNIO_cbf_rec_write> Query_Mode, Get mode received\n");
								pe_get_mode->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
								pe_get_mode->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
								pe_get_mode->Service_Header_Response.Status = 0x01;
								pe_get_mode->Service_Header_Response.Data_Structure_Identifier_RS = 0x02;
								pe_get_mode->Service_Data_Response.PE_Mode_ID = 0x01;
								pe_get_mode->Service_Data_Response.PE_Mode_Attributes = 0x00;
								pe_get_mode->Service_Data_Response.Time_Min_Pause = 0x20BF0200;
								pe_get_mode->Service_Data_Response.Time_To_Pause = 0x20BF0200;
								pe_get_mode->Service_Data_Response.Time_To_Operate = 0x20BF0200;
								pe_get_mode->Service_Data_Response.Time_Min_Length_of_Stay = 0x00000000;
								pe_get_mode->Service_Data_Response.Time_Max_Length_of_Stay = 0xFFFFFFFF;
								pe_get_mode->Service_Data_Response.Mode_Power_Consumption = 0x00000000;
								pe_get_mode->Service_Data_Response.Energy_Consumption_To_Pause = 0x00000000;
								pe_get_mode->Service_Data_Response.Energy_Consumption_To_Operate = 0x00000000;
								pe_fail->Response_Header.Service_Req_ID = 0x00;
							break;
							default:// invalid modifier
								pe_fail->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
								pe_fail->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
								pe_fail->Service_Header_Response.Data_Structure_Identifier_RS = 0xFF;
								pe_fail->Service_Header_Response.Status = 0x02;
								pe_fail->Service_Data_Response.Error_Code = 0x03;
							break;
						}
						PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
						PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					
				  break;


				case 0x04: // PEM_Status
					printf("PNIO_cbf_rec_write> PEM_Status received\n");
					{
						pe_pem_status->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
						pe_pem_status->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
						pe_pem_status->Service_Header_Response.Status = 0x01;
						pe_pem_status->Service_Header_Response.Data_Structure_Identifier_RS = 0x01;

						if (pe_pem_status->Service_Data_Response.PE_Mode_ID_Source
								< pe_pem_status->Service_Data_Response.PE_Mode_ID_Destination)
						{
							pe_pem_status->Service_Data_Response.Remaining_Time_To_Destination = 0x20BF0200;
							pe_pem_status->Service_Data_Response.Time_To_Operate = 0x20BF0200;
						}	else	{
							pe_pem_status->Service_Data_Response.Remaining_Time_To_Destination = 0x00000000;
							pe_pem_status->Service_Data_Response.Time_To_Operate = 0x00000000;
						}

						pe_pem_status->Service_Data_Response.Mode_Power_Consumption = 0.00f;
						pe_pem_status->Service_Data_Response.Energy_Consumption_To_Destination = 0.00f;
						pe_pem_status->Service_Data_Response.Energy_Consumption_To_Operate = 0.00f;
						pe_fail->Response_Header.Service_Req_ID = 0x00;
	          PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
	          PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					}
			  	break;


				case 0x05: // PE_Identify
					printf("PNIO_cbf_rec_write> PE_Identify received\n");
					{
						pe_identify->Response_Header.Request_Ref = pe_req->Request_Header.Request_Ref;
						pe_identify->Response_Header.Service_Req_ID = pe_req->Request_Header.Service_Req_ID;
						pe_identify->Service_Header_Response.Status = 0x01;
						pe_identify->Service_Header_Response.Data_Structure_Identifier_RS = 0x01;
						pe_identify->Service_Data_Response.Count = 0x05;
						pe_identify->Service_Data_Response.SupportedServices[0x00] = 0x01;
						pe_identify->Service_Data_Response.SupportedServices[0x01] = 0x02;
						pe_identify->Service_Data_Response.SupportedServices[0x02] = 0x03;
						pe_identify->Service_Data_Response.SupportedServices[0x03] = 0x04;
						pe_identify->Service_Data_Response.SupportedServices[0x04] = 0x05;
						pe_fail->Response_Header.Service_Req_ID = 0x00;
						PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
						PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					}
				break;
			}
			PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
			PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
			return;
			break;

			default:
				PE_PRINTF("%s : :: PNIO_cbf_rec_write> Datensatz read for Record 0x%x and Slot 0x%x and Subslot 0x%x called\n", turn_out_time(),
						                               RecordIndex, pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);
			break;
	}

	PE_PRINTF("%s : :: Datarecord 0x%x write is not supported, Buffer to write is 0x%x\n", turn_out_time(), RecordIndex, *pBufLen);
	*pBufLen=0x00;
	pPnioState->ErrCode=0xdf;
	pPnioState->ErrDecode=0x80;
	pPnioState->ErrCode1=0xb0;
	pPnioState->ErrCode2=0x0a;
	pPnioState->AddValue1=0x00;
	pPnioState->AddValue2=0x00;
	PE_PRINTF("%s : :: PNIO_cbf_rec_write> CALLBACK END.\n", turn_out_time());
	PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
}



void PNIO_cbf_rec_read_pe  (
    PNIO_UINT32          DevHndl,
    PNIO_UINT32          Api,
    PNIO_UINT16          ArNumber,
    PNIO_UINT16          SessionKey,
    PNIO_UINT32          SequenceNum,
    PNIO_DEV_ADDR      * pAddr,          /* geographical address */
    PNIO_UINT32          RecordIndex,
    PNIO_UINT32        * pBufLen,        /* [in, out] in: length to read, out: length, read by user */
    PNIO_UINT8         * pBuffer,        /* [out] buffer pointer */
    PNIO_ERR_STAT      * pPnioState)     /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                                   ErrCode2), see IEC61158-6 */
{
    PE_PRINTF("%s : //===================================================================================\n", turn_out_time());
	  PE_PRINTF("%s : :: PNIO_cbf_rec_read> CALLBACK.\n", turn_out_time());
	pPnioState->ErrCode=0x00;
	pPnioState->ErrDecode=0x00;
	pPnioState->ErrCode1=0x00;
	pPnioState->ErrCode2=0x00;
	pPnioState->AddValue1=0x00;
	pPnioState->AddValue2=0x00;

  PE_PRINTF("%s : :: PNIO_cbf_rec_read> RecordIndex=0x%x (%d.)\n", turn_out_time(), RecordIndex, RecordIndex);


	switch (RecordIndex)
	{
    case 0x80A0:
   	  if (Service_Req_ID == pe_fail->Response_Header.Service_Req_ID)
   		{
   			*pBufLen = sizeof(PE_response_failure);
   			memcpy(pBuffer, pe_fail, *pBufLen);
				PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
				PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
   			return;
   		}
   		else
   		{
         PE_PRINTF("%s : :: PNIO_cbf_rec_read> Service_Req_ID=0x%x (%d.)\n", turn_out_time(), Service_Req_ID, Service_Req_ID);
   			 switch(Service_Req_ID)
   			   {
				case 0x01: // Start Pause
					printf("PNIO_cbf_rec_read> Start Pause Rec Received\n");
					*pBufLen = sizeof(PE_Service_Start_Pause_Response);
					memcpy(pBuffer, pe_start_pause, *pBufLen);
					is_shutdown_routine_internal_active = 0x01;
	        PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	        PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
					return;
				break;
				case 0x02: // End_Pause
					print_end_pause (NULL);
	        PE_PRINTF("%s : :: PNIO_cbf_rec_read > End_Pause_Response.\n", turn_out_time());
					*pBufLen = sizeof(PE_Service_End_Pause_Response);
					memcpy(pBuffer, pe_end_pause, *pBufLen);
	        PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	        PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
					is_wake_up_routine_internal_active = 0x01;
					Sleep(1000);
					return;
				break;

				case 0x03: // Query_Modes
					print_query_modes (NULL);
					switch (Modifier)
					{
					case 0x01:
						*pBufLen = sizeof(PE_Service_Query_Mode_List_Energy_Saving_Modes_Response);
			      memcpy(pBuffer, pe_list_energy_saving_modes, *pBufLen);
	          PE_PRINTF("%s : :: PNIO_cbf_rec_read > Modifier:0x%x CALLBACK END.\n", turn_out_time(), Modifier);
	          PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					break;
					case 0x02:
						*pBufLen = sizeof(PE_Service_Query_Mode_Get_Mode_Response);
						memcpy(pBuffer, pe_get_mode, *pBufLen);
	          PE_PRINTF("%s : :: PNIO_cbf_rec_read > Modifier:0x%x CALLBACK END.\n", turn_out_time(), Modifier);
	          PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					break;
					default:// invalid modifier
						*pBufLen = sizeof(PE_response_failure);
						memcpy(pBuffer, pe_fail, *pBufLen);
	          PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	          PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
						return;
					break;
					}
				break;
				case 0x04: // PEM_Status
					*pBufLen = sizeof(PE_Service_PEM_Status_Response);
					memcpy(pBuffer, pe_pem_status, *pBufLen);
	        PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	        PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
					return;
				break;
				case 0x05: // PE_Identify
					*pBufLen = sizeof(PE_Service_PE_Identify_Response);
					memcpy(pBuffer, pe_identify, *pBufLen);
	        PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	        PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
					return;
				  break;
   		}
   		   }
   		break;
		default:
			printf("PNIO_cbf_rec_read> Datarecord 0x%x is not supported\n", RecordIndex);
		break;
	}

	PE_print_error (NULL);
	printf("PNIO_cbf_rec_read> Datarecord 0x%x is not supported\n", RecordIndex);
   *pBufLen=0x00;
	pPnioState->ErrCode=0xde;
	pPnioState->ErrDecode=0x80;
	pPnioState->ErrCode1=0xb0;
	pPnioState->ErrCode2=0x0a;
	pPnioState->AddValue1=0x00;
	pPnioState->AddValue2=0x00;
	  PE_PRINTF("%s : :: PNIO_cbf_rec_read > CALLBACK END.\n", turn_out_time());
	  PE_PRINTF("%s : \\\\===================================================================================\n", turn_out_time());
}





#endif
