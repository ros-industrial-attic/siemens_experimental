/*------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*------------------------------------------------------------------------*/
/*                                                                        */
/*   Project           :                                                  */
/*   Filename          : easy_dev_irt1.c                                  */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
/* Description: This sample program shows how to do the following         */
/*              operations with PNIO device                               */
/*              1. Initialize and Uninitialize                            */
/*              2. Reading / Writing IRT IO data                          */
/*                                                                        */
/*------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all   */
/*             printf statements should be synchronized. But this sample  */
/*             application doesn't synchronize for simplicity  !          */
/*------------------------------------------------------------------------*/
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
#else
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <termios.h>
#include <string.h>
#define Sleep(x) usleep(x*1000)
#endif

#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

#include "easy_dev_cfg.h"

/*----------------------------------------------------------------------------------------------------*/
/*    FUNCTION PROTOTYPES                                                                             */
/*----------------------------------------------------------------------------------------------------*/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub);

/*----------------------------------------------------------------------------------------------------*/
/*                                          DEFINES                                                   */
/*----------------------------------------------------------------------------------------------------*/
#define PNIO_HANDLE     PNIO_UINT32           /* Declaring specific type as handle                    */

#define NUMOF_SLOTS              10           /* slot 0...8   +  1                                    */
#define NUMOF_SUBSLOTS            2           /* Every slot has 1 subslots                            */
#define NUMOF_BYTES_PER_SUBSLOT 256           /* Maximum data length as configured in the sample      */
                                              /* Step7 project                                        */
#define MAX_DUMP_DATA            16           /* Maximum data bytes to dump per subslot               */

/*----------------------------------------------------------------------------------------------------*/
/*                                          GLOBALS                                                   */
/*----------------------------------------------------------------------------------------------------*/

PNIO_HANDLE g_hDevice       = 0; /* global handle for device, 0 means invalid handle */
PNIO_UINT16 g_SessionKey    = 0; /* session identifier, this is obtained in application-relation
                                    information callback i.e. PNIO_cbf_ar_info_ind.  */
PNIO_UINT16 g_arNumber      = 0; /* application relation number */

/* Global variables to count callback events */
volatile PNIO_UINT32  g_OpFaultCount      = 0;
volatile PNIO_UINT32  g_StartOpCount      = 0;
volatile PNIO_UINT32  g_DataExchangeCount = 0;

/* Global variables to count errors */
volatile PNIO_UINT32  g_readErrors        = 0;
volatile PNIO_UINT32  g_writeErrors       = 0;
volatile PNIO_UINT32  g_badRemoteState    = 0;

/* Global variables to control the behavoir of the DataExchange function */
volatile int          g_data              = 0;
volatile int          g_set               = 1;
volatile int          g_mirror            = 0;
volatile int          g_increment         = 0;
volatile int          g_DataExchange      = 0;
volatile int          g_device_stopped    = 0;
volatile int          g_device_stop_req   = 0;

volatile int first_startop_done = 0;
volatile int ar_info_done       = 0;
volatile int prm_end_done       = 0;

PNIO_UINT32 g_IRTReadCount  = 0;
PNIO_UINT32 g_IRTWriteCount = 0;

/* Global variables for indices of special submodules */
int g_set_index       = 0;
int g_mirror_index    = 0;
int g_increment_index = 0;

/*
The structure described below contains all the module/submodule information. For simplicity, this structure
is hard-coded in the configuration file, in the actual program it can be built via the '.ini' file.
*/
static device_data_t device_data[] =
{
    DEVICE_DATA
};

device_data_t *g_device_data = NULL;

/* Total no of slots as configured in the sample Step7 project */
static const int gDevArraySize = sizeof(device_data) / sizeof(device_data_t);

int idxTbl[DEVICE_DATA_ENTRIES];       /* an array of slot ids, sub-slot entries will contain -1 */


/**** Output Data  (IO Controller ==> IO Device) */
PNIO_UINT8     OutData    [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    OutDatLen  [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     OutDatIocs [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     OutDatIops [NUMOF_SLOTS][NUMOF_SUBSLOTS];

/**** Input Data  (IO Device ==> IO Controller) */
PNIO_UINT8     InData     [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    InDatLen   [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     InDatIops  [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     InDatIocs  [NUMOF_SLOTS][NUMOF_SUBSLOTS];

/*------------------------------------------------------------------------*/
/* forward declaration of helper functions                                */
/*------------------------------------------------------------------------*/

int getCharWithTimeout(void);

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
/*  and handles them to the stack.                                               */
/*                                                                               */
/*  This function will be called within the IRT cycle and is time critical.      */
/*  Do not perform time consuming operations like output to console or file.     */
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

    /*
        printf("## PNIO_cbf_data_write(..., len=%u, Iocs=%x) for devHandle 0x%x, slot %u, subslot %u\n",
           BufLen, Iocs, DevHndl, slot_num, subslot_num);
    */

    /* Copy the application data to the stack */
    if(BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT) {
        memcpy(pBuffer, &InData[slot_num][subslot_num][0], BufLen);
    } else {
        /* this should not happen */
        printf("\n\t!!! PNIO_cbf_data_write: Buflen=%u > allowed size (%u)!!!\n",
            BufLen, NUMOF_BYTES_PER_SUBSLOT);
    }

    InDatLen [slot_num][subslot_num] = BufLen;  /* save data length (only for debugging)     */
    InDatIocs[slot_num][subslot_num] = Iocs;    /* consumer status (of remote io controller) */

    if(Iocs == PNIO_S_BAD && slot_num)
        ++g_badRemoteState;

    ++g_IRTWriteCount;                          /* increase the write count    */

    return (InDatIops[slot_num][subslot_num]);  /* return local provider state */
}

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_data_read (...)                                                     */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  Passes the output data from the stack to the application.                    */
/*  The application takes the data and writes them to the specified              */
/*  output module.                                                               */
/*                                                                               */
/*  This function will be called within the IRT cycle and is time critical.      */
/*  Do not perform time consuming operations like output to console or file.     */
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

    /*
    printf("## PNIO_cbf_data_read(..., len=%u, Iops=%x) for devHandle 0x%x, slot %u, subslot %u\n",
           BufLen, Iops, DevHndl, slot_num, subslot_num);
    */

    /* Copy the data from the stack to the application buffer */
    if(BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT) {
        memcpy(&OutData[slot_num][subslot_num][0], pBuffer, BufLen);
    } else {
        /* this should not happen */
        printf("\n\t!!! PNIO_cbf_data_read: Buflen=%u > allowed size (%u)!!!\n",
            BufLen, NUMOF_BYTES_PER_SUBSLOT);
    }

    OutDatLen [slot_num][subslot_num] = BufLen; /* save data length (only for debugging)     */
    OutDatIops[slot_num][subslot_num] = Iops;   /* provider status (of remote io controller) */

    if(Iops == PNIO_S_BAD && slot_num)
        ++g_badRemoteState;

    ++g_IRTReadCount;                           /* increase the read count */

    return (OutDatIocs[slot_num][subslot_num]); /* consumer state (of local io device) */
}

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_rec_write (...)                                                     */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*  This callback is called to notify that a write record request                */
/*  has been received from the pnio controller. The user has to                  */
/*  read the record data from the specified source buffer.                       */
/*  The length of the provided data are specified in function parameter          */
/*  *pBufLen. The user has to change this pointer, if the size of                */
/*  the accepted data differs from the size of the provided data.                */
/*  After serving this function, the user returns the success state.             */
/*-------------------------------------------------------------------------------*/
void  PNIO_cbf_rec_write(
    PNIO_UINT32    DevHndl,
    PNIO_UINT32    Api,
    PNIO_UINT16    ArNumber,
    PNIO_UINT16    SessionKey,
    PNIO_UINT32    SequenceNum,
    PNIO_DEV_ADDR* pAddr,       /* geographical or logical address */
    PNIO_UINT32    RecordIndex,
    PNIO_UINT32*   pBufLen,     /* [in, out] in: length to write, out: length, written by user */
    PNIO_UINT8*    pBuf,        /* [in] buffer pointer */
    PNIO_ERR_STAT* pPnioState)  /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                ErrCode2), see IEC61158-6 */
{
    /***** copy dummy data into the buffer, set data-size ****/
    PNIO_UINT8    WriteRecDummyData[50];
    PNIO_UINT32   i;
    PNIO_UINT32   Status = PNIO_OK;

    /**** check data size (accepted data >= provided data ?? */
    if(*pBufLen > sizeof(WriteRecDummyData)) {
        *pBufLen = sizeof(WriteRecDummyData);
    }

    /**** copy the record data into a buffer for further use ***/
    memcpy(WriteRecDummyData,  /* destination pointer for record data */
        pBuf,                  /* source pointer for record data      */
        *pBufLen);             /* length of the accepted data         */

    printf("\n## WRITE_RECORD Request, Api=%u Slot=%u Subslot=%u Index=%u, Length=%u, Sequence_nr=%u\n\n",
        Api, pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot,
        RecordIndex, *pBufLen, SequenceNum);

    printf("## RECORD_DATA =");
    for(i = 0; i < *pBufLen; ++i) {
        if(i && (i % 16 == 0))
            printf("\n                ");
        printf(" 0x%02x", WriteRecDummyData[i]);
    }
    printf("\n");

    if(Status == PNIO_OK) {
        memset(pPnioState, 0, sizeof(*pPnioState));
        return;
    } else {
        /**** if an error occured, you must specify it according IEC 61158-6 */
        pPnioState->ErrCode   = 0xdf;  /* IODWriteRes with ErrorDecode = PNIORW */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                                */
        pPnioState->ErrCode1  = 0xa9;  /* example: Error Class 10 = application, ErrorNr 9 = "feature not supported" */
        pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
        pPnioState->AddValue1 = 0;     /* here dont care                                                             */
        pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        return;
    }
}

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_rec_read (...)                                                      */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/* This callback is called to notify that a read record request                  */
/* has been received from the pnio controller. This function has to              */
/* provide the record data and copies them to the specified buffer               */
/* address. The maximum pBufLen is also provided in the function                 */
/* parameters and can not be exceeded !.                                         */
/* After serving this function, this function returns the real copied data       */
/* length and the success state (PNIO_OK),                                       */
/*-------------------------------------------------------------------------------*/
void  PNIO_cbf_rec_read(
    PNIO_UINT32    DevHndl,
    PNIO_UINT32    Api,
    PNIO_UINT16    ArNumber,
    PNIO_UINT16    SessionKey,
    PNIO_UINT32    SequenceNum,
    PNIO_DEV_ADDR* pAddr,        /* geographical or logical address                         */
    PNIO_UINT32    RecordIndex,
    PNIO_UINT32*   pBufLen,      /* [in, out] in: length to read, out: length, read by user */
    PNIO_UINT8*    pBuf,         /* [in] buffer pointer                                     */
    PNIO_ERR_STAT* pPnioState)   /* 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1,
                                    ErrCode2), see IEC61158-6                               */
{
    PNIO_UINT32   Status = PNIO_OK;

    /***** copy dummy data into the buffer, set data-size ****/
    PNIO_UINT8    ReadRecDummyData[] = {"**Data1234 ReadRecord**"};

    if(*pBufLen > sizeof(ReadRecDummyData)) {
        *pBufLen = sizeof(ReadRecDummyData);
    }

    /*----------------------------------------------*/
    /*  copy the data to the specified buffer       */
    /*----------------------------------------------*/
    memcpy(pBuf, ReadRecDummyData, *pBufLen);

    if(Status == PNIO_OK) {
        memset(pPnioState, 0, sizeof(*pPnioState));
        return;
    } else {
        /**** if an error occured, it must be specified according IEC 61158-6    */
        pPnioState->ErrCode   = 0xde;  /* IODReadRes with ErrorDecode = PNIORW   */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                                 */
        pPnioState->ErrCode1  = 0xa9;  /* example: Error Class 10 = application, */
                                       /* ErrorNr 9 = "feature not supported"    */
        pPnioState->ErrCode2  = 0;     /* here dont care                         */
        pPnioState->AddValue1 = 0;     /* here dont care                         */
        pPnioState->AddValue2 = 0;     /* here dont care                         */
        return;
    }
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_check_ind (...)                                                            */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*  This call back is called for each and every sub-module of the IO Base Device        */
/*  interface -where the configuration does not match with that of the IO controller.   */
/*  The user program is consequently given the option of changing the sub-module        */
/*  layout or marking the sub-module as compatible or incorrect.                        */
/*  In case of agreement of the configurations, this call back is not called.           */
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

    printf ("## CHECK_IND slot=%u, subslot=%u, ModIdent=%u, State (%u), SubIdent=%u, State (%u)\n",
        pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, *pModIdent, *pModState, *pSubIdent, *pSubState);

    /* get the index in our configuration */
    idx = GetSubmodNum(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);

    /*
    Check the configuration sent by controller against the configuration_data structure.
    If there is any mismatch, return error.
    */
    if((idx != -1) && (g_device_data[idx].subslot == pAddr->u.Geo.Subslot) &&
        (g_device_data[idx].modId == *pModIdent) && (g_device_data[idx].subslotId == *pSubIdent)) {
        *pModIdent = g_device_data[idx].modId;
        *pSubIdent = g_device_data[idx].subslotId;
        *pModState = PNIO_MOD_STATE_PROPER_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_OK;
    } else {
        printf ("## the configuration of plugged modules is inconsistent to HWCONFIG\n");
        printf ("## please check your configuration first!\n");
        *pModState = PNIO_MOD_STATE_WRONG_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_WRONG;
    }
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_check_ind (...)                                                         */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*  This callback is called by the IO Base Device interface as soon an IO               */
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
    printf("\n## PNIO_cbf_ar_check_ind (Station %s, IP %d.%d.%d.%d)\n\n",
           stname,lc.c[0], lc.c[1], lc.c[2], lc.c[3]);
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
    int i;
    int j;

    g_arNumber   = ArNumber;      /* Store the AR number */
    g_SessionKey = SessionKey;    /* Store the session key */

    printf("\n## AR-INFO_IND new AR from PNIO controller established, SessionKey=0x%x\n\n", SessionKey);

    /*-------------------------------------------------------------------*/
    /*   set all io data in submodules to inital value = 0               */
    /*-------------------------------------------------------------------*/
    memset(&InData,     0, sizeof(InData));     /* io data (input)        */
    memset(&InDatLen,   0, sizeof(InDatLen));   /* length of input data   */
    memset(&InDatIops,  0, sizeof(InDatIops));  /* local provider status  */
    memset(&InDatIocs,  0, sizeof(InDatIocs));  /* remote consumer status */
    memset(&OutData,    0, sizeof(OutData));    /* io data (output)       */
    memset(&OutDatLen,  0, sizeof(OutDatLen));  /* length of output data  */
    memset(&OutDatIocs, 0, sizeof(OutDatIocs)); /* local consumer status  */
    memset(&OutDatIops, 0, sizeof(OutDatIops)); /* remote provider status */

    /*----------------------------------------------------------------------*/
    /* set local provider status preset values for all input/output slots   */
    /*----------------------------------------------------------------------*/
    for(i = 0; i < gDevArraySize; ++i) {
        for(j = 0; j < 1 /* g_device_data[i].maxSubslots */; ++j) {
            /*** set local provider state = GOOD for input data***/
            if(i == 0) {
                if(g_device_data[i].modState == 1) { /* plugged */
                    InDatIops[i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops[i][j] = PNIO_S_BAD;
                }
            } else {
                if(g_device_data[i].modState == 1 &&
                    g_device_data[i + j].subState == 1) { /* plugged */
                    InDatIops[i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops[i][j] = PNIO_S_BAD;
                }
            }
        }
    }

    /*---------------------------------------------------------------*/
    /* set local consumer status for all output slots                */
    /*---------------------------------------------------------------*/
    for(i = 0; i < gDevArraySize; ++i) {
        for(j = 0; j < g_device_data[0].maxSubslots; ++j) {
            if(i == 0) {
                if(g_device_data[i].modState == 1) { /* plugged */
                    OutDatIocs[i][j] = PNIO_S_GOOD;
                } else {
                    OutDatIocs[i][j] = PNIO_S_BAD;
                }
            } else {
                if(g_device_data[i].modState == 1 &&
                    g_device_data[i + j].subState == 1) { /* plugged */
                    OutDatIocs[i][j] = PNIO_S_GOOD;
                } else {
                    OutDatIocs[i][j] = PNIO_S_BAD;
                }
            }
        }
    }

    /*---------------------------------------------------------------*/
    /* signal startop handler that module and submodule states are   */
    /* initialized. Now it is time to call                           */
    /* PNIO_initiate_data_read_ext() and                             */
    /* PNIO_initiate_data_write_ext() for the first time.            */
    /*---------------------------------------------------------------*/
    ar_info_done = 1;
 }

/*-------------------------------------------------------------------------------*/
/*                                                                               */
/*  PNIO_cbf_ar_indata_ind (...)                                                 */
/*                                                                               */
/*-------------------------------------------------------------------------------*/
/*  This call back is called by the IO Base Device interface as soon as an       */
/*  IO controller has transmitted the IO data for the first time. Signalling     */
/*  the beginning of cyclical data inter-change.                                 */
/*-------------------------------------------------------------------------------*/
void PNIO_cbf_ar_indata_ind(
    PNIO_UINT32     DevHndl,        /* [in] Handle for Multidevice */
    PNIO_UINT16     ArNumber,       /* [in] Application-relation number */
    PNIO_UINT16     SessionKey)     /* [in] session key */
{
    printf("\n## AR IN-Data event indication has been received, ArNumber=0x%x\n\n", ArNumber);

    /* start data exchange in startop callback */
    g_DataExchange = 1;
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_abort_ind (...)                                                         */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface as soon as the          */
/*      connection is terminated after a data exchange with the IO Controller began     */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_abort_ind(
    PNIO_UINT32     DevHndl,     /* [in] Handle for Multidevice */
    PNIO_UINT16     ArNumber,    /* [in] Application-relation number */
    PNIO_UINT16     SessionKey,  /* [in] session key */
    PNIO_AR_REASON  ReasonCode)  /* [in] reason code */
{
    /* AR abort after ArInData-indication */
    printf ("\n## AR ABORT indication, ArNumber = 0x%x, Reason = 0x%x\n\n",
        ArNumber, ReasonCode);
}

/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*  PNIO_cbf_ar_offline_ind (...)                                                       */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface as soon as the          */
/*      connection is terminated before a data exchange with the IO Controller began    */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/
void PNIO_cbf_ar_offline_ind(
    PNIO_UINT32     DevHndl,       /* [in] Handle for Multidevice */
    PNIO_UINT16     ArNumber,      /* [in] Application-relation number */
    PNIO_UINT16     SessionKey,    /* [in] session key */
    PNIO_AR_REASON  ReasonCode)    /* [in] reason code */
{
    printf ("\n## AR Offline indication, ArNumber = 0x%x, Reason = 0x%x\n\n",
        ArNumber, ReasonCode);
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
    printf("\n\n## Event parameterised phase -Application ready- received, ArNumber=0x%x\n", ArNumber);

    prm_end_done = 1;
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_cp_stop_req ()                                        */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*  Input:     ---                                                 */
/*  Output:    ---                                                 */
/*-----------------------------------------------------------------*/
void PNIO_cbf_cp_stop_req(PNIO_UINT32 DevHndl)
{
    printf("\n## PNIO_cbf_cp_stop_req\n");
    printf("press 'q' to quit...\n\n");

    g_device_stop_req = 1;
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_device_stopped ()                                     */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*  This callback is called by the IO Base Device interface after  */
/*  the device stop request is received.                           */
/*-----------------------------------------------------------------*/
void PNIO_cbf_device_stopped(
    PNIO_UINT32 DevHndl,  /* [in] Handle for Multidevice */
    PNIO_UINT32 Reserved) /* Reserved for future use     */
{
    printf("\n## PNIO_cbf_device_stopped\n\n");

    /*
    Automatic synchronization - The main function calls device-stop function
    and then the main function waits for g_device_stopped set to 1
    in this callback to achieve synchronization.
    */
    g_device_stopped = 1;
}

/*-----------------------------------------------------------------*/
/* NOT USED CALLBACKS                                              */
/*-----------------------------------------------------------------*/

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_req_done()                                            */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This Callback is called if firmware wants Host to stop          */
/* communication                                                   */
/*-----------------------------------------------------------------*/
void PNIO_cbf_req_done(
    PNIO_UINT32    DevHndl,    /* Handle for Multidevice */
    PNIO_UINT32    UserHndl,   /* user defined handle    */
    PNIO_UINT32    Status,
    PNIO_ERR_STAT* pPnioState)
{
    printf("\n## req_done not supported\n\n");
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_apdu_status_ind()                                     */
/*                                                                 */
/*-----------------------------------------------------------------*/
/* This Callback is called if firmware wants Host to stop          */
/* communication                                                   */
/*-----------------------------------------------------------------*/
void PNIO_cbf_apdu_status_ind(
    PNIO_UINT32          DevHndl,
    PNIO_UINT16          ArNumber,
    PNIO_UINT16          SessionKey,
    PNIO_APDU_STATUS_IND ApduStatus)
{
    printf("\n## APDU Status Change not supported\n\n");
}

/*******************************************************************/
/* DataExchange                                                    */
/*******************************************************************/
/* This function is called in the StartOP handler                  */
/*******************************************************************/
void DataExchange(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 ErrorCode;
    int SlotNum;
    int SubNum;

    ++g_DataExchangeCount;

    /* Read data from controller */
    ErrorCode = PNIO_initiate_data_read_ext(DevHndl, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
    if(ErrorCode != PNIO_OK)
        ++g_readErrors;

    /* calculate output */
    if(g_set) {
        SlotNum = g_device_data[g_set_index].slot;
        SubNum  = g_device_data[g_set_index].subslot;
        memset((void*)&InData[SlotNum][SubNum][0],
            g_data,
            InDatLen[SlotNum][SubNum]);
        g_set = 0;
    }

    if(g_increment) {
        SlotNum = g_device_data[g_increment_index].slot;
        SubNum  = g_device_data[g_increment_index].subslot;
        if(InDatLen[SlotNum][SubNum] == 4)
            ++*(PNIO_UINT32*)&InData[SlotNum][SubNum][0];
    }

    if(g_mirror) {
        SlotNum = g_device_data[g_mirror_index].slot;
        SubNum  = g_device_data[g_mirror_index].subslot;
        if(OutDatLen[SlotNum][SubNum] == InDatLen[SlotNum][SubNum])
            memcpy((void*)&InData[SlotNum][SubNum][0],
                (void*)&OutData[SlotNum][SubNum][0],
                OutDatLen[SlotNum][SubNum]);
    }

    /* write data to controller */
    ErrorCode = PNIO_initiate_data_write_ext(DevHndl, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);
    if(ErrorCode != PNIO_OK)
        ++g_writeErrors;
}

/*-----------------------------------------------------------------*/
/*  IRT Start OP                                                   */
/*-----------------------------------------------------------------*/
/* This callback is called at the start of IRT cycle               */
/*-----------------------------------------------------------------*/
void Cbf_StartOP_ind(PNIO_CP_CBE_PRM *prm)
{
    ++g_StartOpCount;

    if(!first_startop_done && ar_info_done) {
        PNIO_UINT32 ErrorCode;

        /* Read data from controller */
        ErrorCode = PNIO_initiate_data_read_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);

        /* write data to controller */
        ErrorCode = PNIO_initiate_data_write_ext(prm->u.StartOp.AppHandle, NULL, PNIO_ACCESS_IRT_WITHOUT_LOCK);

        /* signal that data processing is done for this cycle */
        PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);

        first_startop_done = 1;
    }
    else {
        if(g_DataExchange)
            /* process data */
            DataExchange(prm->u.StartOp.AppHandle);

        /* signal that data processing is done for this cycle */
        PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);
    }
}

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
/*                                          FUNCTIONS                                                 */
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------*/
/*                                                                */
/*  GetSubmodNum                                                  */
/*                                                                */
/*----------------------------------------------------------------*/
/*  This function return the index into the IO data array         */
/*  for the given submodule.                                      */
/*----------------------------------------------------------------*/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub)
{
    int entries = gDevArraySize;
    int i, j;

    for(i = 0; i < entries; ++i) {
        if((int)mod == idxTbl[i]) {
            for(j = 0; j < g_device_data[i].maxSubslots; ++j) {
                if(g_device_data[i + j].subslot == (int)sub) {
                    return i + j;
                }
            }
        }
    }

    return -1;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :       AddModSubMod                                 */
/*                                                                */
/*----------------------------------------------------------------*/
/*  This function adds all the modules and submodules of the      */
/*      device in serial order.                                   */
/*----------------------------------------------------------------*/
PNIO_UINT32 AddModSubMod(void)
{
    PNIO_UINT32   status = PNIO_OK;
    PNIO_DEV_ADDR addr;  /* location (module/submodule) */
    int           slot = 0;
    int           entries = gDevArraySize;
    int           i;
    int           state;

    /*-----------------------------------------*/
    /*  add all modules first                  */
    /*-----------------------------------------*/

    addr.AddrType = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */

    /*
    in the for-loop below, i is not incremented contineously.
    Instead, 'i' only goes through those entries of g_device_data array, which are a
    slot, and it jumps over the sub-slots, because in this loop we are adding ONLY modules
    */
    for(i = 0; i < entries;) {
        addr.u.Geo.Slot    = g_device_data[i].slot;    /* plug module at correct slot    */
        addr.u.Geo.Subslot = g_device_data[i].subslot; /* get the corresponding sub-slot */

        status = PNIO_mod_plug(
            g_hDevice,               /* device handle            */
            g_device_data[i].api,    /* api number               */
            &addr,                   /* location (slot, subslot) */
            g_device_data[i].modId); /* submodule 0  identifier  */

        if(status == PNIO_OK) {
            printf("Module plug\t");
            state = 1;
        } else {
            printf("Module plug failed");
            state = 0;
        }

        printf("\t: api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
            g_device_data[i].api,
            g_device_data[i].slot,
            g_device_data[i].subslot,
            g_device_data[i].maxSubslots,
            g_device_data[i].modId);

        /*
        update the device data structure, with the state of module
        plugged or unplugged
        */

        g_device_data[i].modState = state;
        if(status == PNIO_OK) {
            /*
            advance in the g_device_data structure, jumping over all the sub-module entries
            to reach the next module entry in the structure
            */
            i += g_device_data[i].maxSubslots;
        } else {
            /*
            go to the next entry in g_device_data table
            */
            ++i;
        }
    }/*end for*/

    /*-----------------------------------------*/
    /*  add  submodules                        */
    /*-----------------------------------------*/
    for(i = 0; i < entries; ++i) {
        /*
        in g_device_data structure, every module entry has got at-least 1 sub-slot associated with it
        whereas all the sub-module entries are having zero sub-slot.
        */
        if(g_device_data[i].maxSubslots > 0) { /* beginning of a new slot */
            slot = i; /* index of the corresponding slot for a given subslot */

            g_device_data[slot].subState = 1; /* assume that the sub-modules for this slot are
                                                 going to be successfully added, if any module is not added
                                                 correctly, it will be later set to 0 */
        }

        if(g_device_data[slot].modState) { /* add sub-module only if the module is added */
            addr.u.Geo.Slot    = g_device_data[i].slot;
            addr.u.Geo.Subslot = g_device_data[i].subslot;

            status = PNIO_sub_plug(g_hDevice,                   /* device handle            */
                                   g_device_data[i].api,        /* api number               */
                                   &addr,                       /* location (slot, subslot) */
                                   g_device_data[i].subslotId); /* submodule identifier     */

            if(status == PNIO_OK) {
                printf("sub-module plug\t");

                g_device_data[i].subState = 1;
            } else {
                printf("sub-module plug failed");

                g_device_data[i].subState    = 0;
                g_device_data[slot].subState = 0;
            }

            printf("\t: api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
                g_device_data[i].api,
                g_device_data[i].slot,
                g_device_data[i].subslot,
                g_device_data[i].maxSubslots,
                g_device_data[i].modId);
        }
    }/*end for*/

    /* if all the modules/submodules are plugged correctly, set the device state to OK */
    for(i = 0; i < entries; ++i) {
        if(g_device_data[i].subState == 0)
            break;
    }

    if(i == entries) { /* for loop completed successfully */
        printf("Set device state to PNIO_DEVSTAT_OK...\n");
        status = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_OK);
    }

    return status;
}


/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :       RemoveModSubMod                              */
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

    for(i = entries - 1; i >= 0 && status == PNIO_OK; i--) {
        if(g_device_data[i].subState == 1) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot; /* slot number */
            addr.u.Geo.Subslot  = g_device_data[i].subslot;

            /*-----------------------------------------*/
            /*  remove  submodules                     */
            /*-----------------------------------------*/
            status = PNIO_sub_pull(g_hDevice, g_device_data[i].api, &addr);

            if(status == PNIO_OK) {
                g_device_data[i].subState = 0;

                printf("Submodule pull\t\t:");
            } else {
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

        if(status == PNIO_OK && g_device_data[i].modState == 1) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot;
            addr.u.Geo.Subslot  = 1;                         /* dont care */

            /*-----------------------------------------*/
            /*  remove  modules                                                */
            /*-----------------------------------------*/

            status = PNIO_mod_pull(g_hDevice, g_device_data[i].api, &addr);

            if(status == PNIO_OK) {
                printf("Module pull\t\t:");
                g_device_data[i].modState = 0;
            } else {
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
    for(i = j = 0; i < gDevArraySize; i++) {
        /* read api from our configuration data */
        api = g_device_data[i].api;

        /* look if api added at a prior position */
        for(j = 0; j < i; j++) {
            if(api == g_device_data[j].api) {
                /* api was added */
                break;
            }
        }

        if(i == j) { /* not added, add a new api */
            /* calculate highest slot and subslot number for this api */
            highestSlotsNumber   = g_device_data[j].slot;
            highestSubslotNumber = g_device_data[j].subslot;

            /*
            check if the api exists in the slots ahead,
            if yes, then update highest slot/subslot number accordingly
            */
            for(j = i+1; j <  gDevArraySize; j++) {
                if(api == g_device_data[j].api) {
                   if(g_device_data[j].slot > highestSlotsNumber)
                       highestSlotsNumber = g_device_data[j].slot;
                   if(g_device_data[j].subslot > highestSubslotNumber)
                       highestSubslotNumber = g_device_data[j].subslot;
                }
            }

            status = PNIO_api_add(
                g_hDevice,
                api,
                highestSlotsNumber,
                highestSubslotNumber);

            if(status != PNIO_OK)
                printf("\nPNIO_api_add failed\n\n");
            else
                printf("\nPNIO_api_add successful\n\n");
        }
    }

    return status;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  RemoveApi()                                                   */
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
    for(i = j = 0; i < entries && status == PNIO_OK; i++) {
        /* read api from our configuration data */
        api = g_device_data[i].api;

        /* look if the api has been added at a prior position in our g_device_data structure */
        for(j = 0; j < i; j++) {
            if(api == g_device_data[j].api) {
                /* api added at a prior position, hence it has already been removed*/
                break;
            }
        }

        if(i == j) { /* api not removed yet */
            status = PNIO_api_remove(g_hDevice, api);
            if(status != PNIO_OK) {
                    printf("\nApi remove failed\t:");
            } else {
                    printf("\nApi removed\t\t:");
            }
            printf(" api=%u\n\n", api);
        }
    }
    return status;
}

/*-------------------------------------------------------------------*/
/*                                                                   */
/*  Initialize()                                                     */
/*                                                                   */
/*-------------------------------------------------------------------*/
/*      This function does the initialization of the PNIO            */
/*      device. Registration of callbacks is part of initialization  */
/*-------------------------------------------------------------------*/
PNIO_UINT32 Initialize(void)
{
    PNIO_CFB_FUNCTIONS  structCBFunctions;
    PNIO_UINT32         Handle    = 0; /* 0 is invalid handle */
    PNIO_UINT32         uiMaxAR   = 2; /* maximum application relationships supported. */
                                       /* must be 2 for isochronous realtime           */
    PNIO_UINT32         ErrorCode = PNIO_OK;

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
    structCBFunctions.cbf_pull_plug_conf    = NULL;

    printf("Initializing PNIO device...\n");

    do {
        ErrorCode = PNIO_device_open(
            /*in*/  CP_INDEX,                 /* index of communication processor */
            /*in*/  PNIO_CEP_MODE_CTRL,       /* permission to change operation mode */
            /*in*/  VENDOR_ID,                /* vendor ID */
            /*in*/  DEVICE_ID,                /* device ID */
            /*in*/  INSTANCE_ID,              /* instance ID */
            /*in*/  uiMaxAR,                  /* max AR count */
            /*in*/  &structPNIOAnnotation,    /* annotation */
            /*in*/  &structCBFunctions,       /* callback functions information */
            /*out*/ &Handle);                 /* device handle */

        if(ErrorCode != PNIO_OK) {
            int ch = getCharWithTimeout();
            if(('q'==ch) || ('Q'==ch))
                return ErrorCode;
        }
    }
    while(ErrorCode == PNIO_ERR_CONFIG_IN_UPDATE);

    if(ErrorCode == PNIO_OK)
        g_hDevice = Handle;

    return ErrorCode;
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Uninitialize()                                                  */
/*                                                                  */
/*------------------------------------------------------------------*/
/*      This function does the de-initialization of the PNIO device */
/*------------------------------------------------------------------*/
PNIO_UINT32 Uninitialize(void)
{
    return PNIO_device_close(g_hDevice);
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  ConfigureDeviceData()                                           */
/*                                                                  */
/*------------------------------------------------------------------*/
/*  In the sample program #define is used to write the config       */
/*      data. This function creates a structure out of the config   */
/*  data, and fills the unfilled members whereever necessary.       */
/*      In practice the developer can create a structure similar to */
/*      this from the '.ini' file.                                  */
/*------------------------------------------------------------------*/
void ConfigureDeviceData(void)
{
    int i            = 0;
    int beginNewSlot = 0;
    int idx          = 0;      /* counter for idxTbl */

    /*
    copy the predefined structure to g_device_data as it is.
    This piece of code can be replaced by one with '.ini' file reading
    and creating the elementary structure g_device_data.
    */

    g_device_data = device_data;
    for(i = 0; i < DEVICE_DATA_ENTRIES; ++i) {
        g_device_data->maxSubslots = 0;
        g_device_data->modState    = 0;
        g_device_data->subState    = 0;
        g_device_data->dir         = 0;
    }

    /* fill idxTbl with -1, only entries corresponding to a slot will have the right slot id. */
    memset(idxTbl, -1, DEVICE_DATA_ENTRIES * sizeof(int));
    idxTbl[idx++] = g_device_data[0].slot;

    /*
    our predefined structure has the maxsubslots field set to 0, even though
    there are subslots for every module. Here we dynamically calculate the
    subslots in each module and update the g_device_data structure accordingly
    */

    /* browsing through the device_data structure */
    for(i = 0; i < DEVICE_DATA_ENTRIES; i++) {
        if(g_device_data[i].slot == g_device_data[beginNewSlot].slot) {
            /* we are still in same slot and new sub-slot */
            /*
            if we are in the same slot, then the information in this row
            is regarding a new subslot. So increment the sub-slot count by 1
            */
            g_device_data[beginNewSlot].maxSubslots++;

            /*
            we assign the slotId/modId to the corresponding sub-slot
            */
            g_device_data[i].modId = g_device_data[beginNewSlot].modId;
        } else {
            /* new slot information has started */
            beginNewSlot = i;                            /* index corresponding to the beginning of the new slot */
            g_device_data[beginNewSlot].maxSubslots = 1; /* every new module/slot has min one sub-slot */
            idxTbl[idx++] = g_device_data[i].slot;       /* store the entry of the new slot in idxTbl */
        }
    }
}


void usage()
{
    int SlotNum = g_device_data[g_mirror_index].slot;
    int SubNum  = g_device_data[g_mirror_index].subslot;
    printf("m - toggle mirroring of submodule %d.%d OUT data to submodule %d.%d IN data\n",
        SlotNum, SubNum, SlotNum, SubNum);
    SlotNum = g_device_data[g_increment_index].slot;
    SubNum  = g_device_data[g_increment_index].subslot;
    printf("i - toggle increment of submodule %d.%d IN data (interpreted as unsigned long value)\n",
        SlotNum, SubNum);
    SlotNum = g_device_data[g_set_index].slot;
    SubNum  = g_device_data[g_set_index].subslot;
    printf("s - set value of submodule %d.%d IN data (1 byte)\n",
        SlotNum, SubNum);
    printf("l - list all IN and OUT modules\n");
    printf("c - list counters for StartOp, DataExchange and OpFault\n");
    printf("h - print this help\n");
    printf("x - toggle data exchange state\n");
    printf("q - stop data exchange and quit application\n");
}


void print_module_list()
{
    int   i, j, length;
    int   slot, subslot;
    char *ext;

    printf("\nmodule  slot sub length IOPS IOCS data\n");
    for(i = 0; i < gDevArraySize; ++i) {
        slot    = g_device_data[i].slot;
        subslot = g_device_data[i].subslot;
        length  = InDatLen[slot][subslot];

        printf("%2d. IN  %4d.%-3d %6d %4s %4s",
               i, slot, subslot, InDatLen[slot][subslot],
               InDatIops[slot][subslot] == PNIO_S_GOOD ? "good" : "bad",
               InDatIocs[slot][subslot] == PNIO_S_GOOD ? "good" : "bad");
        if(length > MAX_DUMP_DATA) {
            length = MAX_DUMP_DATA;
            ext = " ...";
        } else {
            ext = "";
        }

        for(j = 0; j < length; ++j)
            printf(" %02x", InData[slot][subslot][j]);

        printf("%s\n", ext);

        length = OutDatLen[slot][subslot];
        printf("    OUT %4d.%-3d %6d %4s %4s",
               slot, subslot, OutDatLen[slot][subslot],
               OutDatIops[slot][subslot] == PNIO_S_GOOD ? "good" : "bad",
               OutDatIocs[slot][subslot] == PNIO_S_GOOD ? "good" : "bad");
        if(length > MAX_DUMP_DATA) {
            length = MAX_DUMP_DATA;
            ext = " ...";
        } else {
            ext = "";
        }

        for(j = 0; j < length; ++j)
            printf(" %02x", OutData[slot][subslot][j]);

        printf("%s\n", ext);
    }
}


int main()
{
    PNIO_UINT32               ErrorCode = PNIO_OK;
    PNIO_APPL_READY_LIST_TYPE readyListType;

    int                       slot;
    int                       subslot;
    int                       i;
    char                      data[50] = {0};
    int                       ch = '\0';
    int                       restart = 0;
        unsigned long             seconds;

    /*----------------------------------------------------------------*/
    /*                 Program Menu                                   */
    /*----------------------------------------------------------------*/

    printf("\n--------------------------------------------------------------------------------\n");
    printf("\n------                      PNIO Device Sample Program                     -----\n");
    printf("\n--------------------------------------------------------------------------------\n");

    printf("This sample program shows how to do the following operations with PNIO device:\n");
    printf("\t1. How to initialize PNIO device\n");
    printf("\t2. How to Read/Write IRT IO data\n");

    printf("\nPress 's' to start, 'q' to quit ...\n");

    /* loop to wait for the user input */
    while(1) {
        ch = getCharWithTimeout();
        if(('q'==ch) || ('Q'==ch))
            return 0;

        if(('s'==ch) || ('S'==ch))
            break;
    }
    printf("\n");

RESTART_DEVICE:
    restart = 0;

    /*-------------------------*/
    /* initialize global flags */
    /*-------------------------*/
    g_data              = 0;
    g_set               = 1;
    g_mirror            = 0;
    g_increment         = 0;
    g_DataExchange      = 0;
    g_device_stopped    = 0;
    g_device_stop_req   = 0;

    /*----------------------------------------*/
    /* initialize the g_device_data structure */
    /*----------------------------------------*/
    ConfigureDeviceData();

    g_increment_index = GetSubmodNum(2, 1);
    g_mirror_index    = GetSubmodNum(2, 1);
    g_set_index       = GetSubmodNum(1, 1);
    printf("DEBUG: set index = %d, mirror index = %d, increment index = %d\n",
        g_set_index, g_mirror_index, g_increment_index);

    /*---------------------------------*/
    /* set first_startop flag to false */
    /*---------------------------------*/
    first_startop_done = 0;
    ar_info_done       = 0;
    prm_end_done       = 0;

    /*-----------------------------------*/
    /* Open device                       */
    /*-----------------------------------*/

    ErrorCode = Initialize();
    if((ErrorCode != PNIO_OK) || (0 == g_hDevice)) {
        printf("Error in initializing the device. Error# 0x%08x\n",
            ErrorCode);
        return -1;
    }

    printf("Device initialized successfully. Handle=0x%08x\n",
        g_hDevice);

    /*-----------------------------------*/
    /* Add Profile                       */
    /*-----------------------------------*/

    ErrorCode = AddApi();
    if(ErrorCode != PNIO_OK) {
        /* appropriate messages are displayed in function AddModules */
        goto CLEAN_UP;
    }

    /*-----------------------------------------------*/
    /* Add modules and sub modulesto the device      */
    /*-----------------------------------------------*/

    ErrorCode = AddModSubMod();
    if(ErrorCode != PNIO_OK) {
        /* appropriate messages are displayed in function AddModules */
        goto CLEAN_UP;
    }

    /*----------------------------------------------*/
    /* Register for IRT callbacks                   */
    /*----------------------------------------------*/
    ErrorCode = PNIO_CP_register_cbf(g_hDevice, PNIO_CP_CBE_OPFAULT_IND, Cbf_OPFault_ind);
    if(ErrorCode != PNIO_OK) {
        printf("\n\tError (0x%08x) while registering OpFault callback function, check if you are logged in as root\n",
            ErrorCode);
        goto CLEAN_UP;
    }

    ErrorCode = PNIO_CP_register_cbf(g_hDevice, PNIO_CP_CBE_STARTOP_IND, Cbf_StartOP_ind);
    if(ErrorCode != PNIO_OK) {
        printf("\n\tError (0x%08x) while registering StartOP callback function, check if you are logged in as root\n",
            ErrorCode);
        goto CLEAN_UP;
    }

    /*----------------------------------------------*/
    /* Initialize data                              */
    /*----------------------------------------------*/
    printf("Initializing read-write buffers...\n");

    /*** set initial values for input data ***/
    /* loop corresponding to slots */
    for(i = 0; i < gDevArraySize; ++i) {
        memset(data, 0, 50);

        slot    = g_device_data[i].slot;
        subslot = g_device_data[i].subslot;

        sprintf(data, "SLOT_ID=%d, SUBSLOT_ID=%d, DATA=%d.%d\n", slot, subslot, slot, subslot);
        memcpy(InData[slot][subslot], data, 50);
    }

    /*----------------------------------------------*/
    /* Start the device                             */
    /*----------------------------------------------*/
    printf("Starting PNIO device...\n");
    ErrorCode = PNIO_device_start(g_hDevice);
    if(ErrorCode != PNIO_OK) {
        printf("\n\tError in starting the device. Error# 0x%08x\n", ErrorCode);
        goto CLEAN_UP;
    }

    /*------------------------------------------------------------------------*/
    /* Attention : Callbacks are running concurrent in other threads so all   */
    /*             printf statements should be synchronized. But this sample  */
    /*             application doesn't synchronize for simplicity  !          */
    /*------------------------------------------------------------------------*/
    printf("\nCallbacks awaited ....\n");
    printf("\nOnce you receive expected callbacks, "
           "press 'q' to close the device or 'x' to toggle data exchange mode ...\n\n");

    seconds = 0;
    while(!prm_end_done || !first_startop_done) {
        printf("*** waiting for prm_end and first startop callback (%lu seconds)\r", seconds);
        fflush(stdout);
        ch = getCharWithTimeout();
        if(('q' == ch) || ('Q' == ch)) {
            /* abort by user */
            printf("\n\n*** waiting for PRM END event aborted...\n");
            goto CLEAN_UP;
        }
        if(g_device_stop_req) {
            g_device_stop_req = 0;
            restart = 1;
            goto CLEAN_UP;
        }
        Sleep(900);
        ++seconds;
    }
    printf("\n");

    /* Here we need to call PNIO_set_appl_state_ready so that                                */
    /* the IO Base Device user program registers with the IO controller, a list of the       */
    /* non-functional sub-modules and the extent of readiness to get into a data interchange */

    memset(&readyListType, 0, sizeof(readyListType));
    readyListType.ap_list.Flink = NULL;
    readyListType.ap_list.Blink = NULL;

    ErrorCode = PNIO_set_appl_state_ready(g_hDevice, g_arNumber, g_SessionKey, &readyListType);
    if(ErrorCode == PNIO_OK) {
        printf("\nDevice is ready...\n");
    } else {
        printf("\n\tError in setting appl state ready. Error# 0x%08x\n", ErrorCode);
        goto CLEAN_UP;
    }

    /* wait for AR IN-Data event indication */
    while(!g_DataExchange) {
        ch = getCharWithTimeout();
        if(('q' == ch) || ('Q' == ch)) {
            /* abort by user */
            printf("\n*** waiting for AR IN-Data event aborted...\n");
            goto CLEAN_UP;
        }
    }

    usage();

    while (1)
    {
        ch = getCharWithTimeout();

        switch (ch) {
        case 'c': /* show statistics */
            printf("Statistics: %8u StartOp callbacks, %8u DataExchange calls, %8u OpFault callbacks\n",
                g_StartOpCount, g_DataExchangeCount, g_OpFaultCount);
            printf("            %8u Data write errors, %8u Data read errors,   %8u Bad remote status\n",
                g_writeErrors, g_readErrors, g_badRemoteState);
            break;

        case 'x': /* toggle data exchange state */
            g_DataExchange = !g_DataExchange;
            printf("data exchange is %s\n", g_DataExchange ? "on" : "off");
            break;

        case 'i': /* toggle increment */
            g_increment = !g_increment;
            printf("increment is %s\n", g_increment ? "on" : "off");
            break;

        case 'm': /* toggle mirroring */
            g_mirror = !g_mirror;
            printf("mirror is %s\n", g_mirror ? "on" : "off");
            break;

        case 's': /* set data */
            printf("data is %d (0x%02x) change to --> ", g_data, g_data);
            scanf("%i", &g_data);
            printf("new data is %d (0x%02x)\n", g_data, g_data);
            g_set = 1;
            break;

        case 'l': /* list modules */
            print_module_list();
            break;

        case 'h': /* print help */
            usage();
            break;

        case 'q': /* set data */
            goto CLEAN_UP;
            break;
        }
    }

CLEAN_UP:
    printf("\nStopping PNIO device...\n");

    g_DataExchange = 0;

    /* Stop the device */
    ErrorCode = PNIO_device_stop(g_hDevice);

    if(ErrorCode != PNIO_OK) {
        printf("\n\tError in stopping the device. Error# 0x%08x\n", ErrorCode);
        return -1;
    }

    /*----------------------------------------------*/
    /* Callback synchronization                     */
    /*----------------------------------------------*/

    /*
    after stopping the device a callback "PNIO_cbf_device_stopped" comes,
    in this callback we set g_device_stopped to 1 and ask user to press 'q' to quit
    */
    Sleep(1);

    while(!g_device_stopped) {
        ch = getCharWithTimeout();
        if(('q' == ch) || ('Q' == ch)) {
            /* abort by user */
            printf("\n*** waiting for PNIO_cbf_device_stopped aborted...\n");
            g_device_stopped = 1;
        }
    }

    /* Remove the modules and submodules */
    ErrorCode = RemoveModSubMod();
    if(ErrorCode != PNIO_OK) {
        return -1;
    }

    /* Remove the API */
    ErrorCode = RemoveApi();
    if(ErrorCode != PNIO_OK) {
        return -1;
    }

    /* Uninitialize the device */
    ErrorCode = Uninitialize();
    if(ErrorCode != PNIO_OK) {
        printf("Error in uninitializing the device. Error# 0x%08x\n", ErrorCode);
        return -1;
    }

    g_hDevice = 0;  /* set hDevice back to an invalid value */
    printf("Device uninitialized successfully\n\n");

    if(restart)
        goto RESTART_DEVICE;

    return 0;
}


/*************************************************************/
/* Get a Character from console                              */
/*************************************************************/
int getCharWithTimeout()
{
    int                   key = 0;
#ifndef WIN32
    struct pollfd         pollfd[1];
    static int            init = 0;
    static struct termios termiosOld;
    static struct termios termios;

    if(!init) {
        tcgetattr(fileno(stdin), &termios);
        memcpy(&termiosOld, &termios, sizeof(termios));
        //cfmakeraw(&termios);
        termios.c_lflag &= ~ICANON;
        termios.c_lflag &= ~ECHO;
        termios.c_lflag &= ~ISIG;
        termios.c_oflag |= ONLCR | OPOST;
        termios.c_cc[VMIN] = 0;
        termios.c_cc[VTIME] = 0;
        ++init;
    }

    /* put terminal into raw mode */
    tcsetattr(fileno(stdin), TCSANOW, &termios);
    pollfd->fd      = fileno(stdin);
    pollfd->events  = POLLIN;
    pollfd->revents = 0;
    poll(pollfd, 1, 100);
    if(pollfd->revents & POLLIN)
        key = getchar();

    tcsetattr(fileno(stdin), TCSANOW, &termiosOld);
#else
    Sleep(100);
    if(_kbhit()) {
        key = _getch();
    }
#endif

    return key;
}
