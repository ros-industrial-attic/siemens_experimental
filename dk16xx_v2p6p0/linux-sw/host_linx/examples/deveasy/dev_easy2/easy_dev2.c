/*------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*------------------------------------------------------------------------*/
/*                                                                        */
/*   Project           :                                                  */
/*   Filename          : easy_dev2.c                                      */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
/* Description: This sample program shows how to do the following         */
/*            operations with the PNIO device                             */
/*             1. Initialize and Uninitialize                             */
/*             2. Reading / Writing IO data                               */
/*             3. Processing the data record read/write request           */
/*                of the IO controller                                    */
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
#include <string.h>
#include <time.h>
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
/*    DEFINES                                                                                         */
/*----------------------------------------------------------------------------------------------------*/
#define NUMOF_SLOTS             9     /* slot 0...8   +  1 */
#define NUMOF_SUBSLOTS          2      /* Every slot has 1 subslot */
#define NUMOF_BYTES_PER_SUBSLOT 256    /* Maximum data length as configured in the sample */
//Maximalwerte für Zählvariable
#define MAX_ZAEHL 100	
#define MAX_ZAEHL1 500

/*----------------------------------------------------------------------------------------------------*/
/*    GLOBALS                                                                                         */
/*----------------------------------------------------------------------------------------------------*/
PNIO_UINT32 g_hDevice = 0;              /* global handle for device */
PNIO_UINT16 g_SessionKey = 0;           /* session identifier, this is obtained in application-relation
                                           information callback i.e. PNIO_cbf_ar_info_ind. */
PNIO_UINT16 g_arNumber = 0;             /* application relation number */

/*
The structure described below contains all the module/submodule information. For simplicity, this structure
is hard-coded in the configuration file, in the actual program it can be built via the '.ini' file.
*/
static device_data_t device_data[] =
{
    DEVICE_DATA
};

device_data_t *g_device_data = NULL;
static int gDevArraySize = DEVICE_DATA_ENTRIES;  /* Total no of slots as configured in the sample Step7 project */
int idxTbl[DEVICE_DATA_ENTRIES];                 /* an array of slot ids, sub-slot entries will contain -1 */

/*
Some callback-flags for the callbacks we have to wait for in order to initialize the device properly
*/
static int AR_INFO_IND_flag = 0;                        /* Callback: Verbindung zum Controller ist aufgebaut */
static int PRM_END_IND_flag = 0;                        /* Callback: Ende Parametrierphase */
static int INDATA_IND_flag = 0;                         /* Callback: erste Datenübermittlung von IO-Controller*/


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
    PNIO_UINT32 i;

    PNIO_UINT32 slot_num    = pAddr->u.Geo.Slot;
    PNIO_UINT32 subslot_num = pAddr->u.Geo.Subslot;

    printf("## PNIO_cbf_data_write(..., len=%u, Iocs=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iocs, DevHndl, slot_num, subslot_num);


    InDatLen  [slot_num][subslot_num] = BufLen;  /* save data length (only for debugging) */
    InDatIocs [slot_num][subslot_num] = Iocs;    /* consumer status (of remote IO controller) */


    if(BufLen == 0) {
        printf(" BufLen = 0, nothing to write to..\n");
        InDatIops [slot_num][subslot_num] = PNIO_S_GOOD;
    } else if(BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT) {
        memcpy (pBuffer, &InData[slot_num][subslot_num][0], BufLen); /* Copy the application data to the stack */
            InDatIops [slot_num][subslot_num] = PNIO_S_GOOD; /* assume everything is ok */

            printf(" IO Data buffer:\n");
            for(i = 0; i < BufLen; i++) {
                if(i%16 == 0 && i!=0)
                    printf("\n");

                printf(" 0x%02x", pBuffer[i]);
            }
            printf("\n");
    } else {
        printf("!!! PNIO_cbf_data_write: Buflen=%lu > allowed size (%u)!!! Abort writing..\n",
            (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
        InDatIops [slot_num][subslot_num] = PNIO_S_BAD; /* set local status to bad */
    }

    return (InDatIops [slot_num][subslot_num]); /* return local provider status */
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
    PNIO_UINT32 i;
    PNIO_UINT32 slot_num    = pAddr->u.Geo.Slot;
    PNIO_UINT32 subslot_num = pAddr->u.Geo.Subslot;

    printf("## PNIO_cbf_data_read(..., len=%u, Iops=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iops, DevHndl, slot_num, subslot_num);

    OutDatLen  [slot_num][subslot_num] = BufLen;  /* save data length (only for debugging) */
    OutDatIops [slot_num][subslot_num] = Iops;    /* provider status (of remote IO controller) */

    if(BufLen == 0) {
        printf(" BufLen = 0, nothing to read..\n");
        OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD;
    } else if(BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT) {
        memcpy (&OutData[slot_num][subslot_num][0], pBuffer, BufLen); /*Copy the data from the stack to the application buffer*/
        OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD; /* assume everything is ok */

        printf(" IO Data buffer:\n");
        for(i = 0; i < BufLen; i++) {
            if(i%16 == 0 && i!=0)
                printf("\n");

            printf(" 0x%02x", OutData[slot_num][subslot_num][i]);
        }
        printf("\n");
    } else {
        printf("!!! PNIO_cbf_data_read: Buflen=%lu > allowed size (%u)!!! Abort reading...\n",
            (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
        OutDatIocs [slot_num][subslot_num] = PNIO_S_BAD; /* set local status to bad */
    }

    return ( OutDatIocs [slot_num][subslot_num]); /* consumer state (of local IO device) */
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
    PNIO_UINT8    WriteRecDummyData[50];
    PNIO_UINT32   i;
    PNIO_UINT32   Status = PNIO_OK;

    printf ("\n## WRITE_RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu\n",
        (unsigned long)Api, (unsigned long)pAddr->u.Geo.Slot, (unsigned long)pAddr->u.Geo.Subslot,
        (unsigned long)RecordIndex, (unsigned long)*pBufLen, (unsigned long)SequenceNum);

    /**** check data size (accepted data < provided data) */
    if(*pBufLen > sizeof (WriteRecDummyData)) {
        *pBufLen = sizeof (WriteRecDummyData);
        printf("## WARNING: Can not write all data, not enough space..\n");
    }

    /**** copy the record data into a buffer for further use ***/
    memcpy (WriteRecDummyData,      /* destination pointer for record data */
        pBuf,                       /* source pointer for record data      */
        *pBufLen);                  /* length of the accepted data         */

    printf ("## RECORD_DATA written:");
    for(i=0; i < *pBufLen; i++) {
        if(i%16 == 0)
                printf("\n");

        printf ("0x%02lx ", (long)WriteRecDummyData[i]);
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
    PNIO_UINT32 i;
    PNIO_UINT32   Status    = PNIO_OK;

    /***** copy dummy data into the buffer, set data-size ****/
    PNIO_UINT8    ReadRecDummyData[] = {"**Data1234 ReadRecord**"};

    if(*pBufLen > sizeof (ReadRecDummyData)) {
        *pBufLen = sizeof (ReadRecDummyData);
    }

    printf ("\n## READ_RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu\n",
        (unsigned long)Api, (unsigned long)pAddr->u.Geo.Slot, (unsigned long)pAddr->u.Geo.Subslot,
        (unsigned long)RecordIndex, (unsigned long)*pBufLen, (unsigned long)SequenceNum);

    /*----------------------------------------------*/
    /*  copy the data to the specified buffer       */
    /*----------------------------------------------*/
    if(*pBufLen < sizeof(ReadRecDummyData))
        printf("## WARNING: Can not transmit all data, buffer too small..\n");

    memcpy (pBuf,ReadRecDummyData,*pBufLen);

    printf ("## RECORD_DATA transmitted:");
    for(i=0; i < *pBufLen; i++) {
        if(i%16 == 0)
                printf("\n");

        printf ("0x%02lx ", (long)pBuf[i]);
    }
    printf("\n");

    if(Status == PNIO_OK) {
        memset(pPnioState, 0, sizeof(*pPnioState));
        return;
    } else {
        /**** if an error occured, it must be specified according IEC 61158-6 */
        pPnioState->ErrCode   = 0xde;  /* IODReadRes with ErrorDecode = PNIORW */
        pPnioState->ErrDecode = 0x80;  /* PNIORW                               */
        pPnioState->ErrCode1  = 0xa9;  /* example: Error Class 10 = application, ErrorNr 9 = "feature not supported" */
        pPnioState->ErrCode2  = 0;     /* here dont care                                                             */
        pPnioState->AddValue1 = 0;     /* here dont care                                                             */
        pPnioState->AddValue2 = 0;     /* here dont care                                                             */
        return;
    }
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

    printf ("## CHECK_IND slot=%u, subslot=%u, ModIdent=%u, State (%u), SubIdent=%u, State (%u)\n",
        pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, *pModIdent, *pModState, *pSubIdent, *pSubState);

    /* get the index int of our configuration */
    idx = GetSubmodNum(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);

    /*
    Check the configuration sent by controller against the configuration_data structure.
    If there is any mismatch, return error.
    */
    if((idx != -1) && ((unsigned int) g_device_data[idx].subslot == pAddr->u.Geo.Subslot) &&
        (g_device_data[idx].modId == *pModIdent) && (g_device_data[idx].subslotId == *pSubIdent)) {
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
    printf("## PNIO_cbf_ar_check_ind (Station %s, IP %d.%d.%d.%d)\n\n",
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
    int i, j;
    g_arNumber = ArNumber;     /* Store the AR number */
    g_SessionKey = SessionKey; /* Store the session key */

    printf ("## AR-INFO_IND new AR from PNIO controller established SessionKey %x\n",
        SessionKey);


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
	/* set local consumer status for all output slots						*/
    /*----------------------------------------------------------------------*/
    for(i = 0; i < gDevArraySize ; i++) {
        for(j = 0; j < 1 /*g_device_data[i].maxSubslots*/; j++) {
            /*** set local provider state = GOOD for input data***/
            if(i == 0) {
                if(g_device_data[i].modState == 1) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
					OutDatIocs [i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops [i][j] = PNIO_S_BAD;
					OutDatIocs [i][j] = PNIO_S_BAD;
                }
            } else {
                if(g_device_data[i].modState == 1 &&
                    g_device_data[i+j].subState == 1) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
					OutDatIocs [i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops [i][j] = PNIO_S_BAD;
					OutDatIocs [i][j] = PNIO_S_BAD;
                }
            }
        }
    }

    AR_INFO_IND_flag = 1;
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
    printf ("## AR IN-Data event indication has been received, ArNumber = %x\n\n",
        ArNumber);

    INDATA_IND_flag = 1;
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
    printf ("## AR ABORT indication, ArNumber = %x, Reason = %x\n\n",
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
    printf ("## AR Offline indication, ArNumber = %x, Reason = %x\n\n",
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
    int zaehl=0;
	// Warten auf Aufruf des PNIO_cbf_ar_info_ind() Callbacks
	while (AR_INFO_IND_flag == 0)
	{
        if (zaehl==MAX_ZAEHL1)
		{
			printf("Kein Aufruf des Callbacks PNIO_cbf_ar_info_ind()\n\n");
			return;
		}
		zaehl++;
		Sleep(10);
	}

    printf ("\n## Event parametrizing phase -Application ready- received, ArNumber = %x\n\n",
        ArNumber);

    PRM_END_IND_flag = 1;
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
    printf("## PNIO_cbf_cp_stop_req\n");

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
    printf("## PNIO_cbf_device_stopped\n\n");

    //Manual synchronization - The main function calls device-stop function
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
    printf("req_done not supported\n");
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
    printf("APDU Status Change not supported");
}

/*----------------------------------------------------------------------------------------------------*/
/*    FUNCTIONS                                                                                       */
/*----------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  Function :		do_after_prm_end_ind_cbf()                     */
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
    PNIO_UINT32 ErrorCode;
    PNIO_APPL_READY_LIST_TYPE readyListType;

    /* Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user       */
    /* program can initialize the  incoming data (from the perspective of the IO controller) */
    /* for the functional submodules and set the local status to "GOOD". For all             */
    /* non-functional submodules, the local status should be set to "BAD"                    */
    /* We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback       */
    ErrorCode = PNIO_initiate_data_write(g_hDevice);
    if(PNIO_OK != ErrorCode) {
        printf("## Error - PNIO_init_data_write. 0x%x\n", ErrorCode);
    }

    /* We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program  */
    /* can set the local status for functional submodules of all the outgoing data             */
    /* (from the perspective of the IO controller) to GOOD. For all non-functional submodules, */
    /* the local status has to be set to "BAD".                                                */
    ErrorCode = PNIO_initiate_data_read(g_hDevice);
    if(PNIO_OK != ErrorCode) {
        printf("## Error - PNIO_init_data_read. 0x%x\n", ErrorCode);
    }

    /* Here we need to call PNIO_set_appl_state_ready so that                                  */
    /* the IO Base Device user program registers a list of the non-functional submodules       */
    /* and the extent of readiness to get into a data exchange at the IO controller            */
    memset(&readyListType, 0, sizeof(readyListType));
    readyListType.ap_list.Flink = NULL;
    readyListType.ap_list.Blink = NULL;

    ErrorCode = PNIO_set_appl_state_ready(g_hDevice,
        g_arNumber, g_SessionKey, &readyListType);

    if(ErrorCode == PNIO_OK)
        printf("\nDevice is ready...\n\n");
    else
        printf("\nError in setting appl state ready\n");
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  Function :		do_after_indata_ind_cbf()                      */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This function is called after the PNIO_cbf_ar_indata_ind        */
/* has been called ( see main() ), it calls					       */
/* PNIO_initiate_data_write() and PNIO_initiate_data_read()        */
/*                                                                 */
/*-----------------------------------------------------------------*/
void do_after_indata_ind_cbf(void)
{
    PNIO_UINT32 ErrorCode;

    /* Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user       */
    /* program can initialize the  incoming data (from the perspective of the IO controller) */
    /* for the functional submodules and set the local status to "GOOD". For all             */
    /* non-functional submodules, the local status should be set to "BAD"                    */
    /* We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback        */

    ErrorCode = PNIO_initiate_data_write(g_hDevice);
    if(PNIO_OK != ErrorCode) {
        printf("## Error - PNIO_init_data_write. 0x%x\n", ErrorCode);
    }

    /* We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program   */
    /* can set the local status for functional submodules of all the outgoing data              */
    /* (from the perspective of the IO controller) to GOOD. For all non-functional submodules,  */
    /* the local status has to be set to "BAD".                                                 */

    ErrorCode = PNIO_initiate_data_read(g_hDevice);
    if(PNIO_OK != ErrorCode) {
        printf("## Error - PNIO_init_data_read. 0x%x\n", ErrorCode);
    }
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		GetSubmodNum()                                */
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

    for(i = 0; i < entries; i++) {
        if((int)mod == idxTbl[i])		//Finden des Modulindex
            break;
    }

    if(i == entries)		//kein Modul gefunden? => auch kein Submodul
        return -1;

    for(j = 0; j < g_device_data[i].maxSubslots; j++) {
        if(g_device_data[i+j].subslot == (int)sub)		//Finden des Submodulindex
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

    /*------------------------------------------------------------------*/
    /*  add module 0 and corresponding submodule first                  */
    /*------------------------------------------------------------------*/

    printf("Start plugging modules and submodules..\n");
    printf("First we'll plug module 0 and the corresponding submodule..\n");

    addr.u.Geo.Slot    = g_device_data[0].slot;    /* plug module 0 */
    addr.u.Geo.Subslot = g_device_data[0].subslot; /* get the corresponding sub-slot */

    status = PNIO_mod_plug (g_hDevice,      /* device handle */
        g_device_data[0].api,               /* api number */
        &addr,                              /* location (slot, subslot) */
        g_device_data[0].modId);            /* module 0  identifier */

    if(status == PNIO_OK) {
        printf("Module plug\t\t:");
        g_device_data[0].modState = 1;
    } else {
        printf("Module plug failed\t:");
        g_device_data[0].modState = 0;
    }

    printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
        g_device_data[0].api,
        g_device_data[0].slot,
        g_device_data[0].subslot,
        g_device_data[0].maxSubslots,
        g_device_data[0].modId);

    if(!g_device_data[0].modState) {
        printf("ERROR: Failure in plugging module 0 -> no other module / submodule will be plugged...\n");
        return (status);
    }

    // now plug submodule corresponding to module 0
    status = PNIO_sub_plug (
        g_hDevice,                    /* device handle */
        g_device_data[0].api,         /* api number */
        &addr,                        /* location (slot, subslot) */
        g_device_data[0].subslotId);  /* submodule identifier */

    if(status == PNIO_OK) {
        printf("Submodule plug\t\t:");
        g_device_data[0].subState = 1;
    } else {
        printf("Submodule plug failed\t:");
        g_device_data[0].subState = 0;
    }

    printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
        g_device_data[0].api,
        g_device_data[0].slot,
        g_device_data[0].subslot,
        g_device_data[0].maxSubslots,
        g_device_data[0].modId);

    if(!g_device_data[0].subState) {
        printf("ERROR: Failure in plugging the submodule corresponding to module 0\n");
        printf(" -> no other module / submodule will be plugged...\n");
        return (status);
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
    for(i = 1; i < entries;) {
        addr.u.Geo.Slot    = g_device_data[i].slot;    /* plug module at correct slot */
        addr.u.Geo.Subslot = g_device_data[i].subslot; /* get the corresponding sub-slot*/

        status = PNIO_mod_plug (
            g_hDevice,               /* device handle */
            g_device_data[i].api,    /* api number */
            &addr,                   /* location (slot, subslot) */
            g_device_data[i].modId); /* module identifier */

        if(status == PNIO_OK) {
            printf("Module plug\t\t:");
            g_device_data[i].modState = 1;
        } else {
            printf("Module plug failed\t:");
            g_device_data[i].modState = 0;
        }

        printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
            g_device_data[i].api,
            g_device_data[i].slot,
            g_device_data[i].subslot,
            g_device_data[i].maxSubslots,
            g_device_data[i].modId);

        if(status == PNIO_OK) {
            /*
            advance in the g_device_data structure, jumping over all the submodule entries
            to reach the next module entry in the structure
            */
            i += g_device_data[i].maxSubslots;
        } else {
            /*
            go to the next entry in g_device_data table
            */
            i++;
        }
    }/*end for*/


    /*-----------------------------------------*/
    /*  add  submodules                        */
    /*-----------------------------------------*/
    for(i = 1; i < entries; i++) {
        /*
        in the g_device_data structure, each module entry has got at least 1 sub-slot associated with it (maxSubslot > 0)
        whereas all the submodule entries are having no sub-slots (maxSubslots = 0).
        */
        if(g_device_data[i].maxSubslots > 0) {
            /* beginning of a new slot */
            slot = i; /* index of the corresponding slot for a given subslot */

            g_device_data[slot].subState = 1; /* assume that the submodules for this slot are
                                                 going to be successfully added, if any module is not added
                                                 correctly, it will be later set to 0 */
        }

        if(g_device_data[slot].modState) {
            /* add submodule only if the module is added */
            addr.u.Geo.Slot         = g_device_data[i].slot;
            addr.u.Geo.Subslot      = g_device_data[i].subslot;

            status = PNIO_sub_plug (
                g_hDevice,                    /* device handle */
                g_device_data[i].api,         /* api number */
                &addr,                        /* location (slot, subslot) */
                g_device_data[i].subslotId);  /* submodule identifier */

            if(status == PNIO_OK) {
                printf("Submodule plug\t\t:");

                g_device_data[i].subState = 1;
            } else {
                printf("Submodule plug failed\t:");

                g_device_data[i].subState = 0;
                g_device_data[slot].subState = 0;
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
                g_device_data[i].api,
                g_device_data[i].slot,
                g_device_data[i].subslot,
                g_device_data[i].maxSubslots,
                g_device_data[i].modId);
        }
    }/*end for*/

    /*if not all the modules/submodules are plugged correctly, print warning*/
    for(i = 0; i < entries; i++) {
        if(g_device_data[i].subState == 0) {
            printf("WARNING: Not all modules or submodules were plugged correctly!!\n");
            break;
        }
    }

    return (status);
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

    for(i = entries - 1; i >= 0 && status == PNIO_OK; i--) {
        if(g_device_data[i].subState == 1) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot; /* slot number */
            addr.u.Geo.Subslot  = g_device_data[i].subslot;

            /*-----------------------------------------*/
            /*  remove  submodules                     */
            /*-----------------------------------------*/
            status = PNIO_sub_pull (g_hDevice,g_device_data[i].api, &addr);

            if(status == PNIO_OK) {
                g_device_data[i].subState = 0;

                printf("Submodule pull\t\t:");
            } else {
                printf("Submodule pull failed\t:");
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
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

            status = PNIO_mod_pull(g_hDevice,g_device_data[i].api,&addr);

            if(status == PNIO_OK) {
                printf("Module pull\t\t:");
                g_device_data[i].modState = 0;
            } else {
                printf("Module pull failed\t:");
            }

            printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
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
				g_hDevice,				/* in */
                api,					/* in */
                (PNIO_UINT16) highestSlotsNumber,		/* in */
                (PNIO_UINT16) highestSubslotNumber);	/* in */

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
/*  Function :		RemoveApi()                                   */
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
/*  Function :		Initialize()                                     */
/*                                                                   */
/*-------------------------------------------------------------------*/
/*      This function does the initialization of the PNIO            */
/*      device. Registration of callbacks is part of initialization  */
/*-------------------------------------------------------------------*/
PNIO_UINT32 Initialize(void)
{
    PNIO_CFB_FUNCTIONS      structCBFunctions;
    PNIO_UINT32 Handle;
    PNIO_UINT32 uiMaxAR=1; /* maximum application relationships supported. */
    PNIO_UINT32 ErrorCode = PNIO_OK;


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

    printf("Initializing PNIO device...\n");

    ErrorCode = PNIO_device_open(
        /*in*/  CP_INDEX,                 /* index of communication processor */
        /*in*/  PNIO_CEP_MODE_CTRL,       /* permission to change operation mode */
        /*in*/  VENDOR_ID,                /* vendor ID */
        /*in*/  DEVICE_ID,                /* device ID */
        /*in*/  INSTANCE_ID,              /* instance ID */
        /*in*/  uiMaxAR,                  /* max AR count*/
        /*in*/  &structPNIOAnnotation,    /* annotation */
        /*in*/  &structCBFunctions,       /* callback functions information*/
        /*out*/ &Handle);               /* device handle */


    if((ErrorCode != PNIO_OK) || (0 == Handle))
        return ErrorCode;

    g_hDevice = Handle;
    return  PNIO_OK;
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :		Uninitialize()                                  */
/*                                                                  */
/*------------------------------------------------------------------*/
/*      This function does the de-initialization of the PNIO device */
/*------------------------------------------------------------------*/
PNIO_UINT32 Uninitialize(void)
{
    return PNIO_device_close(g_hDevice);
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		PrintIOData()                                 */
/*                                                                */
/*----------------------------------------------------------------*/
/*      This function displays the buffer contents on screen      */
/*----------------------------------------------------------------*/

void PrintIOData()
{
    int SlotNum;
    int SubNum;
    PNIO_UINT32 IoInd;

    int i;

    printf("\nOutput data...\n");
    /*---------------------------------------------------------------*/
    /*  print output data                                            */
    /*---------------------------------------------------------------*/
    /* loop corresponding to slots */
    for(i = 0; i < gDevArraySize ; i++) {
        SlotNum = g_device_data[i].slot;
        SubNum = g_device_data[i].subslot;

        if(OutDatLen[SlotNum][SubNum] != 0) {
            printf("\nRead Module: Slot = %u, Subslot = %u, DataLen = %u, Local (consumer) status = %u\n",
                SlotNum, SubNum, OutDatLen[SlotNum][SubNum], OutDatIocs[SlotNum][SubNum]);

            for(IoInd=0; IoInd < OutDatLen[SlotNum][SubNum]; IoInd++) {
                if(IoInd % 16 == 0 && IoInd != 0)
                    printf("\n");

                printf(" 0x%02x", (unsigned int)OutData[SlotNum][SubNum][IoInd]);
            }
            printf("\n");
        }
    }

    printf("\nInput data...\n");
    /*---------------------------------------------------------------*/
    /*  print input data                                             */
    /*---------------------------------------------------------------*/
    /* loop corresponding to slots */
    for(i = 0; i < gDevArraySize ; i++) {
        SlotNum = g_device_data[i].slot;
        SubNum = g_device_data[i].subslot;

        if(InDatLen[SlotNum][SubNum] != 0) {
            printf("\nWrite Module: Slot = %u, Subslot = %u, DataLen = %u, Local (provider) status = %u\n",
                SlotNum, SubNum, InDatLen[SlotNum][SubNum], InDatIops[SlotNum][SubNum]);

            for(IoInd=0; IoInd < InDatLen[SlotNum][SubNum]; IoInd++) {
                if(IoInd % 16 == 0 && IoInd != 0)
                    printf("\n");

                printf(" 0x%02x", (unsigned int)InData[SlotNum][SubNum][IoInd]);
            }
            printf("\n");
        }
    }
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :		ConfigureDeviceData()                           */
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
    int i = 0;
    int beginNewSlot = 0;
    int idx = 0;      /* counter for idxTbl */

    /*
    copy the predefined structure to g_device_data as it is.
    This piece of code can be replaced by one with '.ini' file reading
    and creating the elementary structure g_device_data.
    */

    g_device_data = device_data;

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

int main(void)
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    int ch = '\0';
	int zaehl = 0, i=1;

    /*----------------------------------------------------------------*/
    /*                 Program Menu                                   */
    /*----------------------------------------------------------------*/

    printf("\n-----------------------------------------------------------------------------\n");
    printf("\n------\t\t\t PNIO Device Sample Program 2 \t\t\t-----   \n");
    printf("\n-----------------------------------------------------------------------------\n");

    printf("This sample program shows how to do the following operations with the\n");
    printf("PNIO device:\n");
    printf("\t1. How to initialize PNIO device\n");
    printf("\t2. How to Read/Write IO data\n");
    printf("\t3. How to process the data record read/write request of IO controller\n");
    printf("\t4. How to uninitialize the device and do the rest of the clean-up\n");
    printf("\t   process\n");

    printf("\n Press 's' to start, 'q' to quit ... \n");

    /* loop to wait for the user input */
    do {
        ch = getchar();
        if(((int)'q'==ch) || ((int)'Q'==ch))
            return 0;
    }while(((int)'s'!=ch) && ((int)'S'!=ch));
    printf("\n");

    /*----------------------------------------*/
    /* initialize the g_device_data structure */
    /*----------------------------------------*/
    ConfigureDeviceData();

    /*-----------------------------------*/
    /* Open device                       */
    /*-----------------------------------*/

    ErrorCode = Initialize();
    if((ErrorCode != PNIO_OK) || (0 == g_hDevice)) {
        printf("Error in initializing the device. Error# 0x%x\n",
            ErrorCode);
        return -1;
    }

    printf("Device initialized successfully. Handle=0x%x\n",
        g_hDevice);

    /*-----------------------------------*/
    /* Add Profile                       */
    /*-----------------------------------*/

    ErrorCode = AddApi();
    if(ErrorCode != PNIO_OK) {
        printf("Error in AddApi(). Error# 0x%x\n", ErrorCode);
       goto CLEAN_UP;
    }


    /*-----------------------------------------------*/
    /* Add modules and sub modules to the device      */
    /*-----------------------------------------------*/

    ErrorCode = AddModSubMod();
    if(ErrorCode != PNIO_OK) {
        printf("Error in AddModSubMod(). Error# 0x%x\n", ErrorCode);
		goto CLEAN_UP;
    }

    /*----------------------------------------------*/
    /* Start the device                             */
    /*----------------------------------------------*/

    printf("\nStarting PNIO device...\n");
    ErrorCode = PNIO_device_start(g_hDevice);
    if(ErrorCode != PNIO_OK) {
        printf("Error in starting the device. Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }

    printf("\nSetting device state OK...\n");
    ErrorCode = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_OK);
    if(ErrorCode != PNIO_OK) {
        printf("Error in setting device state OK. Error# 0x%x\n", ErrorCode);
        goto CLEAN_UP;
    }

    /*------------------------------------------------------------------------*/
    /* Attention : Callbacks are running concurrent in other threads so all   */
    /*             printf statements should be synchronized. But this sample  */
    /*             application doesn't synchronize for simplicity  !          */
    /*------------------------------------------------------------------------*/
    printf("\nWaiting for callbacks...\n\n");

    /* Wait for the necessary callbacks */
    while(zaehl != MAX_ZAEHL) {
		/* PNIO_cbf_prm_end_ind() callback already called? */
        if(PRM_END_IND_flag == 1) {
            do_after_prm_end_ind_cbf();
            break;
        }
		zaehl++;
        Sleep(100);
    }
	if (zaehl == MAX_ZAEHL)
		goto CLEAN_UP;
	zaehl=0;

    while(zaehl != MAX_ZAEHL) {
		/* PNIO_cbf_indata_ind() callback already called? */
        if(INDATA_IND_flag == 1) {
            do_after_indata_ind_cbf();
            break;
        }
		zaehl++;
        Sleep(100);
    }
	if (zaehl == MAX_ZAEHL)
		goto CLEAN_UP;

    printf("\n\nAll necessary callbacks called, press 'q' to close the device or 'p' to do some IO operations ...\n");
    /*loop to wait for the user input*/
    do {
        ch = getchar();
        if(('p'==ch) || ('P'==ch)) {
             /* if 'p' is hit - do IO operations */

            int i, slot, subslot;  /* i corresponds to every element in g_device_data */
            char data[50] = {0};

            printf("Initializing read-write buffers...\n");
            /* loop corresponding to slots */
            for(i = 0; i < gDevArraySize ; i++) {
                /*** set initial values for input data ***/
                memset(data, 0, 50);

                slot = g_device_data[i].slot;
                subslot = g_device_data[i].subslot;

                sprintf(data,"SLOT_ID=%d, SUBSLOT_ID=%d, DATA=%d.%d",
                    slot, subslot, slot, subslot);
                memcpy(InData[slot][subslot], data, 50);
            }

            break;
        }

        Sleep(100);
    } while (((int)'q'!=ch) && ((int)'Q'!=ch));
	if ((ch == (int)'q')|| (ch == (int)'Q'))
		goto CLEAN_UP;

    printf("\n\tPress\t'r' to initiate data read,\n\t\t'w' to initiate data write, \n\t\t'p' to print the IO data, \n\t\t'q' to quit...\n\n");
    do {
        ch = getchar();
		
        /* if 'p' is hit - display buffer contents */
        if(((int)'p'==ch) || ((int)'P'==ch)) {
            printf("\nPrinting IO data\n");
            PrintIOData();
        }

        /* if 'r' is hit - call PNIO_initiate_data_read */
        if(((int)'r'==ch) || ((int)'R'==ch)) {
            printf("\nCalled PNIO_initiate_data_read for device handle 0x%x,\nCallbacks awaited ....\n" ,
                g_hDevice);
            ErrorCode = PNIO_initiate_data_read(g_hDevice);
            if(PNIO_OK != ErrorCode) {
                printf("## Error - PNIO_init_data_read. 0x%x\n", ErrorCode);
            }
        }

        /* if 'w' is hit - call PNIO_initiate_data_write */
        if(((int)'w'==ch) || ((int)'W'==ch)) {
            printf("\nCalled PNIO_initiate_data_write for device handle 0x%x,\nCallbacks awaited ....\n" ,
                g_hDevice);
            ErrorCode = PNIO_initiate_data_write(g_hDevice);
            if(PNIO_OK != ErrorCode) {
                printf("## Error - PNIO_init_data_write. 0x%x\n", ErrorCode);
            }
        }
//-----------------------------------------------------------------------------
		/* show options of the menu*/
		if(!(((int)'q'==ch) || ((int)'Q'==ch))) {
			if (i==0){
			printf("\n\tPress\t'r' to initiate data read,\n\t\t'w' to initiate data write, \n\t\t'p' to print the IO data, \n\t\t'q' to quit...\n\n");
			i++;
			}
			else { i=0; }
		}
//-----------------------------------------------------------------------------
	}while (((int)'q'!=ch) && ((int)'Q'!=ch));

CLEAN_UP:
    printf("\nStopping PNIO device...\n\n");
    /* Stop the device */
    ErrorCode = PNIO_device_stop(g_hDevice);
    if(ErrorCode != PNIO_OK) {
        printf("Error in stopping the device. Error# 0x%x\n", ErrorCode);
        return -1;
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
        printf("Error in uninitializing the device. Error# 0x%x\n", ErrorCode);
        return -1;
    }

    g_hDevice = 0;  /* set hDevice back to an invalid value */

    printf("Device uninitalized successfully\n\n");

//________________________________________________________________________   
	printf("press 'q' to quit...\n");
	do {
        ch = getchar();
    }while(((int)'q'!=ch) && ((int)'Q'!=ch));  
//________________________________________________________________________  

    return 0;
}
