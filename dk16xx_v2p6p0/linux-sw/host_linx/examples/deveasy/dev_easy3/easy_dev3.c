/*------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*------------------------------------------------------------------------*/
/*                                                                        */
/*   Project           :                                                  */
/*   Filename          : easy_dev3.c                                      */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
/* Description: This sample program shows how to do the following         */
/*            operations with the PNIO device                             */
/*             1. Initialize and Uninitialize                             */
/*             2. Sending diagnostic alarms                               */
/*             3. Resetting diagnostic alarms                             */
/*             4. Sending process alarms                                  */
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

#define USER_HANDLE             0x12341234 /* User identifier, in case of multiusers this constant
                                              value should be replaced by variable which will give
                                              different user different IDs */
//Maximalwerte f?r Z?hlvariable
#define MAX_ZAEHL 100
#define MAX_ZAEHL1 500

/*------------------------------------DIAGNOSE ALARMS-------------------------------------------------*/
#define CH_ERR_OVERLOAD       0x0004
#define CH_ERR_DATA_TRANS_IMP 0x8000
#define CH_ERR_PS_FAIL        0x0011
#define CH_ERR_MANSPEC        0x0101
#define CH_ERR_GROUND_FAULT   0x0014


/*----------------------------------------------------------------------------------------------------*/
/*    TYPEDEFS                                                                                        */
/*----------------------------------------------------------------------------------------------------*/

/*
User defined data type - used for sending diagnostic alarms.
This structure carries additional information about the alarm data.
*/

typedef struct {
    unsigned short chanNum;
    unsigned short chanProp;
    unsigned short chanErrType;
} diagAlarmData_t;

typedef struct {
    unsigned short chanNum;
    unsigned short chanProp;
    unsigned short chanErrType;
    unsigned short ExtChannelErrType;
    unsigned long ExtChannelAddValue;
} diagAlarmDataExt_t;
/*----------------------------------------------------------------------------------------------------*/
/*    GLOBALS                                                                                         */
/*----------------------------------------------------------------------------------------------------*/
PNIO_UINT32 g_hDevice = 0;              /* global handle for device */
PNIO_UINT16 g_SessionKey = 0;           /* session identifier, this is obtained in application-relation
                                           information callback i.e. PNIO_cbf_ar_info_ind. */
PNIO_UINT16 g_arNumber = 0;             /* application relation number */

#define max_alarm 16
diagAlarmDataExt_t arr_diag_alarm_prop[max_alarm];		/* global array with pointer to alarm properties */


/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :		has_diag_data(...)                              */
/*                                                                  */
/*------------------------------------------------------------------*/
/*  This function is checking if there are diagnostic data for      */
/*                           the id or not.                         */
/*            0 if no diag data for id available                    */
/*            1 if diag data for id available                       */
/*           -1 if id > max array size                              */
/*------------------------------------------------------------------*/
int has_diag_data(PNIO_UINT16 id){
	if(id > max_alarm ) return -1;

	if(arr_diag_alarm_prop[id].chanErrType == 0 
      && arr_diag_alarm_prop[id].chanNum == 0 
      && arr_diag_alarm_prop[id].chanProp == 0
      && arr_diag_alarm_prop[id].ExtChannelAddValue == 0
      && arr_diag_alarm_prop[id].ExtChannelErrType == 0) return 0;
	else return 1;
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :		check_diag_or_ext_diag(...)                       */
/*                                                                  */
/*------------------------------------------------------------------*/
/*  This function is checking if there is diagnostic data for       */
/*                           the id or extended diagnostic data.    */
/*            0 diag                                                */
/*            1 ext diag                                            */
/*           -1 if id > max array size                              */
/*------------------------------------------------------------------*/
int check_diag_or_ext_diag(PNIO_UINT16 id){
	if(id > max_alarm ) return -1;

	if( arr_diag_alarm_prop[id].ExtChannelAddValue == 0
      && arr_diag_alarm_prop[id].ExtChannelErrType == 0) return 0;
	else return 1;
}

/*------------------------------------------------------------------*/
/*                                                                  */
/*  Function :		reset_diag_data()                                 */
/*                                                                  */
/*------------------------------------------------------------------*/
/*  Reset/initialize the array containing the diagnostic data       */
/*            for the diagnostic alarms (id = 0)                    */
/*            Or reset one field with id != 0                       */
/*------------------------------------------------------------------*/
void reset_diag_data(PNIO_UINT16 id ){
	int i = 0;

	if( id == 0 ){
		for(i=0; i<max_alarm; i++){
			arr_diag_alarm_prop[i].chanErrType 	= 0;
			arr_diag_alarm_prop[i].chanNum 		= 0;
			arr_diag_alarm_prop[i].chanProp 	= 0;
      arr_diag_alarm_prop[i].ExtChannelAddValue 	= 0;
			arr_diag_alarm_prop[i].ExtChannelErrType 	= 0;
		}
	} else if( id < max_alarm ){
		arr_diag_alarm_prop[id].chanErrType 	= 0;
		arr_diag_alarm_prop[id].chanNum 		= 0;
		arr_diag_alarm_prop[id].chanProp 		= 0;
    arr_diag_alarm_prop[id].ExtChannelAddValue 	= 0;
		arr_diag_alarm_prop[id].ExtChannelErrType 	= 0;
	}
}



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
static int INDATA_IND_flag = 0;                         /* Callback: erste Daten?bermittlung von IO-Controller*/

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

    printf("## PNIO_cbf_data_write(..., len=%u, Iocs=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iocs, DevHndl, slot_num, subslot_num);

    return PNIO_S_GOOD; /* return local provider status */
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

    printf("## PNIO_cbf_data_read(..., len=%u, Iops=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iops, DevHndl, slot_num, subslot_num);

    return PNIO_S_GOOD; /* consumer status (of local io device) */
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
    g_arNumber = ArNumber;          /* Store the AR number */
    g_SessionKey = SessionKey; /* Store the session key */

    printf ("## AR-INFO_IND new AR from PNIO controller established SessionKey %x\n",
        SessionKey);

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
    int zaehl = 0;
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

/*-----------------------------------------------------------------------------------*/
/*                                                                                   */
/*  PNIO_cbf_req_done (...)                                                          */
/*                                                                                   */
/*-----------------------------------------------------------------------------------*/
/*      This callback is called by the IO Base Device interface, as soon as an alarm */
/*      is sent                                                                      */
/*-----------------------------------------------------------------------------------*/
void PNIO_cbf_req_done(
    PNIO_UINT32  DevHndl,        /* [in] Handle for Multidevice */
    PNIO_UINT32  UserHndl,       /* [in] user defined handle */
    PNIO_UINT32  Status,
    PNIO_ERR_STAT * pPnioState)
{
    printf ("## PNIO_cbf_req_done UserHndl = %x, Status = %x\n",
        UserHndl, Status);
}

/*-----------------------------------------------------------------*/
/*                                                                 */
/*  PNIO_cbf_cp_stop_req ()                                        */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/*                                                                 */
/*  Input:     ---                                                 */
/*  Output:    ---                                                 */
/*-----------------------------------------------------------------*/
void PNIO_cbf_cp_stop_req (PNIO_UINT32 DevHndl)
{
    printf ("## PNIO_cbf_cp_stop_req\n");

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

    /*
    Manual synchronization - The main function calls device-stop function
    and then the main function waits for 'q' to quit - but the message is displayed
    in this callback to achieve synchronization.
    */
    printf("press 'q' to quit...\n");
}

/*-----------------------------------------------------------------*/
/* NOT USED CALLBACKS                                              */
/*-----------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/*                                                                  */
/*  PNIO_cbf_rec_read()                                             */
/*                                                                  */
/*------------------------------------------------------------------*/
/*                                                                  */
/* This callback is called if the Controller wants to read a record */
/*                                                                  */
/*------------------------------------------------------------------*/
void PNIO_cbf_rec_read  (
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
    *pBufLen=0;
    pPnioState->ErrCode=0xde;
    pPnioState->ErrDecode=0x80;
    pPnioState->ErrCode1=9;
    pPnioState->ErrCode2=0;
    pPnioState->AddValue1=0;
    pPnioState->AddValue2=0;
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
void PNIO_cbf_rec_write  (
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
    *pBufLen=0;
    pPnioState->ErrCode=0xdf;
    pPnioState->ErrDecode=0x80;
    pPnioState->ErrCode1=9;
    pPnioState->ErrCode2=0;
    pPnioState->AddValue1=0;
    pPnioState->AddValue2=0;
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
    PNIO_UINT32 ErrorCode;
    PNIO_APPL_READY_LIST_TYPE readyListType;

    /* Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user       */
    /* program can initialize the  incoming data (from the perspective of the IO controller) */
    /* for the functional submodules and set the local status to "GOOD". For all             */
    /* non-functional submodules, the local status should be set to "BAD"                    */
    /* We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback        */
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
/*  Function :      do_after_indata_ind_cbf()                      */
/*                                                                 */
/*-----------------------------------------------------------------*/
/*                                                                 */
/* This function is called after the PNIO_cbf_ar_indata_ind        */
/* has been called ( see main() ), it calls                        */
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

    for(i = 0; i < entries; i++) {
        if((int)mod == idxTbl[i])		//Finden des Modulindex
            break;
    }

    if(i == entries)			//kein Modul gefunden? => auch kein Submodul
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
        g_device_data[0].modId);            /* module 0 identifier */

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

            /*------------------------------------------------------------------*/
			/* set the device state is no longer necessary!   					*/
			/*------------------------------------------------------------------*/
            /*status = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);*/
        }

        if(status == PNIO_OK && g_device_data[i].modState == 1) {
            addr.AddrType       = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
            addr.u.Geo.Slot     = g_device_data[i].slot;
            addr.u.Geo.Subslot  = 1;                         /* dont care */

            /*-----------------------------------------*/
            /*  remove  modules                        */
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

            /*------------------------------------------------------------------*/
			/* set the device state is no longer necessary!   					*/
			/*------------------------------------------------------------------*/
            /*status = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);*/
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
/*  Function :      Uninitialize()                                  */
/*                                                                  */
/*------------------------------------------------------------------*/
/*      This function does the de-initialization of the PNIO device */
/*------------------------------------------------------------------*/
PNIO_UINT32 Uninitialize(void)
{
    return PNIO_device_close(g_hDevice);
}

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

    if(tstVal.by[0] == 0x01) {
        /* higher byte has gone at address 0 - byte ordering is present */
        return val;
    } else {
        /* reverse byte order */
        return (val >> 8) | ((val&0xff) << 8);
    }
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :      sendProcessAlarm()                            */
/*																  */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function sends a process alarm message to the PNIO       */
/*      controller.                                               */
/*----------------------------------------------------------------*/
PNIO_UINT32 SendProcessAlarm(
    PNIO_UINT16 diagTag)  /* diagnostic tag/alarm handle */
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    char ProcessAlarmData[50]; /* dummy data for process alarm */
    int  modIdx = diagTag;
        PNIO_DEV_ADDR   addr; /* Address info of the module */

    printf("Sending a process alarm...\n");

    sprintf(ProcessAlarmData, "Process Alarm Slot %d, Subslot 1", modIdx);

    addr.AddrType       = PNIO_ADDR_GEO;
    addr.u.Geo.Slot     = modIdx;
    addr.u.Geo.Subslot  = 1; /* subslot 1 */


    ErrorCode = PNIO_process_alarm_send(
        g_hDevice,                     /* Device handle */
        g_device_data[modIdx].api,     /* Api number */
        g_arNumber,                    /* received in PNIO_cbf_ar_info_ind */
        g_SessionKey,                  /* received in PNIO_cbf_ar_info_ind */
        &addr,                         /* location (slot, subslot) */
        (PNIO_UINT8*)ProcessAlarmData, /* alarm data */
        sizeof (ProcessAlarmData),     /* alarm data length */
        0x004d,                        /* 0...0x7fff: user struct. is manufac. specific */
        USER_HANDLE);                  /* user handle, to identify user in a multi-user scenario */

    if(ErrorCode != PNIO_OK) {
        printf("Error in sending a process alarm\n");
    }

    return ErrorCode;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		SendDiagnosticAlarm()                         */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function sends a diagnostic alarm message to the PNIO    */
/*      controller.                                               */
/*----------------------------------------------------------------*/
PNIO_UINT32 SendDiagnosticAlarm(
    PNIO_UINT32     channelErrType, /* Type of error - alarm type */
    PNIO_UINT16 diagTag)            /* diagnostic tag/alarm handle */
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    PNIO_UINT16 uiDiagAlarmProp = 0; /* Channel property to be created and passed to the channel add function */
    int modIdx = 1;  /* index of the module */
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

    if(ErrorCode != PNIO_OK) {
        printf("Error in adding a diagnostic channel\n");
        return ErrorCode;
    }

    /*------------------------------------------------------------------*/
	/* set the device state is no longer necessary!   					*/
	/*------------------------------------------------------------------*/
    /*ErrorCode = PNIO_set_dev_state (g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
     *if(ErrorCode != PNIO_OK) {
     *    printf("Error in changing the device state\n");
     *    return ErrorCode;
     *}
     */

    /*------------------------------------------------------------------------------------------------*/
    /* Send diagnosis alarm to the PNIO controller, handle the rest in the callback PNIO_cbf_req_done */
    /*------------------------------------------------------------------------------------------------*/
    diagData.chanProp    = setNWOrder(uiDiagAlarmProp);
    diagData.chanNum     = setNWOrder(0x8000);
    diagData.chanErrType = setNWOrder((unsigned short)diagTag);

    /*----------------------------------------------*/
	/* Copy the alarm properties to reset the alarm */
	/*----------------------------------------------*/
    arr_diag_alarm_prop[diagTag].chanProp	 = diagData.chanProp;
    arr_diag_alarm_prop[diagTag].chanNum	 = diagData.chanNum ;
    arr_diag_alarm_prop[diagTag].chanErrType = diagData.chanErrType;

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
        USER_HANDLE);                          /* user handle, to identify user in multi-user scenario */

    if(ErrorCode != PNIO_OK) {
        printf("Error in sending a diagnostic alarm\n");
    }

    return ErrorCode;
}


/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		SendExtDiagnosticAlarm()                        */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function sends a extended diagnostic alarm message to    */
/*      the PNIO controller.                                      */
/*----------------------------------------------------------------*/
PNIO_UINT32 SendExtDiagnosticAlarm(
    PNIO_UINT32     channelErrType, /* Type of error - alarm type */
    PNIO_UINT16 diagTag)            /* diagnostic tag/alarm handle */
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    PNIO_UINT16 uiDiagAlarmProp = 0; /* Channel property to be created and passed to the channel add function */
    int modIdx = 1;  /* index of the module */
    diagAlarmDataExt_t diagData;        /* extended diagnosis data */

    PNIO_DEV_ADDR   addr;            /* Address info of the module */

    addr.AddrType      = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
    addr.u.Geo.Slot    = modIdx;        /* module index*/
    addr.u.Geo.Subslot = 1;             /* for our test application always 1 */

    printf("Sending a extended diagnostic alarm...\n");

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
    ErrorCode = PNIO_diag_ext_channel_add(
        g_hDevice,                             /* Device handle */
        g_device_data[modIdx].api,             /* Api number */
        &addr,                                 /* Address */
        1,                                     /* subslot number always 1 here */
        uiDiagAlarmProp,                       /* channel properties */
        channelErrType,                        /* errortype - function param */
        0x8000,                                /* ext channel error type */
        0x00000001,                            /* ext channel add value */
        diagTag);                              /* diag tag/ alarm handle */

    if(ErrorCode != PNIO_OK) {
        printf("Error in adding a diagnostic channel\n");
        return ErrorCode;
    }

    /*------------------------------------------------------------------*/
	  /* set the device state is no longer necessary!   					        */
	  /*------------------------------------------------------------------*/
    /*ErrorCode = PNIO_set_dev_state (g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
     *if(ErrorCode != PNIO_OK) {
     *    printf("Error in changing the device state\n");
     *    return ErrorCode;
     *}
     */

    /*------------------------------------------------------------------------------------------------*/
    /* Send diagnosis alarm to the PNIO controller, handle the rest in the callback PNIO_cbf_req_done */
    /*------------------------------------------------------------------------------------------------*/
    diagData.chanProp           = setNWOrder(uiDiagAlarmProp);
    diagData.chanNum            = setNWOrder(0x8000);
    diagData.chanErrType        = setNWOrder((unsigned short)diagTag);
    diagData.ExtChannelErrType  = setNWOrder(0x8000);
    diagData.ExtChannelAddValue = setNWOrder(0x00000001);

    /*----------------------------------------------*/
	  /* Copy the alarm properties to reset the alarm */
	  /*----------------------------------------------*/
    arr_diag_alarm_prop[diagTag].chanProp	 = diagData.chanProp;
    arr_diag_alarm_prop[diagTag].chanNum	 = diagData.chanNum ;
    arr_diag_alarm_prop[diagTag].chanErrType = diagData.chanErrType;
    arr_diag_alarm_prop[diagTag].ExtChannelAddValue = diagData.ExtChannelAddValue;
    arr_diag_alarm_prop[diagTag].ExtChannelErrType = diagData.ExtChannelErrType;

    ErrorCode = PNIO_diag_alarm_send(
        g_hDevice,                             /* Device Handle */
        g_device_data[modIdx].api,             /* Api number */
        g_arNumber,                            /* received in PNIO_cbf_ar_info_ind */
        g_SessionKey,                          /* received in PNIO_cbf_ar_info_ind */
		PNIO_STATE_ALARM_APPEARS,
        &addr,                                 /* location (slot, subslot) */
        (PNIO_UINT8 *)&diagData,               /* Alarm Data */
        sizeof(diagData),                      /* size of the structure sent */
        0x8002,                                /* means, pData contains diagnosis information */
        USER_HANDLE);                          /* user handle, to identify user in multi-user scenario */

    if(ErrorCode != PNIO_OK) {
        printf("Error in sending a extended diagnostic alarm\n");
    }

    return ErrorCode;
}

/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		ResetDiagnosticAlarm()                          */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*                                                                */
/*----------------------------------------------------------------*/
PNIO_UINT32 ResetDiagnosticAlarm(PNIO_UINT16 diagTag)
{
    int i;
    PNIO_UINT32 ErrorCode = PNIO_OK;
    int modIdx = 1;    /* index of the module, assumed 1 */
    int entries = gDevArraySize;

    PNIO_DEV_ADDR   addr;               /* Address info of the module */
    addr.AddrType      = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
    addr.u.Geo.Slot    = modIdx;        /* module index */
    addr.u.Geo.Subslot = 1;             /* for our test application always 1 */

    if(diagTag != 0) printf("Resetting the diagnostic alarm...\n");
    else printf("Resetting the (all) diagnostic and/or maintenance alarm(s)...\n");

    /*--------------------------------------------*/
    /* Storing diagnosis data in a sub-slot       */
    /* and remove diag channel or extdiag channel */
    /*--------------------------------------------*/
    if(diagTag == 0 || !check_diag_or_ext_diag(diagTag) ){
      ErrorCode = PNIO_diag_channel_remove(
          g_hDevice,                             /* Device handle */
          g_device_data[modIdx].api,             /* Api number */
          &addr,                                 /* Address */
          diagTag);                              /* diag tag/ alarm handle */
    } else {
      ErrorCode = PNIO_diag_ext_channel_remove(
          g_hDevice,                             /* Device handle */
          g_device_data[modIdx].api,             /* Api number */
          &addr,                                 /* Address */
          diagTag);                              /* diag tag/ alarm handle */
    }

    if(ErrorCode != PNIO_OK) {
        printf("Error in removing a diagnostic channel\n");
        return ErrorCode;
    }

    /* do this only if all modules / submodules are plugged */
    for(i = 0; i < entries; i++) {
        if(g_device_data[i].subState == 0) {
            break;
        }
    }

    /*------------------------------------------------------------------*/
	  /* set the device state is no longer necessary!   					        */
	  /*------------------------------------------------------------------*/
    /*if(i == entries) {												*/
        /*--------------------------------------------------------------*/
        /* set the device state to PNIO_DEVSTAT_OK       				*/
        /*--------------------------------------------------------------*/
    /*    ErrorCode = PNIO_set_dev_state (g_hDevice, PNIO_DEVSTAT_OK);	*/
    /*    if(ErrorCode != PNIO_OK) {									*/
    /*        printf("Error in changing the device state\n");			*/
    /*       return ErrorCode;											*/
    /*    }																*/


    /*-----------------------------------------------*/
    /* to reset all alarms alarm data must be NULL,  */
    /* for a special alarm alarm data has to be used!*/
    /*-----------------------------------------------*/
    if( diagTag == 0) {
		ErrorCode = PNIO_diag_alarm_send(
			g_hDevice,                             /* Device Handle */
			g_device_data[modIdx].api,             /* Api number */
			g_arNumber,                            /* received in PNIO_cbf_ar_info_ind */
			g_SessionKey,                          /* received in PNIO_cbf_ar_info_ind */
			PNIO_STATE_ALARM_DISAPPEARS,
			&addr,
			NULL,                                  /* Alarm Data */
			0,                                     /* size of the structure sent */
			0x8001,                                /* means that pData contains diagnosis information */
			USER_HANDLE);                          /* user handle, to identify user in multi-user scenario */

		reset_diag_data(0);
    } else {
		ErrorCode = PNIO_diag_alarm_send(
			g_hDevice,                            		 	/* Device Handle */
			g_device_data[modIdx].api,             			/* Api number */
			g_arNumber,                            			/* received in PNIO_cbf_ar_info_ind */
			g_SessionKey,                          			/* received in PNIO_cbf_ar_info_ind */
			PNIO_STATE_ALARM_DISAPPEARS,
			&addr,
			(PNIO_UINT8 *)& arr_diag_alarm_prop[diagTag],   /* Alarm Data */
			0,                                     			/* size of the structure sent */
			0x8001,                                			/* means that pData contains diagnosis information */
			USER_HANDLE);                          			/* user handle, to identify user in multi-user scenario */
		reset_diag_data(diagTag);
    }
    if(ErrorCode != PNIO_OK) {
        printf("Error in resetting a diagnostic alarm\n");
    }

    return ErrorCode;
}


/*----------------------------------------------------------------*/
/*                                                                */
/*  Function :		SendMaintenanceAlarm()                         */
/*                                                                */
/*----------------------------------------------------------------*/
/*                                                                */
/*  This function sends a maintenance alarm message to the PNIO   */
/*      controller.                                               */
/*----------------------------------------------------------------*/
PNIO_UINT32 SendMaintenanceAlarm(
    PNIO_UINT32     channelErrType, /* Type of error - alarm type */
    PNIO_UINT16 diagTag,            /* diagnostic tag/alarm handle */
    PNIO_UINT8 maintRequired)             /* 1 = Maintenance required alarm, 0 = Maintenance demanded alarm */
{
    PNIO_UINT32 ErrorCode = PNIO_OK;
    PNIO_UINT16 uiDiagAlarmProp = 0; /* Channel property to be created and passed to the channel add function */
    int modIdx = 1;  /* index of the module */
    diagAlarmData_t diagData;        /* diagnosis data */

    PNIO_DEV_ADDR   addr;            /* Address info of the module */

    addr.AddrType      = PNIO_ADDR_GEO; /* must be PNIO_ADDR_GEO */
    addr.u.Geo.Slot    = modIdx;        /* module index*/
    addr.u.Geo.Subslot = 1;             /* for our test application always 1 */

    if(maintRequired)
      printf("Sending a maintenance required alarm...\n");
    else
      printf("Sending a maintenance demanded alarm...\n");

    /*--------------------------------------*/
    /* Build channel properties             */
    /*--------------------------------------*/
    uiDiagAlarmProp = PNIO_build_channel_properties(
        g_hDevice,                             /* Device handle */
        PNIO_DIAG_CHANPROP_TYPE_SUBMOD,        /* for our dev. always this  */
        PNIO_DIAG_CHAN_SPEC_ERROR_APPEARS,     /* Der Fehler kommt (aus Sicht des IO-Controllers)*/
        PNIO_DIAG_CHAN_DIRECTION_MANUFACTURE); /* 0: = specific */

    /*----------------------------------------------------------------------------*/
    /* IMPORTANT: Add this for maintenance required or demanded alarm!!!          */
    /*----------------------------------------------------------------------------*/
    if(maintRequired)
      uiDiagAlarmProp |= PNIO_CHAN_PROP_MAINT_REQUIRED_BIT;
    else
      uiDiagAlarmProp |= PNIO_CHAN_PROP_MAINT_DEMANDED_BIT;

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

    if(ErrorCode != PNIO_OK) {
        printf("Error in adding a maintenance diagnostic channel\n");
        return ErrorCode;
    }

    /*------------------------------------------------------------------*/
	/* set the device state is no longer necessary!   					*/
	/*------------------------------------------------------------------*/
    /*
     * ErrorCode = PNIO_set_dev_state (g_hDevice, PNIO_DEVSTAT_STATION_PROBLEM);
     * if(ErrorCode != PNIO_OK) {
     *    printf("Error in changing the device state\n");
     *    return ErrorCode;
     * }
   	 */

    /*------------------------------------------------------------------------------------------------*/
    /* Send maintenance alarm to the PNIO controller, handle the rest in the callback PNIO_cbf_req_done */
    /*------------------------------------------------------------------------------------------------*/
    diagData.chanProp    = setNWOrder(uiDiagAlarmProp);
    diagData.chanNum     = setNWOrder(0x8000);
    diagData.chanErrType = setNWOrder((unsigned short)diagTag);

    /*----------------------------------------------*/
	/* Copy the alarm properties to reset the alarm */
	/*----------------------------------------------*/
    arr_diag_alarm_prop[diagTag].chanProp	 = diagData.chanProp;
    arr_diag_alarm_prop[diagTag].chanNum	 = diagData.chanNum ;
    arr_diag_alarm_prop[diagTag].chanErrType = diagData.chanErrType;

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
        USER_HANDLE);                          /* user handle, to identify user in multi-user scenario */

    if(ErrorCode != PNIO_OK) {
        printf("Error in sending a diagnostic alarm\n");
    }

    return ErrorCode;
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
    int ch = 0;
	int zaehl = 0;
	int choice=0;
    /*----------------------------------------------------------------*/
    /*                 Program Menu                                   */
    /*----------------------------------------------------------------*/

    printf("\n-----------------------------------------------------------------------------\n");
    printf("\n------\t\t\t PNIO Device Sample Program 3 \t\t\t-----\n");
    printf("\n-----------------------------------------------------------------------------\n");

    printf("This sample program shows how to do the following operations with the\n");
    printf("PNIO device:\n");
    printf("\t1. How to initialize PNIO device\n");
    printf("\t2. How set and reset diagnostic alarms\n");
    printf("\t3. How to set process alarms\n");
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

    /*---------------------------------------------*/
	/* set the diag alarm properties array to NULL */
	/*---------------------------------------------*/
    reset_diag_data(0);

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

    /*------------------------------------------------------------------*/
	/* set the device state is no longer necessary!   					*/
	/*------------------------------------------------------------------*/
	/*
	 *  printf("\nSetting device state OK...\n");
     *	ErrorCode = PNIO_set_dev_state(g_hDevice, PNIO_DEVSTAT_OK);
     *  if(ErrorCode != PNIO_OK) {
     *    printf("Error in setting device state OK. Error# 0x%x\n", ErrorCode);
     *    goto CLEAN_UP;
     *  }
     */

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
        Sleep(100);
		zaehl++;
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

    printf("\nAll necessary callbacks called...\n");

    /* loop to wait for the user input */

	 do {

        choice=0;
		printf("\nSelect the alarm type -\n");
        printf("\t1. Set/Reset Diagnostic alarm - OVERLOAD\n");
        printf("\t2. Set/Reset Diagnostic alarm - PS FAIL\n");
        printf("\t3. Set/Reset Diagnostic alarm - MANUFACTURE SPECIFIC\n");
        printf("\t4. Set/Reset Extended Diagnostic alarm - DATA TRANSMISSION\n");
        printf("\t5. Send Process alarm\n");
        printf("\t6. Set/Reset Maintenance Required alarm - GROUND FAULT\n");
        printf("\t7. Set/Reset Maintenance Demanded alarm - GROUND FAULT\n");
        printf("\t8. Reset all Diagnostic/Maintenance alarms\n");
        printf("\n\t0. exit\n");

        scanf("%d",&choice);

        switch(choice) {
            /*----------------------------------*/
            /* Send diagnostic alarm            */
            /*----------------------------------*/
        case 1:
            if( !has_diag_data(1) ) ErrorCode = SendDiagnosticAlarm(CH_ERR_OVERLOAD, 1);
            else ErrorCode = ResetDiagnosticAlarm(1);
            break;
        case 2:
        	if( !has_diag_data(2) ) ErrorCode = SendDiagnosticAlarm(CH_ERR_PS_FAIL, 2);
        	else ErrorCode = ResetDiagnosticAlarm(2);
            break;
        case 3:
        	if( !has_diag_data(3) ) ErrorCode = SendDiagnosticAlarm(CH_ERR_MANSPEC, 3);
        	else ErrorCode = ResetDiagnosticAlarm(3);
            break;
            /*-------------------------------------------*/
            /* Send extended diagnostic alarm            */
            /*-------------------------------------------*/
        case 4:
        	if( !has_diag_data(4) ) ErrorCode = SendExtDiagnosticAlarm(CH_ERR_DATA_TRANS_IMP, 4);
        	else ErrorCode = ResetDiagnosticAlarm(4);
            break;
        case 5:
            /*----------------------------------*/
            /* Send  process alarm              */
            /*----------------------------------*/
            ErrorCode = SendProcessAlarm(1);
            break;
        case 6:
            /*----------------------------------*/
            /* Send  maintenance required alarm              */
            /*----------------------------------*/
        	if( !has_diag_data(10) ) ErrorCode = SendMaintenanceAlarm(CH_ERR_GROUND_FAULT,10,1);
        	else ErrorCode = ResetDiagnosticAlarm(10);
            break;
        case 7:
            /*----------------------------------*/
            /* Send  maintenance demanded alarm              */
            /*----------------------------------*/
        	if( !has_diag_data(11) ) ErrorCode = SendMaintenanceAlarm(CH_ERR_GROUND_FAULT,11,0);
        	else ErrorCode = ResetDiagnosticAlarm(11);
            break;
        case 8:
            /*----------------------------------*/
            /* Reset every alarm           */
            /*----------------------------------*/
            ErrorCode = ResetDiagnosticAlarm(0);
            break;
		case (int)'q':
			goto CLEAN_UP;
		default: goto CLEAN_UP;
        }

		if( ErrorCode != PNIO_OK)
		    printf("      Error# 0x%x\n", ErrorCode);
    }while(choice != 0);

CLEAN_UP:
    printf("\nStopping PNIO device...\n\n");

    /* Stop the device */
    ErrorCode = PNIO_device_stop(g_hDevice);
    if(ErrorCode != PNIO_OK) {
        printf("Error in stopping the device. Error# 0x%x\n", ErrorCode);
        return -1;
    }

    /*----------------------------------------------*/
    /* Callback synchronization                     */
    /*----------------------------------------------*/

    /*
    after stopping the device the callback 'PNIO_cbf_device_stopped' is called,
    in this callback we ask the user to press 'q' to quit
    */
     do {
        ch = getchar();
    }while(((int)'q'!=ch) && ((int)'Q'!=ch));

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

    return 0;
}
