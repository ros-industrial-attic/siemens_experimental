/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           :                                                     */
/*   Filename          : pnioeasy3.cpp                                       */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:                                                            */
/*               This is a sample application to demonstrate the IO-Base     */
/*               user interface. This sample does following steps            */
/*                   1. Initialize (with the use of change mode indication   */
/*                                  callback)                                */
/*                   2. Aktivieren und Deaktivieren des ersten IO-Ger�tes    */
/*                      by pressing 'a' and 'd'                              */
/*                   3. Printig alarm information                            */
/*                   4. Do some IO Transfer                                  */
/*                   5. Stop device                                          */
/*                   6. uninitialize                                         */
/*---------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all      */
/*             printf statements should be synchronized. But this sample     */
/*             application doesn't synchronize for simplicity  !             */
/*                                                                           */
/*             For proper display performance use a console width of more    */
/*             than 130 characters                                           */
/*                                                                           */
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
#include <sys/timeb.h>
#define CRLF "\n"
#else
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <sys/timeb.h>
#define Sleep(x) usleep(x*1000)
#define CRLF "\r\n"
#endif


#include "pniousrx.h"
#include "pnioerrx.h"

/*------------------------------------------------------------------------*/
/* Global Data for Communication Processor                                */
/*------------------------------------------------------------------------*/

/* Index of CP given by project */
PNIO_UINT32 g_dwCpId = 1;


/* Don't forget! Callbacks are running in concurrent threads !            */
/* 'volatile PNIO_MODE_TYPE g_currentMode' statement is                   */
/* required to force the compiler to generate unoptimized memory access   */
/* to 'g_currentMode'.                                                    */
/*                                                                        */
/* Example                                                                */
/*  while(g_currentMode == PNIO_MODE_OFFLINE){...};                       */
/* will not be optimized                                                  */


volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;
const char roll[]={'-','\\','|','/','-','\\','|','/'};

#define SWAP_16(var)                  \
(  ((( (var)) & 0xFF00L) >>  8)   \
+ ((( (var)) & 0x00FFL) << 8))

#define SWAP_32(var)                     \
(  ((( (var)) & 0xFF000000L) >> 24)  \
+ ((( (var)) & 0x00FF0000L) >>  8)   \
+ ((( (var)) & 0x0000FF00L) <<  8)   \
+ ((( (var)) & 0x000000FFL) << 24))

volatile bool cp_stop_req = false;
/*------------------------------------------------------------------------*/
/* Global Data for Device                                                 */
/*------------------------------------------------------------------------*/


volatile PNIO_IOXS g_localState = PNIO_S_GOOD;

#define DEVICE_INPUT_COUNT 2       // number of input modules

volatile PNIO_IOXS g_deviceInputState[DEVICE_INPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceInputAddress[DEVICE_INPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_IN, 0}, // input address of first  input module
    {PNIO_ADDR_LOG, PNIO_IO_IN, 1}  // input address of second input module
};

PNIO_UINT32 g_deviceInputLength[DEVICE_INPUT_COUNT] = {
    1,                          // length in bytes of first input module
    1                           // length in bytes of second input module
};
#define MAX_INPUT_LEN 1   //length of largest input module
PNIO_UINT8 g_deviceInputData[DEVICE_INPUT_COUNT][MAX_INPUT_LEN]={0};
//array with two dimensions in case of modules with more than 1 Byte data ( not necessary in this example)

#define DEVICE_OUTPUT_COUNT  2      // number of output modules
volatile PNIO_IOXS g_deviceOutputState[DEVICE_OUTPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceOutputAddress[DEVICE_OUTPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 0}, // output address of first  output module
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 1}  // output address of second  output module
};

PNIO_UINT32 g_deviceOutputLength[DEVICE_OUTPUT_COUNT] = {
    1,                          // length in byte of first output module
    1                           // length in byte of second output module
};
#define MAX_OUTPUT_LEN 1   //length of largest output module
PNIO_UINT8 g_deviceOutputData[DEVICE_OUTPUT_COUNT][MAX_OUTPUT_LEN]={0};  


// for dataset records
#define RECORD_INDEX 0xE000
#define RECORD_INDEX_LENGTH 0x100



/*-----------------------------------------------------------------------*/
/* mandatory callbacks                                                   */
/*-----------------------------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm);

/*-----------------------------------------------------------------------*/
/* mandatory callbacks                                                   */
/* but not used in this sample application                               */
/* da hier keine Datens�tze geschrieben werden                           */
/*-----------------------------------------------------------------------*/

void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm);

/*-----------------------------------------------------------------------*/
/* useful but not mandatory callbacks                                    */
/*-----------------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm);
void callback_for_device_activation(PNIO_CBE_PRM * pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm);
void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm);


/*------------------------------------------------------------------------*/
/* forward declaration of helper functions                                */
/*------------------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32 *pdwHandle);
void UnInitialize(PNIO_UINT32 dwHandle);
void UpdateCyclicOutputData(PNIO_UINT32 dwHandle);
void UpdateCyclicInputData(PNIO_UINT32 dwHandle);
int getCharWithTimeout(void);
void GetSystemTimeString(char *tstring);





/*------------------------------------------------------------------------*/
/* Here starts the Application Code                                       */
/*------------------------------------------------------------------------*/

int main(void)
{

    PNIO_UINT32 dwHandle = 0;
    int key = 0;
    int r;
    char time_strs[13];
    PNIO_IOXS tmpOutState[2] = {PNIO_S_BAD, PNIO_S_BAD},tmpInState[2]={PNIO_S_BAD, PNIO_S_BAD};



    printf("This sample application does following tasks" CRLF);
    printf("1.increases/decreases the value of the first/second output byte once per 100 milliseconds" CRLF);
    printf("2.reads input byte once per 100 milliseconds and displays it on screen" CRLF);
    printf("3.writes all called callbacks like alarm callback on screen" CRLF);
    printf("  (e.g. (un)plugging of modules" CRLF);
    printf("4.if you press key 'd' first device will be deactivated" CRLF);
    printf("5.if you press key 'a' first device will be activated" CRLF);
    printf("6.if you press key 'r' read data record from first module" CRLF);

    printf(CRLF CRLF" Press 's' to start sample application" CRLF);
    printf("To stop sample application press 'q'." CRLF);

    do {
        key = getCharWithTimeout();
        if(key == (int)'q' || key == (int)'Q')
            return 0;
    } while(key != (int)'s' && key != (int)'S');

    r=0;
    

    if(PNIO_OK != Initialize(g_dwCpId, &dwHandle))
        return 0;


    do {
        g_deviceOutputData[0][0]++;    /* Increase Output Data byte 0 of device #1 */
        g_deviceOutputData[1][0]--;    /* Decrease Output Data byte 0 of device #2 */
        

        
        UpdateCyclicOutputData(dwHandle);
        UpdateCyclicInputData(dwHandle);
    

        if((tmpOutState[0]!=g_deviceOutputState[0])
            ||(tmpInState[0]!=g_deviceInputState[0])
            ||(tmpOutState[1]!=g_deviceOutputState[1])
            ||(tmpInState[1]!=g_deviceInputState[1]))
        {
            GetSystemTimeString(time_strs);
            printf(CRLF"PNIO state change at %s"CRLF,time_strs);
             /* in case of any state change begin new line */
        }

        printf("%c out0:0x%02X state: %s out1:0x%02X state: %s   in0:0x%02X state: %s in1:0x%02X state: %s \r",
                roll[r++],
                g_deviceOutputData[0][0],
                ((g_deviceOutputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceOutputData[1][0],
                ((g_deviceOutputState[1] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceInputData[0][0],
                ((g_deviceInputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceInputData[1][0],
                ((g_deviceInputState[1] == PNIO_S_GOOD) ? "good" : "bad "));


        fflush(stdout);
        if(r==sizeof(roll))
            r=0;
        tmpOutState[0]=g_deviceOutputState[0];
        tmpInState[0]=g_deviceInputState[0];
        tmpOutState[1]=g_deviceOutputState[1];
        tmpInState[1]=g_deviceInputState[1];

        key = getCharWithTimeout();

        switch (key) {
        case 'd':
            PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_FALSE);
            key = ' ';
            break;
        case 'a':
            PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_TRUE);
            key = ' ';
            break;
        case 'r':
            /* Reading record data */
            PNIO_rec_read_req(dwHandle, &g_deviceInputAddress[1], /* Address of device */
                RECORD_INDEX,                 /* Expexted Configuration                */
                0,                            /* userdefined Handle (not used here)    */
                RECORD_INDEX_LENGTH);         /* maximum bytes to read                 */

            key = ' ';
            break;
        }

    } while(key != (int)'q' && key != (int)'Q' && !cp_stop_req );


    UnInitialize(dwHandle);

    exit (0);
}

/*-------------------------------------------------------------*/
/* this function sets operational mode                         */
/*                                                             */
/* do not call before Initialize was called successfully       */
/* because it needs PNIO_CBE_MODE_IND callback to be registered*/
/* returns error code                                          */
/*-------------------------------------------------------------*/

PNIO_UINT32 ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
    PNIO_UINT32 dwErrorCode;

    /*setting  mode asynchronously                                 */
    dwErrorCode = PNIO_set_mode(dwHandle, mode);

    if(dwErrorCode != PNIO_OK) {
        printf(CRLF" Error in ChangeAndWaitForPnioMode " CRLF);
        printf(CRLF" PNIO_set_mode returned 0x%x" CRLF, dwErrorCode);
        PNIO_close(dwHandle);
        return(dwErrorCode);                // return
    };

    /* wait for callback_for_mode_change_indication to be called. */
    /* callback_for_mode_change_indication sets g_currentMode     */
    printf(CRLF"waiting for changing operation mode" CRLF);
    while(g_currentMode != mode) {
        Sleep(100);
        printf(".");
    };

    printf("" CRLF);

    return(PNIO_OK);
}

/*----------------------------------------------------------------------*/
/* this function initializes the IO-BASE and returns a handle           */
/* by reference necessary for all subsequent calls to IO-BASE functions */
/* return value is PNIO error code                                      */
/*----------------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32* pdwHandle)
{

   
    PNIO_UINT32 dwErrorCode = PNIO_OK;

    /* Connect to Communication Processor and obtain a handle */

    dwErrorCode = PNIO_controller_open(
             /*in*/  CP_INDEX,                     /* index of communication processor      */
             /*in*/  PNIO_CEP_MODE_CTRL,           /* permission to change operation mode   */
             /*in*/  callback_for_ds_read_conf,    /* mandatory  callback                   */
             /*in*/  callback_for_ds_write_conf,   /* mandatory callback                    */
             /*in*/  callback_for_alarm_indication,/* alarm callback                        */
             /*out*/ pdwHandle                     /* handle                                */
        );
    printf(" Device Handle dwHandle = %d " CRLF, *pdwHandle);
    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_controller_open returned 0x%x" CRLF, dwErrorCode);
        return(dwErrorCode);                // return
    };

    /* here we register the callback                         */
    /* PNIO_CBE_MODE_IND    for Mode changes  confirmation   */
    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_MODE_IND,
        /*in */ callback_for_mode_change_indication
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_MODE_IND,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return
    }

    /* here we register the callback                           */
    /* PNIO_CBE_DEV_ACT     for device activation confirmation */

    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_DEV_ACT_CONF,
        /*in */ callback_for_device_activation
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_DEV_ACT_CONF,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return
    }

    /* here we register the callback                               */
    /* PNIO_CBE_CP_STOP_REQ     for device activation confirmation */

    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_CP_STOP_REQ,
        /*in */ callback_for_cp_stop_req
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_CP_STOP_REQ,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return



    }



    /* here we change the mode to PNIO_MODE_OPERATE  */
    return(ChangeAndWaitForPnioMode(*pdwHandle, PNIO_MODE_OPERATE));

}



/*-------------------------------------------------------------*/
/* this function uninitializes the IO-BASE                     */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/

void UnInitialize(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode = PNIO_OK;

    dwErrorCode = ChangeAndWaitForPnioMode(dwHandle, PNIO_MODE_OFFLINE);
    if(dwErrorCode != PNIO_OK) {
        return;
    }
    dwErrorCode = PNIO_close(dwHandle);

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in UnInitialize " CRLF);
        printf(" PNIO_close returned 0x%x" CRLF, dwErrorCode);
    };
}


/*-------------------------------------------------------------*/
/* this function writes the cyclic output data to IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/

void UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    for(int i = 0; i < DEVICE_OUTPUT_COUNT; i++) {
        dwErrorCode = PNIO_data_write(
                       /*in*/ dwHandle,                             /* handle                            */
                       /*in*/ &(g_deviceOutputAddress[i]),          /* pointer to device output address */
                       /*in*/ g_deviceOutputLength[i],              /* length in bytes of output        */
                       /*in*/ &(g_deviceOutputData[i][0]),          /* pointer to output data           */
                       /*in*/ g_localState,                         /* local status                     */
                       /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i]) /* remote status                    */
            );
    //if(dwErrorCode != PNIO_OK)
        //printf(CRLF"Error 0x%X in PNIO_data_write() Device %d "CRLF,dwErrorCode,i);
    }
}

/*-------------------------------------------------------------*/
/* this function reads the cyclic input data from IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/
void UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    PNIO_UINT32 dwBytesReaded;
    for(int i = 0; i < DEVICE_INPUT_COUNT; i++) {
        dwErrorCode = PNIO_data_read(
                       /*in*/  dwHandle,                           /* handle                           */
                       /*in*/  &g_deviceInputAddress[i],           /* pointer to device input address */
                       /*in*/  g_deviceInputLength[i],             /* length in bytes of input        */
                       /*out*/ &dwBytesReaded,                     /* number of bytes read            */
                       /*in*/  &g_deviceInputData[i][0],           /* pointer to input data           */
                       /*in*/  g_localState,                       /* local status                    */
                       /*out*/(PNIO_IOXS*)&(g_deviceInputState[i]) /* remote status                   */
                       );
    //if(dwErrorCode != PNIO_OK)
        //printf(CRLF"Error 0x%X in PNIO_data_read() Device %d "CRLF,dwErrorCode,i);   
    }
}


/*--------------------------------------------------*/
/* mandatory callback                               */
/*--------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    printf( CRLF"callback_for_ds_read_conf " CRLF);

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_REC_READ_CONF) {
        printf("Record read returned errorcode %x" CRLF, pCbfPrm->RecReadConf.Err);
        if(pCbfPrm->RecReadConf.Err.ErrCode == 0) {
            printf("Record read returned %x bytes " CRLF, pCbfPrm->RecReadConf.Length);
            printf("Record read returned recordIndex %x  " CRLF,
                pCbfPrm->RecReadConf.RecordIndex);
            printf("Record read has read form adress %x  " CRLF,
                pCbfPrm->RecReadConf.pAddr->u.Addr);
            printf("Record read has returned :" CRLF);

            for(unsigned int i = 0; i < pCbfPrm->RecReadConf.Length; i++) {
                if(i % 16 == 0)
                    printf("" CRLF);
                printf("%2x ", pCbfPrm->RecReadConf.pBuffer[i]);
            }
            printf("" CRLF);
        }


    }
}

/*--------------------------------------------------*/
/* mandatory callback but not used in this sample   */
/*--------------------------------------------------*/

void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/
    printf( CRLF"callback_for_ds_read_conf " CRLF);
    printf("this callback must not occur in this sample application" CRLF);
}

/*--------------------------------------------------*/
/* useful callbacks                                 */
/*--------------------------------------------------*/


/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal a change*/
/* in the opreation mode                                       */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm)
{

    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_MODE_IND) {
        /* Callback has correct type so check mode change */
        /* and set global variable                        */
        switch (pCbfPrm->ModeInd.Mode) {
        case PNIO_MODE_OFFLINE:
            g_currentMode = PNIO_MODE_OFFLINE;
            break;
        case PNIO_MODE_CLEAR:
            g_currentMode = PNIO_MODE_CLEAR;
            break;
        case PNIO_MODE_OPERATE:
            g_currentMode = PNIO_MODE_OPERATE;
            break;
        default:
            printf("callback_for_mode_change_indication called with wrong mode" CRLF);
            break;
        };
    };

    printf("callback_for_mode_change_indication was called" CRLF);
}


/*-------------------------------------------------------------*/
/* this function will help to print alarm-callbacks           */
/*                                                             */
/* parameters                                                  */
/*    adata         : AlarmData				                   */
/*-------------------------------------------------------------*/
#define VORSATZ "             "

const char *MyPNIO_get_ctrl_alarmtype_description(PNIO_ALARM_TYPE type)
{
    static char tmp[200];
    switch(type) {
    case PNIO_ALARM_DIAGNOSTIC:
            return "PNIO_ALARM_DIAGNOSTIC";
    case PNIO_ALARM_PROCESS:
            return "PNIO_ALARM_PROCESS";
    case PNIO_ALARM_PULL:
            return "PNIO_ALARM_PULL";
    case PNIO_ALARM_PLUG:
            return "PNIO_ALARM_PLUG";
    case PNIO_ALARM_STATUS:
            return "PNIO_ALARM_STATUS";
    case PNIO_ALARM_UPDATE:
            return "PNIO_ALARM_UPDATE";
    case PNIO_ALARM_REDUNDANCY:
            return "PNIO_ALARM_REDUNDANCY";
    case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
            return "PNIO_ALARM_CONTROLLED_BY_SUPERVISOR";
    case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
            return "PNIO_ALARM_RELEASED_BY_SUPERVISOR";
    case PNIO_ALARM_PLUG_WRONG:
            return "PNIO_ALARM_PLUG_WRONG";
    case PNIO_ALARM_RETURN_OF_SUBMODULE:
            return "PNIO_ALARM_RETURN_OF_SUBMODULE";
    case PNIO_ALARM_DIAGNOSTIC_DISAPPEARS:
            return "PNIO_ALARM_DIAGNOSTIC_DISAPPEARS";
    case PNIO_ALARM_MCR_MISMATCH:
            return "PNIO_ALARM_MCR_MISMATCH";
    case PNIO_ALARM_PORT_DATA_CHANGED:
            return "PNIO_ALARM_PORT_DATA_CHANGED";
    case PNIO_ALARM_SYNC_DATA_CHANGED:
            return "PNIO_ALARM_SYNC_DATA_CHANGED";
    case PNIO_ALARM_ISOCHRONE_MODE_PROBLEM:
            return "PNIO_ALARM_ISOCHRONE_MODE_PROBLEM";
    case PNIO_ALARM_NETWORK_COMPONENT_PROBLEM:
            return "PNIO_ALARM_NETWORK_COMPONENT_PROBLEM";
    case PNIO_ALARM_UPLOAD_AND_STORAGE:
            return "PNIO_ALARM_UPLOAD_AND_STORAGE";
    case PNIO_ALARM_PULL_MODULE:
            return "PNIO_ALARM_PULL_MODULE";
    case PNIO_ALARM_DEV_FAILURE:
            return "PNIO_ALARM_DEV_FAILURE";
    case PNIO_ALARM_DEV_RETURN:
            return "PNIO_ALARM_DEV_RETURN";
    default:
            sprintf(tmp, "%d, unknown alarm", type);
            return tmp;
    }
}

const char *MyPNIO_get_ctrl_alarmprio_description(PNIO_APRIO_TYPE type)
{
    static char tmp[200];
    switch(type) {
    case PNIO_APRIO_LOW:
        return "PNIO_APRIO_LOW";
    case PNIO_APRIO_HIGH:
        return "PNIO_APRIO_HIGH";
    default:
        sprintf(tmp, "%d, unknown alarm prio", type);
        return tmp;
    }
}


void hex_dump (const char *prefix, const PNIO_UINT8 *data, unsigned long number, unsigned long oneline=16)
{
        unsigned long i, j;
        unsigned long rows = (number / oneline) + 1;
        unsigned long onelinelen = (unsigned long)strlen(prefix) + oneline * 3 + 1;
        unsigned long tmplen = onelinelen * rows;

        char *tmpbuf = new char[tmplen];  //-V121

        if(!tmpbuf) {
                printf ("%s", prefix);
                for (i = 0, j = 0; i < number; i++, j++) {
                        if (j >= oneline) {
                                printf ("\n%s", prefix);
                                j = 0;
                        }
                        printf (" %02x", data[i]);
                }
                printf( CRLF);
        } else {
                char tmp2[15];
                strcpy(tmpbuf, prefix);
                for (i = 0, j = 0; i < number; i++, j++) {
                        if (j >= oneline) {
                                strcat(tmpbuf, CRLF);
                                strcat(tmpbuf, prefix);
                                j = 0;
                        }
                        sprintf(tmp2, " %02x", data[i]);
                        strcat(tmpbuf, tmp2);
                }
                strcat(tmpbuf, CRLF);
                printf(tmpbuf);
                delete [] tmpbuf;
        }
}

static void alarm_dump(const PNIO_CTRL_ALARM_DATA *adata)
{
        printf(VORSATZ "alarm: AlarmType=%s(%u), AlarmPriority=%s(%u), DeviceNum=%u" CRLF,
                MyPNIO_get_ctrl_alarmtype_description(adata->AlarmType), adata->AlarmType,
                MyPNIO_get_ctrl_alarmprio_description(adata->AlarmPriority), adata->AlarmPriority,
                adata->DeviceNum);
        printf(VORSATZ "alarm: SlotNum=%u(0x%04x), SubslotNum=%u(0x%04x)" CRLF,
               adata->SlotNum, adata->SlotNum, adata->SubslotNum, adata->SubslotNum);


		printf(VORSATZ "alarm: CpuAlarmPriority=%u, PnioCompatModtype=0x%x" CRLF,
				adata->CpuAlarmPriority,
				adata->PnioCompatModtype);

		printf(VORSATZ "alarm, AlarmTinfo: CompatDevGeoaddr=%u, ProfileType=%u, AinfoType=%u, ControllerFlags=%u" CRLF,
				adata->AlarmTinfo.CompatDevGeoaddr,
				adata->AlarmTinfo.ProfileType,
				adata->AlarmTinfo.AinfoType,
				adata->AlarmTinfo.ControllerFlags);
		printf(VORSATZ "alarm, AlarmTinfo: DeviceFlag=%u, PnioVendorIdent=0x%04x, PnioDevIdent=0x%04x, PnioDevInstance=%u" CRLF,
				adata->AlarmTinfo.DeviceFlag,
				adata->AlarmTinfo.PnioVendorIdent,
				adata->AlarmTinfo.PnioDevIdent,
				adata->AlarmTinfo.PnioDevInstance);

		hex_dump(VORSATZ "alarm: DiagDs      -> ", adata->DiagDs, 4);
		hex_dump(VORSATZ "alarm: PrAlarmInfo -> ", adata->PrAlarmInfo, 4);

		printf(VORSATZ "alarm, AlarmAinfo: BlockType=%u, BlockVersion=%u, Api=%u" CRLF,
				adata->AlarmAinfo.BlockType,
				adata->AlarmAinfo.BlockVersion,
				adata->AlarmAinfo.Api);
		printf(VORSATZ "alarm, AlarmAinfo: AlarmSpecifier=%u, ModIdent=0x%x, SubIdent=0x%x" CRLF,
				adata->AlarmAinfo.AlarmSpecifier,
				adata->AlarmAinfo.ModIdent,
				adata->AlarmAinfo.SubIdent);

		if(adata->AlarmAinfo.UserAlarmDataLen) {
				printf(VORSATZ "alarm, AlarmAinfo: UserStrucIdent=%u, UserAlarmDataLen=%u" CRLF,
					adata->AlarmAinfo.UserStrucIdent,
					adata->AlarmAinfo.UserAlarmDataLen);

				hex_dump(VORSATZ "alarm AlarmAinfo: ", adata->AlarmAinfo.UAData.UserAlarmData , adata->AlarmAinfo.UserAlarmDataLen);
		} else {
				printf(VORSATZ "alarm, AlarmAinfo: UserAlarmDataLen=0" CRLF);
		}

		printf(VORSATZ "geoaddr: ?.%u.%u.%u.?\n",
				adata->DeviceNum, adata->SlotNum, adata->SubslotNum);

}


/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* a alarm has been received                                   */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/

void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm)
{
    char time_str[13];
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_ALARM_IND) {
    	GetSystemTimeString(time_str);

        switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType) {
        case PNIO_ALARM_DIAGNOSTIC:

        	/********************************************************************/
			/* Check what kind of diagnostic alarm                              */
        	/*   UserStrucIdent = user structure identifier for user alarm data */
			/********************************************************************/
        	switch(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UserStrucIdent) {
        		case 0x8000:	/* 0x8000 :  diagnosis alarm                             */
        			printf(CRLF"PNIO_ALARM_DIAGNOSTIC -> diagnosis alarm at %s" CRLF, time_str);

        			printf("\tChannelNumber    = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.diag.ChannelNumber));
        			printf("\tChannelProperties = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.diag.ChannelProperties));
        			printf("\tChannelErrorType  = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.diag.ChannelErrorType));

        			break;
        		case 0x8002:    /* 0x8002 :  extended diagnosis alarm                    */
        			printf(CRLF"PNIO_ALARM_DIAGNOSTIC -> extended diagnosis alarm  at %s" CRLF, time_str);

        			printf("\tChannelNumber      = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.ext_diag.ChannelNumber));
					printf("\tChannelProperties   = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.ext_diag.ChannelProperties));
					printf("\tChannelErrorType    = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.ext_diag.ChannelErrorType));
					printf("\tExtChannelErrorType = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.ext_diag.ExtChannelErrorType));
					printf("\tExtChannelAddValue  = 0x%08x"CRLF, SWAP_32(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.ext_diag.ExtChannelAddValue));

        			break;
        		case 0x8100:    /* 0x8100 :  maintenance diagnosis alarm                 */
        			/********************************************************************/
        			/*  check if maintenance required or demanded!                      */
        			/********************************************************************/
        			if( SWAP_32(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.maintInfo.MaintenanceStatus) & 0x1)
        				printf(CRLF"PNIO_ALARM_DIAGNOSTIC -> maintenance required alarm at %s" CRLF, time_str);
        			else if( SWAP_32(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.maintInfo.MaintenanceStatus) & 0x2)
        			    printf(CRLF"PNIO_ALARM_DIAGNOSTIC -> maintenance demanded alarm at %s" CRLF, time_str);
        			else{
        				printf(CRLF"!!!ERROR!!! PNIO_ALARM_DIAGNOSTIC -> unknown maintenance (0x%08x) alarm at %s" CRLF, SWAP_32(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.maintInfo.MaintenanceStatus), time_str);
        				break;
        			}

        			/********************************************************************/
        			/*   check if diagnostic or extended diagnostic maintenance alarm   */
        			/********************************************************************/
        			if( SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.UserStrucIdent) == 0x8000){
        				/********************************************************************/
						/*   diagnostic maintenance alarm 							        */
						/********************************************************************/
        				printf("\tChannelNumber    = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.ChannelNumber));
						printf("\tChannelProperties = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.ChannelProperties));
						printf("\tChannelErrorType  = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.ChannelErrorType));
        			} else if( SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.UserStrucIdent) == 0x8002){
        				/********************************************************************/
						/*   extended diagnostic maintenance alarm 					        */
						/********************************************************************/
            			printf("\tChannelNumber      = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.mext_diag.ChannelNumber));
    					printf("\tChannelProperties   = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.mext_diag.ChannelProperties));
    					printf("\tChannelErrorType    = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.mext_diag.ChannelErrorType));
    					printf("\tExtChannelErrorType = 0x%04x"CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.mext_diag.ExtChannelErrorType));
    					printf("\tExtChannelAddValue  = 0x%08x"CRLF, SWAP_32(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.mext_diag.ExtChannelAddValue));
        			}else {
        				printf(CRLF"!!!ERROR!!! unknown maintenance alarm USI=0x%04x at %s" CRLF, SWAP_16(pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UAData.m_diag.UserStrucIdent), time_str);
        			}

        			break;
        		default:
        			printf(CRLF"PNIO_ALARM_DIAGNOSTIC USI = 0x%04x at %s" CRLF, pCbfPrm->AlarmInd.pAlarmData->AlarmAinfo.UserStrucIdent, time_str);
        			break;
        	}

            break;

        case PNIO_ALARM_DIAGNOSTIC_DISAPPEARS:
        	printf(CRLF"PNIO_ALARM_DIAGNOSTIC_DISAPPEARS at %s" CRLF, time_str);
        	break;

        case PNIO_ALARM_PROCESS:
            printf(CRLF"PNIO_ALARM_PROCESS at %s" CRLF, time_str);
            break;
        case PNIO_ALARM_PULL:
            printf(CRLF"PNIO_ALARM_PULL at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_PLUG:
            printf(CRLF"PNIO_ALARM_PLUG at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_STATUS:
            printf(CRLF"PNIO_ALARM_STATUS at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_UPDATE:
            printf(CRLF"PNIO_ALARM_UPDATE at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_REDUNDANCY:
            printf(CRLF"PNIO_ALARM_REDUNDACY at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
            printf(CRLF"PNIO_ALARM_CONTROLLED_BY_SUPERVISOR at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
            printf(CRLF"PNIO_ALARM_RELEASED_BY_SUPERVISOR at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_PLUG_WRONG:
            printf(CRLF"PNIO_ALARM_PLUG_WRONG at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_RETURN_OF_SUBMODULE:
            printf(CRLF"PNIO_ALARM_RETURN_OF_SUBMODULE  at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_DEV_FAILURE:
            printf(CRLF"PNIO_ALARM_DEV_FAILURE at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_DEV_RETURN:
            printf(CRLF"PNIO_ALARM_DEV_RETURN at %s" CRLF, time_str);
            break;
        default:
            printf(CRLF"callback_for_alarm_indication called with wrong type at %s" CRLF, time_str);
            break;
        }

        alarm_dump(pCbfPrm->AlarmInd.pAlarmData);
    }


}

/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* a device was activated or deactivated                       */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/
void callback_for_device_activation(PNIO_CBE_PRM * pCbfPrm)
{

    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/
    char time_str[13];
    GetSystemTimeString(time_str);
    switch (pCbfPrm->DevActConf.Mode) {
    case PNIO_DA_TRUE:
        printf(CRLF"device activation was send device with Address %d with result %x at %s" CRLF,
            pCbfPrm->DevActConf.pAddr->u.Addr, pCbfPrm->DevActConf.Result, time_str);
        break;
    case PNIO_DA_FALSE:
        printf(CRLF"device deactivation was send to device with Address %d with result %x at %s"
            CRLF, pCbfPrm->DevActConf.pAddr->u.Addr, pCbfPrm->DevActConf.Result, time_str);
        break;
    };
}

void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm)
{
    printf (CRLF"CP_STOP_REQUEST received, terminating Programm ..."CRLF);
    cp_stop_req = true;
};
/*****************************************************************/
/* This functions reads a character from console                 */
/*****************************************************************/

int getCharWithTimeout()
{
    int key = 0;
#ifndef WIN32
    struct pollfd pollfd[1];
    static int init = 0;
    static struct termios termiosOld;
    static struct termios termios;

    if(!init) {
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


/***************************************************************************/
/* GetSystemTimeString needs a char array with length of at least 13 Bytes */
/* returns the local time hh:mm:ss.mmm                                     */
/***************************************************************************/

void GetSystemTimeString(char *tstring)
{

#ifdef WIN32
    #define my_timeb  _timeb
    #define my_ftime  _ftime
#else
    #define my_timeb  timeb
    #define my_ftime  ftime
#endif

    struct my_timeb timebuffer;
    char *timeline;

    my_ftime( &timebuffer );
    timeline = ctime( & ( timebuffer.time ) );


    sprintf(tstring,"%.8s.%03u", &timeline[11], timebuffer.millitm);


    return;
}
